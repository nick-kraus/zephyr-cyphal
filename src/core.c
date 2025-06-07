#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/slist.h>

LOG_MODULE_REGISTER(cyphal, CONFIG_CAN_LOG_LEVEL);

#include "zcyphal/core.h"

#define CYPHAL_MAX_NODE_ID (127)
#define CYPHAL_MAX_SERVICE_ID (511)
#define CYPHAL_MAX_SUBJECT_ID (8191)

#define CYPHAL_FRAME_MTU COND_CODE_1(CONFIG_ZCYPHAL_CAN_FD, (64), (8))

static int32_t zcy_make_canid(uint8_t priority,
                              bool is_service,
                              bool is_request,
                              uint16_t service_id,
                              uint16_t subject_id,
                              uint8_t destination_id,
                              uint8_t source_id) {
    /* Error on arguments beyond their maximum value. */
    if (priority > ZCY_PRIO_OPTIONAL || service_id > CYPHAL_MAX_SERVICE_ID ||
        subject_id > CYPHAL_MAX_SUBJECT_ID || destination_id > CYPHAL_MAX_NODE_ID ||
        source_id > CYPHAL_MAX_NODE_ID) {
        return -EINVAL;
    }

    const int32_t priority_shift = 26;
    const int32_t priority_mask = GENMASK(28, 26);
    const int32_t service = BIT(25);
    const int32_t request = BIT(24);
    const int32_t message_reserved = BIT(22) | BIT(21);
    const int32_t service_id_shift = 14;
    const int32_t service_id_mask = GENMASK(22, 14);
    const int32_t subject_id_shift = 8;
    const int32_t subject_id_mask = GENMASK(20, 8);
    const int32_t destination_id_shift = 7;
    const int32_t destination_id_mask = GENMASK(13, 7);
    const int32_t source_id_mask = GENMASK(6, 0);

    int32_t canid = ((int32_t)priority << priority_shift) & priority_mask;
    if (is_service) {
        canid |= service;
        canid |= is_request ? request : 0;
        canid |= (service_id << service_id_shift) & service_id_mask;
        canid |= (destination_id << destination_id_shift) & destination_id_mask;
    } else {
        canid |= message_reserved;
        canid |= (subject_id << subject_id_shift) & subject_id_mask;
    }

    canid |= source_id & source_id_mask;
    return canid;
}

static uint8_t zcy_make_tail_byte(bool start,
                                  bool end,
                                  bool toggle,
                                  uint8_t transfer_id) {
    const uint8_t start_bit = BIT(7);
    const uint8_t end_bit = BIT(6);
    const uint8_t toggle_bit = BIT(5);
    const uint8_t transfer_id_mask = 0x1F;

    return (transfer_id & transfer_id_mask) | (start ? start_bit : 0) |
           (end ? end_bit : 0) | (toggle ? toggle_bit : 0);
}

static void zcy_tx_push(zcy_inst_t* inst, zcy_tx_t* xfer) {
    zcy_tx_t* prev = NULL;
    zcy_tx_t* cur;

    SYS_SLIST_FOR_EACH_CONTAINER(&inst->tx_queue, cur, node) {
        if (xfer->id < cur->id) { break; }
        prev = cur;
    }

    if (prev == NULL) {
        sys_slist_prepend(&inst->tx_queue, &xfer->node);
    } else {
        sys_slist_insert(&inst->tx_queue, &prev->node, &xfer->node);
    }
}

void zcy_tx_callback(const struct device* dev, int error, void* user_data) {
    zcy_tx_t* xfer = (zcy_tx_t*)user_data;

    if (error != 0) {
        xfer->status = error;
        atomic_clear(&xfer->pending);
    } else {
        /* Be extra cautious to not decrement past 0. */
        if (!atomic_cas(&xfer->pending, 1, 0)) { atomic_dec(&xfer->pending); }
    }

    k_work_schedule(&xfer->inst->tx_work, K_NO_WAIT);
}

static void zcy_tx_work_handler(struct k_work* work) {
    struct k_work_delayable* dwork = k_work_delayable_from_work(work);
    zcy_inst_t* inst = CONTAINER_OF(dwork, zcy_inst_t, tx_work);

    int32_t ret = k_mutex_lock(&inst->mutex, K_NO_WAIT);
    if (ret < 0) {
        /* Wait some period before rescheduling, to reduce thread contention. */
        k_work_schedule(&inst->tx_work, K_USEC(100));
        return;
    }

    zcy_tx_t* xfer = SYS_SLIST_PEEK_HEAD_CONTAINER(&inst->tx_queue, xfer, node);
    if (!xfer) {
        /* No more pending transfers, no need to resubmit work. */
        k_mutex_unlock(&inst->mutex);
        return;
    }

    bool expired = sys_timepoint_expired(xfer->end);
    if (expired || atomic_get(&xfer->pending) == 0) {
        if (expired && atomic_get(&xfer->pending) > 0) { xfer->status = -ETIMEDOUT; }
        goto xfer_remove;
    }

    size_t payload_remaining = xfer->payload_len - xfer->payload_written;
    bool start = xfer->payload_written == 0;
    bool end = (payload_remaining + (start ? 0 : 2)) < CYPHAL_FRAME_MTU;
    uint8_t tail = zcy_make_tail_byte(start, end, xfer->toggle, xfer->transfer_id);

    struct can_frame frame;
    frame.id = xfer->id;
    frame.flags =
        CAN_FRAME_IDE | COND_CODE_1(ZCYPHAL_CAN_FD, (CAN_FRAME_FDF | CAN_FRAME_BRS), (0));

    /* Write as much remaining payload as possible. */
    uint8_t frame_payload_len = 0;
    if (payload_remaining > 0) {
        frame_payload_len = MIN(payload_remaining, (CYPHAL_FRAME_MTU - 1));
        memcpy(frame.data, &xfer->payload[xfer->payload_written], frame_payload_len);

        if (!start || !end) {
            xfer->crc = crc16_itu_t(
                xfer->crc, &xfer->payload[xfer->payload_written], frame_payload_len);
        }
    }

    /* (Maybe) Write padding if this is the final frame. */
    uint8_t frame_padding_len = 0;
    if (end) {
        /* If start frame as well, single frame transfer, no CRC. */
        uint8_t frame_len = start ? frame_payload_len + 1 : frame_payload_len + 3;
        frame_padding_len = can_dlc_to_bytes(can_bytes_to_dlc(frame_len)) - frame_len;
        memset(&frame.data[frame_payload_len], 0, frame_padding_len);

        xfer->crc =
            crc16_itu_t(xfer->crc, &frame.data[frame_payload_len], frame_padding_len);
    }

    /* (Maybe) Write CRC if entire payload is written. */
    uint8_t frame_crc_len = 0;
    if (!start && payload_remaining == frame_payload_len) {
        /* Minimum of space remaining in frame for CRC or CRC bytes to be written. */
        frame_crc_len =
            MIN((CYPHAL_FRAME_MTU - 1) - frame_payload_len - frame_padding_len,
                2 - xfer->crc_written);

        if (frame_crc_len >= 2 && xfer->crc_written == 0) {
            frame.data[frame_payload_len + frame_padding_len] = (uint8_t)(xfer->crc >> 8);
            frame.data[frame_payload_len + frame_padding_len + 1] = (uint8_t)xfer->crc;
        } else if (frame_crc_len >= 1 && xfer->crc_written == 0) {
            frame.data[frame_payload_len + frame_padding_len] = (uint8_t)(xfer->crc >> 8);
        } else if (frame_crc_len >= 1) {
            frame.data[frame_payload_len + frame_padding_len] = (uint8_t)xfer->crc;
        }
    }

    /* Write tail byte. */
    frame.data[frame_payload_len + frame_padding_len + frame_crc_len] = tail;
    frame.dlc =
        can_bytes_to_dlc(frame_payload_len + frame_padding_len + frame_crc_len + 1);

    k_timeout_t send_timeout = sys_timepoint_timeout(xfer->end);
    ret = can_send(inst->canbus, &frame, send_timeout, zcy_tx_callback, xfer);
    if (ret == -EAGAIN) {
        /* CAN driver is currently busy, reschedule work after a delay. */
        k_work_schedule(&inst->tx_work, K_USEC(100));
        k_mutex_unlock(&inst->mutex);
        return;
    } else if (ret < 0) {
        /* Fail the transfer.*/
        xfer->status = ret;
        goto xfer_remove;
    }

    /* CAN frame successfully sent to tx mailbox, the next work item will be scheduled
     * from the callback after transmission. */

    /* Advance transfer state. */
    xfer->payload_written += frame_payload_len;
    xfer->crc_written += frame_crc_len;
    xfer->toggle = !xfer->toggle;

    k_mutex_unlock(&inst->mutex);
    return;

xfer_remove:
    sys_slist_get_not_empty(&inst->tx_queue);

    if (xfer->done) { k_sem_give(xfer->done); }
    atomic_clear(&xfer->pending);

    if (!sys_slist_is_empty(&inst->tx_queue)) {
        k_work_schedule(&inst->tx_work, K_NO_WAIT);
    }
    k_mutex_unlock(&inst->mutex);
}

int32_t zcy_init(zcy_inst_t* inst, const struct device* canbus, uint8_t node_id) {
    int32_t ret;

    if (!inst || node_id > CYPHAL_MAX_NODE_ID) {
        return -EINVAL;
    } else if (!device_is_ready(canbus)) {
        return -ENODEV;
    }

    inst->canbus = canbus;
    inst->node_id = node_id;
    ret = k_mutex_init(&inst->mutex);
    if (ret < 0) { return ret; }
    sys_slist_init(&inst->tx_queue);
    k_work_init_delayable(&inst->tx_work, zcy_tx_work_handler);

    return 0;
}

int32_t zcy_publisher_init(zcy_publisher_t* pub, zcy_inst_t* inst, uint16_t subject_id) {
    if (!pub || !inst || subject_id > CYPHAL_MAX_SUBJECT_ID) { return -EINVAL; }

    memset(pub, 0, sizeof(zcy_publisher_t));

    pub->xfer.inst = inst;
    pub->xfer.end = sys_timepoint_calc(K_NO_WAIT);
    pub->xfer.transfer_id = 0x1F;
    pub->subject_id = subject_id;

    return 0;
}

int32_t zcy_publish(zcy_publisher_t* pub,
                    zcy_prio_t priority,
                    uint8_t* payload,
                    size_t len,
                    k_timeout_t timeout,
                    struct k_sem* done) {
    if (!pub || !payload) { return -EINVAL; }
    atomic_val_t num_frames =
        len < CYPHAL_FRAME_MTU
            ? 1
            : (atomic_val_t)DIV_ROUND_UP(len + 2, (CYPHAL_FRAME_MTU - 1));
    if (!atomic_cas(&pub->xfer.pending, 0, num_frames)) { return -EALREADY; }

    k_timepoint_t end = sys_timepoint_calc(timeout);
    zcy_inst_t* inst = pub->xfer.inst;

    int32_t ret =
        zcy_make_canid(priority, false, false, 0, pub->subject_id, 0, inst->node_id);
    if (ret < 0) { goto err; }
    uint32_t canid = ret;

    pub->xfer.id = canid;
    pub->xfer.end = end;
    pub->xfer.payload = payload;
    pub->xfer.payload_len = len;
    pub->xfer.payload_written = 0;
    /* Toggle always one for first frame of transfer. */
    pub->xfer.toggle = 1;
    /* Increment transfer ID before pushing to queue. */
    pub->xfer.transfer_id = (pub->xfer.transfer_id + 1) & 0x1F;
    pub->xfer.crc_written = 0;
    /* Only generate CRC for multi-frame transfers. */
    pub->xfer.crc = 0xFFFF;
    pub->xfer.done = done;

    ret = k_mutex_lock(&inst->mutex, timeout);
    if (ret < 0) { goto err; }
    zcy_tx_push(inst, &pub->xfer);
    k_mutex_unlock(&inst->mutex);

    ret = k_work_schedule(&inst->tx_work, K_NO_WAIT);
    if (ret < 0) { goto err; }

    return 0;

err:
    atomic_clear(&pub->xfer.pending);
    return ret;
}

int32_t zcy_publish_sync(zcy_publisher_t* pub,
                         zcy_prio_t priority,
                         uint8_t* payload,
                         size_t len,
                         k_timeout_t timeout) {
    struct k_sem sem;
    int32_t ret = k_sem_init(&sem, 0, 1);
    if (ret < 0) { return ret; }

    ret = zcy_publish(pub, priority, payload, len, timeout, &sem);
    if (ret < 0) { return ret; }

    k_sem_take(&sem, K_FOREVER);
    return ret;
}
