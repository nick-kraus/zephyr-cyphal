#include <stdint.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/slist.h>

LOG_MODULE_DECLARE(zyphal);

#include "zyphal/core.h"

#define ZYPHAL_FRAME_MTU COND_CODE_1(CONFIG_ZYPHAL_CAN_FD, (64), (8))

#define CANID_PRIO_SHIFT (26)
#define CANID_PRIO_MASK GENMASK(28, 26)
#define CANID_SERVICE_BIT BIT(25)
#define CANID_REQUEST_BIT BIT(24)
#define CANID_MSG_RESERVED_BITS (BIT(22) | BIT(21))
#define CANID_SERVICE_ID_SHIFT (14)
#define CANID_SERVICE_ID_MASK GENMASK(22, 14)
#define CANID_SUBJECT_ID_SHIFT (8)
#define CANID_SUBJECT_ID_MASK GENMASK(20, 8)
#define CANID_DESTINATION_ID_SHIFT (7)
#define CANID_DESTINATION_ID_MASK GENMASK(13, 7)
#define CANID_SOURCE_ID_MASK GENMASK(6, 0)

#define TAIL_START_BIT BIT(7)
#define TAIL_END_BIT BIT(6)
#define TAIL_TOGGLE_BIT BIT(5)
#define TAIL_TRANSFER_ID_MASK GENMASK(4, 0)

#define TAIL_BYTE_SIZE (1)
#define MULTI_FRAME_CRC_SIZE (2)

static uint32_t make_canid(uint8_t priority,
                           bool is_service,
                           bool is_request,
                           uint16_t service_id,
                           uint16_t subject_id,
                           uint8_t destination_id,
                           uint8_t source_id) {
    uint32_t canid = (priority << CANID_PRIO_SHIFT) & CANID_PRIO_MASK;
    if (is_service) {
        canid |= CANID_SERVICE_BIT;
        canid |= is_request ? CANID_REQUEST_BIT : 0;
        canid |= (service_id << CANID_SERVICE_ID_SHIFT) & CANID_SERVICE_ID_MASK;
        canid |=
            (destination_id << CANID_DESTINATION_ID_SHIFT) & CANID_DESTINATION_ID_MASK;
    } else {
        canid |= CANID_MSG_RESERVED_BITS;
        canid |= (subject_id << CANID_SUBJECT_ID_SHIFT) & CANID_SUBJECT_ID_MASK;
    }
    canid |= source_id & CANID_SOURCE_ID_MASK;
    return canid;
}

static uint8_t make_tail_byte(bool start, bool end, bool toggle, uint8_t transfer_id) {
    return (transfer_id & TAIL_TRANSFER_ID_MASK) | (start ? TAIL_START_BIT : 0) |
           (end ? TAIL_END_BIT : 0) | (toggle ? TAIL_TOGGLE_BIT : 0);
}

struct built_frame {
    struct can_frame frame;
    size_t payload_len;
    uint8_t crc_len;
};

static struct built_frame build_next_frame(zyphal_tx_t* tx) {
    bool start = tx->payload_written == 0;
    bool end = atomic_get(&tx->pending) == 1;
    bool single = start && end;
    size_t payload_remaining = tx->payload_len - tx->payload_written;

    struct built_frame out;
    memset(&out, 0, sizeof(out));

    /* Write as much payload data as frame space allows. */
    out.payload_len = MIN(payload_remaining, (ZYPHAL_FRAME_MTU - TAIL_BYTE_SIZE));
    if (out.payload_len > 0) {
        memcpy(out.frame.data, &tx->payload[tx->payload_written], out.payload_len);
        if (!single) {
            tx->crc =
                crc16_itu_t(tx->crc, &tx->payload[tx->payload_written], out.payload_len);
        }
    }

    /* Calculate how much CRC can be written into the frame, before padding length.
     * Padding will only be added if the full CRC fits. */
    size_t crc_remaining = single ? 0 : MULTI_FRAME_CRC_SIZE - tx->crc_written;
    size_t crc_space = (ZYPHAL_FRAME_MTU - TAIL_BYTE_SIZE) - out.payload_len;
    out.crc_len = MIN(crc_remaining, crc_space);

    /* Padding is only added on the last frame, if there is space in between the message
     * size (with CRC if applicable) and the next largest DLC size. */
    size_t frame_dlc = can_bytes_to_dlc(out.payload_len + out.crc_len + TAIL_BYTE_SIZE);
    size_t padding_len =
        can_dlc_to_bytes(frame_dlc) - (out.payload_len + out.crc_len + TAIL_BYTE_SIZE);
    if (padding_len > 0) {
        memset(&out.frame.data[out.payload_len], 0, padding_len);
        if (!single) {
            tx->crc = crc16_itu_t(tx->crc, &out.frame.data[out.payload_len], padding_len);
        }
    }

    /* Write as many crc bytes as will fit. */
    for (int i = 0; i < out.crc_len; i++) {
        uint8_t crc_byte =
            (tx->crc_written + i == 0) ? (uint8_t)(tx->crc >> 8) : (uint8_t)tx->crc;
        out.frame.data[out.payload_len + padding_len + i] = crc_byte;
    }

    /* Write tail byte. */
    uint8_t tail = make_tail_byte(start, end, tx->toggle, tx->transfer_id);
    out.frame.data[out.payload_len + padding_len + out.crc_len] = tail;

    out.frame.id = tx->id;
    out.frame.flags =
        CAN_FRAME_IDE |
        COND_CODE_1(CONFIG_ZYPHAL_CAN_FD, (CAN_FRAME_FDF | CAN_FRAME_BRS), (0));
    out.frame.dlc =
        can_bytes_to_dlc(out.payload_len + padding_len + out.crc_len + TAIL_BYTE_SIZE);

    return out;
}

static void tx_queue_push(zyphal_inst_t* inst, zyphal_tx_t* tx) {
    zyphal_tx_t* prev = NULL;
    zyphal_tx_t* cur;

    SYS_SLIST_FOR_EACH_CONTAINER(&inst->tx_queue, cur, node) {
        if (tx->id < cur->id) { break; }
        prev = cur;
    }

    if (prev == NULL) {
        sys_slist_prepend(&inst->tx_queue, &tx->node);
    } else {
        sys_slist_insert(&inst->tx_queue, &prev->node, &tx->node);
    }
}

static void tx_queue_remove_head(zyphal_inst_t* inst, int32_t status) {
    if (k_mutex_lock(&inst->mutex, K_NO_WAIT) < 0) { return; }
    sys_snode_t* node = sys_slist_get_not_empty(&inst->tx_queue);
    zyphal_tx_t* tx = SYS_SLIST_CONTAINER(node, tx, node);

    bool expired = sys_timepoint_expired(tx->end);
    bool pending = atomic_get(&tx->pending) > 0;
    if (expired && pending && status == 0) { status = -ETIMEDOUT; }
    atomic_clear(&tx->pending);

    if (tx->done_cb) { tx->done_cb(tx->done_user_data, status); }
    k_mutex_unlock(&inst->mutex);
}

static zyphal_tx_t* tx_queue_get_next(zyphal_inst_t* inst) {
    while (!sys_slist_is_empty(&inst->tx_queue)) {
        zyphal_tx_t* tx = SYS_SLIST_PEEK_HEAD_CONTAINER(&inst->tx_queue, tx, node);

        bool expired = sys_timepoint_expired(tx->end);
        bool pending = atomic_get(&tx->pending) > 0;
        if (!expired && pending) { return tx; }

        tx_queue_remove_head(inst, 0);
    }

    return NULL;
}

void can_send_callback(const struct device* dev, int error, void* user_data) {
    zyphal_tx_t* tx = (zyphal_tx_t*)user_data;
    atomic_val_t old = atomic_dec(&tx->pending);

    if (error != 0 || old <= 1) { tx_queue_remove_head(tx->inst, error); }

    k_work_schedule(&tx->inst->tx_work, K_NO_WAIT);
}

void zyphal_tx_work_handler(struct k_work* work) {
    struct k_work_delayable* dwork = k_work_delayable_from_work(work);
    zyphal_inst_t* inst = CONTAINER_OF(dwork, zyphal_inst_t, tx_work);

    int32_t ret = k_mutex_lock(&inst->mutex, K_NO_WAIT);
    if (ret < 0) {
        /* Wait some period before rescheduling, to reduce thread contention. */
        k_work_schedule(&inst->tx_work, K_USEC(100));
        return;
    }

    /* Retrieve the next non-expired transfer with pending frames. */
    zyphal_tx_t* tx = tx_queue_get_next(inst);
    if (tx == NULL) {
        /* No more pending transfers, no need to resubmit work. */
        goto end;
    }

    struct built_frame next = build_next_frame(tx);
    ret = can_send(inst->canbus, &next.frame, K_NO_WAIT, can_send_callback, tx);
    if (ret == -EAGAIN) {
        /* CAN driver is currently busy, reschedule work after a delay. */
        k_work_schedule(&inst->tx_work, K_USEC(100));
        goto end;
    } else if (ret < 0) {
        /* Fail the transfer.*/
        tx_queue_remove_head(inst, ret);
        k_work_schedule(&inst->tx_work, K_NO_WAIT);
        goto end;
    }

    /* CAN frame successfully sent to tx mailbox, the next work item will be scheduled
     * from the callback after transmission. */

    /* Advance transfer state. */
    tx->payload_written += next.payload_len;
    tx->crc_written += next.crc_len;
    tx->toggle = !tx->toggle;

end:
    k_mutex_unlock(&inst->mutex);
    return;
}

int32_t zyphal_tx_init(zyphal_inst_t* inst, zyphal_tx_t* tx) {
    if (!inst || !tx) { return -EINVAL; }

    memset(tx, 0, sizeof(zyphal_tx_t));

    tx->inst = inst;
    tx->end = sys_timepoint_calc(K_NO_WAIT);
    /* Initialized to max value so the first publish call produces transfer_id 0. */
    tx->transfer_id = TAIL_TRANSFER_ID_MASK;

    return 0;
}

int32_t zyphal_publish(zyphal_tx_t* tx,
                       zyphal_prio_t priority,
                       uint16_t subject_id,
                       uint8_t* payload,
                       size_t len,
                       k_timeout_t timeout,
                       zyphal_tx_done_cb_t cb,
                       void* user_data) {
    if (!tx || priority > ZYPHAL_PRIO_OPTIONAL || subject_id > ZYPHAL_MAX_SUBJECT_ID ||
        (!payload && len > 0)) {
        return -EINVAL;
    }
    atomic_val_t num_frames =
        len < ZYPHAL_FRAME_MTU
            ? 1
            : (atomic_val_t)DIV_ROUND_UP(len + MULTI_FRAME_CRC_SIZE,
                                         ZYPHAL_FRAME_MTU - TAIL_BYTE_SIZE);
    if (!atomic_cas(&tx->pending, 0, num_frames)) { return -EALREADY; }

    k_timepoint_t end = sys_timepoint_calc(timeout);
    zyphal_inst_t* inst = tx->inst;

    tx->id = make_canid(priority, false, false, 0, subject_id, 0, inst->node_id);
    tx->end = end;
    tx->payload = payload;
    tx->payload_len = len;
    tx->payload_written = 0;
    /* Toggle always one for first frame of transfer. */
    tx->toggle = 1;
    /* Increment transfer ID before pushing to queue. */
    tx->transfer_id = (tx->transfer_id + 1) & TAIL_TRANSFER_ID_MASK;
    tx->crc_written = 0;
    tx->crc = UINT16_MAX;
    tx->done_cb = cb;
    tx->done_user_data = user_data;

    int32_t ret = k_mutex_lock(&inst->mutex, timeout);
    if (ret < 0) { goto err; }
    tx_queue_push(inst, tx);
    k_mutex_unlock(&inst->mutex);

    ret = k_work_schedule(&inst->tx_work, K_NO_WAIT);
    if (ret < 0) { goto err; }

    return 0;

err:
    atomic_clear(&tx->pending);
    return ret;
}

struct publish_done_data {
    struct k_sem sem;
    int32_t status;
};

static void publish_done_cb(void* user_data, int32_t status) {
    struct publish_done_data* data = (struct publish_done_data*)user_data;
    data->status = status;
    k_sem_give(&data->sem);
}

int32_t zyphal_publish_wait(zyphal_tx_t* tx,
                            zyphal_prio_t priority,
                            uint16_t subject_id,
                            uint8_t* payload,
                            size_t len,
                            k_timeout_t timeout) {
    struct publish_done_data data = {.status = 0};
    int32_t ret = k_sem_init(&data.sem, 0, 1);
    if (ret < 0) { return ret; }

    ret = zyphal_publish(
        tx, priority, subject_id, payload, len, timeout, publish_done_cb, &data);
    if (ret < 0) { return ret; }

    k_sem_take(&data.sem, K_FOREVER);
    return data.status;
}

bool zyphal_publish_pending(zyphal_tx_t* tx) {
    if (!tx) { return false; }
    return atomic_get(&tx->pending) > 0;
}
