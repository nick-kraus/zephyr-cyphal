#include <stdlib.h>
#include <zephyr/drivers/can/can_fake.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/slist.h>
#include <zephyr/ztest.h>

struct can_fff_history_item {
    struct can_frame frame;
    sys_snode_t node;
};
static sys_slist_t can_fff_history;

static void can_fff_frame_history_append(const struct can_frame* frame) {
    struct can_fff_history_item* item = malloc(sizeof(struct can_fff_history_item));
    memcpy(&item->frame, frame, sizeof(struct can_frame));
    sys_slist_append(&can_fff_history, &item->node);
}

static struct can_frame can_fff_frame_history_get_next(void) {
    zassert_false(sys_slist_is_empty(&can_fff_history), "No frames in history.");

    sys_snode_t* item_node = sys_slist_get_not_empty(&can_fff_history);
    struct can_fff_history_item* item = SYS_SLIST_CONTAINER(item_node, item, node);
    struct can_frame frame = item->frame;

    free(item);
    return frame;
}

static volatile int32_t can_fff_send_custom_return_val = 0;
static int32_t can_fff_send_custom(const struct device* dev,
                                   const struct can_frame* frame,
                                   k_timeout_t timeout,
                                   can_tx_callback_t callback,
                                   void* user_data) {
    /* Add the received frame to the history. */
    if (can_fff_send_custom_return_val == 0) {
        can_fff_frame_history_append(frame);
    }

    /* Running the callback will give a semaphore, preventing deadlock. */
    if (callback) { callback(dev, 0, user_data); }

    return can_fff_send_custom_return_val;
}

void can_fff_history_reset(void) {
    while (!sys_slist_is_empty(&can_fff_history)) {
        can_fff_frame_history_get_next();
    }
}

void can_fff_ztest_before(void) {
    /* Ensure the custom fake handler is always set. */
    fake_can_send_fake.custom_fake = can_fff_send_custom;

    /* Fresh history for every new test. */
    can_fff_history_reset();
}

void can_fff_assert_frames_empty(void) {
    zassert_true(sys_slist_is_empty(&can_fff_history), "Frames remaining in history.");
}

void can_fff_assert_popped_frame_equal(struct can_frame frame) {
    struct can_frame hist = can_fff_frame_history_get_next();

    zassert_equal(frame.id,
                  hist.id,
                  "Expected frame id: 0x%08x, history frame id: 0x%08x",
                  frame.id,
                  hist.id);
    zassert_equal(frame.dlc,
                  hist.dlc,
                  "Expected frame dlc: %u, history frame dlc: %u",
                  frame.dlc,
                  hist.dlc);
    uint8_t bytes = can_dlc_to_bytes(frame.dlc);
    for (uint8_t b = 0; b < bytes; b++) {
        zassert_equal(frame.data[b],
                      hist.data[b],
                      "Expected frame data[%u]: 0x%02x, history frame data[%u]: 0x%02x",
                      b,
                      frame.data[b],
                      b,
                      hist.data[b]);
    }
}

void can_fff_set_send_status(int32_t status) {
    can_fff_send_custom_return_val = status;
}
