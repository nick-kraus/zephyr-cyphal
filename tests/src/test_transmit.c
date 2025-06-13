#include <stdint.h>
#include <stdlib.h>
#include <zephyr/drivers/can/can_fake.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "can_fff.h"
#include "zyphal/core.h"

DEFINE_FFF_GLOBALS;

#define NODE_ID (0x55)
#define SUBJECT_ID (0x1234)

#define FILL_VAL(len, val) ((uint8_t)(val))
#define FILL_ARRAY(len, val) LISTIFY(len, FILL_VAL, (, ), val)

const struct device* canbus = DEVICE_DT_GET(DT_NODELABEL(fake_can));
zyphal_inst_t inst;

static void transmit_suite_before(void* f) {
    zassert_true(device_is_ready(canbus));
    zassert_ok(zyphal_init(&inst, canbus, NODE_ID));

    can_fff_ztest_before();
}

ZTEST(transmit, single_frame_message) {
    zyphal_tx_t tx;
    zassert_ok(zyphal_tx_init(&inst, &tx));

    /* Maximum sized single frame. */
    uint8_t pl1[] = {FILL_ARRAY(63, 0x11)};
    zassert_ok(
        zyphal_publish_wait(&tx, ZYPHAL_PRIO_NOMINAL, SUBJECT_ID, pl1, 63, K_MSEC(10)));
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 15, .data = {FILL_ARRAY(63, 0x11), 0xE0}});
    can_fff_assert_frames_empty();

    /* Single frame with padding. */
    uint8_t pl2[] = {FILL_ARRAY(32, 0x22)};
    zassert_ok(
        zyphal_publish_wait(&tx, ZYPHAL_PRIO_NOMINAL, SUBJECT_ID, pl2, 32, K_MSEC(10)));
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x10723455,
                           .dlc = 14,
                           .data = {FILL_ARRAY(32, 0x22), FILL_ARRAY(15, 0), 0xE1}});
    can_fff_assert_frames_empty();

    /* Minimum sized single frame (no payload). */
    uint8_t pl3[] = {};
    zassert_ok(
        zyphal_publish_wait(&tx, ZYPHAL_PRIO_NOMINAL, SUBJECT_ID, pl3, 0, K_MSEC(10)));
    can_fff_assert_popped_frame_equal((struct can_frame){.id = 0x10723455,
                                                         .dlc = 1,
                                                         .data = {
                                                             0xE2,
                                                         }});
    can_fff_assert_frames_empty();
}

ZTEST(transmit, multi_frame_message) {
    zyphal_tx_t tx;
    zassert_ok(zyphal_tx_init(&inst, &tx));

    /* Three full frames. */
    uint8_t pl1[] = {FILL_ARRAY(187, 0x33)};
    zassert_ok(
        zyphal_publish_wait(&tx, ZYPHAL_PRIO_NOMINAL, SUBJECT_ID, pl1, 187, K_MSEC(10)));
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 15, .data = {FILL_ARRAY(63, 0x33), 0xA0}});
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 15, .data = {FILL_ARRAY(63, 0x33), 0x00}});
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 15, .data = {FILL_ARRAY(61, 0x33), 0x95, 0x90, 0x60}});
    can_fff_assert_frames_empty();

    /* Three frames, CRC by itself on last frame. */
    uint8_t pl2[] = {FILL_ARRAY(126, 0x44)};
    zassert_ok(
        zyphal_publish_wait(&tx, ZYPHAL_PRIO_NOMINAL, SUBJECT_ID, pl2, 126, K_MSEC(10)));
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 15, .data = {FILL_ARRAY(63, 0x44), 0xA1}});
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 15, .data = {FILL_ARRAY(63, 0x44), 0x01}});
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x10723455, .dlc = 3, .data = {0x27, 0xF0, 0x61}});
    can_fff_assert_frames_empty();

    /* Three frames, CRC split between frames. */
    uint8_t pl3[] = {FILL_ARRAY(125, 0x55)};
    zassert_ok(
        zyphal_publish_wait(&tx, ZYPHAL_PRIO_NOMINAL, SUBJECT_ID, pl3, 125, K_MSEC(10)));
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 15, .data = {FILL_ARRAY(63, 0x55), 0xA2}});
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 15, .data = {FILL_ARRAY(62, 0x55), 0xEE, 0x02}});
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x10723455, .dlc = 2, .data = {0x63, 0x62}});
    can_fff_assert_frames_empty();

    /* Two frames, CRC after padding. */
    uint8_t pl4[] = {FILL_ARRAY(81, 0x66)};
    zassert_ok(
        zyphal_publish_wait(&tx, ZYPHAL_PRIO_NOMINAL, SUBJECT_ID, pl4, 81, K_MSEC(10)));
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 15, .data = {FILL_ARRAY(63, 0x66), 0xA3}});
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455,
        .dlc = 12,
        .data = {FILL_ARRAY(18, 0x66), FILL_ARRAY(3, 0), 0xDE, 0x2D, 0x43}});
    can_fff_assert_frames_empty();
}

static void publish_done_cb(void* user_data, int32_t status) {
    struct k_sem* sem = (struct k_sem*)user_data;
    k_sem_give(sem);
}

ZTEST(transmit, priority_ordering) {
    zyphal_tx_t txs[9];
    for (size_t i = 0; i < ARRAY_SIZE(txs); i++) {
        zassert_ok(zyphal_tx_init(&inst, &txs[i]));
    }

    struct k_sem sem;
    zassert_ok(k_sem_init(&sem, 0, 9));

    /* Publish all possible priorities in reverse order. */
    uint8_t payloads[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t priorities[] = {ZYPHAL_PRIO_OPTIONAL,
                            ZYPHAL_PRIO_SLOW,
                            ZYPHAL_PRIO_LOW,
                            ZYPHAL_PRIO_NOMINAL,
                            ZYPHAL_PRIO_HIGH,
                            ZYPHAL_PRIO_FAST,
                            ZYPHAL_PRIO_IMMEDIATE,
                            ZYPHAL_PRIO_EXCEPTIONAL,
                            ZYPHAL_PRIO_NOMINAL};
    for (size_t i = 0; i < ARRAY_SIZE(txs); i++) {
        zassert_ok(zyphal_publish(&txs[i],
                                  priorities[i],
                                  SUBJECT_ID,
                                  &payloads[i],
                                  1,
                                  K_MSEC(10),
                                  publish_done_cb,
                                  &sem));
    }

    for (size_t i = 0; i < sem.limit; i++) { zassert_ok(k_sem_take(&sem, K_FOREVER)); }

    struct can_frame expected_frames[] = {
        {.id = 0x723455, .dlc = 2, .data = {7, 0xE0}},
        {.id = 0x4723455, .dlc = 2, .data = {6, 0xE0}},
        {.id = 0x8723455, .dlc = 2, .data = {5, 0xE0}},
        {.id = 0xC723455, .dlc = 2, .data = {4, 0xE0}},
        {.id = 0x10723455, .dlc = 2, .data = {3, 0xE0}},
        {.id = 0x10723455, .dlc = 2, .data = {8, 0xE0}},
        {.id = 0x14723455, .dlc = 2, .data = {2, 0xE0}},
        {.id = 0x18723455, .dlc = 2, .data = {1, 0xE0}},
        {.id = 0x1C723455, .dlc = 2, .data = {0, 0xE0}}};
    for (size_t i = 0; i < ARRAY_SIZE(expected_frames); i++) {
        can_fff_assert_popped_frame_equal(expected_frames[i]);
    }
}

ZTEST(transmit, transfer_id) {
    zyphal_tx_t tx;
    zassert_ok(zyphal_tx_init(&inst, &tx));

    uint8_t payload[] = {1};
    for (uint8_t i = 0; i < 32; i++) {
        zassert_ok(zyphal_publish_wait(
            &tx, ZYPHAL_PRIO_NOMINAL, SUBJECT_ID, payload, 1, K_MSEC(10)));
    }

    for (uint8_t i = 0; i < 32; i++) {
        uint8_t transfer_id = i;
        can_fff_assert_popped_frame_equal((struct can_frame){
            .id = 0x10723455, .dlc = 2, .data = {0x01, 0xE0 | transfer_id}});
    }
    can_fff_assert_frames_empty();

    /* Next transfer ID should wrap around to 0. */
    zassert_ok(zyphal_publish_wait(
        &tx, ZYPHAL_PRIO_NOMINAL, SUBJECT_ID, payload, 1, K_MSEC(10)));
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x10723455, .dlc = 2, .data = {0x01, 0xE0}});
    can_fff_assert_frames_empty();
}

ZTEST(transmit, invalid_params) {
    zyphal_tx_t tx;
    zassert_ok(zyphal_tx_init(&inst, &tx));

    uint8_t payload[] = {1};
    /* NULL parameters. */
    zassert_equal(
        zyphal_publish_wait(NULL, ZYPHAL_PRIO_LOW, SUBJECT_ID, payload, 1, K_MSEC(10)),
        -EINVAL);
    zassert_equal(
        zyphal_publish_wait(&tx, ZYPHAL_PRIO_LOW, SUBJECT_ID, NULL, 1, K_MSEC(10)),
        -EINVAL);
    /* Parameters out of range. */
    zassert_equal(zyphal_publish_wait(
                      &tx, ZYPHAL_PRIO_OPTIONAL + 1, SUBJECT_ID, payload, 1, K_MSEC(10)),
                  -EINVAL);
    zassert_equal(
        zyphal_publish_wait(
            &tx, ZYPHAL_PRIO_LOW, ZYPHAL_MAX_SUBJECT_ID + 1, payload, 1, K_MSEC(10)),
        -EINVAL);

    can_fff_assert_frames_empty();
}

ZTEST(transmit, busy_transfer) {
    zyphal_tx_t tx;
    zassert_ok(zyphal_tx_init(&inst, &tx));

    struct k_sem sem;
    zassert_ok(k_sem_init(&sem, 0, 1));

    uint8_t pl[] = {1};
    zassert_ok(zyphal_publish(
        &tx, ZYPHAL_PRIO_LOW, SUBJECT_ID, pl, 1, K_MSEC(10), publish_done_cb, &sem));
    /* Transfer should fail with -EALREADY when first is still pending. */
    zassert_equal(
        zyphal_publish(
            &tx, ZYPHAL_PRIO_LOW, SUBJECT_ID, pl, 1, K_MSEC(10), publish_done_cb, &sem),
        -EALREADY);
    zassert_true(zyphal_publish_pending(&tx));

    /* Once first transfer completes, second transfer should succeed. */
    k_sem_take(&sem, K_FOREVER);
    zassert_ok(zyphal_publish(
        &tx, ZYPHAL_PRIO_LOW, SUBJECT_ID, pl, 1, K_MSEC(10), publish_done_cb, &sem));
    k_sem_take(&sem, K_FOREVER);
    zassert_false(zyphal_publish_pending(&tx));

    /* Verify both frames were sent. */
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x14723455, .dlc = 2, .data = {1, 0xE0}});
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x14723455, .dlc = 2, .data = {1, 0xE1}});
    can_fff_assert_frames_empty();
}

ZTEST_SUITE(transmit, NULL, NULL, transmit_suite_before, NULL, NULL);
