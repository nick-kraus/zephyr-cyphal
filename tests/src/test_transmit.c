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

ZTEST(transmit, priority_ordering) {
    zyphal_tx_t tx1, tx2, tx3, tx4, tx5, tx6, tx7, tx8, tx9;
    zassert_ok(zyphal_tx_init(&inst, &tx1));
    zassert_ok(zyphal_tx_init(&inst, &tx2));
    zassert_ok(zyphal_tx_init(&inst, &tx3));
    zassert_ok(zyphal_tx_init(&inst, &tx4));
    zassert_ok(zyphal_tx_init(&inst, &tx5));
    zassert_ok(zyphal_tx_init(&inst, &tx6));
    zassert_ok(zyphal_tx_init(&inst, &tx7));
    zassert_ok(zyphal_tx_init(&inst, &tx8));
    zassert_ok(zyphal_tx_init(&inst, &tx9));

    struct k_sem sem1, sem2, sem3, sem4, sem5, sem6, sem7, sem8, sem9;
    zassert_ok(k_sem_init(&sem1, 0, 1));
    zassert_ok(k_sem_init(&sem2, 0, 1));
    zassert_ok(k_sem_init(&sem3, 0, 1));
    zassert_ok(k_sem_init(&sem4, 0, 1));
    zassert_ok(k_sem_init(&sem5, 0, 1));
    zassert_ok(k_sem_init(&sem6, 0, 1));
    zassert_ok(k_sem_init(&sem7, 0, 1));
    zassert_ok(k_sem_init(&sem8, 0, 1));
    zassert_ok(k_sem_init(&sem9, 0, 1));

    /* Publish all possible priorities, in reverse order. */
    uint8_t payloads[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    zassert_ok(zyphal_publish(
        &tx1, ZYPHAL_PRIO_OPTIONAL, SUBJECT_ID, &payloads[0], 1, K_MSEC(10), &sem1));
    zassert_ok(zyphal_publish(
        &tx2, ZYPHAL_PRIO_SLOW, SUBJECT_ID, &payloads[1], 1, K_MSEC(10), &sem2));
    zassert_ok(zyphal_publish(
        &tx3, ZYPHAL_PRIO_LOW, SUBJECT_ID, &payloads[2], 1, K_MSEC(10), &sem3));
    zassert_ok(zyphal_publish(
        &tx4, ZYPHAL_PRIO_NOMINAL, SUBJECT_ID, &payloads[3], 1, K_MSEC(10), &sem4));
    zassert_ok(zyphal_publish(
        &tx5, ZYPHAL_PRIO_HIGH, SUBJECT_ID, &payloads[4], 1, K_MSEC(10), &sem5));
    zassert_ok(zyphal_publish(
        &tx6, ZYPHAL_PRIO_FAST, SUBJECT_ID, &payloads[5], 1, K_MSEC(10), &sem6));
    zassert_ok(zyphal_publish(
        &tx7, ZYPHAL_PRIO_IMMEDIATE, SUBJECT_ID, &payloads[6], 1, K_MSEC(10), &sem7));
    zassert_ok(zyphal_publish(
        &tx8, ZYPHAL_PRIO_EXCEPTIONAL, SUBJECT_ID, &payloads[7], 1, K_MSEC(10), &sem8));
    /* This message should send after the other ZYPHAL_PRIO_NOMINAL message. */
    zassert_ok(zyphal_publish(
        &tx9, ZYPHAL_PRIO_NOMINAL, SUBJECT_ID, &payloads[8], 1, K_MSEC(10), &sem9));

    k_sem_take(&sem1, K_FOREVER);
    k_sem_take(&sem2, K_FOREVER);
    k_sem_take(&sem3, K_FOREVER);
    k_sem_take(&sem4, K_FOREVER);
    k_sem_take(&sem5, K_FOREVER);
    k_sem_take(&sem6, K_FOREVER);
    k_sem_take(&sem7, K_FOREVER);
    k_sem_take(&sem8, K_FOREVER);
    k_sem_take(&sem9, K_FOREVER);

    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x723455, .dlc = 2, .data = {7, 0xE0}});
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x4723455, .dlc = 2, .data = {6, 0xE0}});
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x8723455, .dlc = 2, .data = {5, 0xE0}});
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0xC723455, .dlc = 2, .data = {4, 0xE0}});
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x10723455, .dlc = 2, .data = {3, 0xE0}});
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x10723455, .dlc = 2, .data = {8, 0xE0}});
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x14723455, .dlc = 2, .data = {2, 0xE0}});
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x18723455, .dlc = 2, .data = {1, 0xE0}});
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x1C723455, .dlc = 2, .data = {0, 0xE0}});
    can_fff_assert_frames_empty();
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
    /* NULL publisher. */
    zassert_equal(
        zyphal_publish(NULL, ZYPHAL_PRIO_LOW, SUBJECT_ID, payload, 1, K_MSEC(10), NULL),
        -EINVAL);
    /* NULL payload. */
    zassert_equal(
        zyphal_publish(&tx, ZYPHAL_PRIO_LOW, SUBJECT_ID, NULL, 1, K_MSEC(10), NULL),
        -EINVAL);
    /* Invalid priority. */
    zassert_equal(zyphal_publish(&tx, 8, SUBJECT_ID, payload, 1, K_MSEC(10), NULL),
                  -EINVAL);

    can_fff_assert_frames_empty();
}

ZTEST(transmit, busy_transfer) {
    zyphal_tx_t tx;
    zassert_ok(zyphal_tx_init(&inst, &tx));

    struct k_sem sem1, sem2;
    zassert_ok(k_sem_init(&sem1, 0, 1));
    zassert_ok(k_sem_init(&sem2, 0, 1));

    uint8_t pl1[] = {1};
    uint8_t pl2[] = {2};

    zassert_ok(
        zyphal_publish(&tx, ZYPHAL_PRIO_LOW, SUBJECT_ID, pl1, 1, K_MSEC(10), &sem1));
    /* Transfer should fail with -EALREADY when first is still pending. */
    zassert_equal(
        zyphal_publish(&tx, ZYPHAL_PRIO_LOW, SUBJECT_ID, pl2, 1, K_MSEC(10), &sem2),
        -EALREADY);

    /* Once first transfer completes, second transfer should succeed. */
    k_sem_take(&sem1, K_FOREVER);
    zassert_ok(
        zyphal_publish(&tx, ZYPHAL_PRIO_LOW, SUBJECT_ID, pl2, 1, K_MSEC(10), &sem2));
    k_sem_take(&sem2, K_FOREVER);

    /* Verify both frames were sent. */
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x14723455, .dlc = 2, .data = {1, 0xE0}});
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x14723455, .dlc = 2, .data = {2, 0xE1}});
    can_fff_assert_frames_empty();
}

ZTEST(transmit, async_without_semaphore) {
    zyphal_tx_t tx;
    zassert_ok(zyphal_tx_init(&inst, &tx));

    uint8_t payload[] = {1};
    zassert_ok(
        zyphal_publish(&tx, ZYPHAL_PRIO_LOW, SUBJECT_ID, payload, 1, K_MSEC(10), NULL));

    /* Give some time for async processing. */
    k_msleep(10);

    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x14723455, .dlc = 2, .data = {1, 0xE0}});
    can_fff_assert_frames_empty();
}

ZTEST_SUITE(transmit, NULL, NULL, transmit_suite_before, NULL, NULL);
