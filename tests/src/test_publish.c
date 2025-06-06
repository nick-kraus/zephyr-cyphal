#include <stdlib.h>
#include <zephyr/drivers/can/can_fake.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "can_fff.h"
#include "zcyphal/core.h"

DEFINE_FFF_GLOBALS;

#define NODE_ID (0x55)
#define SUBJECT_ID (0x1234)

const struct device* canbus = DEVICE_DT_GET(DT_NODELABEL(fake_can));
zcy_inst_t inst;

static void publish_suite_before(void* f) {
    zassert_true(device_is_ready(canbus));
    zassert_ok(zcy_init(&inst, canbus, NODE_ID));

    can_fff_ztest_before();
}

ZTEST(publish, test_single_frame) {
    zcy_publisher_t pub;
    zassert_ok(zcy_publisher_init(&pub, &inst, SUBJECT_ID));

    /* Maximum sized single frame. */
    uint8_t pl1[] = {1, 2, 3, 4, 5, 6, 7};
    zassert_ok(zcy_publish_sync(&pub, ZCY_PRIO_NOMINAL, pl1, 7, K_MSEC(10)));
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 8, .data = {1, 2, 3, 4, 5, 6, 7, 0xE0}});
    can_fff_assert_frames_empty();

    /* Minimum sized single frame (no payload). */
    uint8_t pl2[] = {};
    zassert_ok(zcy_publish_sync(&pub, ZCY_PRIO_NOMINAL, pl2, 0, K_MSEC(10)));
    can_fff_assert_popped_frame_equal((struct can_frame){.id = 0x10723455,
                                                         .dlc = 1,
                                                         .data = {
                                                             0xE1,
                                                         }});
    can_fff_assert_frames_empty();
}

ZTEST(publish, test_frame_priority) {
    zcy_publisher_t pub1, pub2, pub3;
    zassert_ok(zcy_publisher_init(&pub1, &inst, 1));
    zassert_ok(zcy_publisher_init(&pub2, &inst, 2));
    zassert_ok(zcy_publisher_init(&pub3, &inst, 1));

    struct k_sem sem1, sem2, sem3;
    zassert_ok(k_sem_init(&sem1, 0, 1));
    zassert_ok(k_sem_init(&sem2, 0, 1));
    zassert_ok(k_sem_init(&sem3, 0, 1));

    uint8_t pl1[] = {1};
    zassert_ok(zcy_publish(&pub1, ZCY_PRIO_SLOW, pl1, 1, K_MSEC(10), &sem1));
    uint8_t pl2[] = {2};
    zassert_ok(zcy_publish(&pub2, ZCY_PRIO_FAST, pl2, 1, K_MSEC(10), &sem2));
    uint8_t pl3[] = {3};
    zassert_ok(zcy_publish(&pub3, ZCY_PRIO_SLOW, pl3, 1, K_MSEC(10), &sem3));

    k_sem_take(&sem1, K_FOREVER);
    k_sem_take(&sem2, K_FOREVER);
    k_sem_take(&sem3, K_FOREVER);

    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x8600255, .dlc = 2, .data = {2, 0xE0}});
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x18600155, .dlc = 2, .data = {1, 0xE0}});
    can_fff_assert_popped_frame_equal(
        (struct can_frame){.id = 0x18600155, .dlc = 2, .data = {3, 0xE0}});
    can_fff_assert_frames_empty();
}

ZTEST(publish, test_multi_frame) {
    zcy_publisher_t pub;
    zassert_ok(zcy_publisher_init(&pub, &inst, SUBJECT_ID));

    /* Three full frames. */
    uint8_t pl1[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
    zassert_ok(zcy_publish_sync(&pub, ZCY_PRIO_NOMINAL, pl1, 19, K_MSEC(10)));
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 8, .data = {1, 2, 3, 4, 5, 6, 7, 0xA0}});
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 8, .data = {8, 9, 10, 11, 12, 13, 14, 0x00}});
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 8, .data = {15, 16, 17, 18, 19, 0x1F, 0xAD, 0x60}});
    can_fff_assert_frames_empty();

    /* CRC by itself on last frame. */
    uint8_t pl2[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    zassert_ok(zcy_publish_sync(&pub, ZCY_PRIO_NOMINAL, pl2, 14, K_MSEC(10)));
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 8, .data = {1, 2, 3, 4, 5, 6, 7, 0xA1}});
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 8, .data = {8, 9, 10, 11, 12, 13, 14, 0x01}});
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 3, .data = {0x32, 0xF8, 0x61}});
    can_fff_assert_frames_empty();

    /* CRC split between frames. */
    uint8_t pl3[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
    zassert_ok(zcy_publish_sync(&pub, ZCY_PRIO_NOMINAL, pl3, 13, K_MSEC(10)));
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 8, .data = {1, 2, 3, 4, 5, 6, 7, 0xA2}});
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 8, .data = {8, 9, 10, 11, 12, 13, 0xF9, 0x02}});
    can_fff_assert_popped_frame_equal((struct can_frame){
        .id = 0x10723455, .dlc = 2, .data = {0xAD, 0x62}});
    can_fff_assert_frames_empty();
}

ZTEST_SUITE(publish, NULL, NULL, publish_suite_before, NULL, NULL);
