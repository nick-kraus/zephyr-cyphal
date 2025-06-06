#ifndef CAN_FFF_H
#define CAN_FFF_H

#include <zephyr/drivers/can.h>

/* CAN FFF setup and reset, should be run before any unit tests. */
void can_fff_ztest_before(void);
/* Resets the history of saved CAN frames. */
void can_fff_history_reset(void);

/* ZTest assertion helpers. */
void can_fff_assert_frames_empty(void);
void can_fff_assert_popped_frame_equal(struct can_frame frame);

#endif /* CAN_FFF_H */
