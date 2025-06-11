#ifndef TRANSMIT_H
#define TRANSMIT_H

#include <zephyr/kernel.h>

void zyphal_tx_work_handler(struct k_work* work);

#endif /* TRANSMIT_H */
