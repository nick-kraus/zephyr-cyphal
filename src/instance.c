#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zyphal, CONFIG_CAN_LOG_LEVEL);

#include "transmit.h"
#include "zyphal/core.h"

int32_t zyphal_init(zyphal_inst_t* inst, const struct device* canbus, uint8_t node_id) {
    if (!inst || node_id > ZYPHAL_MAX_NODE_ID) {
        return -EINVAL;
    } else if (!device_is_ready(canbus)) {
        return -ENODEV;
    } else if (k_mutex_init(&inst->mutex) < 0) {
        return -EAGAIN;
    }

    inst->canbus = canbus;
    inst->node_id = node_id;
    sys_slist_init(&inst->tx_queue);
    k_work_init_delayable(&inst->tx_work, zyphal_tx_work_handler);

    return 0;
}
