#ifndef ZCYPHAL_CORE_H
#define ZCYPHAL_CORE_H

#include <stdint.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "zephyr/sys/slist.h"

typedef enum {
    ZCY_PRIO_EXCEPTIONAL = 0,
    ZCY_PRIO_IMMEDIATE = 1,
    ZCY_PRIO_FAST = 2,
    ZCY_PRIO_HIGH = 3,
    ZCY_PRIO_NOMINAL = 4,
    ZCY_PRIO_LOW = 5,
    ZCY_PRIO_SLOW = 6,
    ZCY_PRIO_OPTIONAL = 7,
} zcy_prio_t;

typedef struct {
    /* CAN bus device used for communication. */
    const struct device* canbus;
    /* 7-Bit ID for this cyphal node. */
    uint8_t node_id;
    /* Provides thread-safe access to instances. */
    struct k_mutex mutex;
    /* Transmission data queue and work item. */
    sys_slist_t tx_queue;
    struct k_work_delayable tx_work;
} zcy_inst_t;

typedef struct {
    /* Owning instance. */
    zcy_inst_t* inst;
    /* Transmit priority queue node. */
    sys_snode_t node;
    /* Extended CAN ID, used to determine priority. */
    uint32_t id;
    /* Time after which the transmission is discarded. */
    k_timepoint_t end;
    /* Payload data, length, and amount written. */
    uint8_t* payload;
    size_t payload_len;
    size_t payload_written;
    /* Cyphal transfer tail byte contents, and crc. */
    uint8_t toggle : 1;
    uint8_t transfer_id : 5;
    uint8_t crc_written : 2;
    uint16_t crc;
    /* Number of frames pending transmit. */
    atomic_t pending;
    /* Status of transfer, negative if an error. */
    int32_t status;
    /* Notifies the publishing thread on transfer completion. */
    struct k_sem* done;
} zcy_tx_t;

typedef struct {
    /* Transfer priority queue object. */
    zcy_tx_t xfer;
    /* Cyphal subject ID. */
    uint16_t subject_id;
} zcy_publisher_t;

/* Initializes a cyphal instance. */
int32_t zcy_init(zcy_inst_t* inst, const struct device* canbus, uint8_t node_id);

/* Initializes a publisher object. */
int32_t zcy_publisher_init(zcy_publisher_t* pub, zcy_inst_t* inst, uint16_t subject_id);
/* Publishes a message. */
int32_t zcy_publish(zcy_publisher_t* pub,
                    zcy_prio_t priority,
                    uint8_t* payload,
                    size_t len,
                    k_timeout_t timeout,
                    struct k_sem* done);
/* Publishes a message, and doesn't return until the message is sent. */
int32_t zcy_publish_sync(zcy_publisher_t* pub,
                         zcy_prio_t priority,
                         uint8_t* payload,
                         size_t len,
                         k_timeout_t timeout);

#endif /* ZCYPHAL_CORE_H */
