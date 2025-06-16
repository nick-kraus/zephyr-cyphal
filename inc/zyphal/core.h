#ifndef ZYPHAL_CORE_H
#define ZYPHAL_CORE_H

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/slist.h>

#define ZYPHAL_MAX_NODE_ID (127)
#define ZYPHAL_MAX_SERVICE_ID (511)
#define ZYPHAL_MAX_SUBJECT_ID (8191)

typedef enum {
    ZYPHAL_PRIO_EXCEPTIONAL = 0,
    ZYPHAL_PRIO_IMMEDIATE = 1,
    ZYPHAL_PRIO_FAST = 2,
    ZYPHAL_PRIO_HIGH = 3,
    ZYPHAL_PRIO_NOMINAL = 4,
    ZYPHAL_PRIO_LOW = 5,
    ZYPHAL_PRIO_SLOW = 6,
    ZYPHAL_PRIO_OPTIONAL = 7,
} zyphal_prio_t;

typedef void (*zyphal_tx_done_cb_t)(void* user_data, int32_t status);

/* TODO: Define members in private header. */
typedef struct {
    /* CAN bus device used for communication. */
    const struct device* canbus;
    /* 7-Bit cyphal node ID. */
    uint8_t node_id;
    /* Provides thread-safe access to instances. */
    struct k_mutex mutex;
    /* Transmission data queue and work item. */
    sys_slist_t tx_queue;
    struct k_work_delayable tx_work;
} zyphal_inst_t;

/* TODO: Define members in private header. */
typedef struct {
    /* Owning instance. */
    zyphal_inst_t* inst;
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
    /* Called once full message has been transmitted. */
    zyphal_tx_done_cb_t done_cb;
    void* done_user_data;
} zyphal_tx_t;

/* Initializes a zyphal instance. */
int32_t zyphal_init(zyphal_inst_t* inst, const struct device* canbus, uint8_t node_id);

/* Initializes a transmitter object. */
int32_t zyphal_tx_init(zyphal_inst_t* inst, zyphal_tx_t* tx);

/* Publishes a message. */
int32_t zyphal_publish(zyphal_tx_t* tx,
                       zyphal_prio_t priority,
                       uint16_t subject_id,
                       uint8_t* payload,
                       size_t len,
                       k_timeout_t timeout,
                       zyphal_tx_done_cb_t cb,
                       void* user_data);
/* Publishes a message, returning once the message has been sent. */
int32_t zyphal_publish_wait(zyphal_tx_t* tx,
                            zyphal_prio_t priority,
                            uint16_t subject_id,
                            uint8_t* payload,
                            size_t len,
                            k_timeout_t timeout);

/* Returns true if a transmission is currently pending. */
bool zyphal_tx_pending(zyphal_tx_t* tx);
/* Cancels a currently pending transmission. */
int32_t zyphal_tx_cancel(zyphal_tx_t* tx);

#endif /* ZYPHAL_CORE_H */
