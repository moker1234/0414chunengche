// services/protocol/j1939/opensae_adapter.c
#include "opensae_adapter.h"

#include <string.h>
#include <unistd.h>
#include <pthread.h>

#ifndef OPENSAE_MAX_CAN_CHANNELS
#define OPENSAE_MAX_CAN_CHANNELS 8
#endif

#ifndef OPENSAE_RXQ_CAP
#define OPENSAE_RXQ_CAP 256
#endif

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} opensae_frame_t;

typedef struct {
    pthread_mutex_t mtx;
    opensae_frame_t q[OPENSAE_RXQ_CAP];
    int head; // pop
    int tail; // push
    int count;
} opensae_rxq_t;

static opensae_rxq_t g_rxq[OPENSAE_MAX_CAN_CHANNELS];
static volatile int  g_active_can_index = 0;

/* OpenSAE callback storage (optional) */
static void (*g_cb_send)(uint32_t, uint8_t, uint8_t[]) = 0;
static void (*g_cb_read)(uint32_t*, uint8_t[], bool*) = 0;
static void (*g_cb_traffic)(uint32_t, uint8_t, uint8_t[], bool) = 0;
static void (*g_cb_delay)(uint8_t) = 0;

static void opensae_rxq_init_once_(void) {
    static int inited = 0;
    if (inited) return;
    inited = 1;
    for (int i = 0; i < OPENSAE_MAX_CAN_CHANNELS; ++i) {
        pthread_mutex_init(&g_rxq[i].mtx, 0);
        g_rxq[i].head = g_rxq[i].tail = g_rxq[i].count = 0;
    }
}

/* 由 C++ 层实现：真正把一帧发到 socketcan（通过 DriverManager / CanThread） */
extern void opensae_adapter_tx_dispatch(int can_index, uint32_t can_id, uint8_t dlc, const uint8_t* data);

void opensae_set_active_can_index(int can_index) {
    opensae_rxq_init_once_();
    if (can_index < 0) can_index = 0;
    if (can_index >= OPENSAE_MAX_CAN_CHANNELS) can_index = OPENSAE_MAX_CAN_CHANNELS - 1;
    g_active_can_index = can_index;
}

int opensae_get_active_can_index(void) {
    return g_active_can_index;
}

void opensae_clear_rx_queue(int can_index) {
    opensae_rxq_init_once_();
    if (can_index < 0 || can_index >= OPENSAE_MAX_CAN_CHANNELS) return;

    opensae_rxq_t* q = &g_rxq[can_index];
    pthread_mutex_lock(&q->mtx);
    q->head = q->tail = q->count = 0;
    pthread_mutex_unlock(&q->mtx);
}

void opensae_push_rx_frame(int can_index, uint32_t can_id, uint8_t dlc, const uint8_t* data) {
    opensae_rxq_init_once_();
    if (can_index < 0 || can_index >= OPENSAE_MAX_CAN_CHANNELS) return;

    opensae_rxq_t* q = &g_rxq[can_index];

    pthread_mutex_lock(&q->mtx);

    if (q->count >= OPENSAE_RXQ_CAP) {
        /* drop oldest */
        q->head = (q->head + 1) % OPENSAE_RXQ_CAP;
        q->count--;
    }

    opensae_frame_t* f = &q->q[q->tail];
    f->id = can_id;
    f->dlc = dlc;
    memset(f->data, 0, 8);
    if (data) {
        int n = (dlc > 8) ? 8 : (int)dlc;
        memcpy(f->data, data, (size_t)n);
    }

    q->tail = (q->tail + 1) % OPENSAE_RXQ_CAP;
    q->count++;

    pthread_mutex_unlock(&q->mtx);
}

/* ===== OpenSAE required symbols ===== */

ENUM_J1939_STATUS_CODES CAN_Send_Message(uint32_t ID, uint8_t data[]) {
    const int can_index = opensae_get_active_can_index();

    if (g_cb_send) {
        g_cb_send(ID, 8, data);
    } else {
        opensae_adapter_tx_dispatch(can_index, ID, 8, (const uint8_t*)data);
    }

    if (g_cb_traffic) {
        g_cb_traffic(ID, 8, data, true);
    }
    return STATUS_SEND_OK;
}

ENUM_J1939_STATUS_CODES CAN_Send_Request(uint32_t ID, uint8_t PGN[]) {
    const int can_index = opensae_get_active_can_index();

    if (g_cb_send) {
        g_cb_send(ID, 3, PGN);
    } else {
        opensae_adapter_tx_dispatch(can_index, ID, 3, (const uint8_t*)PGN);
    }

    if (g_cb_traffic) {
        g_cb_traffic(ID, 3, PGN, true);
    }
    return STATUS_SEND_OK;
}

bool CAN_Read_Message(uint32_t* ID, uint8_t data[]) {
    opensae_rxq_init_once_();

    const int can_index = opensae_get_active_can_index();
    if (can_index < 0 || can_index >= OPENSAE_MAX_CAN_CHANNELS) return false;

    opensae_rxq_t* q = &g_rxq[can_index];

    pthread_mutex_lock(&q->mtx);

    if (q->count <= 0) {
        pthread_mutex_unlock(&q->mtx);
        return false;
    }

    opensae_frame_t f = q->q[q->head];
    q->head = (q->head + 1) % OPENSAE_RXQ_CAP;
    q->count--;

    pthread_mutex_unlock(&q->mtx);

    if (ID) *ID = f.id;
    if (data) memcpy(data, f.data, 8);

    if (g_cb_read) {
        bool is_new = true;
        uint32_t tmp_id = f.id;
        uint8_t tmp_data[8];
        memcpy(tmp_data, f.data, 8);
        g_cb_read(&tmp_id, tmp_data, &is_new);
        (void)is_new;
    }

    if (g_cb_traffic) {
        g_cb_traffic(f.id, 8, f.data, false);
    }

    return true;
}

void CAN_Delay(uint8_t milliseconds) {
    if (g_cb_delay) {
        g_cb_delay(milliseconds);
    } else {
        usleep((useconds_t)milliseconds * 1000);
    }
}

void CAN_Set_Callback_Functions(
    void (*Callback_Function_Send_)(uint32_t, uint8_t, uint8_t[]),
    void (*Callback_Function_Read_)(uint32_t*, uint8_t[], bool*),
    void (*Callback_Function_Traffic_)(uint32_t, uint8_t, uint8_t[], bool),
    void (*Callback_Function_Delay_ms_)(uint8_t)
) {
    g_cb_send = Callback_Function_Send_;
    g_cb_read = Callback_Function_Read_;
    g_cb_traffic = Callback_Function_Traffic_;
    g_cb_delay = Callback_Function_Delay_ms_;
}
