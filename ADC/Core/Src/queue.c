/*
 * queue.c
 *
 *  Created on: Jun 16, 2025
 *      Author: sholte
 */

// Circular Queue implementation in C

#include "queue.h"

void queue_init(Queue *q) {
    q->head = q->tail = q->count = 0;
}

bool queue_push(Queue *q, uint16_t value[NUM_VALUES]) {
    if (q->count == BUF_SIZE)
        return false;
    q->buffer[q->head] = value;
    q->head = (q->head + 1) % BUF_SIZE;
    q->count++;
    return true;
}

bool queue_pop(Queue *q, uint16_t *value[NUM_VALUES]) {
    if (q->count == 0)
        return false;
    *value = q->buffer[q->tail];
    q->tail = (q->tail + 1) % BUF_SIZE;
    q->count--;
    return true;
}
