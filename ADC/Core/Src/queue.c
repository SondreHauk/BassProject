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

bool queue_isFull(Queue *q){
	if (q->count == BUF_SIZE)
		return true;
	return false;
}

bool queue_push(Queue *q, uint16_t val_in[NUM_SCAN]) {
    if (q->count == BUF_SIZE)
        return false;
    q->buffer[q->head][0] = val_in[0];
    q->buffer[q->head][1] = val_in[1];  // not modular! Use for-loop to modularise.
    q->head = (q->head + 1) % BUF_SIZE;
    q->count++;
    return true;
}

bool queue_pop(Queue *q, uint16_t *val_out) {
    if (q->count == 0)
        return false;
    val_out[0] = q->buffer[q->tail][0];
    val_out[1] = q->buffer[q->tail][1];
    q->tail = (q->tail + 1) % BUF_SIZE;
    q->count--;
    return true;
}
