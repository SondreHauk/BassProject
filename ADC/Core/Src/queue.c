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

bool queue_push(Queue *q, uint16_t val_in[NUM_CONVERSIONS]) {
    if (q->count == BUF_SIZE)
        return false;
    for(int i = 0; i < NUM_CONVERSIONS; i++){
    	q->buffer[q->head][i] = val_in[i];
    	}
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
