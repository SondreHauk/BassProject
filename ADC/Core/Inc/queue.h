/*
 * queue.h
 * Circular Queue implementation in C
 *  Created on: Jun 16, 2025
 *      Author: sholte
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define BUF_SIZE 3
#define NUM_SCAN 2 // same as NUM_CONV in main...

typedef struct {
	uint16_t buffer[BUF_SIZE][NUM_SCAN];
	uint8_t  head;
	uint8_t  tail;
	uint8_t  count;
} Queue;

void queue_init(Queue *q);
bool queue_isFull(Queue *q);
bool queue_push(Queue *q, uint16_t val_in[NUM_SCAN]);
bool queue_pop(Queue *q, uint16_t *val_out);

#endif /* INC_QUEUE_H_ */
