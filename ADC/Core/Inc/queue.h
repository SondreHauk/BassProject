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

#define BUF_SIZE 4
#define NUM_VALUES 2 //NB: is the same as num_conversions in main. Should be fixed...

typedef struct {
	uint16_t buffer[BUF_SIZE][NUM_VALUES];
	uint8_t head;
	uint8_t tail;
	uint8_t count;
} Queue;

void queue_init(Queue *q);
bool queue_push(Queue *q, uint16_t value[NUM_VALUES]);
bool queue_pop(Queue *q, uint16_t *value[NUM_VALUES]);

#endif /* INC_QUEUE_H_ */
