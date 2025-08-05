/*
 * config.h
 * Communication protocol: 0 for UART, 1 for SSI
 * Buffer size is equal to the sample delay
 * Maximum sample frequency is 8000 Hz
 *
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define COM_PROTOCOL 0
#define BUF_SIZE 3
#define SAMPLE_FREQ 8000

#endif /* INC_CONFIG_H_ */
