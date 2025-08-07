/*
 * config.h
 * Communication protocol: UART or SSI
 * Buffer size is equal to the sample delay
 * Maximum sample frequency is 8000 Hz
 *
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define COM_PROTOCOL UART
#define BUF_SIZE 3
#define ADC_SAMPLE_FREQ 8000
#define SSI_FREQ 4000

/*
 * Do not change anything below this line
 */

//#define UART 0
//#define SSI 1

#endif /* INC_CONFIG_H_ */
