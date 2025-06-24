/*
 * config.h
 *
 *  Created on: Jun 18, 2025
 *      Author: sholte
 *
 *
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define BUF_SIZE 3
#define NUM_CONVERSIONS 2
#define NCDT_SAMPLE_FREQ 8000
#define LDT_SAMPLE_FREQ 20 		  // Minimum 16
#define NCDT_LSB_TO_um (50000.0f / 65536.0f)  // 50000um on 16 bit resolution

#endif /* INC_CONFIG_H_ */
