/*
 * App.h
 *
 *  Created on: 16.03.2019
 *      Author: Bora Ozgen
 */

#ifndef APP_H_
#define APP_H_

#include "stdint.h"
#include "stdbool.h"
#include "cmsis_os.h"
#include "main.h"

#define CHANNEL_COUNT			15
#define CURRENT_FRAME_A_SIZE	8
#define CURRENT_FRAME_B_SIZE	7
#define STATUS_FRAME_SIZE		3

#define CURRENT_FRAME_INTERVAL_MS	100
#define FUSE_TASK_INTERVAL_MS		1

#define CH1_FUSE_CURRENT		15.0F	// Water Fan L
#define CH2_FUSE_CURRENT		15.0F	// Water Fan R
#define CH3_FUSE_CURRENT		15.0F	// Schwimmwinkel
#define CH4_FUSE_CURRENT		20.0F	// Servos + Test Lambdas
#define CH5_FUSE_CURRENT		20.0F	// Ignition + Injection + Fuel Pump
#define CH6_FUSE_CURRENT		5.0F	// LTC
#define CH7_FUSE_CURRENT		5.0F	// ECU
#define CH8_FUSE_CURRENT		5.0F	// Sensors 12V
#define CH9_FUSE_CURRENT		15.0F	// Oil Fan
#define CH10_FUSE_CURRENT		5.0F	// Display (temporary)
#define CH11_FUSE_CURRENT		5.0F	// Not used
#define CH12_FUSE_CURRENT		5.0F	// Water Pump L
#define CH13_FUSE_CURRENT		5.0F	// Water Pump R
#define CH14_FUSE_CURRENT		5.0F	// Test Kurbelwellendruck
#define CH15_FUSE_CURRENT		5.0F	// Beacon

/* Enumeration to map channels to n-1 for optimal array usage */
enum {
	CH1, CH2, CH3, CH4, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15
};

typedef struct {
	GPIO_TypeDef*		input_port;
	uint32_t			input_pin;
	osTimerId			timer_id;
	osTimerDef_t		timer_def;
	uint32_t			retry_time_ms;
	bool				auto_retry;
	float				fuse_current;
	volatile uint16_t*	adc_value_ptr;
	float				k_factor;
	float				r_sense;
} App_channel_t;

int App_init(void);
int App_triggerFuse(uint8_t ch);
int App_turnOffChannel(uint8_t ch);
int App_turnOnChannel(uint8_t ch);

#endif /* APP_H_ */
