/*
 * rtscheduler.h
 *
 *  Created on: May 21, 2023
 *      Author: Samuel Parent
 */

#ifndef INC_RTSCHEDULER_H_
#define INC_RTSCHEDULER_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/time.h>
#include <string.h>
#include "stm32f7xx_hal.h"

#define RTSCHEDULER_MAX_TASKS_PER_RATE 	(10U)
#define RTSCHEDULER_MAX_TASK_RATES 		(3U)
#define RTSCHEDULER_FRC_TICK_RATE_US 	(100U)
// Scheduler tick rate
#define RTSCHEDULER_SYSTICK_RATE_MS 	(5U)

// Scheduler return codes
#define RTSCHEDULER_RUN_FAIL 	(0U)
#define RTSCHEDULER_RUN_SUCCESS (1U)

typedef enum {
	eTASK1_5MS = 0,
	eTASK2_500MS,
	N_RATES
} eRtScheduler_TaskRates;

static const uint32_t RtScheduler_rateMs[N_RATES] = {
	[eTASK1_5MS] 	= 5u,
	[eTASK2_500MS] 	= 500u,
};

typedef void (*RtScheduler_tasks[N_RATES][RTSCHEDULER_MAX_TASKS_PER_RATE])(void);

int RtScheduler_startRunning(RtScheduler_tasks RtScheduler_tasks);

#endif /* INC_RTSCHEDULER_H_ */
