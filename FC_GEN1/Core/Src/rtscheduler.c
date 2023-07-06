/*
 * rtscheduler.c
 *
 *  Created on: May 21, 2023
 *      Author: Samuel Parent
 */


#include "rtscheduler.h"

static uint64_t RtScheduler_systick = 0;
static uint8_t RtScheduler_taskCounter[N_RATES] = {
	0, // 5ms tasks
	0, //500ms tasks
};
// Note: RtScheduler_nTasksPerRate must be updated alongside RtScheduler_tasks when tasks are added/removed
static const uint8_t RtScheduler_nTasksPerRate[N_RATES] = {
	7, // 5ms
	1, // 500ms
};

static bool RtScheduler_taskMarkedForExecution[N_RATES] = {
	false, // 5ms bucket
	false, // 500ms bucket
};
// Track which tasks have been executed during this tick period already
static bool RtScheduler_taskExecutedThisTick[N_RATES] = {
	false, // 5ms bucket
	false, // 500ms bucket
};

static uint64_t RtScheduler_GetTimeMs(void);
static uint64_t RtScheduler_GetTicks(void);
static void RtScheduler_Tick(void);
static uint64_t convertToMs(uint32_t);

int RtScheduler_startRunning(RtScheduler_tasks RtScheduler_tasks) {
	uint8_t tickRate = 0;
	static uint8_t taskCounter = 0;
	static uint64_t RtScheduler_prevFrcTime = 0;
	static uint64_t RtScheduler_currFrcTime = 0;
	static uint64_t RtScheduler_prevTicks = 0;

	while(1) {
		// Free running counter is represented in milliseconds since HAL was initialized
		RtScheduler_currFrcTime = HAL_GetTick();
		if ((RtScheduler_currFrcTime - RtScheduler_prevFrcTime) >= RTSCHEDULER_SYSTICK_RATE_MS) {
			// Tick the scheduler per the set scheduler systick rate
			RtScheduler_Tick();
			RtScheduler_prevFrcTime = RtScheduler_currFrcTime;
			RtScheduler_prevTicks = RtScheduler_GetTicks();
			memset(&RtScheduler_taskExecutedThisTick[0], false, sizeof(bool) * N_RATES);
		}

		for (uint8_t taskRate = eTASK1_5MS; taskRate < N_RATES; taskRate++) {
			if (RtScheduler_taskMarkedForExecution[taskRate]) {
				// Execute your registered task for the pool
				taskCounter = RtScheduler_taskCounter[taskRate];
				RtScheduler_tasks[taskRate][taskCounter]();

				// Calculate number of tasks per rate remaining, mark task rate buckets as complete
				taskCounter++;
				RtScheduler_taskCounter[taskRate] = taskCounter;
				if (taskCounter >= RtScheduler_nTasksPerRate[taskRate]) {
					RtScheduler_taskMarkedForExecution[taskRate] = false;
					RtScheduler_taskExecutedThisTick[taskRate] = true;
					taskCounter = 0;
					RtScheduler_taskCounter[taskRate] = taskCounter;
				}
			} else { // TODO: This is executing multiple times between the 5ms increments
				// Prevent all tasks from running at t = 0ms and from running multiples times between scheduler ticks
				if (RtScheduler_prevTicks > 0) {
					tickRate = RtScheduler_rateMs[taskRate];
					// Schedule pool for execution if rate has fallen on a harmonic of the task period
					if ((0 == (RtScheduler_GetTimeMs() % RtScheduler_rateMs[taskRate])) &&
							(RtScheduler_taskExecutedThisTick[taskRate] == false)) {
					  RtScheduler_taskMarkedForExecution[taskRate] = true;
					}
				}
			}
		}
	}

	return RTSCHEDULER_RUN_SUCCESS;
}

uint64_t convertToMs(uint32_t sysTicks) {
	if ((sysTicks * RTSCHEDULER_SYSTICK_RATE_MS) > UINT64_MAX) {
		return UINT64_MAX;
	}

	return sysTicks * RTSCHEDULER_SYSTICK_RATE_MS;
}

void RtScheduler_Tick(void) {
   ++RtScheduler_systick;
}

uint64_t RtScheduler_GetTicks(void) {
	return RtScheduler_systick;
}

uint64_t RtScheduler_GetTimeMs(void) {
	return convertToMs(RtScheduler_systick);
}



