/*
 * timers_pwm.c
 *
 *  Created on: Oct. 22, 2022
 *      Author: Samuel Parent
 *
 *
 *	Advanced Timers: (4 PWM Channels with Repetition Counter)
 *		tim1
 *		tim8
 *
 *	General Purpose Timers: (1-4 PWM Channels, no Repetition Counter)
 *		tim2
 *		tim3
 *		tim4
 *		tim5
 *		tim9
 *		tim10
 *		tim11
 *		tim12
 *		tim13
 *		tim14
 *
 *	Basic Timers: (0 PWM Channels, no Repetition Counter)
 *		tim6
 *		tim7
 *
*/

/*----------INCLUDES----------*/

#include "timers_pwm.h"

/*----------PRIVATE FUNCTION DEFINITIONS----------*/

// timer_select gets the timer specific information stored in metadata_st and selects the correct timer instance
static TIM_Ret_et timer_select(Timer_st* tim) {
	tim_metadata_st m;

	switch(tim->tim_num) {
		case(1):
			tim->htim->Instance = TIM1;
			m.tim_type = ADVANCED_TIMER;
			m.num_channels = 4;
			break;
		case(2):
			tim->htim->Instance = TIM2;
			m.tim_type = GENERAL_TIMER;
			m.num_channels = 4;
			break;
		case(3):
			tim->htim->Instance = TIM3;
			m.tim_type = GENERAL_TIMER;
			m.num_channels = 4;
			break;
		case(4):
			tim->htim->Instance = TIM4;
			m.tim_type = GENERAL_TIMER;
			m.num_channels = 4;
			break;
		case(5):
			tim->htim->Instance = TIM5;
			m.tim_type = GENERAL_TIMER;
			m.num_channels = 4;
			break;
		case(6):
			tim->htim->Instance = TIM6;
			m.tim_type = BASIC_TIMER;
			m.num_channels = 0;
			break;
		case(7):
			tim->htim->Instance = TIM7;
			m.tim_type = BASIC_TIMER;
			m.num_channels = 0;
			break;
		case(8):
			tim->htim->Instance = TIM8;
			m.tim_type = ADVANCED_TIMER;
			m.num_channels = 4;
			break;
		case(9):
			tim->htim->Instance = TIM9;
			m.tim_type = GENERAL_TIMER;
			m.num_channels = 2;
			break;
		case(10):
			tim->htim->Instance = TIM10;
			m.tim_type = GENERAL_TIMER;
			m.num_channels = 1;
			break;
		case(11):
			tim->htim->Instance = TIM11;
			m.tim_type = GENERAL_TIMER;
			m.num_channels = 1;
			break;
		case(12):
			tim->htim->Instance = TIM12;
			m.tim_type = GENERAL_TIMER;
			m.num_channels = 2;
			break;
		case(13):
			tim->htim->Instance = TIM13;
			m.tim_type = GENERAL_TIMER;
			m.num_channels = 1;
			break;
		case(14):
			tim->htim->Instance = TIM14;
			m.tim_type = GENERAL_TIMER;
			m.num_channels = 1;
			break;
		default:
			return TIM_NUM_INVALID;
	}

	tim->__metadata = m;

	return TIM_OK;
}

// Configure the timing parameters with period
static TIM_Ret_et config_period(TIM_HandleTypeDef* htim, uint32_t period_ms) {
	// Timer period cannot be 0
	if (period_ms == 0) {
		return TIM_PERIOD_ZERO;
	}

	//Not to be confused with a time period (count from 0 up to this value)
	htim->Init.Period = COUNTING_PERIOD_SLOW - 1;
	// f_timer = f_SysClock / (PSC + 1)
	htim->Init.Prescaler = (((uint64_t)HAL_RCC_GetSysClockFreq() * period_ms) / (double)((htim->Init.Period+1) * 1000)) - 1;

	return TIM_OK;
}

// Configure the timing parameters with frequency
static TIM_Ret_et config_freq(TIM_HandleTypeDef* htim, uint32_t freq_hz) {
	// Timer frequency cannot be 0
	if ((freq_hz == 0) || freq_hz > MAX_TIM_FREQ_FAST(HAL_RCC_GetSysClockFreq())) {
		return TIM_FREQ_ZERO;
	}
	else if (freq_hz < MAX_TIM_FREQ_SLOW) {
		// Not to be confused with a time period (count from 0 up to this value)
		htim->Init.Period = COUNTING_PERIOD_SLOW - 1;
	}
	else {
		// Same as above but larger number to count to
		htim->Init.Period = COUNTING_PERIOD_FAST - 1;
	}

	// f_Timer = f_SysClock / (PSC + 1)
	uint64_t f = HAL_RCC_GetSysClockFreq();
	uint64_t d = freq_hz * ((uint64_t)htim->Init.Period+1);
	htim->Init.Prescaler = (uint64_t)(f / d)-1;

	return TIM_OK;
}

// These are the default configurations that do not change based on user input
static void set_timer_defaults(Timer_st* tim){
	// CounterMode determines whether we count up or count down
	tim->htim->Init.CounterMode = TIM_COUNTERMODE_UP;
	// ClockDivision is a system clock divisor. We use prescaler so this is not needed
	tim->htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	// AutoReloadPreload determines what to reset the counter to on timer reload.
	// We do not need this as we set the counter period value on init to determine timing.
	tim->htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
}

// Returns a boolean to see if pwm channels will or will not be used
static uint8_t using_pwm(Channel_Config_st* channels) {
	return(	channels->en_ch1 |
			channels->en_ch2 |
			channels->en_ch3 |
			channels->en_ch4 |
			channels->en_ch_all );
}

// Allows easy enabling of all pwm channels on the timer module
static void enable_all_ch(Channel_Config_st* channels, tim_metadata_st* m) {
	if (m->num_channels >= 1) {
		channels->en_ch1 = 1;
	}
	if (m->num_channels >= 2) {
		channels->en_ch2 = 1;
	}
	if (m->num_channels >= 3) {
		channels->en_ch3 = 1;
	}
	if (m->num_channels == 4) {
		channels->en_ch4 = 1;
	}
}

// Check interrupt config for a timer without rep counter and timing model = period
static TIM_Ret_et noRC_it_config_period(TIM_HandleTypeDef* htim, uint32_t tim_period, uint32_t it_period) {
	// This allows us to have a default interrupt period: if value is not set then it will be timer period
	if (it_period == DEFAULT_IT_PERIOD) {
		it_period = tim_period;
	}
	// Without a repetition counter the only period that the interrupt can have is the timer period
	if (it_period != tim_period) { return TIM_NO_RC; }

	return TIM_OK;
}

// Check interrupt config for a timer without rep counter and timing model = frequency
static TIM_Ret_et noRC_it_config_freq(TIM_HandleTypeDef* htim, uint32_t tim_freq, uint32_t it_freq) {
	// This allows us to have a default interrupt frequency: if value is not set then it will be timer frequency
	if (it_freq == DEFAULT_IT_FREQ) {
		it_freq = tim_freq;
	}
	// Without a repetition counter the only frequency that the interrupt can have is the timer frequency
	if (it_freq != tim_freq) { return TIM_NO_RC; }
	// Interrupts should not be triggered too frequently
	if (it_freq > MAX_IT_FREQ) { return TIM_IT_INVALID_FREQ; }

	return TIM_OK;
}

// Allows a different interrupt frequency than the timer frequency (must be a multiple within (tim_freq/max_rc)
static TIM_Ret_et adv_it_config_freq(TIM_HandleTypeDef* htim, uint32_t tim_freq, uint32_t it_freq) {
	// This allows us to have a default interrupt frequency: if value is not set then it will be timer frequency
	if (it_freq == DEFAULT_IT_FREQ) {
		it_freq = tim_freq;
	}
	// Timer interrupts have max freq, cannot be greater than timer freq, and must be multiple of timer freq
	if 	((it_freq > MAX_IT_FREQ) ||
		(it_freq > tim_freq)  			||
		((tim_freq % it_freq) != 0)) {
		return TIM_IT_INVALID_FREQ;
	}
	// Make sure
	if (((double)tim_freq / (double)MAX_REP_COUNTER) > MAX_IT_FREQ) {
		return TIM_RC_OVERFLOW;
	}

	htim->Init.RepetitionCounter = (tim_freq / it_freq) - 1;


	return TIM_OK;
}

// TODO
static TIM_Ret_et adv_it_config_period(TIM_HandleTypeDef* htim, uint32_t tim_period, uint32_t it_period) {
	// This allows us to have a default interrupt period: if the value is not set then it will be timer period
	if (it_period == 0) {
		it_period = tim_period;
	}
	// Timer interrupt period must be less than timer period and must be a multiple of timer period
	if 	((it_period > tim_period) ||
		((it_period % tim_period) != 0)) {
		return TIM_IT_INVALID_PERIOD;
	}
	// timer_period / Interrupt_period <= max_rep_counter
	if (((double)tim_period / (double)it_period) > MAX_REP_COUNTER) {
		return TIM_RC_OVERFLOW;
	}

	htim->Init.RepetitionCounter = (it_period / tim_period) - 1;

	return TIM_OK;
}

// adv_tim_init initializes an advanced timer
static TIM_Ret_et adv_tim_init(Timer_st* tim) {
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
	TIM_Ret_et ret;

	// Initialize timing parameters for interrupts
	if (tim->it_config.en_it) {
		if (tim->timing  == PERIOD) {
			ret = adv_it_config_period(tim->htim, tim->period_ms, tim->it_config.period_ms);
		}
		else if (tim->timing == FREQ) {
			ret = adv_it_config_freq(tim->htim, tim->freq_hz, tim->it_config.freq_hz);
		}
		if (ret != TIM_OK) {
			return ret;
		}
	}

	if (HAL_TIM_Base_Init(tim->htim) != HAL_OK) { return TIM_BASE_INIT_FAIL; }

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(tim->htim, &sClockSourceConfig) != HAL_OK) { return TIM_CONFIG_CLK_FAIL; }

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(tim->htim, &sMasterConfig) != HAL_OK) { return TIM_MASTER_CONFIG_FAIL; }

	if (using_pwm(&tim->channels))
	{
		if (HAL_TIM_PWM_Init(tim->htim) != HAL_OK) { return TIM_PWM_INIT_FAIL; }

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 0;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

		sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
		sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
		sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
		sBreakDeadTimeConfig.DeadTime = 0;
		sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
		sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
		sBreakDeadTimeConfig.BreakFilter = 0;
		sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
		sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
		sBreakDeadTimeConfig.Break2Filter = 0;
		sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
		if (HAL_TIMEx_ConfigBreakDeadTime(tim->htim, &sBreakDeadTimeConfig) != HAL_OK) { return TIM_CONFIG_DEADTIME_FAIL; }

		if (tim->channels.en_ch_all) {
			enable_all_ch(&tim->channels, &tim->__metadata);
		}

		if (tim->channels.en_ch1 && tim->__metadata.num_channels >= 1)
		{
			if (HAL_TIM_PWM_ConfigChannel(tim->htim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { return TIM_PWM_CONFIG_CH_FAIL; }
		}
		else if (tim->channels.en_ch1)
		{
			return TIM_PWM_CH_UNSUPPORTED;
		}
		if (tim->channels.en_ch2 && tim->__metadata.num_channels >= 2)
		{
			if (HAL_TIM_PWM_ConfigChannel(tim->htim, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { return TIM_PWM_CONFIG_CH_FAIL; }
		}
		else if (tim->channels.en_ch2)
		{
			return TIM_PWM_CH_UNSUPPORTED;
		}
		if (tim->channels.en_ch3 && tim->__metadata.num_channels >= 3)
		{
			if (HAL_TIM_PWM_ConfigChannel(tim->htim, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { return TIM_PWM_CONFIG_CH_FAIL; }
		}
		else if (tim->channels.en_ch3)
		{
			return TIM_PWM_CH_UNSUPPORTED;
		}
		if (tim->channels.en_ch4 && tim->__metadata.num_channels == 4)
		{
			if (HAL_TIM_PWM_ConfigChannel(tim->htim, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { return TIM_PWM_CONFIG_CH_FAIL; }
		}
		else if (tim->channels.en_ch4)
		{
			return TIM_PWM_CH_UNSUPPORTED;
		}

		HAL_TIM_MspPostInit(tim->htim);
	}

	if (tim->it_config.en_it) {
		if (HAL_TIM_Base_Start_IT(tim->htim) != HAL_OK) { return TIM_BASE_START_IT_FAIL; };
	}

	return TIM_OK;
}

// gen_tim_init initialzes a general purpose timer
static TIM_Ret_et gen_tim_init(Timer_st* tim) {
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_Ret_et ret;

	if (tim->it_config.en_it){
		if (tim->timing == PERIOD) {
			ret = noRC_it_config_period(tim->htim, tim->period_ms, tim->it_config.period_ms);
		}
		else {
			ret = noRC_it_config_freq(tim->htim, tim->freq_hz, tim->it_config.freq_hz);
		}
		if (ret != TIM_OK) {
			return ret;
		}
	}

	if (HAL_TIM_Base_Init(tim->htim) != HAL_OK) { return TIM_BASE_INIT_FAIL; }

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(tim->htim, &sClockSourceConfig) != HAL_OK) { return TIM_CONFIG_CLK_FAIL; }

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(tim->htim, &sMasterConfig) != HAL_OK) { return TIM_MASTER_CONFIG_FAIL; }

	if (using_pwm(&tim->channels))
	{
		if (HAL_TIM_PWM_Init(tim->htim) != HAL_OK) { return TIM_PWM_INIT_FAIL; }

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 0;
	  	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	  	if (tim->channels.en_ch_all)
	  	{
	  		enable_all_ch(&tim->channels, &tim->__metadata);
	  	}

	  	if (tim->channels.en_ch1 && tim->__metadata.num_channels >= 1)
	  	{
	  		if (HAL_TIM_PWM_ConfigChannel(tim->htim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { return TIM_PWM_CONFIG_CH_FAIL; }
	  	}
	  	else if (tim->channels.en_ch1)
	  	{
	  		return TIM_PWM_CH_UNSUPPORTED;
	  	}
	  	if (tim->channels.en_ch2 && tim->__metadata.num_channels >= 2)
	  	{
	  		if (HAL_TIM_PWM_ConfigChannel(tim->htim, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { return TIM_PWM_CONFIG_CH_FAIL; }
	  	}
	  	else if (tim->channels.en_ch2)
	  	{
	  		return TIM_PWM_CH_UNSUPPORTED;
	  	}
	  	if (tim->channels.en_ch3 && tim->__metadata.num_channels >= 3)
	  	{
	  		if (HAL_TIM_PWM_ConfigChannel(tim->htim, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { return TIM_PWM_CONFIG_CH_FAIL; }
	  	}
	  	else if (tim->channels.en_ch3)
	  	{
	  		return TIM_PWM_CH_UNSUPPORTED;
	  	}
	  	if (tim->channels.en_ch4 && tim->__metadata.num_channels == 4)
	  	{
	  		if (HAL_TIM_PWM_ConfigChannel(tim->htim, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { return TIM_PWM_CONFIG_CH_FAIL; }
	  	}
	  	else if (tim->channels.en_ch4)
	  	{
	  		return TIM_PWM_CH_UNSUPPORTED;
	  	}

	  	HAL_TIM_MspPostInit(tim->htim);
	}

	if (tim->it_config.en_it)
	{
		if (HAL_TIM_Base_Start_IT(tim->htim) != HAL_OK) { return TIM_BASE_START_IT_FAIL; }
	}

	return TIM_OK;
}

//TODO: comment
static TIM_Ret_et basic_tim_init(Timer_st* tim) {
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_Ret_et ret;

	if (tim->it_config.en_it){
		if (tim->timing == PERIOD) {
			ret = noRC_it_config_period(tim->htim, tim->period_ms, tim->it_config.period_ms);
		}
		else {
			ret = noRC_it_config_freq(tim->htim, tim->freq_hz, tim->it_config.freq_hz);
		}
		if (ret != TIM_OK) {
			return ret;
		}
	}

	// Basic Timers do not have PWM channels
	if (using_pwm(&tim->channels)) {
		return TIM_PWM_UNSUPPORTED;
	}

	if (HAL_TIM_Base_Init(tim->htim) != HAL_OK) { return TIM_BASE_INIT_FAIL; }

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(tim->htim, &sMasterConfig) != HAL_OK) { return TIM_MASTER_CONFIG_FAIL; }

	if (tim->it_config.en_it)
	{
		if (HAL_TIM_Base_Start_IT(tim->htim) != HAL_OK) { return TIM_BASE_START_IT_FAIL; }
	}

	return TIM_OK;
}

// Compare is the value the timer will count to before toggling GPIO to create PWM
static uint32_t calculate_compare_value(uint32_t counting_period, uint8_t duty) {
	return ((counting_period+1) / MAX_DUTY_CYCLE) * (uint32_t)duty;
}

/*----------PUBLIC FUNCTION DEFINITIONS----------*/

// Timer_Init initializes a timer with configurations described in the Timer_st fields
TIM_Ret_et Timer_Init(Timer_st* tim)
{
	TIM_Ret_et ret;

	// Gets timer specific information
	ret = timer_select(tim);
	if (ret != TIM_OK) {
		return ret;
	}

	// Configure Timing
	if (tim->timing == PERIOD) {
		ret = config_period(tim->htim, tim->period_ms);
		if (ret != TIM_OK) {
			return ret;
		}
	}
	else if (tim->timing == FREQ) {
		ret = config_freq(tim->htim, tim->freq_hz);
		if (ret != TIM_OK) {
			return ret;
		}
	}

	// Defaults - these stay the same as the autogen code and generally do not need to be changed
	set_timer_defaults(tim);

	// The remainder of the initialization will depend on the timer type
	switch(tim->__metadata.tim_type) {
	case(ADVANCED_TIMER):
		ret = adv_tim_init(tim);
		break;
	case(GENERAL_TIMER):
		ret = gen_tim_init(tim);
		break;
	case(BASIC_TIMER):
		ret = basic_tim_init(tim);
		break;
	}

	// Timer is now initialized
	tim->__metadata.tim_initialized = 1;

	return ret;
}

// Timer_Stop de-initializes a timer and resets the proper flag
TIM_Ret_et Timer_Stop(Timer_st* tim) {
	if (HAL_TIM_Base_DeInit(tim->htim) != HAL_OK) { return TIM_BASE_DEINIT_FAIL; }

	// Reset timer_init flag
	tim->__metadata.tim_initialized = 0;

	return TIM_OK;
}

//TODO: comment
PWM_Ret_et PWM_Init(PWM_st* pwm)
{
	// Differs from the simple channel number, see enum types in the switch case statement
	uint32_t chan;
	// This is the value that the timer will count to before shutting making the PWM line low
	uint32_t compare;
	// This value could be different than pwm->duty if pwm->is_inverted is enabled
	uint8_t duty;

	// Make sure all pwm fields are valid
	if (pwm->duty > MAX_DUTY_CYCLE) { return PWM_INVALID_DUTY; }
	if (pwm->target_duty > MAX_DUTY_CYCLE) { return PWM_INVALID_TRGT_DUTY; }
	if (pwm->chan_num == 0 || pwm->chan_num > 4) { return PWM_INVALID_CH_NUM; }
	if (pwm->chan_num > pwm->tim->__metadata.num_channels) { return PWM_CH_UNSUPPORTED; }

	// Make sure the correct timer has been initialized
	if (!pwm->tim->__metadata.tim_initialized) { return PWM_TIM_UNINIT; }


	duty = pwm->duty;
	if (pwm->is_inverted) {
		duty = MAX_DUTY_CYCLE - duty;
	}

	compare = calculate_compare_value(pwm->tim->htim->Init.Period, duty);

	// Find desired channel
	switch(pwm->chan_num)
	{
		case(1):
			if (!pwm->tim->channels.en_ch1) { return PWM_CH_NOT_ENABLED; }
			chan = TIM_CHANNEL_1;
			break;
		case(2):
			if (!pwm->tim->channels.en_ch2) { return PWM_CH_NOT_ENABLED; }
			chan = TIM_CHANNEL_2;
			break;
		case(3):
			if (!pwm->tim->channels.en_ch3) { return PWM_CH_NOT_ENABLED; }
			chan = TIM_CHANNEL_3;
			break;
		case(4):
			if (!pwm->tim->channels.en_ch4) { return PWM_CH_NOT_ENABLED; }
			chan = TIM_CHANNEL_4;
			break;
	}

	// Initiate the PWM and desired duty cycle
	if (HAL_TIM_PWM_Start(pwm->tim->htim, chan) != HAL_OK) { return PWM_START_FAIL; }
	__HAL_TIM_SET_COMPARE(pwm->tim->htim, chan, compare);

	return PWM_OK;
}

//TODO: comment
PWM_Ret_et PWM_Stop(PWM_st* pwm) {
	// Differs from the simple channel number, see enum types in the switch case statement
	uint32_t chan;

	// Find desired channel
	switch(pwm->chan_num)
	{
		case(1):
			if (!pwm->tim->channels.en_ch1) { return PWM_CH_NOT_ENABLED; }
			chan = TIM_CHANNEL_1;
			break;
		case(2):
			if (!pwm->tim->channels.en_ch2) { return PWM_CH_NOT_ENABLED; }
			chan = TIM_CHANNEL_2;
			break;
		case(3):
				if (!pwm->tim->channels.en_ch3) { return PWM_CH_NOT_ENABLED; }
			chan = TIM_CHANNEL_3;
			break;
		case(4):
				if (!pwm->tim->channels.en_ch4) { return PWM_CH_NOT_ENABLED; }
			chan = TIM_CHANNEL_4;
			break;
	}

	if (HAL_TIM_PWM_Stop(pwm->tim->htim, chan) != HAL_OK) { return PWM_STOP_FAIL; };

	return PWM_OK;
}

// PWM_Move_Towards_Target moves the pwm duty cycle to the target duty cycle by duty_step_size %
PWM_Ret_et PWM_Move_Towards_Target(PWM_st* pwm)
{
	// Can be different from duty_step_size if |duty - target_duty| = duty_step_size
	uint8_t step;
	PWM_Ret_et ret;

	// If the duty cycle is at the target, exit
	if(pwm->duty == pwm->target_duty) {
		return PWM_OK;
	}

	else if(pwm->duty > pwm->target_duty) {
		// Make sure duty does not go below min
		step = 	((pwm->duty - pwm->target_duty) > pwm->duty_step_size) ?
				(pwm->duty - pwm->target_duty) :
				(pwm->duty_step_size);

		// Update the duty cycle
		pwm->duty -= step;
	}
	else if(pwm->duty < pwm->target_duty) {
		//make sure duty does not go above max
		step = 	((pwm->target_duty - pwm->duty) > pwm->duty_step_size) ?
				 (pwm->target_duty - pwm->duty) :
				 (pwm->duty_step_size);

		// Update the duty cycle
		pwm->duty -= step;
	}

	// These errors should never happen
	if (pwm->target_duty > MAX_DUTY_CYCLE) { return PWM_INVALID_TRGT_DUTY; }
	if (pwm->duty > MAX_DUTY_CYCLE) { return PWM_INVALID_DUTY; }

	ret = PWM_Stop(pwm);
	if (ret != PWM_OK) {
		return ret;
	}

	ret = PWM_Init(pwm);
	if (ret != PWM_OK) {
		return ret;
	}

	return PWM_OK;
}

// PWM_Update_Target updates the pwm duty cycle target to new_target if it is valid
PWM_Ret_et PWM_Update_Target(PWM_st* pwm, uint8_t new_target)
{
	if (new_target > MAX_DUTY_CYCLE) {
		return PWM_INVALID_TRGT_DUTY;
	}

	pwm->target_duty = new_target;

	return PWM_OK;
}
