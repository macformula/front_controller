/**
 ********************************************************************************
 * @file    projectMain.c
 * @author  Samuel Parent
 * @date    2023-07-16
 * @brief   
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "main.h"
#include "projectMain.h"
#include "controller_autogen.h"
#include "rtwtypes.h"
#include "canal.h"
#include "timers_pwm.h"
#include "adc_lib.h"
#include "usart.h"
#include "can.h"
#include "tim.h"
#include "adc.h"

/************************************
 * STATIC VARIABLES
 ************************************/
static uint16_t apps1_buffer[APPS1_BUFFER_LEN] = {0};
static uint16_t apps2_buffer[APPS2_BUFFER_LEN] = {0};
static uint16_t steering_angle_sensor_buffer[STEERING_ANGLE_SENSOR_BUFFER_LEN]= {0};
static uint16_t bpps_buffered_buffer[BPPS_BUFFERED_BUFFER_LEN] = {0};
static uint16_t adc_values[NUM_ADC_CHANNELS];

static RtScheduler_tasks tasks = {
	[eTASK1_5MS] = {
		readFromAdc,
		setControlSystemInputs,
		controller_autogen_step,
		getControlSystemOutputs,
		transmitToAMKMotors,
		transmitToBMS,
		setDigitalOutputs,
	},
	[eTASK2_500MS] = {
		setPWMOutputs,
	},
};

/************************************
 * EXTERN VARIABLES
 ************************************/
uint8_t driverSpeakerEn; 
uint8_t brakeLightEn;
uint8_t statusLightDutyCycle; 
uint16_t statusLightFreq;

TsCanAL pt1_can = {
	.hcan = &hcan1,
	.canNum = PT1_CAN_INST,
	.baud = PT1_CAN_BAUD,
	.mode = PT1_CAN_MODE,
};

TsCanAL pt2_can = {
	.hcan = &hcan2,
	.canNum = PT2_CAN_INST,
	.baud = PT2_CAN_BAUD,
	.mode = PT2_CAN_MODE,
};

TsCanAL veh_can = {
	.hcan = &hcan3,
	.canNum = VEH_CAN_INST,
	.baud = VEH_CAN_BAUD,
	.mode = VEH_CAN_MODE,
};

ADC_Channel_st adc1_channels[ADC1_NUM_CHANNELS] = {
	{
		.channel_number = APPS1_ADC_CHANNEL,
		.sample_time = ADC_15CYCLES,
		.buffer_len = APPS1_BUFFER_LEN,
		.buffer = apps1_buffer,
		.convert = NULL,
	},
	{
		.channel_number = APPS2_ADC_CHANNEL,
		.sample_time = ADC_15CYCLES,
		.buffer_len = APPS2_BUFFER_LEN,
		.buffer = apps2_buffer,
		.convert = NULL,
	},
	{
		.channel_number = STEERING_ANGLE_SENSOR_ADC_CHANNEL,
		.sample_time = ADC_15CYCLES,
		.buffer_len = STEERING_ANGLE_SENSOR_BUFFER_LEN,
		.buffer = steering_angle_sensor_buffer,
		.convert = NULL,
	},
	{
		.channel_number = BPPS_BUFFERED_ADC_CHANNEL,
		.sample_time = ADC_15CYCLES,
		.buffer_len = BPPS_BUFFERED_BUFFER_LEN,
		.buffer = bpps_buffered_buffer,
		.convert = NULL,
	},
};

ADC_st adc1 = {
	.hadc = &hadc1,
	.adc_num = 1,
	.num_channels = ADC1_NUM_CHANNELS,
	.channels = adc1_channels,
};

Timer_st tim2 = {
    .htim = &htim2,
    .tim_num = 2,
    .channels = {0,0,0,1},
    .timing = FREQ,
    .freq_hz = 1,
    .it_config = {0},
};

PWM_st status_led_pwm = {
    .tim = &tim2,
    .chan_num = 4,
    .duty = 0,
    .is_inverted = 0,
};

UART_st debug_uart = {
	.uart_num = 3,
	.huart = &huart3,
	.bit_position = LSB_First,
	.baudrate = UART_115200,
	.mode = UART_TX_RX,
	.datasize = UART_Datasize_8,
};

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * PRIVATE TYPEDEFS
 ************************************/

/************************************
 * GLOBAL VARIABLES
 ************************************/

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/
static void getRollingAvg(uint16_t* data_points, uint16_t* output_averages);

/************************************
 * STATIC FUNCTIONS
 ************************************/
void getRollingAvg(uint16_t* data_points, uint16_t* output_averages)
{
    static uint32_t sums [NUM_ADC_CHANNELS] = {};
    static uint16_t num_vals = 0;
    static uint16_t value_windows[NUM_ADC_CHANNELS][ADC_ROLLING_AVG_WIN_SIZE] = {};

    if (num_vals == ADC_ROLLING_AVG_WIN_SIZE)
    {
        for (int i = 0; i<NUM_ADC_CHANNELS; i++)
        {
            sums[i] = sums[i] - value_windows[i][0] + data_points[i];
            output_averages[i] = sums[i]/num_vals;

            //Shift the values
            for (int j = 0; j < ADC_ROLLING_AVG_WIN_SIZE - 1; j++)
            {
                value_windows[i][j] = value_windows[i][j+1];
            }
            value_windows[i][ADC_ROLLING_AVG_WIN_SIZE - 1] = data_points[i];
        }
    }
    else
    {
        num_vals++;
        for (int i = 0; i < NUM_ADC_CHANNELS; i++)
        {
            sums[i] += data_points[i];
            value_windows[i][num_vals-1] = data_points[i];
            output_averages[i] = sums[i]/(num_vals);
        }
    }
}

void readFromAdc() {
  ADC_Scan(&adc1);
  uint16_t raw_adc_values[NUM_ADC_CHANNELS] = {
		  [APPS1_IDX] = apps1_buffer[0],
		  [APPS2_IDX] = apps2_buffer[0],
		  [STEERING_ANGLE_SENSOR_IDX] = steering_angle_sensor_buffer[0],
		  [BPPS_BUFFERED_IDX] = bpps_buffered_buffer[0],
  };

  getRollingAvg(raw_adc_values, adc_values);
}

void setControlSystemInputs() {
  controller_autogen_U.AMK_ActualVelocity_R = AMK1_ActualValues1.AMK_ActualVelocity;
  controller_autogen_U.AMK_MagnetizingCurrent_R = AMK1_ActualValues1.AMK_MagnetizingCurrent;
  controller_autogen_U.AMK_TorqueCurrent_R = AMK1_ActualValues1.AMK_TorqueCurrent;
  controller_autogen_U.AMK_bDcOn_R = AMK1_ActualValues1.AMK_bDcOn;
  controller_autogen_U.AMK_bDerating_R = AMK1_ActualValues1.AMK_bDerating;
  controller_autogen_U.AMK_bError_R = AMK1_ActualValues1.AMK_bError;
  controller_autogen_U.AMK_bInverterOn_R = AMK1_ActualValues1.AMK_bInverterOn;
  controller_autogen_U.AMK_bQuitDcOn_R = AMK1_ActualValues1.AMK_bQuitDcOn;
  controller_autogen_U.AMK_bQuitInverterOn_R = AMK1_ActualValues1.AMK_bQuitInverterOn;
  controller_autogen_U.AMK_bSystemReady_R = AMK1_ActualValues1.AMK_bSystemReady;
  controller_autogen_U.AMK_bWarn_R = AMK1_ActualValues1.AMK_bWarn;
  controller_autogen_U.AMK_ErrorInfo_R = AMK1_ActualValues2.AMK_ErrorInfo;
  controller_autogen_U.AMK_TempIGBT_R = AMK1_ActualValues2.AMK_TempIGBT;
  controller_autogen_U.AMK_TempInverter_R = AMK1_ActualValues2.AMK_TempInverter;
  controller_autogen_U.AMK_TempMotor_R = AMK1_ActualValues2.AMK_TempMotor;
  controller_autogen_U.AMK_ActualVelocity_L = AMK0_ActualValues1.AMK_ActualVelocity;
  controller_autogen_U.AMK_MagnetizingCurrent_L = AMK0_ActualValues1.AMK_MagnetizingCurrent;
  controller_autogen_U.AMK_TorqueCurrent_L = AMK0_ActualValues1.AMK_TorqueCurrent;
  controller_autogen_U.AMK_bDcOn_L = AMK0_ActualValues1.AMK_bDcOn;
  controller_autogen_U.AMK_bDerating_L = AMK0_ActualValues1.AMK_bDerating;
  controller_autogen_U.AMK_bError_L = AMK0_ActualValues1.AMK_bError;
  controller_autogen_U.AMK_bInverterOn_L = AMK0_ActualValues1.AMK_bInverterOn;
  controller_autogen_U.AMK_bQuitDcOn_L = AMK0_ActualValues1.AMK_bQuitDcOn;
  controller_autogen_U.AMK_bQuitInverterOn_L = AMK0_ActualValues1.AMK_bQuitInverterOn;
  controller_autogen_U.AMK_bSystemReady_L = AMK0_ActualValues1.AMK_bSystemReady;
  controller_autogen_U.AMK_bWarn_L = AMK0_ActualValues1.AMK_bWarn;
  controller_autogen_U.AMK_ErrorInfo_L = AMK0_ActualValues2.AMK_ErrorInfo;
  controller_autogen_U.AMK_TempIGBT_L = AMK0_ActualValues2.AMK_TempIGBT;
  controller_autogen_U.AMK_TempInverter_L = AMK0_ActualValues2.AMK_TempInverter;
  controller_autogen_U.AMK_TempMotor_L = AMK0_ActualValues2.AMK_TempMotor;
  controller_autogen_U.DI_V_SteeringAngle = adc_values[STEERING_ANGLE_SENSOR_IDX];
  controller_autogen_U.DI_V_BrakePedalPos = adc_values[BPPS_BUFFERED_IDX];
  controller_autogen_U.DI_b_DriverButton = HAL_GPIO_ReadPin(START_BUTTON_N_GPIO_Port, START_BUTTON_N_Pin);
  controller_autogen_U.DI_V_AccelPedalPos1 = adc_values[APPS1_IDX];
  controller_autogen_U.DI_V_AccelPedalPos2 = adc_values[APPS2_IDX];
  controller_autogen_U.BM_b_prechrgContactorSts = Contactor_Feedback.Pack_Precharge_Feedback;
  controller_autogen_U.BM_b_HVposContactorSts = Contactor_Feedback.Pack_Positive_Feedback;
  controller_autogen_U.BM_b_HVnegContactorSts = Contactor_Feedback.Pack_Negative_Feedback;
}

void getControlSystemOutputs() {
  AMK1_SetPoints1.AMK_bInverterOn = controller_autogen_Y.AMK_bInverterOn_tx_R;
  AMK1_SetPoints1.AMK_bDcOn = controller_autogen_Y.AMK_bDcOn_tx_R;
  AMK1_SetPoints1.AMK_bEnable = controller_autogen_Y.AMK_bEnable_R;
  AMK1_SetPoints1.AMK_bErrorReset = controller_autogen_Y.AMK_bErrorReset_R;
  AMK1_SetPoints1.AMK_TargetVelocity = controller_autogen_Y.AMK_TargetVelocity_R;
  AMK1_SetPoints1.AMK_TorqueLimitPositiv = controller_autogen_Y.AMK_TorqueLimitPositiv_R;
  AMK1_SetPoints1.AMK_TorqueLimitNegativ = controller_autogen_Y.AMK_TorqueLimitNegativ_R;

  AMK0_SetPoints1.AMK_bInverterOn = controller_autogen_Y.AMK_bInverterOn_tx_L;
  AMK0_SetPoints1.AMK_bDcOn = controller_autogen_Y.AMK_bDcOn_tx_L;
  AMK0_SetPoints1.AMK_bEnable = controller_autogen_Y.AMK_bEnable_L;
  AMK0_SetPoints1.AMK_bErrorReset = controller_autogen_Y.AMK_bErrorReset_L;
  AMK0_SetPoints1.AMK_TargetVelocity = controller_autogen_Y.AMK_TargetVelocity_L;
  AMK0_SetPoints1.AMK_TorqueLimitPositiv = controller_autogen_Y.AMK_TorqueLimitPositiv_L;
  AMK0_SetPoints1.AMK_TorqueLimitNegativ = controller_autogen_Y.AMK_TorqueLimitNegativ_L;

  Contactor_States.Pack_Precharge = controller_autogen_Y.BM_b_prechargeContactorCMD;
  Contactor_States.Pack_Positive = controller_autogen_Y.BM_b_HVposContactorCMD;
  Contactor_States.Pack_Negative = controller_autogen_Y.BM_b_HVnegContactorCMD;

  driverSpeakerEn = controller_autogen_Y.DI_b_driverSpeaker;
  brakeLightEn = controller_autogen_Y.DI_b_brakeLightEn;
  status_led_pwm.duty = controller_autogen_Y.DI_p_PWMstatusLightCycle;
  tim2.freq_hz = controller_autogen_Y.DI_p_PWMstatusLightFreq;
}

void setDigitalOutputs() {
  HAL_GPIO_WritePin(RTDS_EN_GPIO_Port, RTDS_EN_Pin, driverSpeakerEn);
  HAL_GPIO_WritePin(BRAKE_LIGHT_EN_GPIO_Port, BRAKE_LIGHT_EN_Pin, brakeLightEn);
}

void setPWMOutputs(){
  static uint8_t prevDutyCycle = 0;
  static uint32_t prevFreq = 0;

  if (tim2.freq_hz != prevFreq || status_led_pwm.duty != prevDutyCycle) {
	Timer_Init(&tim2);
	PWM_Init(&status_led_pwm);

	prevDutyCycle = status_led_pwm.duty;
	prevFreq = tim2.freq_hz;
  }
}

void transmitToAMKMotors() {
	CanAL_Transmit(&pt1_can, AMK0_SETPOINTS1_CANAL_ID);
	CanAL_Transmit(&pt1_can, AMK1_SETPOINTS1_CANAL_ID);
}

void transmitToBMS() {
	CanAL_Transmit(&veh_can, CONTACTOR_STATES_CANAL_ID);
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
void projectMain()
{
    Timer_Init(&tim2);
    PWM_Init(&status_led_pwm);
    ADC_Init(&adc1);
    CanAL_Init(&veh_can);
    CanAL_Init(&pt1_can);
    #if DEBUG_MODE 
    Printf_Init();
    #endif // DEBUG_MODE
    controller_autogen_initialize();

    RtScheduler_startRunning(tasks);

    while (1)
    {
        
    }
}
