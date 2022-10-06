#include <stdio.h>
#include "acm_GO422.h"
#include "GopherCAN.h"

// set up enables for each power supply (automatically set to on, turn off if power strays from usable range)

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* acm_hcan;

// Use this to define what module this board and other boards will be
#define THIS_MODULE_ID ACM_ID
#define OTHER_MODULE DLM_ID

// global variables
// need to make a parameter for every possible data channel

FLOAT_CAN_STRUCT* data_channels[VALID_CHANNELS] =
{
		// these can change depending on what people want to test overall
	    &brake_pressure_rear,
	    &steering_anglels,
	    &yaw_rate,
	    &throttle_position,
	    &acceleration,
};

U8 last_button_state;
U8 drs_state = 0;
U8 active_aero_state = 0;
U8 valid_channel = 0; // 0 for yes, -1 for no

data_thresholds channel_1, channel_2, channel_3;

// initialize GopherCAN
// What needs to happen on startup in order to run GopherCAN
data_thresholds used_data_channels[NUM_CHANNELS];

data_thresholds* init(CAN_HandleTypeDef* hcan_ptr)
{
	acm_hcan = hcan_ptr;

	// initialize CAN
	// NOTE: CAN will also need to be added in CubeMX and code must be generated
	// Check the STM_CAN repo for the file "F0xx CAN Config Settings.pptx" for the correct settings
	if (init_can(acm_hcan, THIS_MODULE_ID, BXTYPE_MASTER))
	{
		// an error has occurred, stay here
		while (1);
	}

	// enable updating for data channels
	DATA_CHANNEL_1.update_enabled = TRUE;
	DATA_CHANNEL_2.update_enabled = TRUE;
	DATA_CHANNEL_3.update_enabled = TRUE;

	// enable all of the variables in GopherCAN for testing
	set_all_params_state(TRUE);

	// create threshold structs
	channel_1.channel_name = &DATA_CHANNEL_1;
	channel_1.high = DATA_CHANNEL_1_HIGH;
	channel_1.mid = DATA_CHANNEL_1_MID;
	channel_1.low = DATA_CHANNEL_1_LOW;
	channel_1.max = DATA_CHANNEL_1_MAX;
	channel_1.min = DATA_CHANNEL_1_MIN;

	channel_2.channel_name = &DATA_CHANNEL_2;
	channel_2.high = DATA_CHANNEL_2_HIGH;
	channel_2.mid = DATA_CHANNEL_2_MID;
	channel_2.low = DATA_CHANNEL_2_LOW;
	channel_2.max = DATA_CHANNEL_2_MAX;
	channel_2.min = DATA_CHANNEL_2_MIN;

	channel_3.channel_name = &DATA_CHANNEL_3;
	channel_3.high = DATA_CHANNEL_3_HIGH;
	channel_3.mid = DATA_CHANNEL_3_MID;
	channel_3.low = DATA_CHANNEL_3_LOW;
	channel_3.max = DATA_CHANNEL_3_MAX;
	channel_3.min = DATA_CHANNEL_3_MIN;

	used_data_channels[0] = channel_1;
	used_data_channels[1] = channel_2;
	used_data_channels[2]= channel_3;
	return used_data_channels;
}

// can_buffer_handling_loop
//  This loop will handle CAN RX software task and CAN TX hardware task. Should be
//  called every 1ms or as often as received messages should be handled
void can_buffer_handling_loop()
{
	// handle each RX message in the buffer
	if (service_can_rx_buffer())
	{
		// an error has occurred
	}

	// handle the transmission hardware for each CAN bus
	service_can_tx_hardware(acm_hcan);
}

int valid_data_channel(FLOAT_CAN_STRUCT* data_channel)
{
	for(int i = 0; i < NUM_CHANNELS; i++) //need to figure out why these macros aren't transfering properly
	{
		if(data_channel->param_id == data_channels[i]->param_id)
		{
			return 0;
		}
	}
	return -1;
}

// function to be called in the main() function to determine which state the rear wing should be
// at based on the data requested
int calculate_rear_wing_angle(FLOAT_CAN_STRUCT* data_channel, data_thresholds thresh)
{
	if(data_channel->data > thresh.max || data_channel->data < thresh.min)
	{
		return ERR_DRS;
	}
	else if(data_channel->data > thresh.high )
	{
		return HIGH_REAR_ANGLE;
	}
	else if(data_channel->data > thresh.mid)
	{
		return MID_HIGH_REAR_ANGLE;
	}
	else if(data_channel->data > thresh.low)
	{
		return MID_LOW_REAR_ANGLE;
	}
	else
	{
		return LOW_REAR_ANGLE;
	}
}

// function to be called in the main() function to determine which state the front wing should be
// at based on the data requested
int calculate_front_wing_angle(FLOAT_CAN_STRUCT* data_channel, data_thresholds thresh)
{
	if(data_channel->data > thresh.max || data_channel->data < thresh.min)
	{
		return ERR_FRONT;
	}
	else if(data_channel->data > thresh.high)
	{
		return HIGH_FRONT_ANGLE;
	}
	else if(data_channel->data > thresh.mid)
	{
		return MID_HIGH_FRONT_ANGLE;
	}
	else if(data_channel->data > thresh.low)
	{
		return MID_LOW_FRONT_ANGLE;
	}
	else
	{
		return LOW_FRONT_ANGLE;
	}
}

int determine_wing_angle(int angles[NUM_CHANNELS])
{
	int highest_downforce = 0;
	for(int i = 0; i < NUM_CHANNELS; i++)
	{
		if(angles[i] > highest_downforce)
		{
			highest_downforce = angles[i];
		}
	}
	return highest_downforce;
}

// manual control of DRS, switch from high to low position when driver presses button
int drs_button(int cur_state)
{
	request_parameter(PRIO_HIGH, TCM_ID, SW_DRS_ID); // todo make button_press GCAN_PARAM_ID
	if(sw_drs.data == 1)
	{
		if(cur_state < (HIGH_REAR_ANGLE-LOW_REAR_ANGLE)/2)
		{
			return HIGH_REAR_ANGLE;
		}
		else
		{
			return LOW_REAR_ANGLE;
		}
	}
	return 0;
}

// manual control of active aero
int aa_button(int cur_state)
{
	request_parameter(PRIO_HIGH, TCM_ID, SW_AUX2_ID);
	if(sw_aux2.data == 1)
	{
		if(cur_state < (HIGH_FRONT_ANGLE-LOW_FRONT_ANGLE)/2)
		{
			return HIGH_FRONT_ANGLE;
		}
		else
		{
			return LOW_FRONT_ANGLE;
		}
	}
	return 0;
}

// parameter request
void parameter_request(FLOAT_CAN_STRUCT* data_channel)
{
	service_can_rx_buffer();
	request_parameter(PRIO_HIGH, OTHER_MODULE, data_channel->param_id);
	service_can_tx_hardware(acm_hcan);
}

// function to convert angle to PWM
void PWM_generation(int drs_state, int active_aero_state)
{
	TIM2->CCR1 = active_aero_state; // right
	TIM2->CCR2 = active_aero_state; // left
	TIM2->CCR3 = drs_state; // rear
}
