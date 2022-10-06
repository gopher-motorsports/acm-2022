#include "GopherCAN.h"
// specify which channels you want to use for data input
#define DATA_CHANNEL_1 brake_pressure_rear
#define DATA_CHANNEL_2 acceleration
#define DATA_CHANNEL_3 steering_anglels
#define DATA_CHANNEL_4 engine_temp

// thresholds for data channel 1
#define DATA_CHANNEL_1_MAX 100
#define DATA_CHANNEL_1_MIN 0
#define DATA_CHANNEL_1_HIGH 75
#define DATA_CHANNEL_1_MID 50
#define DATA_CHANNEL_1_LOW 25

// thresholds for data channel 2
#define DATA_CHANNEL_2_MAX 0
#define DATA_CHANNEL_2_MIN 0
#define DATA_CHANNEL_2_HIGH 0
#define DATA_CHANNEL_2_MID 0
#define DATA_CHANNEL_2_LOW 0

// thresholds for data channel 3
#define DATA_CHANNEL_3_MAX 0
#define DATA_CHANNEL_3_MIN 0
#define DATA_CHANNEL_3_HIGH 0
#define DATA_CHANNEL_3_MID 0
#define DATA_CHANNEL_3_LOW 0

// number of channels right now should be 1,2, or 3
#define NUM_CHANNELS 1
#define VALID_CHANNELS 5

// DRS: Angle of attack, 0 to 50, that will be chosen from based on the data passed into the function
// Might want to change naming conventions since idk which angle refers to a high downforce positon
#define HIGH_REAR_ANGLE 2067
#define MID_HIGH_REAR_ANGLE 1878
#define MID_LOW_REAR_ANGLE 1689
#define LOW_REAR_ANGLE 1500 // high downforce or low downforce position?
#define ERR_DRS 2067 //assuming 50 is high downforce position

// Active Aero: Angle of attack, 0 to 30, that will be chosen from based on the data passed into the function
#define HIGH_FRONT_ANGLE 1833
#define MID_HIGH_FRONT_ANGLE 1722
#define MID_LOW_FRONT_ANGLE 1611
#define LOW_FRONT_ANGLE 1500 // high downforce or low downforce position?
#define ERR_FRONT 1833 //assuming 30 is high downforce


typedef struct data_thresholds{
	FLOAT_CAN_STRUCT* channel_name;
	int high;
	int mid;
	int low;
	int max;
	int min;
} data_thresholds;


data_thresholds* init(CAN_HandleTypeDef* hcan_ptr);
void can_buffer_handling_loop();
int valid_data_channel(FLOAT_CAN_STRUCT* data_channel);
int calculate_rear_wing_angle(FLOAT_CAN_STRUCT *data_channel, data_thresholds thresh);
int calculate_front_wing_angle(FLOAT_CAN_STRUCT *data_channel, data_thresholds thresh);
int determine_wing_angle(int angles[NUM_CHANNELS]);
int drs_button(int cur_state);
int aa_button(int cur_state);
void parameter_request(FLOAT_CAN_STRUCT* data_channel);
void PWM_generation(int drs_state, int active_aero_state);
