/*
    ...

*/
#ifndef __FB_PROJDEFS_H__
#define __FB_PROJDEFS_H__
//
// ======================================================================
// Global variables
//
// Probably not good practise to use these many global variables.
// However, seems easier at the beginning. Should sort out vars later on
//
#define DATA_ACQ_MAX_SAMPLES_PER_CYCLE  (300)
#define DATA_ACQ_NUM_OF_CHANNELS        (2)
#define DATA_ACQ_NUM_OF_FILTER_COEFFICIANTS (4)
// === Global variable ("object") for data acquisition ===
typedef struct{
    float data[DATA_ACQ_MAX_SAMPLES_PER_CYCLE];  // data of one full cycle [cN]
    float valid_data[DATA_ACQ_MAX_SAMPLES_PER_CYCLE]; // valid full data of one cycle
    int idx;        // amount of datapoints in valid dataset
    float mean;    // last valid single mean value [cN]
    float act_value;  // current actual sensor value [cN]
    float act_value_filtered; // current value filtered
    float act_value_filtered_old_array[DATA_ACQ_NUM_OF_FILTER_COEFFICIANTS]; // array of last actual (filtered) values
    float integral_approx; // [cNs] approximated integral of one full measurement (i.e. 254 or 127 dataPoints)
} dataAcq_t;
// array of objects for btsr sensor data
dataAcq_t dataAcq[DATA_ACQ_NUM_OF_CHANNELS];
// event group for data Acquisition. defined in main()
EventGroupHandle_t dataAcq_event_group; 
// define event bits
// Bit to signal HallS high detected
#define BIT_0_HALLS             (1 << 0) // Bits if HallS was triggered (isr)
//#define BIT_VALID_MEASUREMENT   (1 << 1) // Bit if new valid measurement has been completed
#define BIT_DATA_ACQ_IS_ACTIVE  (1 << 2) // Bit if dataAcq is acquiring data
#define BIT_DATA_ACQ_IS_IDLE    (1 << 3) // Bit if dataAcq is idle

//
// event group for each hdrive. defined in main()
EventGroupHandle_t hdrive_event_group_array[2];
// define event bits for hdrive
//#define BIT_REQ_DISABLE_HDRIVE       (1 << 0)    // Bit 0 to request disable hdrive
#define BIT_CHANGE_SETPOINT_HDRIVE   (1 << 1)    // Bit 1 to request change in hdrive position
#define BIT_CONTROL_MODE_MANUAL       (1 << 2)    // Bit 2 to signal control Mode is Manual
#define BIT_CONTROL_MODE_MEAN         (1 << 3)    // Bit 3 to signal control Mode is mean value control
#define BIT_VALID_MEASUREMENT        (1 << 4) // Bit 4 if new valid measurement has been completed
#define BIT_HDRIVE_READY_TO_ENABLE   (1 << 5)    // Bit 5 hdrive ready to be enabled
#define BIT_HDRIVE_ALLOWED_TO_ENABLE (1 << 6)    // Bit 6 hdrive allowed to be enabled
#define BIT_HDRIVE_RUNNING            (1 << 7)  // Bit 7 hdrive is running (getting constant triggers)
//

// === Global variable ("object") for wifi ===
typedef struct{
    float setpoint_hdrive;                  // setpoint for MANUAL hdrive position from wifi, [%]
    float setpoint_mean_control;            // setpoint for AUTO mean control from wifi, [cN]
} wifi_fb_t;
//
wifi_fb_t wifi_fb[2];
//
EventGroupHandle_t wifi_event_group;
#define BIT_REQ_CHANGE_TO_MODE_0       (1 << 0)    // Bit 0 to request change to control Mode 0 (Manual)
#define BIT_REQ_CHANGE_TO_MODE_1       (1 << 1)    // Bit 1 to request change to control Mode 1 (Auto)
#define BIT_REQ_DISABLE_HDRIVE  (1 << 2)    // Bit 2 to request disable hdrive
#define BIT_REQ_ENABLE_HDRIVE   (1 << 3)    // Bit 3 to request enable hdrive

//
// === Global variable ("object") for mean control ===
typedef struct{
    float actual_single_mean_value_old; 
    float actual_mean_value_old;
    //
    //
    float actual_single_mean_value; // actual single mean tension value @ 348-352° [cN]
    float actual_mean_value;        // actual filtered mean tension for mean control [cN]
    float actual_mean_a[2];
    float actual_mean_b[2];
    float actual_mean_x[2];
    float actual_mean_y[2];
    //
    //
    //
    float mean_setpoint;
    float mean_setpoint_filtered;
    double a[3]; // array of a coefficiants for setpoint filtering
    double b[3]; // array of b coefficiants for setpoint filtering
    double y[3]; // array of y for setpoint filtering
    double x[3]; // array of x for setpoint filtering
    //
    float mean_control_output;      // [cN]
    float control_integral;         // [cN]
    //
    float display_mean;
    float display_mean_old;
} mean_control_t;
//
mean_control_t mean_control[2];
#define CONTROL_MODE_MANUAL     (0)
#define CONTROL_MODE_MEAN       (1)
uint8_t current_control_mode[2];   // current wanted control Mode of hdrives (0=manual, 1=mean)

// === Global variable ("object") for hdrive ===
// struct for hdrive object, with enum etc.
typedef enum{
    HD_DISABLED,
    HD_ENABLE,
    HD_INITIALIZE,
    HD_CLOSING,
    HD_CLOSED,
    HD_OPENING,
    HD_OPEN,
    HD_ERROR,
    HD_REQ_DISABLED     // from hmi requested disable
} hdrive_step_enum_t;
typedef struct{
    float setpointOpen;     // setpoint for open position, in angle [°]
    float setpointClosed;   // setpoint for closed position, in angle [°]
    float actualPosition;
    hdrive_step_enum_t step;
    int trigger_gpio_ch;
    float currentSetpointClosedPercent;    // closed setpoint of hdrive in percent for wifi [%]
}hdrive_t;
hdrive_t hdrive[2];

// === Global variable ("object") for nvs storage ===
typedef struct {
    uint32_t nvs_Setpoint_Manual;
    uint32_t nvs_Setpoint_Auto;
    uint32_t nvs_control_mode;
}nvs_data_t;
nvs_data_t nvs_data[2];


#include "driver/periph_ctrl.h"
#include "driver/timer.h"
// global timer
timer_group_t global_timer_group;
timer_idx_t global_timer_idx;



// === Global variables for LED stuff  ======
#define NUM_LEDS 280
//
EventGroupHandle_t led_event_group;
#define BIT_LED_CHANGE_TO_SINGLE        (1 << 0)    // Bit 0 to request change to Mode 0
#define BIT_LED_CHANGE_TO_BLINK_LINE    (1 << 1)    // Bit 1 to request change to Mode 1
#define BIT_LED_CHANGE_TO_BLEND         (1 << 2)    // Bit 1 to request change to Mode 2

typedef struct{
    int setpoint_grb; // desired grb
} led_grb_t;
//
led_grb_t led_grb;

// ======================================================================







#endif /* __FB_PROJDEFS_H__ */