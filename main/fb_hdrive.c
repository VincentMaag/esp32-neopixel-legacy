/*
    ...

*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"

#include "driver/ledc.h"
#include "esp_err.h"

#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "fb_projdefs.h"
#include "fb_hdrive.h"

//
static const char *TAG = "fb_hdrive";
//
// define digital Inputs
#define GPIO_HALL1_INPUT         GPIO_NUM_33
#define GPIO_HALL2_INPUT         GPIO_NUM_25
#define GPIO_HALL3_INPUT         GPIO_NUM_26
#define GPIO_TRIGGER1_INPUT      GPIO_NUM_18     // GPIO18 für triggerschaltung, 17 für pwm in ch3
#define GPIO_TRIGGER2_INPUT      GPIO_NUM_19     // GPIO19 für triggerschaltung, 5 für pwm in ch4
#define GPIO_TRIGGER3_INPUT      GPIO_NUM_21
#define GPIO_TRIGGER4_INPUT      GPIO_NUM_22
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_HALL1_INPUT) | (1ULL<<GPIO_HALL2_INPUT) | (1ULL<<GPIO_HALL3_INPUT) \
                            | (1ULL<<GPIO_TRIGGER1_INPUT) | (1ULL<<GPIO_TRIGGER2_INPUT) | (1ULL<<GPIO_TRIGGER3_INPUT) | (1ULL<<GPIO_TRIGGER4_INPUT))
// define digital Outputs
#define GPIO_ENABLE_OUTPUT       GPIO_NUM_15
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_ENABLE_OUTPUT))
// define PWM Outputs
#define GPIO_PWM_OUTPUT_CH0	         GPIO_NUM_27
#define GPIO_PWM_OUTPUT_CH1	         GPIO_NUM_14
#define GPIO_PWM_OUTPUT_CH2	         GPIO_NUM_12
#define GPIO_PWM_OUTPUT_CH3    	     GPIO_NUM_13
#define PWM_NUM_OUTPUT_CHANNELS  	 (2)
// pwm input & output resolution
#define PWM_OUTPUT_RESOLUTION        LEDC_TIMER_12_BIT
#define PWM_INPUT_RESOLUTION         LEDC_TIMER_12_BIT
// number of pwm input channels
#define GPIO_PWM_INPUT_CH0           GPIO_NUM_4
#define GPIO_PWM_INPUT_CH1           GPIO_NUM_16
//#define GPIO_PWM_INPUT_CH2           GPIO_NUM_17
//#define GPIO_PWM_INPUT_CH3           GPIO_NUM_5
#define PWM_NUM_INPUT_CHANNELS  	 (2)
#define PWM_INPUT_FREQ               (75)
// define hdrive channels
#define HDRIVE_CH0                  (0)
#define HDRIVE_CH1                  (1)
#define HDRIVE_CH2                  (2)
#define HDRIVE_CH3                  (3)
#define HDRIVE_NUM_OF_CHANNELS      (2)
// Hardware timer clock divider
#define TIMER_DIVIDER               (16)  
#define TIMER_TICK_FREQ             ( 80000000 / 16 )
// Global timer group & index for pwm input reading. Timer 1, index 0
timer_group_t timer_group       = TIMER_GROUP_1;
timer_idx_t timer_idx           = TIMER_0;
// Global led channel "object"
ledc_channel_config_t ledc_channel[PWM_NUM_OUTPUT_CHANNELS];
// Global variables for interrupt handling
uint64_t isrTicksAtRising[PWM_NUM_INPUT_CHANNELS];
uint64_t isrTicksAtFalling[PWM_NUM_INPUT_CHANNELS];
uint64_t isrTickDifference[PWM_NUM_INPUT_CHANNELS];
// array to select gpio number from hdrive channel number (for example in isr)
int gpio_ch_select[HDRIVE_NUM_OF_CHANNELS] = {GPIO_PWM_INPUT_CH0,GPIO_PWM_INPUT_CH1};
//
//
// === PARAMETERS ======================================================
// measured angles for minimum and maximum force of hdrives
float minimumForceAngle = 169;
float maximumForceAngle = 150;
float absoluteOpenAngle = 185;
//
uint8_t delayUntilStep = 54; // [ms] to delay. 54ms delay to do a optional Step
bool enableOptionalStep = false;
float optionalStepAngle = 1.0; // [°]
//
uint8_t delayUntilOpen = 10; // [ms] to delay. 10ms delay to open @ defined sample time
uint8_t delayUntilClose = 4; // [ms] to delay. 4ms delay to open @ defined sample time
//
// === END PARAMETERS ==================================================
//
//
//
// =========================================================
// fuction to initialize all gpio's, specifically for hdrive
void init_gpio_hdrive(){
    // configure DigIns
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 1; /// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    // configure DigOut(s) and initialize "low"
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_ENABLE_OUTPUT,0);
    // prepare and configure pwm ouputs.
    // first, prepare individual configuration of each LED (pwm) channel,
    // we use timer 0, channels 0-3
    for(int i = 0; i < PWM_NUM_OUTPUT_CHANNELS; i++){
		//ledc_channel[i].channel    = LEDC_CHANNEL_0;
        ledc_channel[i].duty       = 0;
		//ledc_channel[i].gpio_num   = GPIO_PWM_OUTPUT_CH0;
		ledc_channel[i].speed_mode = LEDC_HIGH_SPEED_MODE;
		ledc_channel[i].timer_sel  = LEDC_TIMER_0;   
    };
    // configure the ones we actualy use (2 from 4)
    ledc_channel[0].channel    = LEDC_CHANNEL_0;
    ledc_channel[0].gpio_num   = GPIO_PWM_OUTPUT_CH0;
    ledc_channel[1].channel    = LEDC_CHANNEL_1;
    ledc_channel[1].gpio_num   = GPIO_PWM_OUTPUT_CH1;   
    // next, configure the LED timer itself (timer 0) and initialize
       ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_OUTPUT_RESOLUTION,
        .freq_hz = 5000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&ledc_timer);
    // now, initialize all configured led pwm channels
    int ch;
    for (ch = 0; ch < PWM_NUM_OUTPUT_CHANNELS; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }
    // configure free running timer (timer 1 idx 0) and initialize
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_START;
    config.alarm_en = TIMER_ALARM_DIS;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = false;
    timer_init(timer_group, timer_idx, &config);
    timer_start(timer_group,timer_idx);
    // configure and initialize isr
    // first, config gpios
    gpio_set_intr_type(GPIO_PWM_INPUT_CH0, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(GPIO_PWM_INPUT_CH1, GPIO_INTR_ANYEDGE);
    // next, install isr service, with esp_intr_flag_defult = 0
    gpio_install_isr_service(0);
    // finally, hook isr to individual gpios
    gpio_isr_handler_add(GPIO_PWM_INPUT_CH0, gpio_isr_handler_tickTimeBetweenEdges, (void*) HDRIVE_CH0);
    gpio_isr_handler_add(GPIO_PWM_INPUT_CH1, gpio_isr_handler_tickTimeBetweenEdges, (void*) HDRIVE_CH1);
    // initialize tick vectors
    for(ch=0;ch<PWM_NUM_INPUT_CHANNELS;ch++){
        isrTicksAtRising[ch]    =0;
        isrTicksAtFalling[ch]   =0;
        isrTickDifference[ch]   =0;
    }
}
// =========================================================
// function to calculate duty cycle output (setpoint hdrive) out of SET angle in degrees
int hdrive_degree_2_duty(float degree){
    float a = 0.018, b = -2.65; // these values leed to: 150°-200° = 0.05-0.95 dc
    float duty = (degree * a + b);
    // make sure duty cycle is inbetween 5% and 95%
    if(duty<0.05){
        duty = 0.05;
    }else if(duty >0.95){
        duty = 0.95;
    }
    float dutyRes  = PWM_OUTPUT_RESOLUTION;
    return (int) (duty * (pow(2,dutyRes) - 1)); // calc duty in x-bit resolution
}
// =========================================================
// function to calculate angle out of a scaled percentage input (from wifi) from 0-100%
float percent_2_angle(float percent){
    float a = (maximumForceAngle - minimumForceAngle)/100.0, b = minimumForceAngle;
    float angle = (percent * a + b);
    // make sure angle is inbetween 150° and 170°
    if(angle< maximumForceAngle){
        angle = maximumForceAngle;
    }else if(angle > minimumForceAngle){
        angle = minimumForceAngle;
    }
    return angle;
}
// =========================================================
// function to calculate scaled percent out of angle input (for wifi) from 150-170%
float angle_2_percent(float angle){
    float a = 100.0/(maximumForceAngle - minimumForceAngle), b = -100.0*minimumForceAngle/(maximumForceAngle - minimumForceAngle);
    float percent = (angle * a + b);
    // make sure percent is valid
    if(percent< 0){
        percent = 0;
    }else if(percent > 100){
        percent = 100;
    }
    return percent;
}
// =========================================================
// function to SET position (angle in degrees) of individual hdrive channels
void hdrive_set_position(int hdrive_channel, float hdrive_setAngle){
    uint32_t duty = (uint32_t) hdrive_degree_2_duty(hdrive_setAngle);
    ledc_set_duty(ledc_channel[hdrive_channel].speed_mode, ledc_channel[hdrive_channel].channel, duty);
    ledc_update_duty(ledc_channel[hdrive_channel].speed_mode, ledc_channel[hdrive_channel].channel);
}
// =========================================================
// function to GET position (angle in degrees) of individual hdrive channels
// returns hdrive position in [degrees]
float hdrive_get_position(int hdrive_channel){
    float timeDiff = ((float)isrTickDifference[hdrive_channel]) / ((float)TIMER_TICK_FREQ);
    float pwmFreq  = (float)PWM_INPUT_FREQ;
    float duty     = timeDiff * pwmFreq; // dc in [%/100]
    //float a = 360/0.975, b = 0;   // a=360 & b=0 lead to 0-360° for dc 0-0.975
    // change to this transformation of angle if hdrive is built on viergelenk
    float a = -360/0.975, b = 360;   // a=360 & b=0 lead to 0-360° for dc 0-0.975
    float angle = a*duty + b;
    if(angle>360){
        angle = 360;
    }else if(angle < 0){
        angle = 0;
    }
    return angle;
}
// =========================================================
// interrupt service routine to measure ticks between edges
// This version does not measure time between rising edges!
static void IRAM_ATTR gpio_isr_handler_tickTimeBetweenEdges(void* hdrive_ch){
    // check edge of gpio, because interrupt is thrown on both edges
    int ch = (int)hdrive_ch;
    if(gpio_get_level(gpio_ch_select[ch])){
        timer_get_counter_value(timer_group,timer_idx,&isrTicksAtRising[ch]);
    }else{
        timer_get_counter_value(timer_group,timer_idx,&isrTicksAtFalling[ch]);
        isrTickDifference[ch] = isrTicksAtFalling[ch] - isrTicksAtRising[ch];
    }   
}
// =========================================================
// =========================================================
// h-drive task
void hdrive_task(void* arg){
    // =====================================================
    // local variables
    // variable to store last tick time for exakt cycle time. Updates every cycle
    TickType_t previousWakeTime = xTaskGetTickCount();
    TickType_t pauseTime = 2; // cycle time in [ms]
    int cycleCount = 0; // count hdrive swicth cycles
    uint64_t actual_counter_value = 0; // counter value for testing/debugging
    uint64_t before_counter_value = 0; // counter value for testing/debugging
    uint64_t after_counter_value = 0; // counter value for testing/debugging
    int difference_hdrive = 0;
    uint8_t testing0 = 0;
    uint8_t testing1 = 0;
    int step_counter[HDRIVE_NUM_OF_CHANNELS] = {0,0};
    //
    uint8_t delayCounter = 0;                   // we may use one variable for all drives because drives dont work at the same time!
    //
    uint8_t delayStepWhileClosedCounter[2] = {0,0};
    //
    int hdriveRunningTimeoutCounter[HDRIVE_NUM_OF_CHANNELS] =  {0,0}; // timeout for detecting idle channel$
    bool hdriveTriggerOld[HDRIVE_NUM_OF_CHANNELS] = {false,false};
    // ====================================================
    // Initialize stuff
    init_gpio_hdrive();
    //...
    // init global hdrive objects
    hdrive[0].setpointClosed = percent_2_angle(wifi_fb[0].setpoint_hdrive);
    hdrive[0].setpointOpen   = absoluteOpenAngle;
    hdrive[0].step           = HD_DISABLED;
    hdrive[0].actualPosition = hdrive_get_position(0);
    hdrive[0].trigger_gpio_ch= GPIO_TRIGGER1_INPUT;
    hdrive[0].currentSetpointClosedPercent = 0;
    //
    hdrive[1].setpointClosed = percent_2_angle(wifi_fb[1].setpoint_hdrive);
    hdrive[1].setpointOpen   = absoluteOpenAngle;
    hdrive[1].step           = HD_DISABLED;
    hdrive[1].actualPosition = hdrive_get_position(1);
    hdrive[1].trigger_gpio_ch= GPIO_TRIGGER2_INPUT;
    hdrive[1].currentSetpointClosedPercent = 0;
    // Disable hdrives
    gpio_set_level(GPIO_ENABLE_OUTPUT,0);
    // initialize control mode to manual, check last control mode
    if(current_control_mode[0]){
        xEventGroupSetBits(hdrive_event_group_array[0],BIT_CONTROL_MODE_MEAN);
    }else{
        xEventGroupSetBits(hdrive_event_group_array[0],BIT_CONTROL_MODE_MANUAL);
    }
    if(current_control_mode[1]){
        xEventGroupSetBits(hdrive_event_group_array[1],BIT_CONTROL_MODE_MEAN);
    }else{
        xEventGroupSetBits(hdrive_event_group_array[1],BIT_CONTROL_MODE_MANUAL);
    }
    // ====================================================
    while(1){

        //timer_get_counter_value(timer_group,timer_idx,&before_counter_value);
        // update parameters
        hdrive[0].actualPosition = hdrive_get_position(0);
        hdrive[1].actualPosition = hdrive_get_position(1);
        // ====================================================
        //
        //
        // =========================================================================================================================        
        // === ENABLE / DISABLE ====================================================================================================
        //
        // watch for disable command from browser/wifi. Set in hdrive channel 0!
        if((xEventGroupGetBits(wifi_event_group) & BIT_REQ_DISABLE_HDRIVE) == BIT_REQ_DISABLE_HDRIVE){
            xEventGroupClearBits(wifi_event_group,BIT_REQ_DISABLE_HDRIVE);
            // move state machines to requested disable
            hdrive[0].step = HD_REQ_DISABLED;
            hdrive[1].step = HD_REQ_DISABLED;
            // disable hdrives
            printf("master disabling hdrives\n");
            gpio_set_level(GPIO_ENABLE_OUTPUT,0);
            //
        // wait for request of re-enabling hdrives. Only if hdrives are in fact disabled!
        } else if((xEventGroupGetBits(wifi_event_group) & BIT_REQ_ENABLE_HDRIVE) == BIT_REQ_ENABLE_HDRIVE){
            xEventGroupClearBits(wifi_event_group,BIT_REQ_ENABLE_HDRIVE);
            // check if hdrives are in disabled state, and if so, let them try to enable again
            if((hdrive[0].step == HD_REQ_DISABLED) && (hdrive[1].step == HD_REQ_DISABLED)){
                printf("master letting hdrives try to enable again\n");
                hdrive[0].step = HD_ENABLE;
                hdrive[1].step = HD_ENABLE;
            } else {
                printf("hdrives are not both in requested disabled state. doing nothing...\n");
            }
        // wait for both hdrives to be ready to enable. If so, allow them to be enabled in state machine and enable via gpio
        } else if((xEventGroupGetBits(hdrive_event_group_array[0]) & BIT_HDRIVE_READY_TO_ENABLE) == BIT_HDRIVE_READY_TO_ENABLE){
            //printf("master saw group 0\n");
            if((xEventGroupGetBits(hdrive_event_group_array[1]) & BIT_HDRIVE_READY_TO_ENABLE) == BIT_HDRIVE_READY_TO_ENABLE){
                printf("master saw two ready drives. setting bits to enable\n");
                xEventGroupSetBits(hdrive_event_group_array[0],BIT_HDRIVE_ALLOWED_TO_ENABLE);
                xEventGroupSetBits(hdrive_event_group_array[1],BIT_HDRIVE_ALLOWED_TO_ENABLE);
                gpio_set_level(GPIO_ENABLE_OUTPUT,1);
            }
        }
        // =========================================================================================================================
        // === CONTROL MODE ========================================================================================================
        // === START OF FOR LOOP ============
        //
        for(int hdrive_ch=0;hdrive_ch<HDRIVE_NUM_OF_CHANNELS;hdrive_ch++){
            //
            // === Control Mode and choose Setpoint Closed =========================================================================
            // check wich control Mode is active (if any)
            // if manual mode:
            if((xEventGroupGetBits(hdrive_event_group_array[hdrive_ch]) & BIT_CONTROL_MODE_MANUAL) == BIT_CONTROL_MODE_MANUAL){
                // if manual mode and new setpoint...
                if((xEventGroupGetBits(hdrive_event_group_array[hdrive_ch]) & BIT_CHANGE_SETPOINT_HDRIVE) == BIT_CHANGE_SETPOINT_HDRIVE){
                ESP_LOGE(TAG,"==== [%i] TRYING TO CHANGE HDRIVE CLOSED POSITION ====\n",hdrive_ch);
                xEventGroupClearBits(hdrive_event_group_array[hdrive_ch],BIT_CHANGE_SETPOINT_HDRIVE);
                // set closed setpoint to wifi setpoint
                //hdrive[hdrive_ch].setpointClosed = wifi_fb[hdrive_ch].setpoint_hdrive;
                hdrive[hdrive_ch].setpointClosed = percent_2_angle(wifi_fb[hdrive_ch].setpoint_hdrive);
                }
            // if mean mode:   
            }else if((xEventGroupGetBits(hdrive_event_group_array[0]) & BIT_CONTROL_MODE_MEAN) == BIT_CONTROL_MODE_MEAN){
                // if mean mode, setpoint = control output:
                hdrive[hdrive_ch].setpointClosed = mean_control[hdrive_ch].mean_control_output;
            }

            // after setpoint closed chosen, check absolute boundries
            if(hdrive[hdrive_ch].setpointClosed < 150){
                hdrive[hdrive_ch].setpointClosed = 150;
            }else if(hdrive[hdrive_ch].setpointClosed > 200){
                hdrive[hdrive_ch].setpointClosed = 200;
            }
            // and copy current setpoint closed for wifi
            hdrive[hdrive_ch].currentSetpointClosedPercent = angle_2_percent(hdrive[hdrive_ch].setpointClosed);
        
        // =========================================================================================================================  
        // === WATCH FOR TIMEOUT 2s ================================================================================================
        if(gpio_get_level(hdrive[hdrive_ch].trigger_gpio_ch) == hdriveTriggerOld[hdrive_ch]){
            if(hdriveRunningTimeoutCounter[hdrive_ch] >= (2000/pauseTime)){
                xEventGroupClearBits(hdrive_event_group_array[hdrive_ch],BIT_HDRIVE_RUNNING);
            }else{
                hdriveRunningTimeoutCounter[hdrive_ch]++;
            }
        }else{
            hdriveRunningTimeoutCounter[hdrive_ch] = 0;
            if(hdrive[hdrive_ch].step > 2){
                xEventGroupSetBits(hdrive_event_group_array[hdrive_ch],BIT_HDRIVE_RUNNING);
            }else{
                xEventGroupClearBits(hdrive_event_group_array[hdrive_ch],BIT_HDRIVE_RUNNING);
            }
        }
        hdriveTriggerOld[hdrive_ch] = gpio_get_level(hdrive[hdrive_ch].trigger_gpio_ch);
        //
        // ==========================================================================================================================
        // === STATE MACHINE ========================================================================================================
        // main state machine here!
        switch (hdrive[hdrive_ch].step){
            // ==========================================================================================
            // DISABLED
            // ==========================================================================================
            case HD_DISABLED:
                step_counter[hdrive_ch]++;
                if(step_counter[hdrive_ch] >= 4000){
                    printf("[%i] Trying to enable hdrive. Checking if position is in-range...\n",hdrive_ch);
                    step_counter[hdrive_ch] = 0;
                    hdrive[hdrive_ch].step = HD_ENABLE;
                }else if(step_counter[hdrive_ch] == 3000){
                    printf("[%i] Try to enable hdrive in 1...\n",hdrive_ch);
                }else if(step_counter[hdrive_ch] == 2000){
                    printf("[%i] Try to enable hdrive in 2...\n",hdrive_ch);
                }else if(step_counter[hdrive_ch] == 1000){
                    printf("[%i] Try to enable hdrive in 3...\n",hdrive_ch);
                }
                //
                hdrive_set_position(hdrive_ch,150.0); // if not enabled, just set position to be closed
                //
            break;
            // ==========================================================================================
            // ENABLE
            // ==========================================================================================
            case HD_ENABLE:
                // check if actual position is in range
                if((hdrive[hdrive_ch].actualPosition>=145) && (hdrive[hdrive_ch].actualPosition<=290)){
                    //printf("[%i] hdrive position in-range: %f. ready to enable...\n",hdrive_ch, hdrive[hdrive_ch].actualPosition);
                    // setposition = actual position. This works if hdrive is between 150-200° (see hdrive_set_position)
                    // actual position between 200 and 290, brake will jump to 200 after enable (that's ok for now)
                    hdrive_set_position(hdrive_ch,hdrive[hdrive_ch].actualPosition);
                    // signal readyness
                    xEventGroupSetBits(hdrive_event_group_array[hdrive_ch],BIT_HDRIVE_READY_TO_ENABLE);
                    // wait for all hdrives to be ready
                    if((xEventGroupGetBits(hdrive_event_group_array[hdrive_ch]) & BIT_HDRIVE_ALLOWED_TO_ENABLE) == BIT_HDRIVE_ALLOWED_TO_ENABLE){
                        xEventGroupClearBits(hdrive_event_group_array[hdrive_ch],BIT_HDRIVE_READY_TO_ENABLE | BIT_HDRIVE_ALLOWED_TO_ENABLE);
                        hdrive[hdrive_ch].step = HD_INITIALIZE;
                    }
                }else{
                    printf("[%i] hdrive position NOT in-range: %f. retrying...\n",hdrive_ch, hdrive[hdrive_ch].actualPosition);
                    xEventGroupClearBits(hdrive_event_group_array[hdrive_ch],BIT_HDRIVE_READY_TO_ENABLE | BIT_HDRIVE_ALLOWED_TO_ENABLE);
                    hdrive[hdrive_ch].step = HD_DISABLED;
                }
            break;
            // ==========================================================================================
            // INITIALIZE
            // ==========================================================================================
            case HD_INITIALIZE:
                // wait in this state
                step_counter[hdrive_ch]++;
                if(step_counter[hdrive_ch]<=1){
                    printf("[%i] hdrive enabled, waiting for 1 second...\n",hdrive_ch);
                }else if(step_counter[hdrive_ch]>=500){
                    printf("[%i] trying to close brake...\n",hdrive_ch);
                    hdrive_set_position(hdrive_ch,hdrive[hdrive_ch].setpointClosed);
                    step_counter[hdrive_ch] = 0;
                    hdrive[hdrive_ch].step = HD_CLOSING;
                }
            break;
            // ==========================================================================================
            // CLOSING
            // ==========================================================================================
            case HD_CLOSING:
                // check if brake has closed:
                if(abs(hdrive[hdrive_ch].setpointClosed-hdrive[hdrive_ch].actualPosition) <= 4 ){
                    //printf("[%i] hdrive has successfully closed...\n",hdrive_ch);
                    //delayStepWhileClosedCounter[hdrive_ch] = 0;
                    hdrive[hdrive_ch].step = HD_CLOSED;
                }
                //
                delayStepWhileClosedCounter[hdrive_ch]++;
                // keep telling hdrive to go to setposition! Must not be done for opening because opensetpoint never changes
                hdrive_set_position(hdrive_ch,hdrive[hdrive_ch].setpointClosed);

            break;
            // ==========================================================================================
            // CLOSED
            // ==========================================================================================
            case HD_CLOSED:
                //printf("%i\n",hdrive[hdrive_ch].step);
                // wait for trigger:
                if(gpio_get_level(hdrive[hdrive_ch].trigger_gpio_ch)){
                    //printf("[%i] hdrive trigger detected high. Trying to open brake after delay time...\n",hdrive_ch);
                    // delay until opening
                    if(delayCounter >= (delayUntilOpen/pauseTime)){
                        hdrive_set_position(hdrive_ch,hdrive[hdrive_ch].setpointOpen);
                        hdrive[hdrive_ch].step = HD_OPENING;
                        delayStepWhileClosedCounter[hdrive_ch] = 0;
                        delayCounter = 0;
                    }else{
                        delayCounter++;
                    }
                }else if(delayStepWhileClosedCounter[hdrive_ch] >= (delayUntilStep/pauseTime)){
                    // if not yet to open, wait for optional step @ 54 ms ab closed trigger
                    if(enableOptionalStep){
                        hdrive_set_position(hdrive_ch,(hdrive[hdrive_ch].setpointClosed - optionalStepAngle));
                    }else{
                        hdrive_set_position(hdrive_ch,(hdrive[hdrive_ch].setpointClosed));
                    }
                }else{
                    delayStepWhileClosedCounter[hdrive_ch]++;
                    hdrive_set_position(hdrive_ch,hdrive[hdrive_ch].setpointClosed);  
                }
                

            break;
            // ==========================================================================================
            // OPENING
            // ==========================================================================================
            case HD_OPENING:
                // check if brake has opened:
                if(abs(hdrive[hdrive_ch].setpointOpen-hdrive[hdrive_ch].actualPosition) <= 4 ){
                    //printf("[%i] hdrive has successfully opened...\n",hdrive_ch);
                    hdrive[hdrive_ch].step = HD_OPEN;
                }
            break;
            // ==========================================================================================
            // OPEN
            // ==========================================================================================
            case HD_OPEN:
                // wait for trigger:
                if(gpio_get_level(hdrive[hdrive_ch].trigger_gpio_ch)==0){
                    //printf("[%i] hdrive trigger detected low. Trying to close brake...\n"),hdrive_ch;
                    // delay until opening
                    if(delayCounter >= (delayUntilClose/pauseTime)){
                        hdrive_set_position(hdrive_ch,hdrive[hdrive_ch].setpointClosed);
                        hdrive[hdrive_ch].step = HD_CLOSING;
                        delayCounter = 0;
                    }else{
                        delayCounter++;
                    }
                    //
                    delayStepWhileClosedCounter[hdrive_ch]++;
                } // else, wait for reset command
            break;
            // ==========================================================================================
            // REQ_DISABLED
            // ==========================================================================================
            case HD_REQ_DISABLED:
                // wait for re-enabling
                // this state is only left if Master re-enables drives through forcing step of state machine
            // ==========================================================================================
            // DEFAULT
            // ==========================================================================================
            default:
                //...
            break;
        } // end switch
        } // end for


 

        //mean_control_filter_data(&mean_control[0], 100, 1);

        





        // for measurement purposes
        //timer_get_counter_value(timer_group,timer_idx,&after_counter_value);
        //difference_hdrive = (int)(after_counter_value - before_counter_value);
        
        // ==========================================================================================
        // print stuff every x cycles, i.e. every 2*x ms
        if(cycleCount>=250){
            //printf("free running timer value [s]: %.5f\n",ftestvar1);
            //printf("= AcPos: %.2f = Step: %i = eventGroupBits: %i, logic: %i\n",hdrive[0].actualPosition,hdrive[0].step,xEventGroupGetBits(hdrive_event_group_array[0]),(xEventGroupGetBits(hdrive_event_group_array[0]) & BIT_REQ_ENABLE_HDRIVE));
            //printf("= AcPos0: %.2f = AcPos1: %.2f =\n",hdrive[0].actualPosition,hdrive[1].actualPosition);
            //printf("= 0: %i = 1: %i =\n",gpio_get_level(GPIO_TRIGGER1_INPUT),gpio_get_level(GPIO_TRIGGER2_INPUT));
            //printf("= step 0: %i = step 1: %i =\n",hdrive[0].step,hdrive[1].step);
            //printf("[0]: setpoint %.1f, feedback: %.2f\n",hdrive[0].setpointClosed,hdrive[0].currentSetpointClosedPercent);
            //printf("[hd1] -%i-\n",difference_hdrive);
            //printf("trig1: %i\n",gpio_get_level(GPIO_TRIGGER1_INPUT));

            //testing0 = (xEventGroupGetBits(hdrive_event_group_array[0]) & BIT_HDRIVE_RUNNING);
            //testing1 = (xEventGroupGetBits(hdrive_event_group_array[1]) & BIT_HDRIVE_RUNNING);
            
            // IB printfs ===============
            //printf("= AcPos0: %.2f = AcPos1: %.2f =\n",hdrive[0].actualPosition,hdrive[1].actualPosition);
            //printf("= 0: %i = 1: %i =\n",gpio_get_level(GPIO_TRIGGER1_INPUT),gpio_get_level(GPIO_TRIGGER2_INPUT));
            // ==========================


            //printf("[0]: %i === [1]: === %i === %i\n",testing0,testing1,hdriveRunningTimeoutCounter[0]);
            cycleCount = 0;
        }
        // ====================================================
        // delay until next cycle, increment cycle counter
        cycleCount++;
        vTaskDelayUntil(&previousWakeTime,pauseTime);  
        //vTaskDelay(1);
        // ====================================================
    }
}




