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
#include "esp_event_loop.h"
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
#include "fb_data_acq.h"
//
static const char *TAG = "fb_data_acq";
//
// define digital Inputs (interrupt DigIns)
#define GPIO_HALLS_INPUT         GPIO_NUM_23 //GPIO_NUM_32           // GPIO_NUM_23 for HallS, 32 for reserveGPIO
// 
#define GPIO_32                  GPIO_NUM_32             // test gpio, take out!! (or define global)
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_HALLS_INPUT))
//
// define Analog Inputs
#define ADC1_BTSR1_INPUT         ADC1_CHANNEL_0
#define ADC1_BTSR2_INPUT         ADC1_CHANNEL_3
#define ADC1_BTSR3_INPUT         ADC1_CHANNEL_6
#define ADC1_BTSR4_INPUT         ADC1_CHANNEL_7
// enum type for data acquisition state machine
typedef enum{
    DA_IDLE,
    DA_ACQUIRING,
    DA_ERROR
} dataAcq_step_enum_t;
dataAcq_step_enum_t step_dataAcq; // one step because all Channels acquire data at the same time
//
bool hallS_flag = false;
//
//
// === PARAMETERS ======================================================
// Parameters for mean value calculation
float fromDegree = 340;
float toDegree = 350;
float approxIntegralThreshhold = 2.0;
//
// === END PARAMETERS ==================================================
//
//
// =========================================================
// fuction to initialize all gpio's, specifically for data acquisition
void dataAcq_init_gpio(){
    // configure DigIns
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    // configure analog Inputs, ADC1, ATTEN_DB_6 for ~0-2.2V
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_BTSR1_INPUT,ADC_ATTEN_DB_6);
    adc1_config_channel_atten(ADC1_BTSR2_INPUT,ADC_ATTEN_DB_6);
    adc1_config_channel_atten(ADC1_BTSR3_INPUT,ADC_ATTEN_DB_6);
    adc1_config_channel_atten(ADC1_BTSR4_INPUT,ADC_ATTEN_DB_6);
    // configure and initialize isr
    // first, config gpios
    gpio_set_intr_type(GPIO_HALLS_INPUT, GPIO_INTR_POSEDGE);
    // next, install isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(GPIO_HALLS_INPUT, gpio_isr_handler_catchHallS, (void*) &hallS_flag);
}
// =========================================================
// fuction to calculate yarn tension out of 12-bit analog input
// return yarn tension in [cN]
float dataAcq_get_yarn_tension(int adc_channel){
    float a = 0.0094, b = 1.61;   // approximated function for 12-bit adc, 0.1V-10V = 0cN-50cN with adc calib. (see matlab/documentation)
    int rawData = adc1_get_raw(adc_channel);
    return (float)rawData * a + b;
}
// =========================================================
// function to calculate mean value at certain time (e.g. 348°-352°)
// of ONE dataset of type "dataAcq_t". Works with adress, no return value
// Needs dataAcq object, from/to degree values [°], amount of samples in 
// curent measurement
void dataAcq_calc_mean(dataAcq_t *pDataAcq, float fromDeg, float toDeg, int amount_of_samples){
    // check if parameters are valid
    if((toDeg < fromDeg) || (fromDeg < 0) || (toDeg > 360)){
        printf("dataAcq_calc_mean function parameters not valid! PANIC!");
        return;
    }
    // calculate indexes of single measurement dependent on measurement length
    // because we have 1kHz measurement sample rate, we index of data is in ms!
    int fromMS = (int)round((float)amount_of_samples / 360.0 *  fromDeg);
    int toMS   = (int)round((float)amount_of_samples / 360.0 *  toDeg);
    // temp variable for sum of valid samples
    float sum = 0;
    // amount of samples for mean calculation
    int num_valid_samples = (toMS - fromMS) + 1; // because, at least one sample must be evaluated!
    // check if valid amount of samples:
    if(num_valid_samples >= 1){
        // calc sum of all valid samples
        for(int n = fromMS; n <= toMS; n++){
            sum += (*pDataAcq).data[n];
        }
        // calc mean of all valid samples
        (*pDataAcq).mean = sum / (float)num_valid_samples;
    } 
}
// =========================================================
// function to filter ONE dataset of type "dataAcq_t"
// works with addresses, no return values
void dataAcq_filter_data(dataAcq_t *pDataAcq){
    //float A = 1, B = 0; // no filter
    //float A = 0.1813, B = 0.8187; // PT1, ZOH, T1=5ms @ Ts=1ms
    //float A = 0.01, B = 0.99; // PT1, ZOH, T1=100ms @ Ts=1ms
    //float A = 0.3297, B = 0.6703; // PT1, ZOH, T1=5ms @ Ts=2ms
    float A = 0.8647, B = 0.1353; // PT1, ZOH, T1=1ms @ Ts=2ms
    // calculate new filtered value through actual value and old filtered value
    (*pDataAcq).act_value_filtered = (*pDataAcq).act_value * A + (*pDataAcq).act_value_filtered_old_array[0] * B;
    // copy actual filtered value into old
    (*pDataAcq).act_value_filtered_old_array[0] = (*pDataAcq).act_value_filtered;
    //...
}
// =========================================================
// interrupt service routine to catch rising edge of HallS
// just send TRUE Flag if rising adge of HallS sensor detected
static void IRAM_ATTR gpio_isr_handler_catchHallS(void* flag){ 
    // Signal that HallS high was detected
    bool* pflag = (bool*)flag;
    *pflag = true;
}
// =========================================================
// =========================================================
// data-acquisition task
void dataAcq_task(void* arg){
    // =====================================================
    // local variables
    // variable to store last tick time for exakt cycle time. Updates every cycle
    TickType_t previousWakeTime0 = xTaskGetTickCount();
    TickType_t pauseTime = 2;           // actual cycle time [ms] off dataAcq loop
    uint64_t actual_counter_value = 0; // counter value for testing/debugging
    uint64_t before_counter_value = 0; // counter value for testing/debugging
    uint64_t after_counter_value = 0; // counter value for testing/debugging
    int difference_dataAcq = 0;
    uint64_t difference_dataAcq_array[15] = {};
    int difference_idx = 0;
    int cycleCount = 0; // count switch cycles
    int idx_data = 0;  // index for 0-max300 datapoint. Only one index for all Sensors because either all ar acquiring or no one is
    //
    bool hallS_old = false;
    // ====================================================
    // Initialize stuff
    dataAcq_init_gpio();
    step_dataAcq = DA_IDLE;
    xEventGroupSetBits(dataAcq_event_group,BIT_DATA_ACQ_IS_IDLE);
    // ====================================================
    while(1){
        //
        // timer_get_counter_value(global_timer_group,global_timer_idx,&before_counter_value);
        //
        // get yarn tension of both channels and filter them
        // channel 1 (ADC1 CH0)
        dataAcq[0].act_value = dataAcq_get_yarn_tension(ADC1_BTSR1_INPUT);
        dataAcq_filter_data(&dataAcq[0]);
        // channel 2 (ADC1 CH3)
        dataAcq[1].act_value = dataAcq_get_yarn_tension(ADC1_BTSR2_INPUT);
        dataAcq_filter_data(&dataAcq[1]);
        //
        switch (step_dataAcq){

            case DA_IDLE:
                // wait for HallS trigger
                if(hallS_flag == true){
                    printf("==== HallS detected from isr. Trying to Aquire Data. ====\n");
                    // first dataPoint:
                    idx_data = 0;
                    dataAcq[0].data[idx_data] = dataAcq[0].act_value_filtered;
                    dataAcq[1].data[idx_data] = dataAcq[1].act_value_filtered;
                    idx_data++;
                    hallS_flag = false;
                    // signal other tasks that dataAcq is active
                    xEventGroupClearBits(dataAcq_event_group,BIT_DATA_ACQ_IS_IDLE);
                    xEventGroupSetBits(dataAcq_event_group,BIT_DATA_ACQ_IS_ACTIVE);
                    step_dataAcq = DA_ACQUIRING;
                }
            break;
            //
            case DA_ACQUIRING:
                // check if HallS detects next cycle
                if(hallS_flag == true){
                    hallS_flag = false;
                    // try to calculate mean out of sample
                    dataAcq_calc_mean(&dataAcq[0], fromDegree, toDegree, idx_data);
                    dataAcq_calc_mean(&dataAcq[1], fromDegree, toDegree, idx_data);
                    // save data and idx only if full measurement is done
                    dataAcq[0].idx = idx_data;
                    dataAcq[1].idx = idx_data;
                    // reset approx. integral
                    dataAcq[0].integral_approx = 0;
                    dataAcq[1].integral_approx = 0;
                    // copy data to array to be sent via wifi, and sum aquired data for approx. integral
                    for(int nn=0;nn<idx_data;nn++){
                        dataAcq[0].valid_data[nn] = dataAcq[0].data[nn];
                        dataAcq[1].valid_data[nn] = dataAcq[1].data[nn];
                        // sum for approx integral
                        dataAcq[0].integral_approx += dataAcq[0].valid_data[nn];
                        dataAcq[1].integral_approx += dataAcq[1].valid_data[nn];
                    }
                    // scale approx integral (sum of elements * Ts)
                    dataAcq[0].integral_approx *= (pauseTime / 1000.0);
                    dataAcq[1].integral_approx *= (pauseTime / 1000.0);
                    //
                    // tell mean_control that new valid data has been saved (one is valid, one is near zero because sensor inactive!)
                    // only set bit if data is in fact valid!
                    if(dataAcq[0].integral_approx >= approxIntegralThreshhold){
                        xEventGroupSetBits(hdrive_event_group_array[0],BIT_VALID_MEASUREMENT);
                    };
                    if(dataAcq[1].integral_approx >= approxIntegralThreshhold){
                        xEventGroupSetBits(hdrive_event_group_array[1],BIT_VALID_MEASUREMENT);
                    };
                    //
                    //ESP_LOGE(TAG,"New Dataset Acquired\n");
                    //printf("samples[1]: %i\n", idx_data);
                    // first data Point:
                    idx_data = 0;
                    dataAcq[0].data[idx_data] = dataAcq[0].act_value_filtered;
                    dataAcq[1].data[idx_data] = dataAcq[1].act_value_filtered;
                    idx_data++;
                // if not next cycle, check if data is not full, i.e. if machine is in fact running at expected speed   
                // if ok, contnue to fill data array 
                }else if(idx_data<DATA_ACQ_MAX_SAMPLES_PER_CYCLE){
                    dataAcq[0].data[idx_data] = dataAcq[0].act_value_filtered;
                    dataAcq[1].data[idx_data] = dataAcq[1].act_value_filtered;
                    idx_data++;
                // if to much data, machine has stopped running (or is to slow!)
                // in this case, go back to idle
                }else{
                    // maybe reset idx, but for debugging lets leave it
                    printf("HallSignal Timeout. Going back to IDLE...\n");
                    // signal other tasks that dataAcq is idle
                    xEventGroupClearBits(dataAcq_event_group,BIT_DATA_ACQ_IS_ACTIVE);
                    xEventGroupSetBits(dataAcq_event_group,BIT_DATA_ACQ_IS_IDLE);
                    step_dataAcq = DA_IDLE;
                }
            break;
            //
            case DA_ERROR:
                // ...
            break;
            //
            default:
                // ...
            break;
        }

        /*
        // testing cycle time of data acquisition --> UNCOMMENT ALSO AT BEGINNING OF WHILE(1)!!!
        timer_get_counter_value(global_timer_group,global_timer_idx,&after_counter_value);
        difference_dataAcq = (int)(after_counter_value - before_counter_value);
        if(difference_idx>10){
            difference_idx = 0;
        }
        difference_dataAcq_array[difference_idx] = before_counter_value;
        difference_idx++;
        */

        // ==========================================================================================
        // print stuff every x cycles, i.e. every x ms
        if(cycleCount>=250){
            //printf("xTaskGetTickCount: %i\n",xTaskGetTickCount());
            //printf("Tension RAW: %.2f = Tension FILTERED: %.2f \n", dataAcq[0].act_value, dataAcq[0].act_value_filtered);

            // IB printfs ===============
            //printf("HallS: %i, BTSR1: %i, BTSR2: %i\n", gpio_get_level(GPIO_HALLS_INPUT),adc1_get_raw(ADC1_BTSR1_INPUT),adc1_get_raw(ADC1_BTSR2_INPUT));
            //printf("mean 1 %.2f. mean 2 %.2f\n", dataAcq[0].mean, dataAcq[1].mean);
            //printf("integral 1 %.2f. integral 2 %.2f\n", dataAcq[0].integral_approx, dataAcq[1].integral_approx);
            // ==========================
            
            //printf("samples[1]: %i - step: %i\n", dataAcq[0].idx,step_dataAcq);

            //printf("[data] -%i- idx: %i\n",difference_dataAcq,dataAcq[0].idx);
            
            //printf("ADC RAW: %i\n", adc1_get_raw(ADC1_BTSR1_INPUT));

            


            //printf("[0] aprx int: %.2f = [1] aprx int: %.2f\n",dataAcq[0].integral_approx,dataAcq[1].integral_approx);
            

            //printf("%llu\n",difference_dataAcq_array[0]);
            //printf("%llu\n",difference_dataAcq_array[1]);
            //printf("%llu\n",difference_dataAcq_array[2]);
            //printf("%llu\n",difference_dataAcq_array[3]);
            //printf("BLABLA\n");
            

            //printf("%i;%.2f;%.2f\n",adc1_get_raw(ADC1_BTSR1_INPUT),dataAcq[0].act_value,dataAcq[0].act_value_filtered);
            

            cycleCount = 0;
        }
        // ====================================================
        // delay until next cycle, increment cycle counter
        cycleCount++;
        vTaskDelayUntil(&previousWakeTime0,pauseTime);  
        //vTaskDelay(1);  
        // ====================================================
    }



}
