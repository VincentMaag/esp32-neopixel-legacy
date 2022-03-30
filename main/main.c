/*  WiFi softAP Example

   //This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "fb_projdefs.h"
#include "fb_wifi.h"
#include "fb_hdrive.h"
#include "fb_data_acq.h"
#include "fb_mean_control.h"
#include "fb_nvs.h"
#include "fb_blinker.h"

#include "neopixel.h"


static const char *TAG          = "main";
TaskHandle_t hdrive_task_handle = NULL;
TaskHandle_t dataAcq_task_handle = NULL;
TaskHandle_t mean_control_task_handle = NULL;
TaskHandle_t nvs_task_handle = NULL;
TaskHandle_t neopixel_task_handle = NULL;
TaskHandle_t wifi_task_handle = NULL;
TaskHandle_t blinker_task_handle = NULL;



void app_main()
{
    ESP_LOGI(TAG, "STARTING MAIN");
    // =====================================================================
    //Initialize NVS
    // vTaskDelay((1000/portTICK_PERIOD_MS));
    // ESP_LOGI(TAG, "1");
    esp_err_t ret = nvs_flash_init();
    // vTaskDelay((1000/portTICK_PERIOD_MS));
    // ESP_LOGI(TAG, "2");
    // vTaskDelay((1000/portTICK_PERIOD_MS));
    // ESP_LOGI(TAG, "3");
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    // =====================================================================
    // initialize wifi
    // wifi_init(); atm doing this in wifi.c
    // =====================================================================
    // init global variables
    //
    // dataAcq
    dataAcq_event_group    = xEventGroupCreate();
    for(int i = 0; i < DATA_ACQ_NUM_OF_CHANNELS; i++){
        dataAcq[i].idx = 0;
        dataAcq[i].act_value = 0;
        dataAcq[i].act_value_filtered = 0;
        dataAcq[i].mean   = 0;
        dataAcq[i].integral_approx = 0;
        for(int ii = 0; ii < DATA_ACQ_MAX_SAMPLES_PER_CYCLE; ii++){
            dataAcq[i].data[ii] = 0;
            dataAcq[i].valid_data[ii] = 0;
        }
        for(int nn = 0; nn < DATA_ACQ_NUM_OF_FILTER_COEFFICIANTS; nn++){
            dataAcq[i].act_value_filtered_old_array[nn] = 0;
        }
    }
    hdrive_event_group_array[0]  = xEventGroupCreate();
    hdrive_event_group_array[1]  = xEventGroupCreate();
    //
    // wifi
    for(int i = 0; i < DATA_ACQ_NUM_OF_CHANNELS; i++){
      wifi_fb[i].setpoint_hdrive = 51;
      wifi_fb[i].setpoint_mean_control = 16;
    }
    wifi_event_group = xEventGroupCreate();
    //
    // mean control
    for(int i = 0; i < DATA_ACQ_NUM_OF_CHANNELS; i++){
      mean_control[i].actual_single_mean_value = 15;
      mean_control[i].actual_mean_value = 15;
      mean_control[i].actual_mean_value_old = 15;
      mean_control[i].control_integral = 160;
      mean_control[i].mean_control_output = 160;
      mean_control[i].mean_setpoint = 15;
      mean_control[i].display_mean = 15;
      mean_control[i].display_mean_old = 15;
    }
    current_control_mode[0] = CONTROL_MODE_MANUAL;
    current_control_mode[1] = CONTROL_MODE_MANUAL;
    //
    // global timer

    // global_timer_group       = TIMER_GROUP_1;
    // global_timer_idx         = TIMER_1;
    // timer_config_t config;
    // config.divider = 16;
    // config.counter_dir = TIMER_COUNT_UP;
    // config.counter_en = TIMER_START;
    // config.alarm_en = TIMER_ALARM_DIS;
    // config.intr_type = TIMER_INTR_LEVEL;
    // config.auto_reload = false;
    // timer_init(global_timer_group, global_timer_idx, &config);
    // timer_start(global_timer_group,global_timer_idx);

    //
    // leds
    led_event_group = xEventGroupCreate();
    

    // =====================================================================
    // initialize nvs data
    //init_nvs_data();




    // =====================================================================

    // =====================================================================
    // create tasks

    // working combo: wifi 5, dataAcq 3, hdrive 2, mean 1

    xTaskCreatePinnedToCore(wifi_task, "wifi_task", 4096, (void *) 0, 5, &wifi_task_handle, 0);

    //xTaskCreatePinnedToCore(dataAcq_task, "dataAcq_task", 2048, (void *) 0, configMAX_PRIORITIES - 1, &dataAcq_task_handle, 1);

    //xTaskCreatePinnedToCore(hdrive_task, "hdrive_task", 4096, (void *) 0, 5, &hdrive_task_handle, 1);

    //xTaskCreatePinnedToCore(mean_control_task, "mean_control_task", 2048, (void *) 0, 2, &mean_control_task_handle, 1);

    //xTaskCreatePinnedToCore(nvs_task, "nvs_task", 2048, (void *) 0, 1, &nvs_task_handle, 1);

    xTaskCreatePinnedToCore(neopixel_task, "neopixel_task", 4096, (void *) 0, 1, &neopixel_task_handle, 1); //--> add .c components in CMakeList!

    xTaskCreatePinnedToCore(blinker_task, "blinker_task", 2048, (void *) 0, 1, &blinker_task_handle, 1); //--> add .c components in CMakeList!


    // try and write to flash, once
    //uint32_t uint32data_write = 51;
    //uint32_t uint32data_read  = 0;

    //ESP_LOGI(TAG, "Trying to write Flash");
    //get_flash_uint32(&uint32data_read,"testData");
    //printf("got data\n");
    //printf("data read: %d\n", uint32data_read);

    //if(uint32data_read < 5){
    //  set_flash_uint32(uint32data_write,"testData");
    //  printf("wrote data\n");
    //  printf("data write: %d\n", uint32data_write);
    //}

    

    //ESP_LOGI(TAG, "Flash written");
    //ESP_LOGI(TAG, "Trying to write Flash");

    

    //ESP_LOGI(TAG, "Flash read");

    //printf("data read: %ld\n", uint32data_read);
    //printf("data write: %ld\n", uint32data_write);



}
