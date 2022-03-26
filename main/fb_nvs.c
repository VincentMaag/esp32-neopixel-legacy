


#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sdkconfig.h"

#include "fb_projdefs.h"
#include "fb_nvs.h"


static const char *TAG = "fb_nvs";


bool set_flash_uint32( uint32_t ip, const char *label ){
	nvs_handle my_handle;
	esp_err_t err = nvs_open(label, NVS_READWRITE, &my_handle);

	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error (%d) opening NVS handle!", err);
	} else {
		//write
		//ESP_LOGI(TAG,"Writing to flash: %s", label);
		err = nvs_set_u32(my_handle, label, ip);

		switch (err) {
		case ESP_OK:
			return true;
		default :
			ESP_LOGE(TAG,"Error (%d) writing to flash", err);
			break;
		}
	}
	return false;
}

bool set_flash_uint8( uint8_t dataToStore, const char *label ){
	nvs_handle my_handle;
	esp_err_t err = nvs_open(label, NVS_READWRITE, &my_handle);

	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error (%d) opening NVS handle!", err);
	} else {
		//write
		//ESP_LOGI(TAG,"Writing to flash: %s", label);
		err = nvs_set_u8(my_handle, label, dataToStore);

		switch (err) {
		case ESP_OK:
			return true;
		default :
			ESP_LOGE(TAG,"Error (%d) writing to flash", err);
			break;
		}
	}
	return false;
}

bool get_flash_uint32( uint32_t *ip, const char *label ){
	nvs_handle my_handle;
	esp_err_t err = nvs_open(label, NVS_READWRITE, &my_handle);

	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error (%d) opening NVS handle!", err);
	} else {
		err = nvs_get_u32(my_handle, label, ip);

		switch (err) {
		case ESP_OK:
			return true;
		case ESP_ERR_NVS_NOT_FOUND:
			//ESP_LOGI(TAG,"The value \"%s\" is not initialized yet", label);
			break;
		default :
			ESP_LOGE(TAG,"Error (%d) reading\n", err);
			break;
		}
	}
	return false;
	
}

bool get_flash_uint8( uint8_t *pDataToRead, const char *label ){
	nvs_handle my_handle;
	esp_err_t err = nvs_open(label, NVS_READWRITE, &my_handle);

	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error (%d) opening NVS handle!", err);
	} else {
		err = nvs_get_u32(my_handle, label, pDataToRead);

		switch (err) {
		case ESP_OK:
			return true;
		case ESP_ERR_NVS_NOT_FOUND:
		//ESP_LOGI(TAG,"The value \"%s\" is not initialized yet", label);
			break;
		default :
			ESP_LOGE(TAG,"Error (%d) reading\n", err);
			break;
		}
	}
	return false;
	
}

void init_nvs_data(){
	// load nvs data into nvs-object:
	// ================================================================
	// Control Mode
	get_flash_uint32(&nvs_data[0].nvs_control_mode,"mode0");
	get_flash_uint32(&nvs_data[1].nvs_control_mode,"mode1");
	// copy to relevant objects:
	current_control_mode[0] = (uint8_t)(nvs_data[0].nvs_control_mode);
	current_control_mode[1] = (uint8_t)(nvs_data[1].nvs_control_mode);
	// check boundaries. If something ist wrong, choose mean mode
	if(current_control_mode[0]>1){
		current_control_mode[0] = CONTROL_MODE_MEAN;
	}
	if(current_control_mode[1]>1){
		current_control_mode[1] = CONTROL_MODE_MEAN;
	}
	// ================================================================
	// Setpoint Auto
	get_flash_uint32(&nvs_data[0].nvs_Setpoint_Auto,"auto0");
	get_flash_uint32(&nvs_data[1].nvs_Setpoint_Auto,"auto1");
	// copy
	wifi_fb[0].setpoint_mean_control = (float)nvs_data[0].nvs_Setpoint_Auto;
	wifi_fb[1].setpoint_mean_control = (float)nvs_data[1].nvs_Setpoint_Auto;
	// check boundaries
	if(wifi_fb[0].setpoint_mean_control>50){
		wifi_fb[0].setpoint_mean_control = 50;
	}
	if(wifi_fb[1].setpoint_mean_control>50){
		wifi_fb[1].setpoint_mean_control = 50;
	}
	// ================================================================
	// Setpoint Manual
	get_flash_uint32(&nvs_data[0].nvs_Setpoint_Manual,"manual0");
	get_flash_uint32(&nvs_data[1].nvs_Setpoint_Manual,"manual1");
	// copy
	wifi_fb[0].setpoint_hdrive = (float)nvs_data[0].nvs_Setpoint_Manual;
	wifi_fb[1].setpoint_hdrive = (float)nvs_data[1].nvs_Setpoint_Manual;
	// check boundaries
	if(wifi_fb[0].setpoint_hdrive>100){
		wifi_fb[0].setpoint_hdrive = 100;
	}
	if(wifi_fb[1].setpoint_hdrive>100){
		wifi_fb[1].setpoint_hdrive = 100;
	}
	//
	ESP_LOGI(TAG,"Control Modes -- 0: %d -- 1: %d",current_control_mode[0],current_control_mode[1]);
	ESP_LOGI(TAG,"Auto Setpoints -- 0: %.2f -- 1: %.2f",wifi_fb[0].setpoint_mean_control,wifi_fb[1].setpoint_mean_control);
	ESP_LOGI(TAG,"Manual Setpoints -- 0: %.2f -- 1: %.2f",wifi_fb[0].setpoint_hdrive,wifi_fb[1].setpoint_hdrive);
}

void nvs_task(void* arg){

	// variables:
	uint8_t current_control_mode_0_old = current_control_mode[0];
	uint8_t current_control_mode_1_old = current_control_mode[1];
	float setpoint_mean_control_0_old = wifi_fb[0].setpoint_mean_control;
	float setpoint_mean_control_1_old = wifi_fb[1].setpoint_mean_control;
	float setpoint_hdrive_0_old = wifi_fb[0].setpoint_hdrive;
	float setpoint_hdrive_1_old = wifi_fb[1].setpoint_hdrive;
	
	while(1){

		if(current_control_mode_0_old != current_control_mode[0]){
			nvs_data[0].nvs_control_mode = (uint32_t)current_control_mode[0];
			set_flash_uint32(nvs_data[0].nvs_control_mode,"mode0");
		}
		if(current_control_mode_1_old != current_control_mode[1]){
			nvs_data[1].nvs_control_mode = (uint32_t)current_control_mode[1];
			set_flash_uint32(nvs_data[1].nvs_control_mode,"mode1");
		}
		if(setpoint_mean_control_0_old != wifi_fb[0].setpoint_mean_control){
			nvs_data[0].nvs_Setpoint_Auto = (uint32_t)wifi_fb[0].setpoint_mean_control;
			set_flash_uint32(nvs_data[0].nvs_Setpoint_Auto,"auto0");
		}
		if(setpoint_mean_control_1_old != wifi_fb[1].setpoint_mean_control){
			nvs_data[1].nvs_Setpoint_Auto = (uint32_t)wifi_fb[1].setpoint_mean_control;
			set_flash_uint32(nvs_data[1].nvs_Setpoint_Auto,"auto1");
		}
		if(setpoint_hdrive_0_old != wifi_fb[0].setpoint_hdrive){
			nvs_data[0].nvs_Setpoint_Manual = (uint32_t)wifi_fb[0].setpoint_hdrive;
			set_flash_uint32(nvs_data[0].nvs_Setpoint_Manual,"manual0");
		}
		if(setpoint_hdrive_1_old != wifi_fb[1].setpoint_hdrive){
			nvs_data[1].nvs_Setpoint_Manual = (uint32_t)wifi_fb[1].setpoint_hdrive;
			set_flash_uint32(nvs_data[1].nvs_Setpoint_Manual,"manual1");
		}
		// timeshift data
		current_control_mode_0_old = current_control_mode[0];
		current_control_mode_1_old = current_control_mode[1];
		setpoint_mean_control_0_old = wifi_fb[0].setpoint_mean_control;
		setpoint_mean_control_1_old = wifi_fb[1].setpoint_mean_control;
		setpoint_hdrive_0_old = wifi_fb[0].setpoint_hdrive;
		setpoint_hdrive_1_old = wifi_fb[1].setpoint_hdrive;
		// timing not essential. check to store every 2 seconds
		vTaskDelay((2000/portTICK_PERIOD_MS)); 
	}
}
