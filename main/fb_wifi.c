/*
    ...

*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "lwip/ip4_addr.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "fb_wifi.h"
#include "fb_projdefs.h"

//#include "index.h"
#include "http_parser.h"

#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_MAX_STA_CONN CONFIG_MAX_STA_CONN

static const char *TAG = "fb_wifi";

static QueueHandle_t client_queue;
const static int client_queue_size = 10;

bool bGlobalConnectionStatus = false;

// =====================================================================
// stuff from wifi_softAP...
//
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
// don't really know what these do here
// think this one should actually be in main.c, however
// wifi_init_softap needs this function, so not the wiser yet...
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id)
	{
	case SYSTEM_EVENT_AP_STACONNECTED:
		ESP_LOGI(TAG, "station:" MACSTR " join, AID=%d",
				 MAC2STR(event->event_info.sta_connected.mac),
				 event->event_info.sta_connected.aid);
		break;
	case SYSTEM_EVENT_AP_STADISCONNECTED:
		ESP_LOGI(TAG, "station:" MACSTR "leave, AID=%d",
				 MAC2STR(event->event_info.sta_disconnected.mac),
				 event->event_info.sta_disconnected.aid);
		break;
	case SYSTEM_EVENT_STA_CONNECTED:
		ESP_LOGI(TAG, "STA CONNECTED");
		bGlobalConnectionStatus = true;
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		ESP_LOGI(TAG, "STA DISCONNECTED");
		bGlobalConnectionStatus = false;
		break;
	default:
		break;
	}
	return ESP_OK;
}
// =====================================================================
// function to connect to STA
void wifi_try_connect_sta()
{

	for (int i = 0; i < 20; i++)
	{
		ESP_LOGE(TAG, "try to disconnect wifi connection");
		ESP_ERROR_CHECK(esp_wifi_disconnect());
		ESP_LOGE(TAG, "disconnecting...");
		vTaskDelay((1000 / portTICK_PERIOD_MS));
		ESP_LOGE(TAG, "trying to connect to router");
		ESP_ERROR_CHECK(esp_wifi_connect());
		ESP_LOGE(TAG, "connecting... ");
		vTaskDelay((5000 / portTICK_PERIOD_MS));
		//vTaskDelay((5000));
		
		if (bGlobalConnectionStatus)
		{
			ESP_LOGE(TAG, "event handler signaling successful connection");
			break;
		}
		else if (i == 19)
		{
			ESP_LOGE(TAG, "timout connection to router! 20 unsuccessful connection attempts");
		}
		else
		{
			ESP_LOGE(TAG, "retry");
			;
		}
	}
}
// =====================================================================
// function to initialize wifi
void wifi_init()
{
	s_wifi_event_group = xEventGroupCreate(); // not sure where this one wants to get used...
	// init global connection status with false
	bGlobalConnectionStatus = false;
	// maximize wifi range
	esp_wifi_set_max_tx_power((uint8_t)60);
	// initialize tcpip adapter
	tcpip_adapter_init();
	// stop client
	ESP_ERROR_CHECK(tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA));
	// initialize event handling
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	// configure default
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	// define ip adress
	tcpip_adapter_ip_info_t IpInfo;
	IpInfo.ip.addr = ipaddr_addr("192.168.178.100");
	IpInfo.gw.addr = ipaddr_addr("192.168.178.1");
	IpInfo.netmask.addr = ipaddr_addr("255.255.255.0");
	// set ip idress
	ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &IpInfo));
	// configure sta mode
	wifi_config_t sta_config = {
		.sta = {
			.ssid = "FRITZ!Box 5490 WT",
			.password = "55940362741817360715",
			.bssid_set = false,
		}};

	// if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0)
	// {
	// 	sta_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
	// 	}
	// set sta mode & sta conifig
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_config));
	// start wifi
	ESP_ERROR_CHECK(esp_wifi_start());
	// connect wifi
	//wifi_try_connect_sta();

}
// =============================================================================================================
// function to handle webserver
static void http_serve(struct netconn *conn)
{
	const static char *TAG2 = "http_server";
	const static char HTML_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/html\n\n";
	//const static char ERROR_HEADER[] = "HTTP/1.1 404 Not Found\nContent-type: text/html\n\n";
	const static char JS_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/javascript\n\n";
	const static char CSS_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/css\n\n";
	//const static char PNG_HEADER[] = "HTTP/1.1 200 OK\nContent-type: image/png\n\n";
	//const static char ICO_HEADER[] = "HTTP/1.1 200 OK\nContent-type: image/x-icon\n\n";
	//const static char PDF_HEADER[] = "HTTP/1.1 200 OK\nContent-type: application/pdf\n\n";
	//const static char EVENT_HEADER[] = "HTTP/1.1 200 OK\nContent-Type: text/event-stream\nCache-Control: no-cache\nretry: 3000\n\n";
	struct netbuf *inbuf;
	static char *buf;
	static uint16_t buflen;
	static err_t err;

	char outbuf[1000];

	// default page
	extern const uint8_t root_html_start[] asm("_binary_index_html_start");
	extern const uint8_t root_html_end[] asm("_binary_index_html_end");
	const uint32_t root_html_len = root_html_end - root_html_start;

	// index.js
	extern const uint8_t test_js_start[] asm("_binary_index_js_start");
	extern const uint8_t test_js_end[] asm("_binary_index_js_end");
	const uint32_t test_js_len = test_js_end - test_js_start;

	// Chart.js
	extern const uint8_t Chart_js_start[] asm("_binary_Chart_js_start");
	extern const uint8_t Chart_js_end[] asm("_binary_Chart_js_end");
	const uint32_t Chart_js_len = Chart_js_end - Chart_js_start;

	// index.css
	extern const uint8_t index_css_start[] asm("_binary_index_css_start");
	extern const uint8_t index_css_end[] asm("_binary_index_css_end");
	const uint32_t index_css_len = index_css_end - index_css_start;

	netconn_set_recvtimeout(conn, 1000); // allow a connection timeout of 1 second
	err = netconn_recv(conn, &inbuf);

	if (err == ERR_OK)
	{
		netbuf_data(inbuf, (void **)&buf, &buflen);
		if (buf)
		{

			// default page
			if (strstr(buf, "GET / ") && !strstr(buf, "Upgrade: websocket"))
			{
				ESP_LOGI(TAG2, "Sending /");
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				netconn_write(conn, root_html_start, root_html_len, NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// default page websocket
			else if (strstr(buf, "GET / ") && strstr(buf, "Upgrade: websocket"))
			{
				ESP_LOGI(TAG2, "Requesting websocket on /");
				ws_server_add_client(conn, buf, buflen, "/", websocket_callback);
				netbuf_delete(inbuf);
			}
			//
			else if (strstr(buf, "GET /index.js "))
			{
				ESP_LOGI(TAG2, "Sending /index.js");
				netconn_write(conn, JS_HEADER, sizeof(JS_HEADER) - 1, NETCONN_NOCOPY);
				netconn_write(conn, test_js_start, test_js_len, NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			else if (strstr(buf, "GET /Chart.js "))
			{
				ESP_LOGI(TAG2, "Sending /Chart.js");
				netconn_write(conn, JS_HEADER, sizeof(JS_HEADER) - 1, NETCONN_NOCOPY);
				netconn_write(conn, Chart_js_start, Chart_js_len, NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
				ESP_LOGI(TAG2, "Sent Chart.js and closed connection");
			}
			else if (strstr(buf, "GET /index.css "))
			{
				ESP_LOGI(TAG2, "Sending /index.css");
				netconn_write(conn, CSS_HEADER, sizeof(CSS_HEADER) - 1, NETCONN_NOCOPY);
				netconn_write(conn, index_css_start, index_css_len, NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// testing stuff
			else if (strstr(buf, "GET /test"))
			{
				ESP_LOGE(TAG2, "Requesting CONSOLE TEST! /");
				//
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// hdrives disable
			else if (strstr(buf, "POST /disableHdrives"))
			{
				ESP_LOGE(TAG2, "Requesting to disable hdrives /");
				// tell via event that hdrive must be disabled
				xEventGroupSetBits(wifi_event_group, BIT_REQ_DISABLE_HDRIVE);
				//
				led_grb.setpoint_grb = 0x14FF93;
				//
				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// hdrives re enable
			else if (strstr(buf, "POST /enableHdrives"))
			{
				ESP_LOGE(TAG2, "Requesting to enable hdrives /");
				// tell via event that hdrive must be enabled
				xEventGroupSetBits(wifi_event_group, BIT_REQ_ENABLE_HDRIVE);
				//
				led_grb.setpoint_grb = 0x0000FF;
				//
				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// adc 0
			else if (strstr(buf, "GET /getData"))
			{
				//ESP_LOGE(TAG2,"Requesting value of btsr sensors \n");
				// create ints out of floats to send single bytes (1 byte <= 255 float value)

				// sent data = [1b control mode] + [2b mean value 1] + [2b mean value 2] + [2b %-setpoint1] + [2b %-setpoint2]

				// control mode of drive 1 (i.e. both drives)
				//int idata00 = 0;
				//if((xEventGroupGetBits(hdrive_event_group_array[0]) & BIT_CONTROL_MODE_MANUAL) == BIT_CONTROL_MODE_MANUAL){
				//	idata00 = 0;
				//}else if((xEventGroupGetBits(hdrive_event_group_array[0]) & BIT_CONTROL_MODE_MEAN) == BIT_CONTROL_MODE_MEAN){
				//	idata00 = 1;
				//}

				uint8_t idata00 = current_control_mode[0];

				//ESP_LOGE(TAG2,"bit0 sent: %i\n",idata00);
				//
				int idata0 = (int)(100 * mean_control[0].display_mean);
				int idata1 = (int)(100 * mean_control[1].display_mean);
				int idata2 = (int)(hdrive[0].currentSetpointClosedPercent);
				int idata3 = (int)(hdrive[1].currentSetpointClosedPercent);
				// information for hmi-init
				int idata4 = (int)(wifi_fb[0].setpoint_mean_control);
				int idata5 = (int)(wifi_fb[1].setpoint_mean_control);
				int idata6 = (int)(wifi_fb[0].setpoint_hdrive);
				int idata7 = (int)(wifi_fb[1].setpoint_hdrive);

				// create buffer
				char outbuf_mean[1000];
				// 0&1 byte = control mode (second byte not used yet)
				outbuf_mean[0] = ((char *)&idata00)[0];
				// 2&3 byte = mean 1
				outbuf_mean[2] = ((char *)&idata0)[0];
				outbuf_mean[3] = ((char *)&idata0)[1];
				// 4&5 byte = mean 2
				outbuf_mean[4] = ((char *)&idata1)[0];
				outbuf_mean[5] = ((char *)&idata1)[1];
				// 6 byte = %-setpoint 1, i.e actual closed position
				outbuf_mean[6] = ((char *)&idata2)[0];
				// 7 byte = %-setpoint 2
				outbuf_mean[7] = ((char *)&idata3)[0];
				// 8 byte = auto setpoint 1
				outbuf_mean[8] = ((char *)&idata4)[0];
				// 9 byte = auto setpoint 2
				outbuf_mean[9] = ((char *)&idata5)[0];
				// 8 byte = manual setpoint 1
				outbuf_mean[10] = ((char *)&idata6)[0];
				// 9 byte = manual setpoint 2
				outbuf_mean[11] = ((char *)&idata7)[0];

				netconn_write(conn, outbuf_mean, sizeof(outbuf_mean), NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);

				// err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
			}
			// =========================================================================
			// =========================================================================
			// adc 0
			else if (strstr(buf, "GET /getArray"))
			{
				//ESP_LOGE(TAG2,"Requesting array \n");
				int i;
				int idata;
				int idata2;
				// create buffer, size of both sensor indecis
				char outbuf_btsr[dataAcq[0].idx + dataAcq[0].idx];
				// fill buffer with Sensor 1 data
				for (i = 0; i < dataAcq[1].idx; i++)
				{
					// convert float datapoint to int, because max 50cN, we stay under 255 (1 byte),
					// this means we use only first byte of every idata (int) value.
					//
					// convert one float value to int, 2 bytes, <255
					idata = (int)dataAcq[0].valid_data[i];
					// convert to char data and copy first byte into buffer (only first byte because value <255)
					outbuf_btsr[i] = ((char *)&idata)[0];
					// do the same for sensor 2 data
					idata2 = (int)dataAcq[1].valid_data[i];
					outbuf_btsr[i + dataAcq[1].idx] = ((char *)&idata2)[0];
				}
				netconn_write(conn, outbuf_btsr, sizeof(outbuf_btsr), NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);

				// err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
			}
			// =========================================================================
			// Setpoint closed hdrive 1
			else if (strstr(buf, "POST /sendhdrive1setpoint"))
			{
				ESP_LOGE(TAG2, "Trying to send some data \n");

				// define how long the request string is
				int length = (int)sizeof("POST /sendhdrive1setpoint");
				// fill int value according to where we know the first char number starts
				// extracted data: first 3 bytes is Manual setpoint [%], next two is Auto setpoint [cN]
				int extractedData = ((buf[length - 1] - '0') * 100 + (buf[length] - '0') * 10 + (buf[length + 1] - '0'));
				// second number
				int extractedData2 = ((buf[length + 2] - '0') * 10 + (buf[length + 3] - '0'));
				// Log
				ESP_LOGE(TAG2, "Requesting repositioning of hdrive 0: %i\n", extractedData);
				//printf("String: %s\nlength: %i\nbuf: %c\nbufExtract: %i\n",buf,length,buf[length-1],test);
				//printf("%s\n",buf);
				printf("first number: %i second number: %i\n", extractedData, extractedData2);
				//
				// copy setoint and signal event
				wifi_fb[0].setpoint_hdrive = (float)extractedData;
				wifi_fb[0].setpoint_mean_control = (float)extractedData2;
				xEventGroupSetBits(hdrive_event_group_array[0], BIT_CHANGE_SETPOINT_HDRIVE);

				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);

				// err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
			}
			// =========================================================================
			// Setpoint closed hdrive 2
			else if (strstr(buf, "POST /sendhdrive2setpoint"))
			{
				ESP_LOGE(TAG2, "Trying to send some data \n");

				// define how long the request string is
				int length = (int)sizeof("POST /sendhdrive2setpoint");
				// fill int value according to where we know the first char number starts
				// extracted data: first 3 bytes is Manual setpoint [%], next two is Auto setpoint [cN]
				int extractedData = ((buf[length - 1] - '0') * 100 + (buf[length] - '0') * 10 + (buf[length + 1] - '0'));
				// second number
				int extractedData2 = ((buf[length + 2] - '0') * 10 + (buf[length + 3] - '0'));
				// Log
				ESP_LOGE(TAG2, "Requesting repositioning of hdrive 1: %i\n", extractedData);
				//printf("String: %s\nlength: %i\nbuf: %c\nbufExtract: %i\n",buf,length,buf[length-1],test);
				//
				// copy setoint and signal event
				wifi_fb[1].setpoint_hdrive = (float)extractedData;
				wifi_fb[1].setpoint_mean_control = (float)extractedData2;
				xEventGroupSetBits(hdrive_event_group_array[1], BIT_CHANGE_SETPOINT_HDRIVE);

				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);

				// err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
			}
			// =========================================================================
			// hdrives request change to Manual Mode
			else if (strstr(buf, "POST /controlMode0"))
			{
				ESP_LOGE(TAG2, "Requesting to change control Mode to 0\n");
				// tell via event that control Mode shpuld change
				xEventGroupSetBits(wifi_event_group, BIT_REQ_CHANGE_TO_MODE_0);
				//
				xEventGroupSetBits(led_event_group, BIT_LED_CHANGE_TO_SINGLE);
				//
				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// hdrives request change to Manual Mode
			else if (strstr(buf, "POST /controlMode1"))
			{
				ESP_LOGE(TAG2, "Requesting to change control Mode to 1\n");
				// tell via event that control Mode shpuld change
				xEventGroupSetBits(wifi_event_group, BIT_REQ_CHANGE_TO_MODE_1);
				//
				xEventGroupSetBits(led_event_group, BIT_LED_CHANGE_TO_BLINK_LINE);
				//
				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// adc 0
			else if (strstr(buf, "GET /iosTest"))
			{
				ESP_LOGE(TAG2, "Requesting ios Test data \n");

				int i;
				int idata;
				int idata2;

				// testdata
				//int test_idx = 254;
				//float test_array[254];
				//for(i=0;i<test_idx;i++){
				//	test_array[i] = i;
				//}

				// fill buffer with Sensor 1 data
				for (i = 0; i < dataAcq[1].idx; i++)
				{
					// convert float datapoint to int, because max 50cN, we stay under 255 (1 byte),
					// this means we use only first byte of every idata (int) value.
					//
					// convert one float value to int, 2 bytes, <255
					idata = (int)dataAcq[0].valid_data[i];
					// convert to char data and copy first byte into buffer
					outbuf[i] = ((char *)&idata)[0];
					// do the same for sensor 2 data
					idata2 = (int)dataAcq[1].valid_data[i];
					outbuf[i + dataAcq[1].idx] = ((char *)&idata2)[0];
				}

				// fill buffer with Sensor 2 data
				//for(i=dataAcq[0].idx;i<(2*dataAcq[0].idx);i++){
				//	idata = (int)dataAcq[0].valid_data[i];
				//	outbuf[i] = ((char*)&idata)[0];
				//}

				char outbuf_ios[1000];
				outbuf_ios[0] = "0";
				//outbuf_ios[1] = "0";

				netconn_write(conn, outbuf_ios, sizeof(outbuf_ios), NETCONN_NOCOPY);
				//netconn_write(conn, outbuf, 4, NETCONN_NOCOPY);

				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);

				// err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
			}
			// =========================================================================
			// change led modes
			else if (strstr(buf, "POST /changeToSingleColor"))
			{
				ESP_LOGE(TAG2, "Requesting to changeToSingleColor\n");
				//
				xEventGroupSetBits(led_event_group, BIT_LED_CHANGE_TO_SINGLE);
				//
				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// change led modes
			else if (strstr(buf, "POST /changeToBlinkLine"))
			{
				ESP_LOGE(TAG2, "Requesting to changeToBlinkLine\n");
				//
				xEventGroupSetBits(led_event_group, BIT_LED_CHANGE_TO_BLINK_LINE);
				//
				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// change led modes
			else if (strstr(buf, "POST /changeToBlend"))
			{
				ESP_LOGE(TAG2, "Requesting to changeToBlend\n");
				//
				xEventGroupSetBits(led_event_group, BIT_LED_CHANGE_TO_BLEND);
				//
				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// Setpoint closed hdrive 2
			else if (strstr(buf, "POST /sendGRB"))
			{
				ESP_LOGE(TAG2, "Trying to send grb data\n");

				// define how long the request string is
				int length = (int)sizeof("POST /sendGRB");

				char grb[6] = "00ff00";

				grb[0] = buf[length - 1];
				grb[1] = buf[length];
				grb[2] = buf[length + 1];
				grb[3] = buf[length + 2];
				grb[4] = buf[length + 3];
				grb[5] = buf[length + 4];

				printf("grb value extracted: = %s = \n", grb);

				int number = (int)strtol(grb, NULL, 16);

				printf("grb value as integer: = %i = \n", number);
				printf("grb value as integer (hex lower case): = %x = \n", number);
				printf("grb value as integer (hex upper case): = %x = \n", number);

				led_grb.setpoint_grb = number;

				// // fill int value according to where we know the first char number starts
				// // extracted data: first 3 bytes is Manual setpoint [%], next two is Auto setpoint [cN]
				// int extractedData = ((buf[length-1] - '0')*100 +(buf[length] - '0')*10 + (buf[length+1] - '0'));
				// // second number
				// int extractedData2 = ((buf[length+2] - '0')*10 +(buf[length+3] - '0'));
				// // Log
				// ESP_LOGE(TAG2,"Requesting repositioning of hdrive 1: %i\n",extractedData);
				// //printf("String: %s\nlength: %i\nbuf: %c\nbufExtract: %i\n",buf,length,buf[length-1],test);
				// //
				// // copy setoint and signal event
				// wifi_fb[1].setpoint_hdrive = (float)extractedData;
				// wifi_fb[1].setpoint_mean_control = (float)extractedData2;
				// xEventGroupSetBits(hdrive_event_group_array[1],BIT_CHANGE_SETPOINT_HDRIVE);

				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);

				// err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
			}
			//
			else
			{
				ESP_LOGI(TAG2, "Unknown request");
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
		}
		else
		{
			ESP_LOGI(TAG2, "Unknown request (empty?...)");
			netconn_close(conn);
			netconn_delete(conn);
			netbuf_delete(inbuf);
		}
	}
	else
	{ // if err!=ERR_OK
		ESP_LOGI(TAG2, "error on read, closing connection");
		netconn_close(conn);
		netconn_delete(conn);
		netbuf_delete(inbuf);
	}
}
// =============================================================================================================
// handles clients when they first connect. passes to a queue
static void server_task(void *pvParameters)
{
	const static char *TAG2 = "server_task";
	struct netconn *conn, *newconn;
	static err_t err;
	client_queue = xQueueCreate(client_queue_size, sizeof(struct netconn *));

	conn = netconn_new(NETCONN_TCP);
	netconn_bind(conn, NULL, 80);
	netconn_listen(conn);
	ESP_LOGI(TAG2, "server listening");
	do
	{
		err = netconn_accept(conn, &newconn);
		//ESP_LOGI(TAG2,"new client");
		if (err == ERR_OK)
		{
			//ESP_LOGI(TAG2, "newconn->socket: %d", newconn->socket);
			xQueueSendToBack(client_queue, &newconn, portMAX_DELAY);
			//http_serve(newconn);
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	} while (err == ERR_OK);
	netconn_close(conn);
	netconn_delete(conn);
	ESP_LOGE(TAG2, "task ending, rebooting board");
	esp_restart();
}
// receives clients from queue, handles them
static void server_handle_task(void *pvParameters)
{
	const static char *TAG2 = "server_handle_task";
	struct netconn *conn;
	ESP_LOGI(TAG2, "task starting");
	for (;;)
	{
		xQueueReceive(client_queue, &conn, portMAX_DELAY);
		if (!conn)
			continue;
		http_serve(conn);
	}
	vTaskDelete(NULL);
}
// handles websocket events
void websocket_callback(uint8_t num, WEBSOCKET_TYPE_t type, char *msg, uint64_t len)
{
	const static char *TAG2 = "websocket_callback";
	//	int value;

	switch (type)
	{
	case WEBSOCKET_CONNECT:
		ESP_LOGI(TAG2, "client %i connected!", num);
		break;
	case WEBSOCKET_DISCONNECT_EXTERNAL:
		ESP_LOGI(TAG2, "client %i sent a disconnect message", num);
		//		led_duty(0);
		break;
	case WEBSOCKET_DISCONNECT_INTERNAL:
		ESP_LOGI(TAG2, "client %i was disconnected", num);
		break;
	case WEBSOCKET_DISCONNECT_ERROR:
		ESP_LOGI(TAG2, "client %i was disconnected due to an error", num);
		//		led_duty(0);
		break;
	case WEBSOCKET_TEXT:
		//		if(len) {
		//			switch(msg[0]) {
		//			case 'L':
		//				if(sscanf(msg,"L%i",&value)) {
		//					ESP_LOGI(TAG2,"LED value: %i",value);
		//					led_duty(value);
		//					ws_server_send_text_all_from_callback(msg,len); // broadcast it!
		//				}
		//			}
		//		}
		break;
	case WEBSOCKET_BIN:
		ESP_LOGI(TAG2, "client %i sent binary message of size %i:\n%s", num, (uint32_t)len, msg);
		break;
	case WEBSOCKET_PING:
		ESP_LOGI(TAG2, "client %i pinged us with message of size %i:\n%s", num, (uint32_t)len, msg);
		break;
	case WEBSOCKET_PONG:
		ESP_LOGI(TAG2, "client %i responded to the ping", num);
		break;
	}
}
// =====================================================================
void wifi_task(void *arg)
{
	// start wifi
	wifi_init();
	// start webserver
	ws_server_stop();
	ws_server_start();
	// create needed tasks
	xTaskCreatePinnedToCore(&server_task, "server_task", 3000, NULL, 5, NULL, 0);
	xTaskCreatePinnedToCore(&server_handle_task, "server_handle_task", 4000, NULL, 5, NULL, 0);
	// inf loop
	while (1)
		{
		// check if wifi disconnected:
		if (bGlobalConnectionStatus == false)
		{
			ESP_LOGE(TAG, "caught disconnection after successful connection");
			wifi_try_connect_sta();
		}

		
		//ESP_LOGE(TAG, "=== 1s? ===");
		//..
		vTaskDelay((1000 / portTICK_PERIOD_MS));
		//vTaskDelay(10000);
		
	}
}
