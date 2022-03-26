
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

#include "fb_projdefs.h" // global stuff
#include "neopixel.h" 

#include "ws2812_control.h" // led functions. go define number of leds there!

// Define colors, GRB, NOT RGB!
#define RED   0x00FF00
#define GREEN 0xFF0000
#define BLUE  0x0000FF
#define PINK  0x14FF93
// define states for state machine
typedef enum{
    LED_SINGLE_COLOR,
    LED_BLINK_LINE,
    LED_BLEND
} led_step_enum_t;
led_step_enum_t iLEDStep = LED_SINGLE_COLOR;





void HSVtoRGB( float *r, float *g, float *b, float h, float s, float v ){
    int i;
    float f, p, q, t;
    if( s == 0 ) {
        // achromatic (grey)
        *r = *g = *b = v;
        return;
    }
    h /= 60;            // sector 0 to 5
    i = floor( h );
    f = h - i;          // factorial part of h
    p = v * ( 1 - s );
    q = v * ( 1 - s * f );
    t = v * ( 1 - s * ( 1 - f ) );
    switch( i ) {
        case 0:
            *r = v;
            *g = t;
            *b = p;
            break;
        case 1:
            *r = q;
            *g = v;
            *b = p;
            break;
        case 2:
            *r = p;
            *g = v;
            *b = t;
            break;
        case 3:
            *r = p;
            *g = q;
            *b = v;
            break;
        case 4:
            *r = t;
            *g = p;
            *b = v;
            break;
        default:        // case 5:
            *r = v;
            *g = p;
            *b = q;
            break;
    }
}







// task is created in main.c
void neopixel_task(void* arg){
    // init library
    ws2812_control_init();

    // init global variables
    led_grb.setpoint_grb = BLUE;

    // initialize leds
    // create led array (type defined in WS2812_control_h, with number of leds defined in fb_projdefs.h)
    struct led_state new_state;
    int i;
    int icolors[3] = {PINK, GREEN, BLUE};
    for (i=2; i<=NUM_LEDS; i=i+3){
        new_state.leds[i-2] = icolors[0];
        new_state.leds[i-1] = icolors[1];
        new_state.leds[i] = icolors[2];
    }

    // init LEDS
    new_state.leds[0] = PINK;
    new_state.leds[1] = GREEN;
    new_state.leds[2] = BLUE;
    ws2812_write_leds(new_state);

    int counter = 0;

    int blendColor;


    float h = 0;
    float s = 0.8;
    float v = 0.8;
    float r = 0;
    float g = 0;
    float b = 0;

    int r_blend = 0;
    int g_blend = 0;
    int b_blend = 0;

    int rgb_blend = 0;
    int grb_blend = 0;
    
    
    
    // cyclic loop
    while(1){

        // state machine
        switch (iLEDStep){

            case LED_SINGLE_COLOR:


                // fill array with single color
                for (i=2; i<=NUM_LEDS; i=i+3){
                new_state.leds[i-2] = led_grb.setpoint_grb;
                new_state.leds[i-1] = led_grb.setpoint_grb;
                new_state.leds[i]   = led_grb.setpoint_grb;
                }
                //

                // wait for wifi to request change of mode
                if((xEventGroupGetBits(led_event_group) & BIT_LED_CHANGE_TO_BLINK_LINE) == BIT_LED_CHANGE_TO_BLINK_LINE){
                    xEventGroupClearBits(led_event_group,BIT_LED_CHANGE_TO_BLINK_LINE);
                    xEventGroupClearBits(led_event_group,BIT_LED_CHANGE_TO_SINGLE);
                    iLEDStep = LED_BLINK_LINE;
                }else if((xEventGroupGetBits(led_event_group) & BIT_LED_CHANGE_TO_BLEND) == BIT_LED_CHANGE_TO_BLEND){
                    xEventGroupClearBits(led_event_group,BIT_LED_CHANGE_TO_BLEND);
                    xEventGroupClearBits(led_event_group,BIT_LED_CHANGE_TO_SINGLE);
                    iLEDStep = LED_BLEND;
                }
                



            break;
        
            case LED_BLINK_LINE:

                if(counter == 0){
                icolors[0] = PINK;
                icolors[1] = GREEN;
                icolors[2] = BLUE;
                }else if(counter == 1){
                icolors[0] = BLUE;
                icolors[1] = PINK;
                icolors[2] = GREEN;
                }else if(counter == 2){
                icolors[0] = GREEN;
                icolors[1] = BLUE;
                icolors[2] = PINK;
                }
                counter = counter + 1;
                if(counter > 2){
                counter = 0;
                }
                for (i=2; i<=NUM_LEDS; i=i+3){
                new_state.leds[i-2] = icolors[0];
                new_state.leds[i-1] = icolors[1];
                new_state.leds[i] = icolors[2];
                }

                // wait for wifi to request change of mode
                if((xEventGroupGetBits(led_event_group) & BIT_LED_CHANGE_TO_SINGLE) == BIT_LED_CHANGE_TO_SINGLE){
                    xEventGroupClearBits(led_event_group,BIT_LED_CHANGE_TO_SINGLE);
                    xEventGroupClearBits(led_event_group,BIT_LED_CHANGE_TO_BLINK_LINE);
                    iLEDStep = LED_SINGLE_COLOR;
                }else if((xEventGroupGetBits(led_event_group) & BIT_LED_CHANGE_TO_BLEND) == BIT_LED_CHANGE_TO_BLEND){
                    xEventGroupClearBits(led_event_group,BIT_LED_CHANGE_TO_BLEND);
                    xEventGroupClearBits(led_event_group,BIT_LED_CHANGE_TO_BLINK_LINE);
                    iLEDStep = LED_BLEND;
                }

            break;

            case LED_BLEND:


                HSVtoRGB(&r,&g,&b,h,s,v);
                if(h<359){
                    h = h+1;
                }else{
                    h = 0;
                }

                r_blend = (int)floor(255.0*r);
                g_blend = (int)floor(255.0*g);
                b_blend = (int)floor(255.0*b);

                rgb_blend = (r_blend << 16 | g_blend << 8 | b_blend);
                grb_blend = (g_blend << 16 | r_blend << 8 | b_blend);
                
                //printf("r=%.2f, g=%.2f, b=%.2f, \n",r,g,b);
                //printf("r=%i, g=%i, b=%i, \n",r_blend,g_blend,b_blend);
                //printf("rgb=%x, grb=%x , h = %.2f\n",rgb_blend,grb_blend,h);

                // fill array with single color
                for (i=2; i<=NUM_LEDS; i=i+3){
                new_state.leds[i-2] = grb_blend;
                new_state.leds[i-1] = grb_blend;
                new_state.leds[i]   = grb_blend;
                }
                //

                // wait for wifi to request change of mode
                if((xEventGroupGetBits(led_event_group) & BIT_LED_CHANGE_TO_SINGLE) == BIT_LED_CHANGE_TO_SINGLE){
                    xEventGroupClearBits(led_event_group,BIT_LED_CHANGE_TO_SINGLE);
                    xEventGroupClearBits(led_event_group,BIT_LED_CHANGE_TO_BLEND);
                    iLEDStep = LED_SINGLE_COLOR;
                }else if((xEventGroupGetBits(led_event_group) & BIT_LED_CHANGE_TO_BLINK_LINE) == BIT_LED_CHANGE_TO_BLINK_LINE){
                    xEventGroupClearBits(led_event_group,BIT_LED_CHANGE_TO_BLINK_LINE);
                    xEventGroupClearBits(led_event_group,BIT_LED_CHANGE_TO_BLEND);
                    iLEDStep = LED_BLINK_LINE;
                }


            break;

            default:

            break;
        }

        

        // ============================================
        // write led states
        ws2812_write_leds(new_state);
    	// timing, every x ms
		vTaskDelay((100/portTICK_PERIOD_MS));
        //vTaskDelay(1000000);
        // ============================================
    }
 
}