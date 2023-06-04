#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <geometry_msgs/msg/twist.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_system.h"
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif
#include "lvgl_helpers.h"
#endif

#define ADC_CHANNEL_X1 0
#define ADC_CHANNEL_Y1 3
#define ADC_CHANNEL_X2 6
#define ADC_CHANNEL_Y2 7

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

static const adc_atten_t atten = ADC_ATTEN_DB_11;

static const adc_bits_width_t width = ADC_WIDTH_BIT_9;


//initialization
rcl_publisher_t publisher;

esp_err_t r1;
esp_err_t r2;
esp_err_t r3;
esp_err_t r4;


gpio_num_t adc_gpio_numX1;//channel 0 
gpio_num_t adc_gpio_numY1;//channel 3 
gpio_num_t adc_gpio_numX2;//channel 6 
gpio_num_t adc_gpio_numY2;//channel 7 


//value map function
float mapValue(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		//initialization of twist msg
		geometry_msgs__msg__Twist twist;
		geometry_msgs__msg__Twist__init(&twist);
		
		//lv_obj_t * X1_label = lv_label_create(lv_scr_act(), NULL);
     	//lv_obj_t * X2_label = lv_label_create(lv_scr_act(), NULL);
		char buff[200];

    //labels
    	/*static lv_style_t label_styleX1;
    	lv_style_init(&label_styleX1);
    	lv_style_set_text_color(&label_styleX1, LV_STATE_DEFAULT, LV_COLOR_BLACK);
    	lv_obj_add_style(X1_label, LV_LABEL_PART_MAIN, &label_styleX1);
    	lv_obj_align(X1_label, NULL, LV_ALIGN_CENTER, -50, -10);
    
    	static lv_style_t label_styleX2;
    	lv_style_init(&label_styleX2);
    	lv_style_set_text_color(&label_styleX2, LV_STATE_DEFAULT, LV_COLOR_BLACK);
    	lv_obj_add_style(X2_label, LV_LABEL_PART_MAIN, &label_styleX2);
    	lv_obj_align(X2_label, NULL, LV_ALIGN_CENTER, -50, 10);*/

		// read values
		int read_rawX1 = adc1_get_raw(ADC_CHANNEL_X1); 	
		int read_rawY1 = adc1_get_raw(ADC_CHANNEL_Y1);
		int read_rawX2 = adc1_get_raw(ADC_CHANNEL_X2);
		int read_rawY2 = adc1_get_raw(ADC_CHANNEL_Y2);

		if(read_rawX1 > 300){
			read_rawX1 = 3;
			/*
			lv_refr_now(NULL);    
            sprintf(buff, "x1: %d\n", read_rawX1);
            lv_label_set_text(X1_label, buff);
            lv_refr_now(NULL);*/
		}
		else if(read_rawX1<150){
			read_rawX1 = -3;
			/*lv_refr_now(NULL);    
            sprintf(buff, "x1: %d\n", read_rawX1);
            lv_label_set_text(X1_label, buff);
            lv_refr_now(NULL);*/
		} 
		else {
			read_rawX1 = 0;
			/*lv_refr_now(NULL);    
            sprintf(buff, "x1: %d\n", read_rawX1);
            lv_label_set_text(X1_label, buff);
            lv_refr_now(NULL);*/
		} 

		if(read_rawX2 > 300)
			read_rawX2 = 3;
		else if(read_rawX2<150)
			read_rawX2 = -3;
		else
			read_rawX2 = 0;

		// assign message to publisher		  		
		// map values from 0 - 511 (ADC input) to (-3) - 3 (for turtlesim speed)

		twist.linear.x =  read_rawX1;  
		twist.angular.z = read_rawX2;

		printf("analog read x= %d \t y= %d \n", read_rawX1 , read_rawX2 );
		printf("twist out x= %f \t z= %f \n", twist.linear.x , twist.angular.z );

		RCSOFTCHECK(rcl_publish(&publisher, &twist, NULL));		

	}	
}


void appMain(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
	
	r1 = adc1_pad_get_io_num( ADC_CHANNEL_X1, &adc_gpio_numX1);
    r2 = adc1_pad_get_io_num( ADC_CHANNEL_Y1, &adc_gpio_numY1);
    r1 = adc1_pad_get_io_num( ADC_CHANNEL_X2, &adc_gpio_numX2);
    r2 = adc1_pad_get_io_num( ADC_CHANNEL_Y2, &adc_gpio_numY2);


    assert( r1 == ESP_OK );
    assert( r2 == ESP_OK );
    assert( r3 == ESP_OK );
    assert( r4 == ESP_OK );

    printf("adc1 channel %d @ GPIO %d. \n", ADC_CHANNEL_X1, adc_gpio_numX1);
    printf("adc1 channel %d @ GPIO %d. \n", ADC_CHANNEL_Y1, adc_gpio_numY1);
    printf("adc1 channel %d @ GPIO %d. \n", ADC_CHANNEL_X2, adc_gpio_numX2);
    printf("adc1 channel %d @ GPIO %d. \n", ADC_CHANNEL_Y2, adc_gpio_numY2);

	//ADC initialization
    printf("adc1_init...\n");    
    
    r1 = adc1_config_channel_atten(ADC_CHANNEL_X1, atten);
    r2 = adc1_config_channel_atten(ADC_CHANNEL_Y1, atten);
    r3 = adc1_config_channel_atten(ADC_CHANNEL_X2, atten);
    r4 = adc1_config_channel_atten(ADC_CHANNEL_Y2, atten);
	adc1_config_width(width);
    assert( r1 == ESP_OK );
    assert( r2 == ESP_OK );
    assert( r3 == ESP_OK );
    assert( r4 == ESP_OK );


	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "freertos_twist_pub", "", &support));
	    
    

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		//"/turtle1/cmd_vel"));
		"/model/vehicle_blue/cmd_vel"));

	
	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 500;  //publish timeout
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));		
		usleep(100000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node))
	RCCHECK(rcl_node_fini(&node))

  	vTaskDelete(NULL);
}