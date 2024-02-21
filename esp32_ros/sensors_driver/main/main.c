#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "PMW3360.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define Main_Tag "Main"

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray send_msg;
std_msgs__msg__Int32 recv_msg;

static QueueHandle_t data_received_queue = NULL;
static Motion_Data curr_data;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
		curr_data = Get_Data();
		printf("Time %.2fs, %.2f, %.2f\n", curr_data.timestamps / 100.0, curr_data.x_dist, curr_data.y_dist);

		send_msg.data.capacity = 10;
		send_msg.data.data = (int32_t*) malloc(send_msg.data.capacity * sizeof(int32_t));
		send_msg.data.size = 0;

		// Assigning static memory to the sequence
		static int32_t memory[10];
		send_msg.data.capacity = 10;
		send_msg.data.data = memory;
		send_msg.data.size = 0;

		// Populate array with three integer data
		send_msg.data.data[0] = curr_data.timestamps;
		send_msg.data.data[1] = curr_data.x_dist * 100;
		send_msg.data.data[2] = curr_data.y_dist * 100;
		send_msg.data.size = 3; // Update size of the array to 3
	}
}

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	uint32_t data = (int)  msg->data;
	if (xQueueSend(data_received_queue, &data, portMAX_DELAY) != pdPASS) {
            printf("Error sending data to queue\n");
        }
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options.
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif
	// Setup support structure.
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// Create node.
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "esp32_motion_sensors_driver", "", &support));

	// Create publisher.
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
		"esp32_motion_sensors_publisher"));

	// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"esp32_CPI_cmd"));

	// Create timer.
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 100;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	// Add timer and subscriber to executor.
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

	// Spin forever.
	// send_msg.data.data = {0,0,0};
	// msg.data.size = sizeof(msg.data.data) / sizeof(msg.data.data[0]); 
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		usleep(1000);
	}

	// Free resources.
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	// RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

static void datain_process_task(void* arg){
	uint32_t data;
    while(1){
		if(xQueueReceive(data_received_queue, &data, portMAX_DELAY)){
			printf("queue received %ld\n",data);
			if (data <= 10000 && data >= 100){
				setCPI(data);
			}
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);


void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    data_received_queue = xQueueCreate(200, sizeof(uint32_t));
    xTaskCreate(datain_process_task, "datain_process_task", 2048, NULL, 2, NULL);

	pmw3360_spi_init(PMW3360_CS_GPIO, PMW3360_SCLK_GPIO, PMW3360_MISO_GPIO, PMW3360_MOSI_GPIO);
    performStartup();

    xTaskCreate(data_collection_task, "data_collection_task", 2048, NULL, 2, NULL);

    // pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}

