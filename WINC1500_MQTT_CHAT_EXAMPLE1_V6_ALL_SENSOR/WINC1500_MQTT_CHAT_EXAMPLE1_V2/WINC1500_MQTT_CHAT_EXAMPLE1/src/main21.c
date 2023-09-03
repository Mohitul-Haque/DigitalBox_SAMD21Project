/**
 *
 * \file
 *
 * \brief WINC1500 MQTT chat example.
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/** \mainpage
 * \section intro Introduction
 * This example demonstrates the use of the WINC1500 with the SAMD21 Xplained Pro
 * board to implement an MQTT based chat.
 * It uses the following hardware:
 * - the SAMD21 Xplained Pro.
 * - the WINC1500 on EXT1.
 *
 * \section files Main Files
 * - main.c : Initialize the WINC1500, connect to MQTT broker and chat with the other devices.
 * - mqtt.h : Implementation of MQTT 3.1
 *
 * \section usage Usage
 * -# Configure below code in the main.h for AP information to be connected.
 * \code
 *    #define MAIN_WLAN_SSID         "DEMO_AP"
 *    #define MAIN_WLAN_AUTH         M2M_WIFI_SEC_WPA_PSK
 *    #define MAIN_WLAN_PSK          "12345678"
 * \endcode
 * -# Build the program and download it into the board.
 * -# On the computer, open and configure a terminal application as the follows.
 * \code
 *    Baud Rate : 115200
 *    Data : 8bit
 *    Parity bit : none
 *    Stop bit : 1bit
 *    Flow control : none
 *    Line-Ending style : LF or CR+LF
 * \endcode
 * -# Start the application.
 * -# In the terminal window, First of all enter the user name through the terminal window.
 * -# And after the text of the following is displayed, please enjoy the chat.
 * -# Initialization operations takes a few minutes according to the network environment.
 * \code
 *    Preparation of the chat has been completed.
 * \endcode
 *
 * \section known_issue Known Issue
 * -# The user name cannot contain space (' ').
 * -# Cannot send more than 128 bytes.
 * -# User name must be unique. If someone uses the same user name, Which one will be disconnected.
 * -# USART interface has not error detection procedure. So sometimes serial input is broken.
 *
 * \section compinfo Compilation Information
 * This software was written for the GNU GCC compiler using Atmel Studio 6.2
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com">Atmel</A>.\n
 */

#include "asf.h"
#include "main.h"
#include "driver/include/m2m_wifi.h"
#include "iot/mqtt/mqtt.h"
#include "iot/sw_timer.h"
#include "socket/include/socket.h"
#include "tick_counter.h"

#include <math.h>
#include "demo_tools.h"
#include "shtc1.h"
#include "ams_voc.h"


//#define MEASUREMENT_INTERVAL_MS 2000

#define Vcc 4.65
#define RL 1000
#define R0 5000


/* Application instruction phrase. */
#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 Wi-Fi MQTT chat example --"STRING_EOL \
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

//Function prototype
void configure_adc(void);

//! [module_inst]
struct adc_module adc_instance;
//! [module_inst]


//! [setup]
void configure_adc(void)
{
	//! [setup_config]
	struct adc_config config_adc;
	//! [setup_config]
	//! [setup_config_defaults]
	adc_get_config_defaults(&config_adc);
	//! [setup_config_defaults]

	config_adc.gain_factor		= ADC_GAIN_FACTOR_DIV2;
	config_adc.clock_prescaler	= ADC_CLOCK_PRESCALER_DIV32;
	config_adc.reference		= ADC_REFERENCE_INTVCC1;
	config_adc.positive_input	= ADC_POSITIVE_INPUT_PIN0; //AIN0 which is PA02 in Xplained pro board
	config_adc.resolution		= ADC_RESOLUTION_12BIT;
	config_adc.freerunning		= true;
	config_adc.left_adjust		= false;
	adc_init(&adc_instance, ADC, &config_adc);
	adc_enable(&adc_instance);
	
	//! [setup_enable]
}
//! [setup]


//function set as static to limit access of getADC_Value() from outside this c file
static uint16_t getADC_Value(void){
	uint16_t sensorVal;
	//status_code_genare_t adc_status;
	adc_start_conversion(&adc_instance);
	
	do {
		/* Wait for conversion to be done and read out result */
	} while (adc_read(&adc_instance, &sensorVal) == STATUS_BUSY);
	//! [get_res]
	return sensorVal;
}










/** UART module for debug. */
static struct usart_module cdc_uart_module;

/** Instance of Timer module. */
struct sw_timer_module swt_module_inst;

/** User name of chat. */
char mqtt_user[64] = "seyam";
//char topic1[64] = "sensor";
//char topic2[64] = "air";

/* Instance of MQTT service. */
static struct mqtt_module mqtt_inst;

/* Receive buffer of the MQTT service. */
static char mqtt_buffer[MAIN_MQTT_BUFFER_SIZE];

/** UART buffer. */
static char uart_buffer[MAIN_CHAT_BUFFER_SIZE];

/** Written size of UART buffer. */
static int uart_buffer_written = 0;

/** A buffer of character from the serial. */
static uint16_t uart_ch_buffer;

/**
 * \brief Callback of USART input.
 *
 * \param[in] module USART module structure.
 */
static void uart_callback(const struct usart_module *const module)
{
	/* If input string is bigger than buffer size limit, ignore the excess part. */
	if (uart_buffer_written < MAIN_CHAT_BUFFER_SIZE) {
		uart_buffer[uart_buffer_written++] = uart_ch_buffer & 0xFF;
	}
}

static void wifi_callback(uint8 msg_type, void *msg_data)
{
	tstrM2mWifiStateChanged *msg_wifi_state;
	uint8 *msg_ip_addr;

	switch (msg_type) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
		msg_wifi_state = (tstrM2mWifiStateChanged *)msg_data;
		if (msg_wifi_state->u8CurrState == M2M_WIFI_CONNECTED) {
			/* If Wi-Fi is connected. */
			printf("Wi-Fi connected\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (msg_wifi_state->u8CurrState == M2M_WIFI_DISCONNECTED) {
			/* If Wi-Fi is disconnected. */
			printf("Wi-Fi disconnected\r\n");
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
					MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
			/* Disconnect from MQTT broker. */
			/* Force close the MQTT connection, because cannot send a disconnect message to the broker when network is broken. */
			mqtt_disconnect(&mqtt_inst, 1);
		}

		break;

	case M2M_WIFI_REQ_DHCP_CONF:
		msg_ip_addr = (uint8 *)msg_data;
		printf("Wi-Fi IP is %u.%u.%u.%u\r\n",
				msg_ip_addr[0], msg_ip_addr[1], msg_ip_addr[2], msg_ip_addr[3]);
		/* Try to connect to MQTT broker when Wi-Fi was connected. */
		mqtt_connect(&mqtt_inst, main_mqtt_broker);
		break;

	default:
		break;
	}
}

/**
 * \brief Callback to get the Socket event.
 *
 * \param[in] Socket descriptor.
 * \param[in] msg_type type of Socket notification. Possible types are:
 *  - [SOCKET_MSG_CONNECT](@ref SOCKET_MSG_CONNECT)
 *  - [SOCKET_MSG_BIND](@ref SOCKET_MSG_BIND)
 *  - [SOCKET_MSG_LISTEN](@ref SOCKET_MSG_LISTEN)
 *  - [SOCKET_MSG_ACCEPT](@ref SOCKET_MSG_ACCEPT)
 *  - [SOCKET_MSG_RECV](@ref SOCKET_MSG_RECV)
 *  - [SOCKET_MSG_SEND](@ref SOCKET_MSG_SEND)
 *  - [SOCKET_MSG_SENDTO](@ref SOCKET_MSG_SENDTO)
 *  - [SOCKET_MSG_RECVFROM](@ref SOCKET_MSG_RECVFROM)
 * \param[in] msg_data A structure contains notification informations.
 */
static void socket_event_handler(SOCKET sock, uint8_t msg_type, void *msg_data)
{
	mqtt_socket_event_handler(sock, msg_type, msg_data);
}

/**
 * \brief Callback of gethostbyname function.
 *
 * \param[in] doamin_name Domain name.
 * \param[in] server_ip IP of server.
 */
static void socket_resolve_handler(uint8_t *doamin_name, uint32_t server_ip)
{
	mqtt_socket_resolve_handler(doamin_name, server_ip);
}

/**
 * \brief Callback to get the MQTT status update.
 *
 * \param[in] conn_id instance id of connection which is being used.
 * \param[in] type type of MQTT notification. Possible types are:
 *  - [MQTT_CALLBACK_SOCK_CONNECTED](@ref MQTT_CALLBACK_SOCK_CONNECTED)
 *  - [MQTT_CALLBACK_CONNECTED](@ref MQTT_CALLBACK_CONNECTED)
 *  - [MQTT_CALLBACK_PUBLISHED](@ref MQTT_CALLBACK_PUBLISHED)
 *  - [MQTT_CALLBACK_SUBSCRIBED](@ref MQTT_CALLBACK_SUBSCRIBED)
 *  - [MQTT_CALLBACK_UNSUBSCRIBED](@ref MQTT_CALLBACK_UNSUBSCRIBED)
 *  - [MQTT_CALLBACK_DISCONNECTED](@ref MQTT_CALLBACK_DISCONNECTED)
 *  - [MQTT_CALLBACK_RECV_PUBLISH](@ref MQTT_CALLBACK_RECV_PUBLISH)
 * \param[in] data A structure contains notification informations. @ref mqtt_data
 */
static void mqtt_callback(struct mqtt_module *module_inst, int type, union mqtt_data *data)
{
	switch (type) {
	case MQTT_CALLBACK_SOCK_CONNECTED:
	{
		/*
		 * If connecting to broker server is complete successfully, Start sending CONNECT message of MQTT.
		 * Or else retry to connect to broker server.
		 */
		if (data->sock_connected.result >= 0) {
			mqtt_connect_broker(module_inst, 1, NULL, NULL, mqtt_user, NULL, NULL, 0, 0, 0);// (clean session, user_id, user_password, clientID, will_topic, will_msg, QoS)
		} else {
			printf("Connect fail to server(%s)! retry it automatically.\r\n", main_mqtt_broker);
			mqtt_connect(module_inst, main_mqtt_broker); /* Retry that. */
		}
	}
	break;

	case MQTT_CALLBACK_CONNECTED:
		if (data->connected.result == MQTT_CONN_RESULT_ACCEPT) {
			/* Subscribe chat topic. */
			mqtt_subscribe(module_inst, MAIN_CHAT_TOPIC "#", 0);
			printf("Subscribed to the topic: %s\r\n", MAIN_CHAT_TOPIC);
			/* Enable USART receiving callback. */
			usart_enable_callback(&cdc_uart_module, USART_CALLBACK_BUFFER_RECEIVED);
			printf("Preparation of the chat has been completed.\r\n");
		} else {
			/* Cannot connect for some reason. */
			printf("MQTT broker decline your access! error code %d\r\n", data->connected.result);
		}

		break;

	case MQTT_CALLBACK_RECV_PUBLISH:
		/* You received publish message which you had subscribed. */
		if (data->recv_publish.topic != NULL && data->recv_publish.msg != NULL) {
			if (!strncmp(data->recv_publish.topic, MAIN_CHAT_TOPIC, strlen(MAIN_CHAT_TOPIC))) {
				/* Print user name and message */
				for (int i = strlen(MAIN_CHAT_TOPIC); i < data->recv_publish.topic_size; i++) {
				//for (int i = 0; i < data->recv_publish.topic_size; i++) {
					printf("%c", data->recv_publish.topic[i]);
				}
				printf(" >> ");
				for (int i = 0; i < data->recv_publish.msg_size; i++) {
					printf("%c", data->recv_publish.msg[i]);
				}
				printf("\r\n");
			}
		}

		break;

	case MQTT_CALLBACK_DISCONNECTED:
		/* Stop timer and USART callback. */
		printf("MQTT disconnected\r\n");
		usart_disable_callback(&cdc_uart_module, USART_CALLBACK_BUFFER_RECEIVED);
		break;
	}
}

/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	struct usart_config usart_conf;

	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	usart_conf.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	usart_conf.baudrate    = 115200;

	stdio_serial_init(&cdc_uart_module, EDBG_CDC_MODULE, &usart_conf);
	/* Register USART callback for receiving user input. */
	usart_register_callback(&cdc_uart_module, (usart_callback_t)uart_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable(&cdc_uart_module);
}

/**
 * \brief Configure Timer module.
 */
static void configure_timer(void)
{
	struct sw_timer_config swt_conf;
	sw_timer_get_config_defaults(&swt_conf);

	sw_timer_init(&swt_module_inst, &swt_conf);
	sw_timer_enable(&swt_module_inst);
}

/**
 * \brief Configure MQTT service.
 */
static void configure_mqtt(void)
{
	struct mqtt_config mqtt_conf;
	int result;

	mqtt_get_config_defaults(&mqtt_conf);
	/* To use the MQTT service, it is necessary to always set the buffer and the timer. */
	mqtt_conf.timer_inst = &swt_module_inst;
	mqtt_conf.recv_buffer = mqtt_buffer;
	mqtt_conf.recv_buffer_size = MAIN_MQTT_BUFFER_SIZE;

	result = mqtt_init(&mqtt_inst, &mqtt_conf);
	if (result < 0) {
		printf("MQTT initialization failed. Error code is (%d)\r\n", result);
		while (1) {
		}
	}

	result = mqtt_register_callback(&mqtt_inst, mqtt_callback);
	if (result < 0) {
		printf("MQTT register callback failed. Error code is (%d)\r\n", result);
		while (1) {
		}
	}
}


//This block not required
/**
 * \brief Checking the USART buffer.
 *
 * Finding the new line character(\n or \r\n) in the USART buffer.
 * If buffer was overflowed, Sending the buffer.
 */

static void sendData(char *topicSensor, int temp, int hum, int gas)
{
	//uint8_t temp = 15;
	//uint8_t hum = 55;
	//uint16_t voc = 600;
	//uint16_t co2 = 400;
	//uint16_t ch4 = 500;
	
	char telemetry_buffer1[32]; //Maximum message length should be shorter than 128 bytes
	sprintf(telemetry_buffer1, "{\"TEMP\":%d,\"HUM\":%d,\"GAS\":%d}", temp/1000, hum/1000, gas);
	
	/* Publish the input string when newline was received or input string is bigger than buffer size limit. */
	//mqtt_publish(&mqtt_inst, topic, data_buffer, MAIN_CHAT_BUFFER_SIZE, 0, 0);
	mqtt_publish(&mqtt_inst, topicSensor, telemetry_buffer1, sizeof(telemetry_buffer1), 0, 0); //Maximum message length should be shorter than 128 bytes
	//printf("Sensor Data sent of bytes %d \r\n",sizeof(telemetry_buffer1));
}


static void sendAMSData(char *topicAMS, int voc, int co2)
{
	
	//uint16_t voc = 600;
	//uint16_t co2 = 400;
	
	char telemetry_buffer2[22]; //Maximum message length should be shorter than 128 bytes
	sprintf(telemetry_buffer2, "{\"VOC\":%d,\"CO2\":%d}", voc, co2);
	
	/* Publish the input string when newline was received or input string is bigger than buffer size limit. */
	//mqtt_publish(&mqtt_inst, topic, data_buffer, MAIN_CHAT_BUFFER_SIZE, 0, 0);
	mqtt_publish(&mqtt_inst, topicAMS, telemetry_buffer2, sizeof(telemetry_buffer2), 0, 0); //Maximum message length should be shorter than 128 bytes
	//printf("AMS Data sent of bytes %d \r\n",sizeof(telemetry_buffer2));
}

/**
 * \brief Main application function.
 *
 * Application entry point.
 *
 * \return program return value.
 */
int main(void)
{
	tstrWifiInitParam param;
	int8_t ret;
	char topicSensor[strlen(MAIN_CHAT_TOPIC) + MAIN_CHAT_USER_NAME_SIZE + 1];
	char topicAMS[strlen(MAIN_CHAT_TOPIC) + MAIN_CHAT_USER_NAME_SIZE + 1];
	
	//char msgBuff[32];
	
	/* Initialize the board. */
	system_init();

	/* Initialize the UART console. */
	configure_console();
	
	
	/* Output example information */
	printf(STRING_HEADER);

	/* Initialize the Timer. */
	configure_timer();

	
	/* Initialize the BSP. */
	nm_bsp_init();
	
	
	/* Initialize the ADC on a specific channel */
	configure_adc();

	

	
	
	/* Setup user name first */
	//printf("Enter the user name (Max %d characters)\r\n", MAIN_CHAT_USER_NAME_SIZE);
	//scanf("%64s", mqtt_user);
	printf("User : %s\r\n", mqtt_user); // Add new line after printing mqtt_user
	sprintf(topicSensor, "%s%s", MAIN_CHAT_TOPIC, "sensor"); //concatenate & save 'MAIN_CHAT_TOPIC' data to the 'topic' char array
	sprintf(topicAMS, "%s%s", MAIN_CHAT_TOPIC,"ams"); //concatenate & save 'MAIN_CHAT_TOPIC' data to the 'topic' char array
	

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_callback; /* Set Wi-Fi event callback. */
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) { /* Loop forever. */
		}
	}

	/* Initialize socket interface. */
	socketInit();
	registerSocketCallback(socket_event_handler, socket_resolve_handler);

	/* Connect to router. */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
			MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
			
			
	/* Initialize the MQTT service. */
	configure_mqtt();	
		
				
	/* values read from sensor, encoded as integer numbers */
    int temp, rh, voc, co2;
	
	int16_t gas;
	
	
    /**
     * temperature (°C), dew point (°C),
     * and relative humidity (%RH)
     */
    //int temp_f, rh_f;
	//int dew_f, k_f, VRL, Rs, ratio;

    /* average reference value to detect air-touch */
    //float rh_avg_f = 0.0f;

    /* statuses of I2C communication */
    enum status_code shtc1_connected;
    enum status_code shtc1_read_status;
	enum status_code ams_read_status;
	
	/* Initialize the I2C console. */
	shtc1_i2c_init();

    delay_s(2);

    printf("\r\nstarting...\r\n");

    /* test if sensor is connected */
    shtc1_connected = shtc1_probe(&i2c_master_instance);
    printf("sensor is %s\r\n",
            shtc1_connected ? "present" : "not present");


	/* Enable SysTick interrupt for non busy wait delay. */
	if (SysTick_Config(system_cpu_clock_get_hz() / 1000))
	{
		printf("%s\r\n","SysTick configuration error");
		while(1);
	}



	while (1) {
		
		m2m_wifi_handle_events(NULL);
		
		sw_timer_task(&swt_module_inst);
		
		
		//delay_s(1);
		
				
		//Code for reading Temp & Humidity
        
		
		
		
		
		//delay_s(1);
		
		//Code for reading AMS VOC
		/***  
			450 – 2000 ppm CO2 equivalents (relative)
			125 – 600 ppb TVOC equivalents (relative)
			Values above the defined sensing range are provided as well
		***/
		
				
		
				
				
				
		//delay_ms(500);
		//gas = getADC_Value();		
		//printf("ADC = (%d)\r\n", gas);
		//VRL = (3.3*gas)/4096;
		//printf(VRL);
		//Rs = ((Vcc/VRL)-1)*RL;
		//printf(Rs);
		//ratio = Rs/R0;
		//printf(ratio);
	
	
	
	
		//Write your timing dependent function to be called after time-out
		if( tick_counter_check_timer())
		{
			//shtc code
			shtc1_read_status = shtc1_read_lpm_sync(&i2c_master_instance, &temp, &rh); //read into the temp and rh variables
			
			delay_ms(100);
			
			if (shtc1_read_status == STATUS_OK)
			{
				//printf("Temp: (%d'C)\r\n",temp/1000);
				//printf("Humidity: (%d%%)\r\n",rh/1000);
				//temp_f = temp / 1000;
				//rh_f = rh / 1000;
				
				
				//sprintf(msgBuff,"%.2f",temp_f); //converts and stores float to char array
				//printf("Data: %s\r\n",msgBuff);
				
				
				//mqtt_publish(&mqtt_inst, topic2, dt, MAIN_MQTT_BUFFER_SIZE, 0, 0); //This time mqtt buffer size is being used, not uart buffer size

			}
			else
			{
				
				printf("Bro I can't measure the Temp & Humidity. Are you sure you plugged the sensor in?\n");
				
			}
			
			
			
			
			
			
			//ams code
			ams_read_status = ams_sensor_read(&i2c_master_instance, &voc, &co2); //read into the address of the variable voc and co2
			delay_ms(100);
			if (ams_read_status==STATUS_OK)
			{
				sendAMSData(topicAMS, voc, co2);
				//printf("VOC = (%d) ppb\r\n",voc);
				//printf("CO2 = (%d) ppm\r\n",co2);
				
			}
			else
			{
				printf("Bro I can't measure the VOC & CO2. Are you sure you plugged the iAQ-Core in?\n");
				
			}
			
			
			//adc read
			delay_ms(500);
			gas = getADC_Value();
			
			
			sendData(topicSensor, temp, rh, gas);
			
			//printSystemTime();
			tick_counter_reset_timer(TICK_COUNTER_INTERVAL);
		}
	
		else{
		
			//printSystemTime();
			//printf("%s\r\n", "System Time is less than interval!" );
		}
				
	}//while(1) loop ends
}//main loop ends
