/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup main.c
 * @{
 * @ingroup iot_sdk_app_lwip
 *
 * @brief This file contains the source code for LwIP based MQTT Client sample application.
 *        This example publishes the topic "kitchen/sensor1" when a pir sensor connected to pin 15 goes off
 *        the state is toggled but the value is not material to this .
 *        Value of 0 or 1 is published as data for the topic based on PIR is turned ON or OFF.
 *        It is derived from the Nordic semiconductor iot mqtt example
 */

#include <stdbool.h>
#include <stdint.h>
#include "bsp/boards.h"
#include "bsp/pca10028.h"
#include "app_timer_appsh.h"
#include "app_scheduler.h"
#include "app_button.h"
#include "nordic_common.h"
#include "softdevice_handler_appsh.h"
#include "ble_advdata.h"
#include "ble_srv_common.h"
#include "ble_ipsp.h"
#include "ble_6lowpan.h"
#include "mem_manager.h"
#include "app_trace.h"
#include "lwip/init.h"
#include "lwip/inet6.h"
#include "lwip/ip6.h"
#include "lwip/ip6_addr.h"
#include "lwip/netif.h"
#include "mqtt.h"
#include "lwip/timers.h"
#include "nrf_platform_port.h"
#include "app_util_platform.h"

#include "nrf_log.h"

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "bsp.h"
#include "components/libraries/timer/app_timer.h"
#include "nrf_error.h"
#include "components/device/nrf51.h"
#include "components/device/nrf51_bitfields.h"
#include "components/drivers_nrf/hal/nrf_gpiote.h"
#include "components/libraries/util/app_error.h"

// @todo additional includes to be cleaned up




#define DEVICE_NAME                         "MQTTSensor1"                                         /**< Device name used in BLE undirected advertisement. */

/** Modify m_broker_addr according to your setup.
 * This address is the ip6 address of your broker
 * It does not have to be the ip6 router gateway
 * you can use 6tunnel to bridge to your ip4 broker
 *  The address provided below is a place holder.  */



static const ip6_addr_t                     m_broker_addr =
{
    .addr =
    {HTONL(0xfe800000),
    0x00000000,
    HTONL(0x021a7dff),
    HTONL(0xfeda710c)}
};




#define APP_TIMER_PRESCALER                 NRF51_DRIVER_TIMER_PRESCALER                            /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS                3                                                       /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE             2
#define LWIP_SYS_TIMER_INTERVAL             APP_TIMER_TICKS(50 , APP_TIMER_PRESCALER)               /**< Interval for timer used as trigger to send. */

#define MQTT_START_INTERVAL                  APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)               /**< mqtt check interval. */



#define SCHED_MAX_EVENT_DATA_SIZE           128                                                     /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                    12                                                      /**< Maximum number of events in the scheduler queue. */


#define APP_ADV_TIMEOUT                     0                                                       /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define APP_ADV_ADV_INTERVAL                MSEC_TO_UNITS(100, UNIT_0_625_MS)                       /**< The advertising interval. This value can vary between 100ms to 10.24s). */
#define ENABLE_DEBUG_LOG_SUPPORT 			1
#define DEAD_BEEF                           0xDEADBEEF                                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define APPL_LOG                   NRF_LOG_PRINTF
//#define APPL_LOG                            app_trace_log                                           /**< Macro for logging application messages on UART, in case ENABLE_DEBUG_LOG_SUPPORT is not defined, no logging occurs. */
#define APPL_DUMP                           app_trace_dump                                          /**< Macro for dumping application data on UART, in case ENABLE_DEBUG_LOG_SUPPORT is not defined, no logging occurs. */

#define APP_MQTT_BROKER_PORT                1883                                                    /**< Port number of MQTT Broker being used. */

#define PIR_PIN       						18


eui64_t                                     eui64_local_iid;                                        /**< Local EUI64 value that is used as the IID for*/
static ble_gap_adv_params_t                 m_adv_params;                                           /**< Parameters to be passed to the stack when starting advertising. */

static app_timer_id_t                       m_sys_timer_id;                                         /**< System Timer used to service LwIP timers periodically. */
static app_timer_id_t                       m_mqtt_timer_id;                                         /**< mqtt Timer used to start mqtt when ip6 is up. */

static mqtt_client_t                        m_app_mqtt_id;                                          /**< MQTT Client instance reference provided by the MQTT module. */
static const char                           m_device_id[] = "kitchensensor1";                      /**< Unique MQTT client identifier. */

static const char 							topic_desc[] = "kitchen/sensor1";						/**< mqtt topic for this chip>*/
static const char 							topic_imalive[] = "kitchen/imalive1";						/**< mqtt alive topic for openhab monitoring>*/

//static const char                           m_pir1_state_on[] = "on";                      			/**< sensor on. */
//static const char                           m_pir1_state_off[] = "off";                      		/**< sensor off. */

static const char                           m_pir1_state_on[] = "1";                      			/**< sensor on. */
static const char                           m_pir1_state_off[] = "0";                      		/**< sensor off. */

static long 								hearbeat_timer = 0;

static bool                                 m_pir1_state  = false;                                   /**< pir state. This is the topic being published by the example MQTT client. */


static bool                                 m_connection_state  = false;                            /**< MQTT Connection state. */


void app_mqtt_evt_handler(const mqtt_client_t * p_client, const mqtt_evt_t * p_evt);
static void app_mqtt_publish(bool pir1_state, char* topic);
static void pir_pin_init(void);
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void mqtt_timer_callback(void * p_context);



/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    //Halt the application and notify of error using the LEDs.
    APPL_LOG("[** ASSERT **]: Error 0x%08lX, Line %ld, File %s\r\n", error_code, line_num, p_file_name);

     for(;;)
    {
    }

    // @note: In case on assert, it is desired to only recover and reset, uncomment the line below.
    //NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the nrf log module.
 */
static void nrf_log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT();
    APP_ERROR_CHECK(err_code);
}



//pir event handler
//publish a message using mqtt when the sensor fires
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (m_connection_state == true)
    {
    	//we have a reading, toggle the sensor state
    	m_pir1_state = !m_pir1_state;
        app_mqtt_publish(m_pir1_state, (uint8_t *)topic_desc);
        //APPL_LOG("[APPL]: sensor publishing\r\n");
    }
    else
    {
        // out of sequence
    	APPL_LOG("[APPL]: sensor not connected, out of sequence\r\n");
    }

}



//setup the pir sensor pin - BSP_BUTTON_3
static void pir_pin_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);


    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PIR_PIN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIR_PIN, true);
}




/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t                err_code;
    ble_advdata_t           advdata;
    uint8_t                 flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    ble_gap_conn_sec_mode_t sec_mode;
    ble_gap_addr_t          my_addr;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_address_get(&my_addr);
    APP_ERROR_CHECK(err_code);

    my_addr.addr[5]   = 0x00;
    my_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;

    err_code = sd_ble_gap_address_set(&my_addr);
    APP_ERROR_CHECK(err_code);

    IPV6_EUI64_CREATE_FROM_EUI48(eui64_local_iid.identifier,
                                 my_addr.addr,
                                 my_addr.addr_type);

    ble_uuid_t adv_uuids[] =
    {
        {BLE_UUID_IPSP_SERVICE, BLE_UUID_TYPE_BLE}
    };

    //Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.flags                   = flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    //Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT;
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    APPL_LOG("[APPL]: Advertising.\r\n");

}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            APPL_LOG ("[APPL]: ble event Connected.\r\n");

            break;
        case BLE_GAP_EVT_DISCONNECTED:
            APPL_LOG ("[APPL]: ble event Disconnected.\r\n");
            advertising_start();
            break;
        default:
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    //APPL_LOG("[APPL]: BLE event 0x%08lx\r\n", p_ble_evt->header.evt_id);
    ble_ipsp_evt_handler(p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_APPSH_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, true);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for initializing IP stack.
 *
 * @details Initialize the IP Stack and its driver.
 */
static void ip_stack_init(void)
{
    uint32_t err_code = nrf51_sdk_mem_init();
    APP_ERROR_CHECK(err_code);

    //Initialize LwIP stack.
    lwip_init();

    //Initialize LwIP stack driver.
    err_code = nrf51_driver_init();
    APP_ERROR_CHECK(err_code);

    mqtt_init();
}


/**@brief Timer callback used for periodic servicing of LwIP protocol timers.
 *        This trigger is also used in the example to trigger sending TCP Connection.
 *
 * @details Timer callback used for periodic servicing of LwIP protocol timers.
 *
 * @param[in]   p_context   Pointer used for passing context. No context used in this application.
 */
static void system_timer_callback(void * p_context)
{
	UNUSED_VARIABLE(p_context);
    sys_check_timeouts();
    // keep the mqtt connection open
    // calls mqtt_ping
    UNUSED_VARIABLE(mqtt_live());
    //send a heart beat, openhab can use this to monitor the sensors state
  //  app_mqtt_publish(m_pir1_state, (uint8_t *)topic_imalive);
}

/**@brief Timer callback used for starting mqtt connection
 *        Will try to connect periodically
 *        this means that the brokercan be reset and the
 *        device will reconnect once ip6 is up
 *
 * @details .
 *
 * @param[in]   p_context   Pointer used for passing context. No context used in this application.
 */
static void mqtt_timer_callback(void * p_context)
{

   // APPL_LOG("[APPL]: mqtt_timer_callback \r\n");

    if (m_connection_state == false)
    {

    	APPL_LOG("[APPL]: about to connect to broker\r\n");
        mqtt_connect_t param;

        param.broker_addr    = m_broker_addr;
        param.broker_port    = APP_MQTT_BROKER_PORT;
        param.evt_cb         = app_mqtt_evt_handler;
        param.device_id      = m_device_id;
        param.p_password     = NULL;
        param.p_user_name    = NULL;

        UNUSED_VARIABLE(mqtt_connect(&m_app_mqtt_id, &param));

        //APPL_LOG("[APPL]: Just tried to connect to broker\r\n");
    }
    else
    {

        //send a heart beat, openhab can use this to monitor the sensor state
      //  about every 30 minutes

    	hearbeat_timer++;
    	//if (hearbeat_timer > 900){
    		if (hearbeat_timer > 3){
    		app_mqtt_publish(m_pir1_state, (uint8_t *)topic_imalive);
    		hearbeat_timer = 0;
    		APPL_LOG("[APPL]: sending heart beat\r\n");
    	}

        // out of sequence
    	//APPL_LOG("[APPL]: mqtt_timer_callback  out of sequence\r\n");
    }

    //APPL_LOG("[APPL]: mqtt_timer_callback   after callback\r\n");


}


/**@brief Publishes pir1 state to MQTT broker.
 *
 * @param[in]   pir1_state   LED state being published.
 */
static void app_mqtt_publish(bool pir1_state, char* topic)
{
    mqtt_topic_t mqtttopic;
    mqtt_data_t  data;

   // char topic_desc[] = "kitchen/sensor1";

    //mqtttopic.p_topic = (uint8_t *)topic_desc;
    mqtttopic.p_topic = topic;
    mqtttopic.topic_len = strlen(topic);


    APPL_LOG("[APPL]: pir1_state %d \r\n", pir1_state);
    data.data_len = 1;
    //data.p_data = (uint8_t *)&pir1_state;

    if (pir1_state == true)
    {
    	//data.data_len = 2;
    	data.p_data = (uint8_t *)&m_pir1_state_on;
    }
    else
    {
    	//data.data_len = 3;
    	data.p_data = (uint8_t *)&m_pir1_state_off;
    }
    uint32_t err_code = mqtt_publish(&m_app_mqtt_id,&mqtttopic, &data);
    //APPL_LOG("[APPL]: mqtt_publish result 0x%08lx\r\n", err_code);
    if (err_code == MQTT_SUCCESS)
    {
    	APPL_LOG("[APPL]: mqtt_publish MQTT_SUCCESS 0x%08lx topic %s\r\n", err_code, topic);

    }
    else
    {

    	APPL_LOG("[APPL]: mqtt_publish fail 0x%08lx\r\n", err_code);


    }
}



/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;
    // Initialize timer module.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);

    // Create timers.
    err_code = app_timer_create(&m_mqtt_timer_id, APP_TIMER_MODE_REPEATED, mqtt_timer_callback);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sys_timer_id, APP_TIMER_MODE_REPEATED, system_timer_callback);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function to handle interface up event. */
void nrf51_driver_interface_up(void)
{
    uint32_t err_code;

    APPL_LOG ("[APPL]: IPv6 Interface Up.\r\n");

    sys_check_timeouts();

    err_code = app_timer_start(m_sys_timer_id, LWIP_SYS_TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    //IP6 is now up
    // start the connection with the mqtt broker
    err_code = app_timer_start(m_mqtt_timer_id, MQTT_START_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function to handle interface down event. */
void nrf51_driver_interface_down(void)
{
    uint32_t err_code;

    APPL_LOG ("[APPL]: IPv6 Interface Down.\r\n");

    err_code = app_timer_stop(m_sys_timer_id);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_stop(m_mqtt_timer_id);
    APP_ERROR_CHECK(err_code);
    m_connection_state = false;
    NVIC_SystemReset();
}


void app_mqtt_evt_handler(const mqtt_client_t * p_client, const mqtt_evt_t * p_evt)
{
    uint32_t err_code;
    switch(p_evt->id)
    {
        case MQTT_EVT_CONNECTED:
        {
            APPL_LOG ("[APPL]: >> MQTT_EVT_CONNECTED\r\n");
            if(p_evt->result == MQTT_SUCCESS)
            {
                m_connection_state = true;

            }
            break;
        }
        case MQTT_EVT_DISCONNECTED:
        {
            APPL_LOG ("[APPL]: >> MQTT_EVT_DISCONNECTED\r\n");
            m_connection_state = false;

            break;
        }
        default:
        	APPL_LOG ("[APPL]: >> MQTT_EVT_ default\r\n");
            break;
    }
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;

    //Initialize.
    app_trace_init();
    //leds_init();
    nrf_log_init();
    timers_init();
    //APPL_LOG (" pir_pin_init\r\n");
    pir_pin_init();
    //APPL_LOG (" ble_stack_init\r\n");
    ble_stack_init();
    //APPL_LOG (" advertising_init\r\n");
    advertising_init();
    //APPL_LOG (" ip_stack_init\r\n");
    ip_stack_init ();
    //APPL_LOG (" scheduler_init\r\n");
    scheduler_init();
    //APPL_LOG ("advertising_start\r\n");
   // nrf_log_init();
    //LEDS_ON(ALL_APP_LED);
    //Start execution.
    advertising_start();
    //printf("OK, ");
    APPL_LOG (" publisher MQTT\r\n");
    //Enter main loop.
    for (;;)
    {
        //Execute event schedule.
        app_sched_execute();

        //Sleep waiting for an application event.
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);

        //LEDS_INVERT(LED_FOUR);
       // err_code = app_timer_start(m_led_blink_timer, LED_BLINK_INTERVAL, NULL);
       // APP_ERROR_CHECK(err_code);


    }
}

/**
 * @}
 */
