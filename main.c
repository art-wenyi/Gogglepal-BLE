/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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
 * @defgroup ble_sdk_app_gogglepal_main main.c
 * @{
 * @ingroup ble_sdk_app_gogglepal
 * @brief GogglePal service Sample Application
 *
 * This file contains the source code for a sample application using the Glucose Meter service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <time.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf51_bitfields.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_util.h"
#include "app_timer.h"
#include "app_trace.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_dis.h"
#include "ble_bas.h"
#include "ble_gogglepal.h"
#include "ble_conn_params.h"
#include "ble_radio_notification.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "device_manager.h"
#include "pstorage.h"


#include <stdbool.h>
#include <stdio.h>
#include "ble_ancs_c.h"
#include "ble_db_discovery.h"
#include "ble_gap.h"
#include "nrf_soc.h"
#include "app_timer_appsh.h"

#define UART_TX_BUF_SIZE 256                                                       /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                                                       /**< UART RX buffer size. */

#define ATTR_DATA_SIZE                  BLE_ANCS_ATTR_DATA_MAX                      /**< Allocated size for attribute data. */

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define DEVICE_NAME                     "GogglePal"                                   /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "AR Devices"                             			/**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUMBER                    "CONNECT"                                 	/**< Model Number. Will be passed to Device Information Service. */
#define SERIAL_NUMBER										"000-00-000"
#define HARDWARE_REVISION           		"2.5.0"                                     	/**< Hardware Revision. Will be passed to Device Information Service. */

																				/* __DATE__ gives this format: Nov 21 2013 
																					 __TIME__ gives this format: 13:21:23 */
#define FIRMWARE_REVISION               "1.0.8"//__DATE__                           /**< Firmware Revision. Will be passed to Device Information Service. */
#define SOFTWARE_REVISION               "2.2.0"//__TIME__                           /**< Software Revision. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                0x8967452301                                /**< DUMMY Manufacturer ID. Will be passed to Device Information Service. You shall use the ID for your Company*/
#define ORG_UNIQUE_ID                  0xEEEBBE                                    /**< DUMMY Organisation Unique ID. Will be passed to Device Information Service. You shall use the Organisation Unique ID relevant for your Company */

#define APP_ADV_INTERVAL               510                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 318.75 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS     0                                         	 /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER            0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS           6                 												   /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE        5                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL              MSEC_TO_UNITS(7.5, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms). */
#define MAX_CONN_INTERVAL              MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (100 ms) */
#define SLAVE_LATENCY                  0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT               MSEC_TO_UNITS(300, UNIT_10_MS)              /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(500, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAM_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS           2                                           /**< Maximum number of users of the GPIOTE handler. */

#define MESSAGE_BUFFER_SIZE            18                                          /**< Size of buffer holding optional messages in notifications. */

#define SECURITY_REQUEST_DELAY         APP_TIMER_TICKS(1500, APP_TIMER_PRESCALER)  /**< Delay after connection until security request is sent, if necessary (ticks). */

#define SEC_PARAM_BOND                 1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                 0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES      BLE_GAP_IO_CAPS_NONE                        /**< Display I/O capabilities. */
#define SEC_PARAM_OOB                  0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE         7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE         16                                          /**< Maximum encryption key size. */

#define TX_POWER_LEVEL                  (4)                                         /**< TX Power Level value. Radio transmit power in dBm (accepted values are -40, -20, -16, -12, -8, -4, 0, and 4 dBm). */

#define FLASH_PAGE_DEMO_DATA       			(PSTORAGE_FLASH_PAGE_END-7)									/**< Flash page used for demo data storage */
#define FLASH_PAGE_SYS_ATTR             (PSTORAGE_FLASH_PAGE_END-3)                 /**< Flash page used for bond manager system attribute information. */
#define FLASH_PAGE_BOND                 (PSTORAGE_FLASH_PAGE_END-1)                 /**< Flash page used for bond manager bonding information. */

#define DEAD_BEEF                      0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_FEATURE_NOT_SUPPORTED      BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

//static ble_gap_sec_params_t            m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                        m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

/********************* TIMER *********************/
#define DATA_TRANSMIT_INTERVAL          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)   /**< Data transmit interval (ticks). */
//static app_timer_id_t                   m_gogglepal_timer_id;                       /**< GogglePal timer. */
 
/********************* APP_UART **********************/

#define APP_UART_RX_FIFO_SIZE						256
#define APP_UART_TX_FIFO_SIZE						256

//static uint8_t sg_uart_rx_buffer[APP_UART_RX_FIFO_SIZE];
//static uint8_t sg_uart_tx_buffer[APP_UART_TX_FIFO_SIZE];
//static app_uart_buffers_t 			sg_uart_buffers    = { sg_uart_rx_buffer,
//																											 APP_UART_RX_FIFO_SIZE,
//																											 sg_uart_tx_buffer,
//																											 APP_UART_TX_FIFO_SIZE};
//static app_uart_comm_params_t 	sg_uart_params 		 = { RX_PIN_NUMBER,
//																											 TX_PIN_NUMBER, 
//																											 RTS_PIN_NUMBER, 
//																											 CTS_PIN_NUMBER, 
//																											 APP_UART_FLOW_CONTROL_DISABLED, 
//																											 false, 
//																											 UART_BAUDRATE_BAUDRATE_Baud115200};
static uint16_t  							  gp_uart_handle_id	 = 0;
static void app_uart_transmit( uint8_t *buf, uint8_t len);
/*********************** END OF APP UART *******************************/

#define SCHED_MAX_EVENT_DATA_SIZE     sizeof(ble_sched_event_t)            /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
//#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, \
//                                            BLE_STACK_HANDLER_SCHED_EVT_SIZE) /**< Maximum size of scheduler events. */

#define SCHED_QUEUE_SIZE                10            	/**< Maximum number of events in the scheduler queue. */
#define SERIAL_IDENTIFIER								0xBE						/**< START CODE to signify the start of a frame over UART */
#define BOOT_INVALID_LENGTH_CRC       	0xFFFFFFFF			/**< signifies illegal length or CRC */

typedef struct BLE_bootloader_s{
		uint32_t firmware_length;
		uint32_t firmware_crc;
		uint32_t firmware_rxbytes;
		uint16_t firmware_rxpackets;
}GogglePal_firmware_t;

typedef struct GogglePal_firmware_hdr_s{
		uint8_t OEMString[8];
		uint32_t MCU_length;
		uint32_t MCU_crc;
		uint32_t BLE_length;
		uint32_t BLE_crc;
		uint8_t VendorSring[8];
}GogglePal_firmware_hdr_t;

static GogglePal_firmware_t								gp_firmware_info = { BOOT_INVALID_LENGTH_CRC, BOOT_INVALID_LENGTH_CRC, BOOT_INVALID_LENGTH_CRC, 0xFFFF };
static uint16_t 													gp_msgCount 						= 0;	
////////// BOOTLOADER END//////////

																						
static ble_gogglepal_t 									m_gogglepal;																/**< Structure used to identify the GogglePal data update service. */
//static ble_bas_t                        m_bas;                                        /**< Structure used to identify the battery service. */
//Variables to store information from the iOS device
static ble_gap_addr_t                   current_addr;
MCU_Send_Again_State_t 									sg_MCU_Send_Again_State = MCU_Send_Again_NONE;
SYNC_Data_Packet_Type_t 								sg_SYNC_Data_Packet_Type = SYNC_Data_Packet_NONE;

uint8_t gogglepal_temp_flash_header[FLASH_HEADER_LEN] 									= {0x00, 0x00, 0x00, 0x00, 0x00, 
																																					 0x00, 0x00, 0x00, 0x00, 0x00, 
																																	         0x00, 0x00, 0x00, 0x00, 0x00, 
																																	         0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t gogglepal_temp_data_buffer[LENGTHOF20] 													= {0x00, 0x00, 0x00, 0x00, 0x00, 
																																	       0x00, 0x00, 0x00, 0x00, 0x00, 
																																	       0x00, 0x00, 0x00, 0x00, 0x00, 
																																	       0x00, 0x00, 0x00, 0x00, 0x00};			

static dm_application_instance_t       m_app_handle;                               /**< Application identifier allocated by device manager. */

static dm_handle_t                     m_dm_handle;                                /**< Device manager's instance handle. */

static ble_uuid_t m_adv_uuids[] = {{ANCS_UUID_SERVICE,									 BLE_UUID_TYPE_VENDOR_BEGIN}
																	 //{GOGGLEPAL_UUID_SERVICE,              BLE_UUID_TYPE_VENDOR_BEGIN},
                                   //{BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
                                   //{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
}; /**< Universally unique service identifiers. */

/* PRIVATE FUNCTIONS */
//static void ble_evt_dispatch(ble_evt_t * p_ble_evt);
//static void sys_evt_dispatch(uint32_t sys_evt);
																	 
static const char * lit_catid[BLE_ANCS_NB_OF_CATEGORY_ID] =
{
    "Other",
    "Incoming Call",
    "Missed Call",
    "Voice Mail",
    "Social",
    "Schedule",
    "Email",
    "News",
    "Health And Fitness",
    "Business And Finance",
    "Location",
    "Entertainment"
};

static const char * lit_eventid[BLE_ANCS_NB_OF_EVT_ID] =
{
    "Added",
    "Modified",
    "Removed"
};

static const char * lit_attrid[BLE_ANCS_NB_OF_ATTRS] =
{
    "App Identifier",
    "Title",
    "Subtitle",
    "Message",
    "Message Size",
    "Date",
    "Positive Action Label",
    "Negative Action Label"
};

static ble_ancs_c_t              m_ancs_c;                                 /**< Structure used to identify the Apple Notification Service Client. */

static ble_db_discovery_t        m_ble_db_discovery;                       /**< Structure used to identify the DB Discovery module. */

static uint8_t                   m_ancs_uuid_type;                         /**< Store ANCS UUID. */
static dm_application_instance_t m_app_handle;                             /**< Application identifier allocated by the Device Manager. */
static dm_handle_t               m_peer_handle;                            /**< Identifies the peer that is currently connected. */
static ble_gap_sec_params_t      m_sec_param;                              /**< Security parameter for use in security requests. */
static app_timer_id_t            m_sec_req_timer_id;                       /**< Security request timer. The timer lets us start pairing request if one does not arrive from the Central. */

static ble_ancs_c_evt_notif_t m_notification_latest;                       /**< Local copy to keep track of the newest arriving notifications. */

static uint8_t m_attr_title[ATTR_DATA_SIZE];                               /**< Buffer to store attribute data. */
static uint8_t m_attr_subtitle[ATTR_DATA_SIZE];                            /**< Buffer to store attribute data. */
static uint8_t m_attr_message[ATTR_DATA_SIZE];                             /**< Buffer to store attribute data. */
static uint8_t m_attr_message_size[ATTR_DATA_SIZE];                        /**< Buffer to store attribute data. */
static uint8_t m_attr_date[ATTR_DATA_SIZE];                                /**< Buffer to store attribute data. */
static uint8_t m_attr_posaction[ATTR_DATA_SIZE];                           /**< Buffer to store attribute data. */
static uint8_t m_attr_negaction[ATTR_DATA_SIZE];                           /**< Buffer to store attribute data. */
																	 
void queueSerialEvent(uint8_t length, uint8_t opCode, uint8_t *data, bool wakeup){
		uint8_t msg_buf[30];
		uint8_t i;

//		if( true == wakeup ){
//			nrf_gpio_pin_clear(MCU_BLE_PIN_NO2);
//			nrf_delay_ms(5);
//			nrf_gpio_pin_set(MCU_BLE_PIN_NO2);
//			nrf_delay_ms(100);			
//		}
		nrf_delay_ms(100);
		memset(msg_buf, 0x00, 30);
		msg_buf[0] = SERIAL_IDENTIFIER;
		msg_buf[1] = length+1;
		msg_buf[2] = opCode;
		
		if(NULL != data){
			memcpy(&msg_buf[3], data, length);
		}
		for(i=1;i<=length+2;i++){
			msg_buf[length+3] += msg_buf[i];
		}
		msg_buf[length+3] = ~msg_buf[length+3];
		app_uart_transmit(msg_buf, length+4);
//		nrf_gpio_pin_clear(MCU_BLE_PIN_NO2);
}

typedef enum mcu_msg_state_e{
	Mcu_msg_sof,
	Mcu_msg_length,
	Mcu_msg_data,
	Mcu_msg_checksum
}mcu_msg_state_t;

static mcu_msg_state_t sg_mcu_msg_state = Mcu_msg_sof;

#define MAX_MSG_LENGTH     120

typedef struct mcu_msg_s{
	/* length of data, excluding opcode */
	uint8_t data_length;
	uint8_t msg_idx;
	uint8_t msg[MAX_MSG_LENGTH];
	uint8_t checksum;
}mcu_msg_t;

mcu_msg_t sg_mcu_msg;

ble_gogglepal_t* ble_gogglepal_get_handle_object(){
	return &m_gogglepal;
}
static void process_mcu_msg(mcu_msg_t *msg){
	uint32_t regret   = 0x00000000;
	uint32_t err_code = NRF_ERROR_FORBIDDEN;
	ble_sched_event_t event;
	memset(&event, 0x00, sizeof(ble_sched_event_t));
	
	//Variables for the BLE Mac Address to MCU
	uint8_t BLE_MAC_ADDRESS_Buffer[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t i,j = 0;
	uint8_t nibble = 0;
	
	if( NULL != msg && msg->data_length > 1){
		event.event_code = msg->msg[0];
		memcpy(event.data, &msg->msg[1], LENGTHOF20);
	}	
	switch (msg->msg[0]) {
			case Mcu_ble_event_FLASH_DATA_CODE:{
				while(err_code != NRF_SUCCESS){
					err_code = app_sched_event_put(&event, sizeof(ble_sched_event_t), ble_gogglepal_flash_data_transmit);
				}
				break;
			}
		case Mcu_ble_event_FLASH_HEADER_CODE: {
				while(err_code != NRF_SUCCESS){
					err_code = app_sched_event_put(&event, sizeof(ble_sched_event_t), ble_gogglepal_flash_header_transmit);
				}
				break;
		}
		case Mcu_ble_event_SCOREBOARD_CODE: {
				while(err_code != NRF_SUCCESS){
					err_code = app_sched_event_put(&event, sizeof(ble_sched_event_t), ble_gogglepal_scoreboard_transmit);
				}
				break;
		}	
		case Mcu_ble_event_start_BATTERY: {
			memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
			while(err_code != NRF_SUCCESS){
				//err_code = app_sched_event_put(&event, sizeof(ble_sched_event_t), send_battery_level);
			}
//			queueSerialEvent(0, Mcu_ble_event_STOP_CMD, NULL, false);
			queueSerialEvent(LENGTHOF20, Mcu_ble_event_STOP_CMD, gogglepal_temp_data_buffer, false);
			break;
		}
//		case Mcu_ble_event_FACTORY_RESET_CODE:
//			regret |= 0x00000002;
		case Mcu_ble_event_HARDWARE_RESET_CODE:
			regret |= 0x00000001;
			sd_softdevice_disable();
			NRF_POWER->GPREGRET = regret;
			NVIC_SystemReset();
			break;		
		case Mcu_ble_event_BLE_MAC_ADDRESS:
			j = 0;
			for (i=3;i>0;i--){
				nibble = (current_addr.addr[i-1]&0xF0) >> 4;
				BLE_MAC_ADDRESS_Buffer[j++] = nibble;
				nibble = (current_addr.addr[i-1]&0x0F);
				BLE_MAC_ADDRESS_Buffer[j++] = nibble;
				}
			queueSerialEvent(LENGTHOF20, Mcu_ble_event_BLE_MAC_ADDRESS, BLE_MAC_ADDRESS_Buffer, true);	
			break;
		case Mcu_ble_event_SHUTDOWN_EVENT:			
			app_uart_close(gp_uart_handle_id);
			sd_power_system_off();
			break;
		default: {
			break;
		}
	}
}

static void process_uart_byte(uint8_t value)
{	
	switch (sg_mcu_msg_state){
		case Mcu_msg_sof:
			if(SERIAL_IDENTIFIER == value){				
				sg_mcu_msg_state = Mcu_msg_length;
				memset(&sg_mcu_msg, 0x00, sizeof(mcu_msg_t));
			}			
			break;
		case Mcu_msg_length:
			sg_mcu_msg.data_length = value;
			sg_mcu_msg_state = (value > 0) && (value <= MAX_MSG_LENGTH) ? Mcu_msg_data:Mcu_msg_sof;
			break;
		case Mcu_msg_data:
			sg_mcu_msg.msg[sg_mcu_msg.msg_idx++] = value;
			sg_mcu_msg.checksum+=value;
			if(sg_mcu_msg.msg_idx == sg_mcu_msg.data_length){
				sg_mcu_msg.checksum = ~sg_mcu_msg.checksum;
				sg_mcu_msg_state = Mcu_msg_checksum;
			}
			break;
		case Mcu_msg_checksum:
			
			if(sg_mcu_msg.checksum == value){
				process_mcu_msg(&sg_mcu_msg);
			}
			sg_mcu_msg_state = Mcu_msg_sof;
			break;
	}	
}

/*****************************************************************************
* Error Handling Functions
*****************************************************************************/
																	 
/**@brief Error handler function, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
		//    						ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}																	 

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
//static void service_error_handler(uint32_t nrf_error)
//{
//    APP_ERROR_HANDLER(nrf_error);
//}

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/**@brief Function for handling the security request timer time-out.
 *
 * @details This function is called each time the security request timer expires.
 *
 * @param[in] p_context  Pointer used for passing context information from the
 *                       app_start_timer() call to the time-out handler.
 */
static void sec_req_timeout_handler(void * p_context)
{
    uint32_t             err_code;
    dm_security_status_t status;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        err_code = dm_security_status_req(&m_peer_handle, &status);
        APP_ERROR_CHECK(err_code);

        // If the link is still not secured by the peer, initiate security procedure.
        if (status == NOT_ENCRYPTED)
        {
            err_code = dm_security_setup_req(&m_peer_handle);
            APP_ERROR_CHECK(err_code);
        }
    }
}

/**@brief Function for setting up GATTC notifications from the Notification Provider.
 *
 * @details This function is called when a successful connection has been established.
 */
static void apple_notification_setup(void)
{
    uint32_t err_code;

    nrf_delay_ms(100); // Delay because we cannot add a CCCD to close to starting encryption. iOS specific.

    err_code = ble_ancs_c_notif_source_notif_enable(&m_ancs_c);
    APP_ERROR_CHECK(err_code);

    err_code = ble_ancs_c_data_source_notif_enable(&m_ancs_c);
    APP_ERROR_CHECK(err_code);

//    printf("Notifications Enabled.\n\r");
}


/**@brief Function for printing an iOS notification.
 *
 * @param[in] p_notif  Pointer to the iOS notification.
 */
static void notif_print(ble_ancs_c_evt_notif_t * p_notif)
{
//    printf("\n\rNotification\n\r");
//    printf("Event:       %s\n", lit_eventid[p_notif->evt_id]);
//    printf("Category ID: %s\n", lit_catid[p_notif->category_id]);
//    printf("Category Cnt:%u\n", (unsigned int) p_notif->category_count);
//    printf("UID:         %u\n\r", (unsigned int) p_notif->notif_uid);
	if((p_notif->category_id == Incoming_Call)&&(p_notif->evt_id != Removed)){
					memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
					queueSerialEvent(LENGTHOF20, Mcu_ble_event_INCOMING_CALL_EVENT, gogglepal_temp_data_buffer, true);
	}
	else if((p_notif->category_id == Social)&&(p_notif->evt_id != Removed)){
					memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
					queueSerialEvent(LENGTHOF20, Mcu_ble_event_SOCIAL_MSG_EVENT, gogglepal_temp_data_buffer, true);
	}
	else if((p_notif->category_id == Email)&&(p_notif->evt_id != Removed)){
					memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
					queueSerialEvent(LENGTHOF20, Mcu_ble_event_EMAIL_EVENT, gogglepal_temp_data_buffer, true);
	}
	else{
	}
		

//    printf("Flags:\n\r");
    if(p_notif->evt_flags.silent == 1)
    {
//        printf(" Silent\n\r");
    }
    if(p_notif->evt_flags.important == 1)
    {
//        printf(" Important\n\r");
    }
    if(p_notif->evt_flags.pre_existing == 1)
    {
//        printf(" Pre-existing\n\r");
    }
    if(p_notif->evt_flags.positive_action == 1)
    {
//        printf(" Positive Action\n\r");
    }
    if(p_notif->evt_flags.negative_action == 1)
    {
//        printf(" Positive Action\n\r");
    }
}


/**@brief Function for printing iOS notification attribute data.
 * 
 * @param[in] p_attr         Pointer to an iOS notification attribute.
 * @param[in] ancs_attr_list Pointer to a list of attributes. Each entry in the list stores 
                             a pointer to its attribute data, which is to be printed.
 */
static void notif_attr_print(ble_ancs_c_evt_notif_attr_t * p_attr,
                             ble_ancs_c_attr_list_t      * ancs_attr_list)
{
    if (p_attr->attr_len != 0)
    {
//        printf("%s: %s\n\r",
//               lit_attrid[p_attr->attr_id],
//               ancs_attr_list[p_attr->attr_id].p_attr_data);
    }
    else if (p_attr->attr_len == 0)
    {
//        printf("%s: (N/A)\n\r", lit_attrid[p_attr->attr_id]);
    }
}

//static void gogglepal_pstorage_cb_handler(pstorage_handle_t * handle,
//                                   uint8_t             op_code,
//                                   uint32_t            result,
//                                   uint8_t           * p_data,
//                                   uint32_t            data_len)
//{
//    if (result != NRF_SUCCESS)
//    {
//        APP_ERROR_HANDLER(result);
//    }
//}

/**@brief Bond Manager module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
//static void bond_manager_error_handler(uint32_t nrf_error)
//{
//    APP_ERROR_HANDLER(nrf_error);
//}

//static void send_scoreboard_data(void)
//{
//		ble_sched_event_t event;
//    uint32_t err_code;

//		memset(&event, 0x00, sizeof(ble_sched_event_t));	
//	  memset(gogglepal_temp_data_buffer, 0x00, sizeof(gogglepal_temp_data_buffer));

//	  memcpy(event.data, gogglepal_temp_data_buffer, LENGTHOF20);

//		while(err_code != NRF_SUCCESS){
//			err_code = app_sched_event_put(&event, sizeof(ble_sched_event_t), ble_gogglepal_scoreboard_transmit);
//		}
//}

//static void gogglepal_timeout_handler(void * p_context)
//{
//		ble_sched_event_t event;
//    uint32_t err_code;
	
//	  UNUSED_PARAMETER(p_context);
//}

/*****************************************************************************
* Static Initialization Functions
*****************************************************************************/

/**@brief Battery measurement Update.
 *
 * @details This function will be called each time the battery level measurement is updated.
 *
 */
//static void send_battery_level(void * p_event_data, uint16_t event_size)
//{
//	ble_sched_event_t *batt_data = (ble_sched_event_t*)p_event_data;
//	if(NULL != batt_data){
//		if(Mcu_ble_event_start_BATTERY == batt_data->event_code){
//			ble_bas_battery_level_update(&m_bas, batt_data->data[0]);
//		}
//	}
//}

/**@brief Function for the Timer initialization.
 *
* @details Initializes the timer module. This creates and starts application timers.
*/
static void gogglepal_timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
//    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

//    // Create timers.
//    err_code = app_timer_create(&m_gogglepal_timer_id,
//                                APP_TIMER_MODE_REPEATED,
//                                gogglepal_timeout_handler);
//    APP_ERROR_CHECK(err_code);
	
	  // Initialize timer module, making it use the scheduler.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);

    // Create security request timer.
    err_code = app_timer_create(&m_sec_req_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                sec_req_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Apple Notification Service client.
 *
 * @details This function is called for all events in the Apple Notification client that
 *          are passed to the application.
 *
 * @param[in] p_evt  Event received from the Apple Notification Service client.
 */
static void on_ancs_c_evt(ble_ancs_c_evt_t * p_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_evt->evt_type)
    {
        case BLE_ANCS_C_EVT_DISCOVER_COMPLETE:
//            printf("Apple Notification Service discovered on the server.\n");
            apple_notification_setup();
            break;

        case BLE_ANCS_C_EVT_NOTIF:
            m_notification_latest = p_evt->notif;
            notif_print(&m_notification_latest);	
            break;

        case BLE_ANCS_C_EVT_NOTIF_ATTRIBUTE:
            notif_attr_print(&p_evt->attr, p_evt->ancs_attr_list);
            break;

        case BLE_ANCS_C_EVT_DISCOVER_FAILED:
            // ANCS not found.
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
//                err_code = sd_ble_gap_disconnect(m_conn_handle,
//                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
//            printf("Apple Notification Service not discovered on the server.\n");
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
																					
		err_code = sd_ble_gap_tx_power_set(TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Apple Notification Service client errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void apple_notification_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void user_profile_write_handler(ble_gogglepal_t * p_gogglepal, uint8_t * new_data)
{	
	update_user_profile(new_data);
	
	queueSerialEvent(USER_PROFILE_LEN, Mcu_ble_event_PROFILE, new_data, true );
}


static void hdsk_cmd_write_handler(ble_gogglepal_t * p_gogglepal, uint8_t * new_data)
{
	
	  uint8_t checkSum = 0;
	  uint8_t i;
		for( i=1; i<4; i++ ) {
			checkSum += new_data[i];
		}
		checkSum = ~checkSum;
		if( (new_data[4] == checkSum) && (new_data[0] == SERIAL_IDENTIFIER)){
			switch(new_data[2]){
				case Ble_OTA_event_SYNC_START:
					set_sync_state(true);
//					queueSerialEvent(0, Mcu_ble_event_start_SYNC, NULL, true);
					memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
					queueSerialEvent(LENGTHOF20, Mcu_ble_event_start_SYNC, gogglepal_temp_data_buffer, true);
				break;
				case Ble_OTA_event_SYNC_STOP:
					set_sync_state(false);
//					queueSerialEvent(0, Mcu_ble_event_stop_SYNC, NULL, true);
					memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
					queueSerialEvent(LENGTHOF20, Mcu_ble_event_stop_SYNC, gogglepal_temp_data_buffer, true);
//					NVIC_SystemReset();
				break;
				case Ble_OTA_event_SCOREBOARD:
				  //send_scoreboard_data();
//					queueSerialEvent(0, Mcu_ble_event_start_SCOREBOARD, NULL, true);
					memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
					queueSerialEvent(LENGTHOF20, Mcu_ble_event_start_SCOREBOARD, gogglepal_temp_data_buffer, true);
				break;
				case Ble_OTA_event_FLASH_HEADER:
					gogglepal_temp_flash_header[0] = new_data[3];
					queueSerialEvent(FLASH_HEADER_LEN, Mcu_ble_event_start_FLASH_HEADER, gogglepal_temp_flash_header, true);
				break;
				case Ble_OTA_event_ERASE_FLASH:
//					queueSerialEvent(0, Mcu_ble_event_erase_FLASH, NULL, true);
					memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
					queueSerialEvent(LENGTHOF20, Mcu_ble_event_erase_FLASH, gogglepal_temp_data_buffer, true);
				break;
				case Ble_OTA_event_BATTERY:
//					queueSerialEvent(0, Mcu_ble_event_start_BATTERY, NULL, true);
					memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
					queueSerialEvent(LENGTHOF20, Mcu_ble_event_start_BATTERY, gogglepal_temp_data_buffer, true);
				break;
				case Ble_OTA_event_SHIPPING_MODE:
					memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
					queueSerialEvent(LENGTHOF20, Mcu_ble_event_SHIPPING_MODE_EVENT, gogglepal_temp_data_buffer, true);
				break;
				case Ble_OTA_event_INCOMING_CALL:
					memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
					queueSerialEvent(LENGTHOF20, Mcu_ble_event_INCOMING_CALL_EVENT, gogglepal_temp_data_buffer, true);
				break;
				case Ble_OTA_event_SOCIAL_MSG:
					memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
					queueSerialEvent(LENGTHOF20, Mcu_ble_event_SOCIAL_MSG_EVENT, gogglepal_temp_data_buffer, true);
				break;
				case Ble_OTA_event_EMAIL:
					memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
					queueSerialEvent(LENGTHOF20, Mcu_ble_event_EMAIL_EVENT, gogglepal_temp_data_buffer, true);
				break;
				case Ble_OTA_event_BOOTLOADER:
					/* reset BOOTLOADER OPTIONS */
					gp_firmware_info.firmware_length = ((new_data[3]<<24)|(new_data[4]<<16)|(new_data[5]<<8)|(new_data[6]));
					gp_firmware_info.firmware_crc		 = ((new_data[7]<<24)|(new_data[8]<<16)|(new_data[9]<<8)|(new_data[10]));
					gp_firmware_info.firmware_rxbytes = 0;
					gp_firmware_info.firmware_rxpackets = (gp_firmware_info.firmware_length/20);
					gp_firmware_info.firmware_rxpackets += ((gp_firmware_info.firmware_length%20) ? 1:0);
					gp_msgCount = 0;
					uint8_t msg_buf[20];
					memset(msg_buf, 0x00, 20);
					memcpy(msg_buf, &gp_firmware_info.firmware_length, sizeof(uint32_t));
					memcpy(msg_buf+sizeof(uint32_t), &gp_firmware_info.firmware_crc, sizeof(uint32_t));
//					queueSerialEvent(8, Mcu_ble_event_bootloader_cmd, msg_buf, true);
					queueSerialEvent(LENGTHOF20, Mcu_ble_event_bootloader_cmd, msg_buf, true);
				break;		

			}
		}
}


static void bootloader_write_handler(ble_gogglepal_t * p_gogglepal, uint8_t * new_data)
{
		/* check that we have handshaked with phone before accepting and writing image to flash */
		if( (gp_firmware_info.firmware_length != BOOT_INVALID_LENGTH_CRC) 
		 && (gp_firmware_info.firmware_crc != BOOT_INVALID_LENGTH_CRC) 
		 && gp_msgCount <= gp_firmware_info.firmware_rxpackets ){
			/* message was expected, check if it's within the expected packet range */
			queueSerialEvent(20, Mcu_ble_event_bootloader_data, new_data, false);
			/* this msg is expected */
			gp_msgCount++;
		}
		else{
			gp_firmware_info.firmware_length = BOOT_INVALID_LENGTH_CRC;
			gp_firmware_info.firmware_crc    = BOOT_INVALID_LENGTH_CRC;
			gp_msgCount = 0;
		}
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Glucose, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_gogglepal_init_t gogglepal_init;
    ble_dis_init_t dis_init;
//    ble_bas_init_t bas_init;
	
		// Initialize ANCS Service.	
///////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	  ble_ancs_c_init_t ancs_init_obj;
    ble_uuid_t        service_uuid;
	
	  err_code = sd_ble_uuid_vs_add(&ble_ancs_base_uuid128, &m_ancs_uuid_type);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_uuid_vs_add(&ble_ancs_cp_base_uuid128, &service_uuid.type);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_uuid_vs_add(&ble_ancs_ns_base_uuid128, &service_uuid.type);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_uuid_vs_add(&ble_ancs_ds_base_uuid128, &service_uuid.type);
    APP_ERROR_CHECK(err_code);

    memset(&ancs_init_obj, 0, sizeof(ancs_init_obj));

    err_code = ble_ancs_c_attr_add(BLE_ANCS_NOTIF_ATTR_ID_TITLE, m_attr_title, ATTR_DATA_SIZE);
    APP_ERROR_CHECK(err_code);
    
    err_code = ble_ancs_c_attr_add(BLE_ANCS_NOTIF_ATTR_ID_SUBTITLE,
                                   m_attr_subtitle,
                                   ATTR_DATA_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = ble_ancs_c_attr_add(BLE_ANCS_NOTIF_ATTR_ID_MESSAGE, m_attr_message, ATTR_DATA_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = ble_ancs_c_attr_add(BLE_ANCS_NOTIF_ATTR_ID_MESSAGE_SIZE,
                                   m_attr_message_size,
                                   ATTR_DATA_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = ble_ancs_c_attr_add(BLE_ANCS_NOTIF_ATTR_ID_DATE, m_attr_date, ATTR_DATA_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = ble_ancs_c_attr_add(BLE_ANCS_NOTIF_ATTR_ID_POSITIVE_ACTION_LABEL,
                                   m_attr_posaction,
                                   ATTR_DATA_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = ble_ancs_c_attr_add(BLE_ANCS_NOTIF_ATTR_ID_NEGATIVE_ACTION_LABEL,
                                   m_attr_negaction,
                                   ATTR_DATA_SIZE);
    APP_ERROR_CHECK(err_code);

    ancs_init_obj.evt_handler   = on_ancs_c_evt;
    ancs_init_obj.error_handler = apple_notification_error_handler;

    err_code = ble_ancs_c_init(&m_ancs_c, &ancs_init_obj);
    APP_ERROR_CHECK(err_code);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////	

    // Initialize Battery Service.
//    memset(&bas_init, 0, sizeof(bas_init));

//    // Here the sec level for the Battery Service can be changed/increased.
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

//    bas_init.evt_handler          = NULL;
//    bas_init.support_notification = true;
//    bas_init.p_report_ref         = NULL;
//    bas_init.initial_batt_level   = 100;

//    err_code = ble_bas_init(&m_bas, &bas_init);
//    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, MODEL_NUMBER);
		

		uint8_t nibble = 0; // high and low nibble
		uint8_t i,j = 0;
		char device_addr[13];
		
		memset(device_addr, 0x00, sizeof(device_addr));
		for (i=6;i>0;i--){
			nibble = (current_addr.addr[i-1]&0xF0) >> 4;
			device_addr[j++] = ((nibble<=9)?(nibble+0x30):(nibble+0x37));
			nibble = (current_addr.addr[i-1]&0x0F);
			device_addr[j++] = ((nibble<=9)?(nibble+0x30):(nibble+0x37));
		}
		ble_srv_ascii_to_utf8(&dis_init.serial_num_str, device_addr);
			
		
//		ble_srv_ascii_to_utf8(&dis_init.serial_num_str, SERIAL_NUMBER);
		ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, FIRMWARE_REVISION);
		ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, HARDWARE_REVISION);
		ble_srv_ascii_to_utf8(&dis_init.sw_rev_str, SOFTWARE_REVISION);

    ble_dis_sys_id_t system_id;
    system_id.manufacturer_id            = MANUFACTURER_ID;
    system_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id                    = &system_id;
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
		
		// Initialize GogglePal Service
    memset(&gogglepal_init, 0, sizeof(gogglepal_init));
	
		gogglepal_init.user_profile_write_handler = user_profile_write_handler;
	  gogglepal_init.hdsk_cmd_write_handler = hdsk_cmd_write_handler;
		gogglepal_init.bootloader_write_handler = bootloader_write_handler;
	
    err_code = ble_gogglepal_init(&m_gogglepal, &gogglepal_init);
    APP_ERROR_CHECK(err_code);
		
}


/**@brief Function for starting application timers.
 */
//static void application_timers_start(void)
//{
//    uint32_t err_code;

//    // Start application timers
//    err_code = app_timer_start(m_gogglepal_timer_id, DATA_TRANSMIT_INTERVAL, NULL);
//    APP_ERROR_CHECK(err_code);
//}

/**@brief Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Function for initializing the database discovery module.
 */
static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Initialize security parameters.
 */
//static void sec_params_init(void)
//{
//    m_sec_params.bond         = SEC_PARAM_BOND;
//    m_sec_params.mitm         = SEC_PARAM_MITM;
//    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
//    m_sec_params.oob          = SEC_PARAM_OOB;  
//    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
//    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
//}


/**@brief Function for handling the Connection Parameter events.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail configuration parameter, but instead we use the
 *                event handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAM_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code;

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_DIRECTED:
            break;

        case BLE_ADV_EVT_FAST:
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            break;

        case BLE_ADV_EVT_SLOW:
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_whitelist_t whitelist;
            ble_gap_addr_t    * p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t     * p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];

            whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            whitelist.irk_count  = BLE_GAP_WHITELIST_IRK_MAX_COUNT;
            whitelist.pp_addrs   = p_whitelist_addr;
            whitelist.pp_irks    = p_whitelist_irk;

            err_code = dm_whitelist_create(&m_app_handle, &whitelist);
            APP_ERROR_CHECK(err_code);

            err_code = ble_advertising_whitelist_reply(&whitelist);
            APP_ERROR_CHECK(err_code);
            break;
        }

        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;
    //ble_gatts_rw_authorize_reply_params_t auth_reply;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
					  //application_timers_start();
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
				
            break;
				
				case BLE_GAP_EVT_DISCONNECTED:
//					  queueSerialEvent(0, Mcu_ble_event_BLE_DISCONNECT, NULL, true);
						memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
						queueSerialEvent(LENGTHOF20, Mcu_ble_event_BLE_DISCONNECT, gogglepal_temp_data_buffer, true);
				 
            m_conn_handle = BLE_CONN_HANDLE_INVALID;		
				
//            err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
//            APP_ERROR_CHECK(err_code);
				
						gp_firmware_info.firmware_length = BOOT_INVALID_LENGTH_CRC;
						gp_firmware_info.firmware_crc    = BOOT_INVALID_LENGTH_CRC;
						gp_msgCount = 0;
				
            break;   
				
				case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server and Client time-out events.
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
				
				case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
//            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
//            APP_ERROR_CHECK(err_code);
            break;
				
        case BLE_EVT_USER_MEM_REQUEST:
            //err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
//            if(p_ble_evt->evt.gatts_evt.params.authorize_request.type
//               != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
//            {
//                if ((p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
//                     == BLE_GATTS_OP_PREP_WRITE_REQ)
//                    || (p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
//                     == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
//                    || (p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
//                     == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
//                {
//                    if (p_ble_evt->evt.gatts_evt.params.authorize_request.type
//                        == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
//                    {
//                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
//                    }
//                    else
//                    {
//                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
//                    }
//                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
//                    err_code = sd_ble_gatts_rw_authorize_reply(m_conn_handle,&auth_reply);
//                    APP_ERROR_CHECK(err_code);
//                }
//            }
            break;
						
				case BLE_EVT_TX_COMPLETE:
					  switch(sg_MCU_Send_Again_State){
							case MCU_Send_Again_FLASH:
//								queueSerialEvent(0, Mcu_ble_event_again_SYNC, NULL, false);
								memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
								queueSerialEvent(LENGTHOF20, Mcu_ble_event_again_SYNC, gogglepal_temp_data_buffer, false);
							  sg_MCU_Send_Again_State = MCU_Send_Again_NONE;
								break;
							case MCU_Send_Again_SCOREBOARD:
//								queueSerialEvent(0, Mcu_ble_event_again_SCOREBOARD, NULL, false);
								memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
								queueSerialEvent(LENGTHOF20, Mcu_ble_event_again_SCOREBOARD, gogglepal_temp_data_buffer, false);
							  sg_MCU_Send_Again_State = MCU_Send_Again_NONE;
								break;
							case MCU_Send_Again_FLASH_HEADER:
//								queueSerialEvent(0, Mcu_ble_event_again_FLASH_HEADER, NULL, false);
								memset(gogglepal_temp_data_buffer, 0x00, LENGTHOF20);
								queueSerialEvent(LENGTHOF20, Mcu_ble_event_again_FLASH_HEADER, gogglepal_temp_data_buffer, false);
							  sg_MCU_Send_Again_State = MCU_Send_Again_NONE;
								break;
							case MCU_Send_Again_NONE:
								break;
						}
						break;

        case BLE_GAP_EVT_AUTH_STATUS:
        case BLE_GAP_EVT_CONN_SEC_UPDATE:
            break;
				
				case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
//            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
//            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
	  ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
	  ble_conn_params_on_ble_evt(p_ble_evt);
	  ble_ancs_c_on_ble_evt(&m_ancs_c, p_ble_evt);
    ble_gogglepal_on_ble_evt(&m_gogglepal, p_ble_evt);
	  on_ble_evt(p_ble_evt);
	  ble_advertising_on_ble_evt(p_ble_evt);
//    ble_bas_on_ble_evt(&m_bas, p_ble_evt);

}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));

    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_evt,
                                           ret_code_t          event_result)
{
    uint32_t err_code;

    m_dm_handle = *p_handle;
    APP_ERROR_CHECK(event_result);
	
	  ble_ancs_c_on_device_manager_evt(&m_ancs_c, p_handle, p_evt);
	
    switch (p_evt->event_id)
    {
        case DM_EVT_CONNECTION:
            // Start Security Request timer.
            if (m_dm_handle.device_id != DM_INVALID_ID)
            {
            }
						
						m_peer_handle = (*p_handle);
            err_code      = app_timer_start(m_sec_req_timer_id, SECURITY_REQUEST_DELAY, NULL);
            APP_ERROR_CHECK(err_code);
            break;
						
				case DM_EVT_LINK_SECURED:
            err_code = ble_db_discovery_start(&m_ble_db_discovery,
                                              p_evt->event_param.p_gap_param->conn_handle);
            APP_ERROR_CHECK(err_code);
            break; 
										
        default:
            break;
    }
    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    register_param.sec_param.mitm    = SEC_PARAM_MITM;
    register_param.sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;

		memcpy(&m_sec_param, &register_param.sec_param, sizeof(ble_gap_sec_params_t));

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
	  //ble_advdata_t scanrsp;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_SHORT_NAME;
    advdata.include_appearance      = false;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;;
    //advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    //advdata.uuids_complete.p_uuids  = m_adv_uuids;
	  advdata.uuids_complete.uuid_cnt  = 0;
    advdata.uuids_complete.p_uuids   = NULL;
//    advdata.uuids_solicited.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
//    advdata.uuids_solicited.p_uuids  = m_adv_uuids;
	
	  //memset(&scanrsp, 0, sizeof(scanrsp));
    //scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    //scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
		options.ble_adv_whitelist_enabled = BLE_ADV_WHITELIST_ENABLED;
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
		//err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

static void uart_evt_handler(app_uart_evt_t * p_evt)
{
    uint8_t value;
    uint32_t err_code;
    switch (p_evt->evt_type)
    {
        case APP_UART_DATA_READY:
            err_code = app_uart_get(&value);
            APP_ERROR_CHECK(err_code);
						process_uart_byte(value);
            break;
				
				case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_evt->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_evt->data.error_code);
            break;
				
        default:
            break;
    }
}

/**@brief Function for initializing the UART.
 */
static void uart_init(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       //uart_error_handle,
											 uart_evt_handler,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
}


static void app_uart_transmit( uint8_t *buf, uint8_t len){
//	uint32_t err_code = NRF_SUCCESS;
	
	uint_fast8_t i = 0;
	for(i=0;i<4;i++){
		app_uart_put(0xA5);
//		err_code = app_uart_put(0xA5);
//		if (err_code == NRF_ERROR_NO_MEM) {
//			NVIC_SystemReset();
//		}
	}
	
	i = 0;
	while (i < len) {
		app_uart_put(buf[i++]);
//    err_code = app_uart_put(buf[i++]);
//		if (err_code == NRF_ERROR_NO_MEM) {
//			NVIC_SystemReset();
//		}
	}	
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


//uint32_t err_code = 0;

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds = false;
	
		uint32_t regret = NRF_POWER->GPREGRET;
		NRF_POWER->GPREGRET = 0x00000000;

    // Initialize.
    app_trace_init();
		
		gogglepal_timers_init();
    uart_init();
		ble_stack_init();
		APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
	  //device_manager_init(erase_bonds);
		
//		if((regret&0x02)>>1){
//			sd_flash_page_erase(FLASH_PAGE_DEMO_DATA);
//		}
		if(regret&0x01){
//			sd_flash_page_erase(FLASH_PAGE_SYS_ATTR);
//			sd_flash_page_erase(FLASH_PAGE_BOND);
			erase_bonds = true;
		}
		device_manager_init(erase_bonds);
		
		nrf_gpio_cfg_output(MCU_BLE_PIN_NO2);
		nrf_gpio_pin_clear(MCU_BLE_PIN_NO2);
		
		db_discovery_init();
		scheduler_init();
		gap_params_init();
		sd_ble_gap_address_get(&current_addr);
		services_init();
		/* set up advertising parameters */
		advertising_init();
		conn_params_init();

		/* start advertising */
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
		// Enter main loop
		for (;;){
			app_sched_execute();
			power_manage();
		}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// Useful Functions ///////////////////////////////////////////////////
//        case BSP_EVENT_DISCONNECT:
//            err_code = sd_ble_gap_disconnect(m_conn_handle,
//                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//            if (err_code != NRF_ERROR_INVALID_STATE)
//            {
//                APP_ERROR_CHECK(err_code);
//            }
//            break;

//        case BSP_EVENT_WHITELIST_OFF:
//            err_code = ble_advertising_restart_without_whitelist();
//            if (err_code != NRF_ERROR_INVALID_STATE)
//            {
//                APP_ERROR_CHECK(err_code);
//            }
//            break;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @}
 */
