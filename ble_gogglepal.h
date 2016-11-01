/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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
 * @defgroup ble_sdk_srv_gogglepal GogglePal Data Update Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief GogglePal Data Update Service module.
 *
 * @details This module implements the GogglePal Data Update Service.
 *
 *
 * @note The application must propagate BLE stack events to the GogglePal Data Update Service module by calling
 *       ble_gogglepal_on_ble_evt() from the from the @ref ble_stack_handler callback.
 */

#ifndef BLE_GOGGLEPAL_H__
#define BLE_GOGGLEPAL_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_date_time.h"

#define GOGGLEPAL_UUID_BASE {0x60, 0x60, 0xD0, 0x74, 0xB0, 0x3F, 0x69, 0x96, 0x41, 0x40, 0x45, 0x58, 0x00, 0x00, 0xD9, 0x99}
#define GOGGLEPAL_UUID_SERVICE 															0xBEE0

#define GOGGLEPAL_UUID_SCOREBOARD_CHAR 											0xBEE1
#define GOGGLEPAL_UUID_FLASH_DATA_CHAR 											0xBEE2
#define GOGGLEPAL_UUID_FLASH_HEADER_CHAR										0xBEE3

#define GOGGLEPAL_UUID_USER_PROFILE_CHAR 										0xBEE4
#define GOGGLEPAL_UUID_HDSK_CMD_CHAR												0xBEE5
#define GOGGLEPAL_UUID_BOOTLOADER_CHAR											0xBEE6

#define USER_PROFILE_LEN																	20
#define HDSK_CMD_LEN																			5
#define BOOTLOADER_LEN     																20
#define LENGTHOF20																				20
#define FLASH_HEADER_LEN																	20

extern uint8_t gogglepal_temp_flash_header[FLASH_HEADER_LEN]; 
extern bool    gogglepal_sync_started;

typedef enum BLE_ANCS_CATEGORY_ID_e{
    Other = 0x00,
    Incoming_Call,
    Missed_Call,
    Voice_Mail,
    Social,
    Schedule,
    Email,
    News,
    Health_And_Fitness,
    Business_And_Finance,
    Location,
    Entertainment
}BLE_ANCS_CATEGORY_ID_t;

typedef enum BLE_ANCS_EVT_ID_e{
    Added = 0x00,
    Modified,
    Removed
}BLE_ANCS_EVT_ID_t;

typedef enum Ble_OTA_events_e{
	Ble_OTA_event_SYNC_START				= 0x01,
	Ble_OTA_event_SYNC_STOP					= 0x02,
	Ble_OTA_event_ERASE_FLASH				= 0x03,
	Ble_OTA_event_BATTERY						= 0x04,
	Ble_OTA_event_FLASH_HEADER			= 0x05,
	Ble_OTA_event_SCOREBOARD				= 0x06,	
	Ble_OTA_event_SHIPPING_MODE			= 0x07,
	Ble_OTA_event_INCOMING_CALL			= 0x08,
	Ble_OTA_event_SOCIAL_MSG				= 0x09,
	Ble_OTA_event_EMAIL							= 0x0A,
	Ble_OTA_event_BOOTLOADER				= 0xFF
}Ble_OTA_events_t;

typedef enum Mcu_ble_events_e{
	
	/* MCU <-> BLE FLASH EVENTS */
	Mcu_ble_event_start_SYNC				= 0x00,
	Mcu_ble_event_new_SYNC,
	Mcu_ble_event_again_SYNC,
	Mcu_ble_event_stop_SYNC,
	/* MCU <-> BLE FLASH HEADER EVENTS */
	Mcu_ble_event_start_FLASH_HEADER  = 0x10,
	Mcu_ble_event_new_FLASH_HEADER		= 0x11,
	Mcu_ble_event_again_FLASH_HEADER	= 0x12,
	Mcu_ble_event_stop_FLASH_HEADER		= 0x13,
	Mcu_ble_event_erase_FLASH         = 0x14,
	/* MCU <-> BLE SCOREBOARD EVENTS */
	Mcu_ble_event_start_SCOREBOARD  = 0x20,
	Mcu_ble_event_new_SCOREBOARD		= 0x21,
	Mcu_ble_event_again_SCOREBOARD	= 0x22,
	Mcu_ble_event_stop_SCOREBOARD		= 0x23,
	/**/
	Mcu_ble_event_start_BATTERY				= 0x30,
	Mcu_ble_event_PROFILE							= 0x31,
	Mcu_ble_event_FLASH_DATA_CODE			= 0x32,
	Mcu_ble_event_FLASH_HEADER_CODE		= 0x33,
	Mcu_ble_event_SCOREBOARD_CODE			= 0x34,
	Mcu_ble_event_FACTORY_RESET_CODE	= 0x35,
	Mcu_ble_event_HARDWARE_RESET_CODE	= 0x36,
	Mcu_ble_event_BLE_DISCONNECT      = 0x37,
	Mcu_ble_event_BLE_MAC_ADDRESS			= 0x38,
	Mcu_ble_event_STOP_CMD            = 0x39,
	Mcu_ble_event_bootloader_cmd			= 0x3A,
	Mcu_ble_event_bootloader_data			= 0x3B,
	Mcu_ble_event_SHUTDOWN_EVENT			= 0x40,
	Mcu_ble_event_INCOMING_CALL_EVENT	= 0x41,
	Mcu_ble_event_SOCIAL_MSG_EVENT	  = 0x42,
	Mcu_ble_event_EMAIL_EVENT					= 0x43,
	Mcu_ble_event_SHIPPING_MODE_EVENT	= 0x44
}Mcu_ble_events_t;

/* MCU Send Again Packet State*/
typedef enum MCU_Send_Again_State_e{
	MCU_Send_Again_FLASH													= 0x01,
	MCU_Send_Again_FLASH_HEADER										= 0x02,
	MCU_Send_Again_SCOREBOARD										  = 0x03,
	MCU_Send_Again_NONE														= 0xFF
}MCU_Send_Again_State_t;

/* SYNC Data Packet Type*/
typedef enum SYNC_Data_Packet_Type_e{
	SYNC_Data_Packet_1													= 0x01,
	SYNC_Data_Packet_2													= 0x02,
	SYNC_Data_Packet_3										 			= 0x03,
	SYNC_Data_Packet_NONE												= 0xFF
}SYNC_Data_Packet_Type_t;

extern MCU_Send_Again_State_t sg_MCU_Send_Again_State;
extern SYNC_Data_Packet_Type_t SYNC_Data_Packet_Type;

/**@brief GogglePal Service event type. */
typedef enum
{
    BLE_GOGGLEPAL_EVT_NOTIFICATION_ENABLED,                          /**< GogglePal value notification enabled event. */
    BLE_GOGGLEPAL_EVT_NOTIFICATION_DISABLED                          /**< GogglePal value notification disabled event. */
} ble_gogglepal_evt_type_t;

/**@brief GogglePal Service event. */
typedef struct
{
    ble_gogglepal_evt_type_t evt_type;                               /**< Type of event. */
} ble_gogglepal_evt_t;

// Forward declaration of the ble_gogglepal_t type. 
typedef struct ble_gogglepal_s ble_gogglepal_t;

/**@brief GogglePal Service event handler type. */
typedef void (*ble_gogglepal_evt_handler_t) (ble_gogglepal_t * p_gogglepal, ble_gogglepal_evt_t * p_evt);

/**@brief GogglePal Data Update Service event handler type. */
typedef void (*ble_gogglepal_user_profile_write_handler_t) (ble_gogglepal_t * p_gogglepal, uint8_t * new_data);
typedef void (*ble_gogglepal_hdsk_cmd_write_handler_t)     (ble_gogglepal_t * p_gogglepal, uint8_t * new_data);
typedef void (*ble_gogglepal_bootloader_write_handler_t) 	 (ble_gogglepal_t * p_gogglepal, uint8_t * new_data);

/**@brief GogglePal Data Update Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
		ble_gogglepal_user_profile_write_handler_t         user_profile_write_handler;
	  ble_gogglepal_hdsk_cmd_write_handler_t						 hdsk_cmd_write_handler;
	  ble_gogglepal_bootloader_write_handler_t						bootloader_write_handler;
} ble_gogglepal_init_t;

//typedef struct
//{
//    ble_gogglepal_evt_handler_t     evt_handler;                     /**< Event handler to be called for handling events in the GogglePal Service. */
//    ble_srv_error_handler_t         error_handler;                   /**< Function to be called in case of an error. */
//} ble_gogglepal_init_t;


/**@brief GogglePal Data Update Service structure. This contains various status information for the service. */
typedef struct ble_gogglepal_s{         
    uint16_t                     								service_handle;    
	  ble_gatts_char_handles_t     								scoreboard_char_handles;
    ble_gatts_char_handles_t     								flash_data_char_handles;
		ble_gatts_char_handles_t     								flash_header_char_handles;      
    ble_gatts_char_handles_t     								user_profile_char_handles;
    ble_gatts_char_handles_t     								hdsk_cmd_char_handles;		
	  ble_gatts_char_handles_t     								bootloader_char_handles;
    uint8_t                      								uuid_type;         
    uint16_t                     								conn_handle;  
    bool                     										is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
	  ble_gogglepal_user_profile_write_handler_t        user_profile_write_handler;
		ble_gogglepal_hdsk_cmd_write_handler_t        		hdsk_cmd_write_handler; 
	  ble_gogglepal_bootloader_write_handler_t        	bootloader_write_handler;
} ble_gogglepal_t;

//struct ble_gogglepal_s
//{
//    ble_gogglepal_evt_handler_t     evt_handler;                     /**< Event handler to be called for handling events in the Glucose Service. */
//    ble_srv_error_handler_t   			error_handler;                   /**< Function to be called in case of an error. */
//    uint16_t                  			service_handle;                  /**< Handle of GogglePal Service (as provided by the BLE stack). */
//    ble_gatts_char_handles_t  			scoreboard_char_handles;         
//    ble_gatts_char_handles_t  			flash_data_char_handles;           
//	  ble_gatts_char_handles_t  			flash_header_char_handles;         
//    ble_gatts_char_handles_t  			user_profile_char_handles;                    
//    ble_gatts_char_handles_t  			hdsk_cmd_char_handles;                    
//    uint16_t                  			conn_handle;                     /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
//};

typedef struct ble_sched_event_s{
	uint32_t event_code;
	uint8_t data[20];
}ble_sched_event_t;

/**@brief Initialize the GogglePal Data Update Service.
 *
 * @param[out]  p_gogglepal       
 * @param[in]   p_gogglepal_init  
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_gogglepal_init(ble_gogglepal_t * p_gogglepal, const ble_gogglepal_init_t * p_gogglepal_init);

/**@brief Sends sync data if notification has been enabled.
 *
 * @details The application calls this function after GogglePal data update timer expires.
 *          If notification has been enabled, the data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_gogglepal              GogglePal Data Update Service structure.
 * @param[in]   data			               New data.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
void ble_gogglepal_scoreboard_transmit(void *p_event_data, uint16_t event_size);
void ble_gogglepal_flash_data_transmit(void *p_event_data, uint16_t event_size);
void ble_gogglepal_flash_header_transmit(void *p_event_data, uint16_t event_size);
void queueSerialEvent(uint8_t length, uint8_t opCode, uint8_t *data, bool wakeup);
void update_user_profile(uint8_t *profile);
ble_gogglepal_t* ble_gogglepal_get_handle_object(void);
void set_sync_state(bool state);

/**@brief GogglePal Data Update Service BLE stack event handler.
 *
 * @details Handles all events from the BLE stack of interest to the GogglePal Data Update Service.
 *
 * @param[in]   p_gogglepal      GogglePal Data Update Service structure.
 * @param[in]   p_ble_evt  			 Event received from the BLE stack.
 */
void ble_gogglepal_on_ble_evt(ble_gogglepal_t * p_gogglepal, ble_evt_t * p_ble_evt);


#endif // BLE_GOGGLEPAL_H__

/** @} */
