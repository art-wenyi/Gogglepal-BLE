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

#include "ble_gogglepal.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "nrf_delay.h"

#define BLE_SEND_RETRIES            5
#define BLANK_DATA_SIZE							20
#define USER_PROFILE_SIZE						BLANK_DATA_SIZE

static uint8_t gp_empty_buffer[BLANK_DATA_SIZE] 	= {0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
																										 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
																										 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
																										 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8_t gp_sync_end[BLANK_DATA_SIZE] 			= {0xFE, 0xFE, 0xFE, 0xFE, 0xFE,
																										 0xFE, 0xFE, 0xFE, 0xFE, 0xFE,
																										 0xFE, 0xFE, 0xFE, 0xFE, 0xFE,
																										 0xFE, 0xFE, 0xFE, 0xFE, 0xFE};

static uint8_t gp_blank_data[BLANK_DATA_SIZE]   	= {0x00, 0x00, 0x00, 0x00, 0x00, 
																										 0x00, 0x00, 0x00, 0x00, 0x00,
																										 0x00, 0x00, 0x00, 0x00, 0x00,
																										 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t gp_user_profile[USER_PROFILE_LEN]  = {0x00, 0x00, 0x00, 0x00, 0x00, 
																										 0x00, 0x00, 0x00, 0x00, 0x00,
																										 0x00, 0x00, 0x00, 0x00, 0x00,
																										 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t gp_temp_data[BLANK_DATA_SIZE]   		= {0x00, 0x00, 0x00, 0x00, 0x00, 
																										 0x00, 0x00, 0x00, 0x00, 0x00,
																										 0x00, 0x00, 0x00, 0x00, 0x00,
																										 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t gp_handshake_command[HDSK_CMD_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00};

bool    gogglepal_sync_started										= false;


/**@brief Connect event handler.
 *
 * @param[in]   p_gogglepal       LEDButton Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_gogglepal_t * p_gogglepal, ble_evt_t * p_ble_evt)
{
    p_gogglepal->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Disconnect event handler.
 *
 * @param[in]   p_gogglepal       LEDButton Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_gogglepal_t * p_gogglepal, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_gogglepal->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the Scoreboard CCCD write event.
 *
 * @param[in] p_gogglepal        Service instance.
 * @param[in] p_evt_write  WRITE event to be handled.
 */
//static void on_scoreboard_cccd_write(ble_gogglepal_t * p_gogglepal, ble_gatts_evt_write_t * p_evt_write)
//{
//    if (p_evt_write->len == 2)
//    {
//        // CCCD written, update notification state
//        ble_gogglepal_evt_t evt;

//        if (ble_srv_is_notification_enabled(p_evt_write->data))
//        {
//            evt.evt_type = BLE_GOGGLEPAL_EVT_NOTIFICATION_ENABLED;
//        }
//        else
//        {
//            evt.evt_type = BLE_GOGGLEPAL_EVT_NOTIFICATION_DISABLED;
//        }

//        if (p_gogglepal->evt_handler != NULL)
//        {
//            p_gogglepal->evt_handler(p_gogglepal, &evt);
//        }
//    }
//}

//static void on_flashdata_cccd_write(ble_gogglepal_t * p_gogglepal, ble_gatts_evt_write_t * p_evt_write)
//{
//    if (p_evt_write->len == 2)
//    {
//        // CCCD written, update notification state
//        ble_gogglepal_evt_t evt;

//        if (ble_srv_is_notification_enabled(p_evt_write->data))
//        {
//            evt.evt_type = BLE_GOGGLEPAL_EVT_NOTIFICATION_ENABLED;
//        }
//        else
//        {
//            evt.evt_type = BLE_GOGGLEPAL_EVT_NOTIFICATION_DISABLED;
//        }

//        if (p_gogglepal->evt_handler != NULL)
//        {
//            p_gogglepal->evt_handler(p_gogglepal, &evt);
//        }
//    }
//}

//static void on_flashheader_cccd_write(ble_gogglepal_t * p_gogglepal, ble_gatts_evt_write_t * p_evt_write)
//{
//    if (p_evt_write->len == 2)
//    {
//        // CCCD written, update notification state
//        ble_gogglepal_evt_t evt;

//        if (ble_srv_is_notification_enabled(p_evt_write->data))
//        {
//            evt.evt_type = BLE_GOGGLEPAL_EVT_NOTIFICATION_ENABLED;
//        }
//        else
//        {
//            evt.evt_type = BLE_GOGGLEPAL_EVT_NOTIFICATION_DISABLED;
//        }

//        if (p_gogglepal->evt_handler != NULL)
//        {
//            p_gogglepal->evt_handler(p_gogglepal, &evt);
//        }
//    }
//}

/**@brief Write event handler.
 *
 * @param[in]   p_gogglepal       GogglePal Data Update Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_gogglepal_t * p_gogglepal, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
//	 if ((p_evt_write->handle == p_gogglepal->user_profile_char_handles.value_handle) &&
//       (p_evt_write->len == USER_PROFILE_LEN) &&
//       (p_gogglepal->user_profile_write_handler != NULL))
//   {
//		   p_gogglepal->user_profile_write_handler(p_gogglepal, p_evt_write->data);
//   }
//	 else if ((p_evt_write->handle == p_gogglepal->hdsk_cmd_char_handles.value_handle) &&
//       (p_evt_write->len == HDSK_CMD_LEN) &&
//       (p_gogglepal->hdsk_cmd_write_handler != NULL))
//   {
//		   p_gogglepal->hdsk_cmd_write_handler(p_gogglepal, p_evt_write->data);
//   }
	
	  if (
        (p_evt_write->handle == p_gogglepal->scoreboard_char_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_gogglepal->is_notification_enabled = true;
        }
        else
        {
            p_gogglepal->is_notification_enabled = false;
        }
    }
		else if (
        (p_evt_write->handle == p_gogglepal->flash_data_char_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_gogglepal->is_notification_enabled = true;
        }
        else
        {
            p_gogglepal->is_notification_enabled = false;
        }
    }
		else if (
        (p_evt_write->handle == p_gogglepal->flash_header_char_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_gogglepal->is_notification_enabled = true;
        }
        else
        {
            p_gogglepal->is_notification_enabled = false;
        }
    }
		else if (
             (p_evt_write->handle == p_gogglepal->hdsk_cmd_char_handles.value_handle)
             &&
             (p_gogglepal->hdsk_cmd_write_handler != NULL)
						 &&	
						 (p_evt_write->len == HDSK_CMD_LEN)
            )
    {
        p_gogglepal->hdsk_cmd_write_handler(p_gogglepal, p_evt_write->data);
    }
		else if (
             (p_evt_write->handle == p_gogglepal->bootloader_char_handles.value_handle)
             &&
             (p_gogglepal->bootloader_write_handler != NULL)
						 &&	
						 (p_evt_write->len == BOOTLOADER_LEN)
            )
    {
        p_gogglepal->bootloader_write_handler(p_gogglepal, p_evt_write->data);
    }
		else if (
             (p_evt_write->handle == p_gogglepal->user_profile_char_handles.value_handle)
             &&
             (p_gogglepal->user_profile_write_handler != NULL)
						 &&	
						 (p_evt_write->len == USER_PROFILE_LEN)
            )
    {
        p_gogglepal->user_profile_write_handler(p_gogglepal, p_evt_write->data);
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
	 
//	  if (p_evt_write->handle == p_gogglepal->scoreboard_char_handles.cccd_handle)
//    {
//        on_scoreboard_cccd_write(p_gogglepal, p_evt_write);
//    }
//		else if (p_evt_write->handle == p_gogglepal->flash_data_char_handles.cccd_handle)
//    {
//        on_flashdata_cccd_write(p_gogglepal, p_evt_write);
//    }
//		else if (p_evt_write->handle == p_gogglepal->flash_header_char_handles.cccd_handle)
//    {
//        on_flashheader_cccd_write(p_gogglepal, p_evt_write);
//    }
//    else if (p_evt_write->handle == p_gogglepal->hdsk_cmd_char_handles.value_handle)
//    {
//        on_hdsk_value_write(p_gogglepal, p_evt_write);
//    }
//		else if (p_evt_write->handle == p_gogglepal->user_profile_char_handles.value_handle)
//    {
//        on_user_value_write(p_gogglepal, p_evt_write);
//    }
}

/**@brief Function for handling the TX_COMPLETE event.
 *
 * @details Handles TX_COMPLETE events from the BLE stack.
 *
 * @param[in] p_gogglepal      GogglePal Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_tx_complete(ble_gogglepal_t * p_gogglepal, ble_evt_t * p_ble_evt)
{


}

/**@brief Function for handling the HVC event.
 *
 * @details Handles HVC events from the BLE stack.
 *
 * @param[in] p_gogglepal      GogglePal Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_hvc(ble_gogglepal_t * p_gogglepal, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_hvc_t * p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;

    if (p_hvc->handle == p_gogglepal->hdsk_cmd_char_handles.value_handle)
    {


				// We did not expect this event in this state. Report error to application.
//				if (p_gogglepal->error_handler != NULL)
//				{
//						p_gogglepal->error_handler(NRF_ERROR_INVALID_STATE);
//				}
        
    }
		else if (p_hvc->handle == p_gogglepal->user_profile_char_handles.value_handle)
    {


				// We did not expect this event in this state. Report error to application.
//				if (p_gogglepal->error_handler != NULL)
//				{
//						p_gogglepal->error_handler(NRF_ERROR_INVALID_STATE);
//				}
        
    }
}


static void on_rw_authorize_request(ble_gogglepal_t * p_gogglepal, ble_gatts_evt_t * p_gatts_evt)
{
    ble_gatts_evt_rw_authorize_request_t * p_auth_req = &p_gatts_evt->params.authorize_request;
    if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
        if (   (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_PREP_WRITE_REQ)
            && (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
            && (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)
           )
        {
					  if (p_auth_req->request.write.handle == p_gogglepal->hdsk_cmd_char_handles.value_handle)
            {
                
            }
						else if (p_auth_req->request.write.handle == p_gogglepal->user_profile_char_handles.value_handle)
            {
                
            }
 
        }
    }
}

void ble_gogglepal_on_ble_evt(ble_gogglepal_t * p_gogglepal, ble_evt_t * p_ble_evt)
{
//    switch (p_ble_evt->header.evt_id)
//    {
//        case BLE_GAP_EVT_CONNECTED:
//            on_connect(p_gogglepal, p_ble_evt);
//            break;
//            
//        case BLE_GAP_EVT_DISCONNECTED:
//            on_disconnect(p_gogglepal, p_ble_evt);
//            break;
//            
//        case BLE_GATTS_EVT_WRITE:
//            on_write(p_gogglepal, p_ble_evt);
//            break;
//            
//        default:
//            break;
//    }
	
	  if ((p_gogglepal == NULL) || (p_ble_evt == NULL))
    {
        return;
    }
	
	  switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            //p_gogglepal->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
				    on_connect(p_gogglepal, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            //p_gogglepal->conn_handle = BLE_CONN_HANDLE_INVALID;
				    on_disconnect(p_gogglepal, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_gogglepal, p_ble_evt);
            break;

        case BLE_EVT_TX_COMPLETE:
            on_tx_complete(p_gogglepal, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            on_rw_authorize_request(p_gogglepal, &p_ble_evt->evt.gatts_evt);
            break;

        case BLE_GATTS_EVT_HVC:
            on_hvc(p_gogglepal, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Add User Profile characteristic.
 *
 * @param[in]   p_gogglepal        GogglePal Data Update Service structure.
 * @param[in]   p_gogglepal_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t user_profile_char_add(ble_gogglepal_t * p_gogglepal, const ble_gogglepal_init_t * p_gogglepal_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
		  
    uint8_t userProfileUserDesp[20] = "User Profile\0";
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = userProfileUserDesp;
		char_md.char_user_desc_max_size = 20;
    char_md.char_user_desc_size = 20;		
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_gogglepal->uuid_type;
    ble_uuid.uuid = GOGGLEPAL_UUID_USER_PROFILE_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
		//attr_md.vlen       = 0;
    attr_md.vlen       = 1;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
		attr_char_value.init_len		 = BLANK_DATA_SIZE;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len		   = BLANK_DATA_SIZE;
    attr_char_value.p_value      = gp_user_profile;
    
    return sd_ble_gatts_characteristic_add(p_gogglepal->service_handle, 
																					 &char_md,
                                           &attr_char_value,
                                           &p_gogglepal->user_profile_char_handles);
}


/**@brief Add Handshake Command characteristic.
 *
 * @param[in]   p_gogglepal        GogglePal Data Update Service structure.
 * @param[in]   p_gogglepal_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t hdsk_cmd_char_add(ble_gogglepal_t * p_gogglepal, const ble_gogglepal_init_t * p_gogglepal_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
	
    uint8_t HandshakeCMDUserDesp[20] = "Handshake Command\0";
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = HandshakeCMDUserDesp;
		char_md.char_user_desc_max_size = 20;
    char_md.char_user_desc_size = 20;		
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_gogglepal->uuid_type;
    ble_uuid.uuid = GOGGLEPAL_UUID_HDSK_CMD_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
		//attr_md.vlen       = 0;
    attr_md.vlen       = 1;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
		attr_char_value.init_len		 = HDSK_CMD_LEN;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len		   = HDSK_CMD_LEN;
    attr_char_value.p_value      = gp_handshake_command;
    
    return sd_ble_gatts_characteristic_add(p_gogglepal->service_handle, 
			                                     &char_md,
                                           &attr_char_value,
                                           &p_gogglepal->hdsk_cmd_char_handles);
}


/**@brief Add Bootloader characteristic.
 *
 * @param[in]   p_gogglepal        GogglePal Data Update Service structure.
 * @param[in]   p_gogglepal_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t bootloader_char_add(ble_gogglepal_t * p_gogglepal, const ble_gogglepal_init_t * p_gogglepal_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
		  
		uint8_t BootloaderUserDesp[20] = "Bootloader\0";	
	  //uint8_t BootloaderUserDesp[20] = "\0";	
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
//		char_md.char_props.write_wo_resp = 1;	
    char_md.p_char_user_desc  = BootloaderUserDesp;
		char_md.char_user_desc_max_size = 20;
    char_md.char_user_desc_size = 20;	
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_gogglepal->uuid_type;
    ble_uuid.uuid = GOGGLEPAL_UUID_BOOTLOADER_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
		attr_char_value.init_len		 = BLANK_DATA_SIZE;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len		   = BLANK_DATA_SIZE;
    attr_char_value.p_value      = gp_blank_data;
    
    return sd_ble_gatts_characteristic_add(p_gogglepal->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_gogglepal->bootloader_char_handles);
}


/**@brief Add Scoreboard data characteristic.
 *
 * @param[in]   p_gogglepal        GogglePal Data Update Service structure.
 * @param[in]   p_gogglepal_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t scoreboard_char_add(ble_gogglepal_t * p_gogglepal, const ble_gogglepal_init_t * p_gogglepal_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
	
		static uint8_t CurrentCountUserDesp[20] = "Scoreboard Data\0";

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
		char_md.p_char_user_desc  = CurrentCountUserDesp;	
		char_md.char_user_desc_max_size = 20;
    char_md.char_user_desc_size = 20;		
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_gogglepal->uuid_type;
    ble_uuid.uuid = GOGGLEPAL_UUID_SCOREBOARD_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    //attr_md.vlen       = 0;
		attr_md.vlen       = 1;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = BLANK_DATA_SIZE;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = BLANK_DATA_SIZE;
    attr_char_value.p_value			 = gp_blank_data;
    
    return sd_ble_gatts_characteristic_add(p_gogglepal->service_handle, 
																				   &char_md,
                                           &attr_char_value,
                                           &p_gogglepal->scoreboard_char_handles);
}


/**@brief Add Flash Data characteristic.
 *
 * @param[in]   p_gogglepal        GogglePal Data Update Service structure.
 * @param[in]   p_gogglepal_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t flash_data_char_add(ble_gogglepal_t * p_gogglepal, const ble_gogglepal_init_t * p_gogglepal_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
	
		static uint8_t FlashDataUserDesp[20] = "Flash Data\0";

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
		char_md.p_char_user_desc  = FlashDataUserDesp;	
		char_md.char_user_desc_max_size = 20;
    char_md.char_user_desc_size = 20;		
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_gogglepal->uuid_type;
    ble_uuid.uuid = GOGGLEPAL_UUID_FLASH_DATA_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    //attr_md.vlen       = 0;
		attr_md.vlen       = 1;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = BLANK_DATA_SIZE;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = BLANK_DATA_SIZE;
    attr_char_value.p_value			 = gp_blank_data;
    
    return sd_ble_gatts_characteristic_add(p_gogglepal->service_handle, 
																					 &char_md,
                                           &attr_char_value,
                                           &p_gogglepal->flash_data_char_handles);
}


/**@brief Add Flash Header characteristic.
 *
 * @param[in]   p_gogglepal        GogglePal Data Update Service structure.
 * @param[in]   p_gogglepal_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t flash_header_char_add(ble_gogglepal_t * p_gogglepal, const ble_gogglepal_init_t * p_gogglepal_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
	
		static uint8_t FlashHeaderUserDesp[20] = "Flash Header\0";

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
		char_md.p_char_user_desc  = FlashHeaderUserDesp;	
		char_md.char_user_desc_max_size = 20;
    char_md.char_user_desc_size = 20;		
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_gogglepal->uuid_type;
    ble_uuid.uuid = GOGGLEPAL_UUID_FLASH_HEADER_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    //attr_md.vlen       = 0;
		attr_md.vlen       = 1;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = BLANK_DATA_SIZE;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = BLANK_DATA_SIZE;
    attr_char_value.p_value			 = gp_blank_data;
    
    return sd_ble_gatts_characteristic_add(p_gogglepal->service_handle, 
			                                     &char_md,
                                           &attr_char_value,
                                           &p_gogglepal->flash_header_char_handles);
}


uint32_t ble_gogglepal_init(ble_gogglepal_t * p_gogglepal, const ble_gogglepal_init_t * p_gogglepal_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
	  ble_uuid128_t base_uuid = GOGGLEPAL_UUID_BASE;
	
	  if ((p_gogglepal == NULL) || (p_gogglepal_init == NULL))
    {
        return NRF_ERROR_NULL;
    }

    // Initialize service structure
    p_gogglepal->conn_handle       					= BLE_CONN_HANDLE_INVALID;
	  p_gogglepal->user_profile_write_handler = p_gogglepal_init->user_profile_write_handler;
	  p_gogglepal->hdsk_cmd_write_handler 		= p_gogglepal_init->hdsk_cmd_write_handler;
		p_gogglepal->bootloader_write_handler 	= p_gogglepal_init->bootloader_write_handler;
		p_gogglepal->is_notification_enabled 		= false;
    
    /**@snippet [Adding proprietary Service to S110 SoftDevice] */
    // Add a custom base UUID.    
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_gogglepal->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ble_uuid.type = p_gogglepal->uuid_type;
    ble_uuid.uuid = GOGGLEPAL_UUID_SERVICE;
	
		// Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_gogglepal->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
		err_code  = scoreboard_char_add(p_gogglepal, p_gogglepal_init);
    if (err_code != NRF_SUCCESS)
		{
        return err_code;
    }	
		
		err_code  = flash_data_char_add(p_gogglepal, p_gogglepal_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
		err_code  = flash_header_char_add(p_gogglepal, p_gogglepal_init);
    if (err_code != NRF_SUCCESS)
		{
        return err_code;
    }
		
		err_code = user_profile_char_add(p_gogglepal, p_gogglepal_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

		err_code = hdsk_cmd_char_add(p_gogglepal, p_gogglepal_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
		err_code = bootloader_char_add(p_gogglepal, p_gogglepal_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
    return NRF_SUCCESS;
}


/* SYNC OPERATION:
	1. mobile device asks for FLASH HEADER
  2. mobile device writes SYNC request
	3. GogglePal notifies FLASH DATA until 20 x 0xFFs are received from UART
  4. GogglePal notifies FLASH HEADER
*/
void ble_gogglepal_flash_data_transmit(void *p_event_data, uint16_t event_size)
{
	ble_sched_event_t *data = (ble_sched_event_t*)p_event_data;
	ble_gogglepal_t *p_gogglepal = NULL;
	uint32_t err_code = NRF_ERROR_FORBIDDEN;
	p_gogglepal = ble_gogglepal_get_handle_object();
	// Send value if connected and notifying
	if(p_gogglepal && (event_size > 0)){
	if (p_gogglepal->conn_handle != BLE_CONN_HANDLE_INVALID){
			uint16_t               hvx_len = 20;
			ble_gatts_hvx_params_t hvx_params;
			memset(&hvx_params, 0, sizeof(hvx_params) && p_event_data);        
			hvx_params.handle   = p_gogglepal->flash_data_char_handles.value_handle;
			hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
			hvx_params.offset   = 0;
			hvx_params.p_len    = &hvx_len;
			hvx_params.p_data   = data->data;        

			err_code = sd_ble_gatts_hvx(p_gogglepal->conn_handle, &hvx_params);
			/* if this packet failed to queue, ask MCU for it again */
			if(NRF_ERROR_BUSY==err_code || 
				 NRF_ERROR_NO_MEM==err_code || 
			   NRF_ERROR_INVALID_STATE==err_code || 
			   BLE_ERROR_INVALID_CONN_HANDLE==err_code || 
			   BLE_ERROR_NO_TX_BUFFERS==err_code ||
			   NRF_SUCCESS != err_code){
//					switch((Mcu_ble_events_t)data->event_code){
//						case Mcu_ble_event_FLASH_DATA_CODE:
//							sg_MCU_Send_Again_State = MCU_Send_Again_FLASH;
//							break;
//						case Mcu_ble_event_FLASH_HEADER_CODE: 
//							sg_MCU_Send_Again_State = MCU_Send_Again_FLASH_HEADER;
//							break;
//					}				
					 sg_MCU_Send_Again_State = MCU_Send_Again_FLASH;
			}else if (NRF_SUCCESS == err_code) {				
				if( 0 != memcmp(data->data, gp_sync_end, 20)){
//					switch((Mcu_ble_events_t)data->event_code){
//						/* if it's not the end of flash data, ask for more */
//					case Mcu_ble_event_FLASH_DATA_CODE:
//						memset(gp_temp_data, 0x00, LENGTHOF20);
//						queueSerialEvent(LENGTHOF20, Mcu_ble_event_new_SYNC, gp_temp_data, true);
//						break;
//					}
						nrf_delay_ms(300);
						memset(gp_temp_data, 0x00, LENGTHOF20);
						queueSerialEvent(LENGTHOF20, Mcu_ble_event_new_SYNC, gp_temp_data, true);
				}/* else stop sync */
				else{
//					switch((Mcu_ble_events_t)data->event_code){
//					case Mcu_ble_event_FLASH_DATA_CODE:
//						memset(gp_temp_data, 0x00, LENGTHOF20);
//						queueSerialEvent(LENGTHOF20, Mcu_ble_event_stop_SYNC, gp_temp_data, true);
//						break;
//					}
						memset(gp_temp_data, 0x00, LENGTHOF20);
						queueSerialEvent(LENGTHOF20, Mcu_ble_event_stop_SYNC, gp_temp_data, true);
				}
			}
		}
	}
}

void ble_gogglepal_scoreboard_transmit(void *p_event_data, uint16_t event_size)
{
	ble_sched_event_t *data = (ble_sched_event_t*)p_event_data;
	uint8_t timeout = 0;
	ble_gogglepal_t *p_gogglepal = NULL;
	uint32_t err_code = NRF_ERROR_FORBIDDEN;
	p_gogglepal = ble_gogglepal_get_handle_object();    
	// Send value if connected and notifying
	if(p_gogglepal && (event_size > 0) && p_event_data){
		if (p_gogglepal->conn_handle != BLE_CONN_HANDLE_INVALID){
			uint16_t               hvx_len = 20;
			ble_gatts_hvx_params_t hvx_params;
			memset(&hvx_params, 0, sizeof(hvx_params));			
			hvx_params.handle   = p_gogglepal->scoreboard_char_handles.value_handle;
			hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
			hvx_params.offset   = 0;
			hvx_params.p_len    = &hvx_len;
			hvx_params.p_data   = data->data;			

			while( NRF_SUCCESS != err_code){
				err_code = sd_ble_gatts_hvx(p_gogglepal->conn_handle, &hvx_params);
				if(timeout++ >= BLE_SEND_RETRIES){
					break;
				}
			}
    }
	}
}

void ble_gogglepal_flash_header_transmit(void *p_event_data, uint16_t event_size)
{
	ble_sched_event_t *data = (ble_sched_event_t*)p_event_data;
	uint8_t timeout = 0;
  ble_gogglepal_t *p_gogglepal = NULL;
	uint32_t err_code = NRF_ERROR_FORBIDDEN;
	p_gogglepal = ble_gogglepal_get_handle_object();    
  // Send value if connected and notifying
	if(p_gogglepal && (event_size > 0) && p_event_data){
    if (p_gogglepal->conn_handle != BLE_CONN_HANDLE_INVALID){
			uint16_t               hvx_len = 20;
			ble_gatts_hvx_params_t hvx_params;
			memset(&hvx_params, 0, sizeof(hvx_params));        
			hvx_params.handle   = p_gogglepal->flash_header_char_handles.value_handle;
			hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
			hvx_params.offset   = 0;
			hvx_params.p_len    = &hvx_len;
			hvx_params.p_data   = data->data;        

			while( NRF_SUCCESS != err_code){
				err_code = sd_ble_gatts_hvx(p_gogglepal->conn_handle, &hvx_params);
				if(timeout++ >= BLE_SEND_RETRIES){
					break;
				}
			}
			if(true == gogglepal_sync_started){
				gogglepal_sync_started = false;
				//memset(gp_temp_data, 0x00, LENGTHOF20);
				//queueSerialEvent(LENGTHOF20, Mcu_ble_event_STOP_CMD, gp_temp_data, false);
			}

    }
	}
}


void update_user_profile(uint8_t *profile)
{
		memcpy(gp_user_profile, profile, USER_PROFILE_LEN);
}

void set_sync_state(bool state)
{
	gogglepal_sync_started = state;
}


