/**
 * Copyright (c) 2012 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_app_hids_keyboard_main main.c
 * @{
 * @ingroup ble_sdk_app_hids_keyboard
 * @brief HID Keyboard Sample Application main file.
 *
 * This file contains is the source code for a sample application using the HID, Battery and Device
 * Information Services for implementing a simple keyboard functionality.
 * Pressing Button 0 will send text 'hello' to the connected peer. On receiving output report,
 * it toggles the state of LED 2 on the mother board based on whether or not Caps Lock is on.
 * This application uses the @ref app_scheduler.
 *
 * Also it would accept pairing requests from any peer device.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "bsp_btn_ble.h"
#include "app_scheduler.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "app_uart.h"
#include "app_fifo.h"
#include "nrf_drv_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define DEVICE_NAME                         "Pidan Keyboard"                      		/**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "Analog Dragon"                   			/**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO               3                                          /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                          /**< A tag identifying the SoftDevice BLE configuration. */

#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(2000)                      /**< Battery level measurement interval (ticks). */
#define APP_INTERVAL         				APP_TIMER_TICKS(1)						   //APP timer

#define PNP_ID_VENDOR_ID_SOURCE             0x02                                       /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                    0x1915                                     /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID                   0xEEEE                                     /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION              0x0001                                     /**< Product Version. */

#define APP_ADV_FAST_INTERVAL               0x0028                                     /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL               0x0C80                                     /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */

#define APP_ADV_FAST_DURATION               3000                                       /**< The advertising duration of fast advertising in units of 10 milliseconds. */
#define APP_ADV_SLOW_DURATION               18000                                      /**< The advertising duration of slow advertising in units of 10 milliseconds. */


/*lint -emacro(524, MIN_CONN_INTERVAL) // Loss of precision */
#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(7.5, UNIT_1_25_MS)           /**< Minimum connection interval (7.5 ms) */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(30, UNIT_1_25_MS)            /**< Maximum connection interval (30 ms). */
#define SLAVE_LATENCY                       6                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(430, UNIT_10_MS)             /**< Connection supervisory timeout (430 ms). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                      /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                     /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                         /**< Maximum encryption key size. */

#define KeepKeyNum							12 
#define OUTPUT_REPORT_INDEX                 0                                          /**< Index of Output Report. */
#define OUTPUT_REPORT_MAX_LEN               8                                          /**< Maximum length of Output Report. */
#define INPUT_REPORT_KEYS_INDEX             0                                          /**< Index of Input Report. */
#define INPUT_REPORT_KEYS_MAX_LEN           (KeepKeyNum + 1 + 2)                       /**< Maximum length of the Input Report characteristic. */
#define INPUT2_REPORT_KEYS_MAX_LEN          2                              			   /**< Maximum length of the Input Report characteristic. */

#define INPUT_REP_REF_ID                    0                                          /**< Id of reference to Keyboard Input Report. */
#define OUTPUT_REP_REF_ID                   0                                          /**< Id of reference to Keyboard Output Report. */

#define BASE_USB_HID_SPEC_VERSION           0x0101                                     /**< Version number of base USB HID Specification implemented by this application. */


#define DEAD_BEEF                           0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE           APP_TIMER_SCHED_EVENT_DATA_SIZE            /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                    20                                         /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                    10                                         /**< Maximum number of events in the scheduler queue. */
#endif




APP_TIMER_DEF(m_battery_timer_id);                                  /**< Battery timer. */
APP_TIMER_DEF(m_app_timer_id);                                  	/**< APP Timer. */
BLE_HIDS_DEF(m_hids,                                                /**< Structure used to identify the HID service. */
             NRF_SDH_BLE_TOTAL_LINK_COUNT,
             INPUT_REPORT_KEYS_MAX_LEN,
             OUTPUT_REPORT_MAX_LEN);
BLE_BAS_DEF(m_bas);                                                 /**< Structure used to identify the battery service. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */

static bool              m_in_boot_mode = false;                    /**< Current protocol mode. */
static uint16_t          m_conn_handle  = BLE_CONN_HANDLE_INVALID;  /**< Handle of the current connection. */
static pm_peer_id_t      m_peer_id;                                 /**< Device reference handle to the current bonded central. */
static uint32_t          m_whitelist_peer_cnt;                      /**< Number of peers currently in the whitelist. */
static pm_peer_id_t      m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];   /**< List of peers currently in the whitelist. */

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}};


#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

#define UART_HEAR_TX        0x5A
#define UART_HEAR_RX        0xA5
#define UART_BLE_ADDR       0x01

//
struct SysTime_REG
{
	volatile uint16_t SysTimeCNT1ms;
	volatile uint16_t SysTimeCNT10ms;
	volatile uint16_t SysTimeCNT100ms;
	volatile uint16_t SysTimeCNT1s;
	volatile uint16_t SysTimeCNT1min;
	volatile uint16_t SysTimeCNT1h;
	
	volatile uint8_t SysTimeFLG10ms;
	volatile uint8_t SysTimeFLG100ms;
	volatile uint8_t SysTimeFLG1s;
	volatile uint8_t SysTimeFLG1min;
	volatile uint8_t SysTimeFLG1h;
}; 

static struct SysTime_REG SysTime = {0};

//用于HID
static uint8_t HID_Input[INPUT_REPORT_KEYS_MAX_LEN] = {0};
static uint8_t HID_Input2[INPUT2_REPORT_KEYS_MAX_LEN] = {0};
static uint8_t HID_Output[OUTPUT_REPORT_MAX_LEN] = {0};

//用于uart
struct BLE_STATE_DEF{
  uint8_t LEDState    :8;
  
  uint8_t LinkID      :4;
  uint8_t LinkSta     :1;
  uint8_t Timeout     :1;
  uint8_t NeedReset   :1;
  uint8_t res0        :1;
  
  uint8_t res1        :8;
  uint8_t res2        :8;
};

union BLE_STATE_UNI{
  struct BLE_STATE_DEF bit;
  uint8_t all[4];
};

union BLE_STATE_UNI BleState;

/*------串口相关全局变量------*/
#define UART_LEN 300

static uint8_t  TxBusyFlag = 0;
static uint16_t TxTime = 0;

static uint8_t  RxBusyFlag = 0;
static uint16_t RxTime = 0;	//接受byte的时间戳
static uint16_t RxCpltTime = 0;	//接受帧时间戳

static uint16_t RxIndex = 0;
static uint16_t RxLength = 0;

static uint16_t RxErrNum = 0;
static uint8_t LinkErr = 0;

static uint16_t RxOtherTime = 0;
static uint8_t RxOtherFlag = 0;

static uint8_t UsartTxBuffer[UART_LEN] = {0};
static uint8_t UsartRxBuffer[UART_LEN] = {0};
static uint8_t UsartRxOut[UART_LEN] = {0};

/*------其他全局变量------*/

static uint8_t key_buff[2 + KeepKeyNum] = {0};
static uint8_t key_buff_bak[sizeof(key_buff)] = {0};
static uint8_t key_fresh = 0;

static uint8_t BatteryLevel = 10;
static uint8_t BLE_CMD = 0;

//记录一个时间戳，超时后直接复位
static uint16_t NeedResetTime = 0;

static uint32_t hid_send_release(uint8_t* pData, uint8_t Len);
void CheckUartData(void);
static void Disconnect_BLE(void);
static void WhitelistOFF_BLE(void);
static void delete_bonds(void);

	
//1ms系统时钟
void SysTimeCall(void){
	
  SysTime.SysTimeCNT1ms++;
  
  if((SysTime.SysTimeCNT1ms%10) == 0){
    if(SysTime.SysTimeCNT1ms>=60000)SysTime.SysTimeCNT1ms = 0;
    SysTime.SysTimeCNT10ms++;
    SysTime.SysTimeFLG10ms = 1;
    
    if((SysTime.SysTimeCNT10ms%10) == 0){
      if(SysTime.SysTimeCNT10ms>=60000)SysTime.SysTimeCNT10ms = 0;
      SysTime.SysTimeCNT100ms++;
      SysTime.SysTimeFLG100ms = 1;
      
      if((SysTime.SysTimeCNT100ms%10) == 0){
        if(SysTime.SysTimeCNT100ms>=60000)SysTime.SysTimeCNT100ms = 0;
        SysTime.SysTimeCNT1s++;
        SysTime.SysTimeFLG1s = 1;
        
        if((SysTime.SysTimeCNT1s%60) == 0){
          if(SysTime.SysTimeCNT1s>=60000)SysTime.SysTimeCNT1s = 0;
          SysTime.SysTimeCNT1min++;
          SysTime.SysTimeFLG1min = 1;
          
          if((SysTime.SysTimeCNT1min%60) == 0){
            if(SysTime.SysTimeCNT1min>=60000)SysTime.SysTimeCNT1min = 0;
            SysTime.SysTimeCNT1h++;
            SysTime.SysTimeFLG1h = 1;
          }
        }
      }
    }
  }
}


//获取时间差
uint16_t GetDtTime(uint16_t TimeBuf,uint16_t TimeBase){
  if(TimeBuf > TimeBase)
    return 60000 - TimeBuf + TimeBase;
  else
    return TimeBase - TimeBuf;
}


//计算CRC
uint16_t CRCCheck(uint8_t *pData, uint16_t Size){
  uint16_t crc_result = 0xFFFF;
  
  for(uint16_t i=0;i<Size;i++)
  {
      crc_result ^= pData[i];
      for(uint8_t j=0;j<8;j++)
      {
          if(crc_result&0x01)
              crc_result=(crc_result>>1)^0xa001;
          else
              crc_result=crc_result>>1;
      }
  }
  return crc_result;
}

static void sleep_mode_enter(void)
{
	//这里需要串口传回一个需要复位的数据
	BleState.bit.NeedReset = 1;
	NRF_LOG_INFO("Need Reset");
	NeedResetTime = SysTime.SysTimeCNT10ms;
}

static void uart_handle(app_uart_evt_t * p_event){
	uint8_t res = 0;
	
	switch(p_event->evt_type){
		
		case APP_UART_DATA_READY:
			while(app_uart_get(&res) == NRF_SUCCESS){
				RxTime = SysTime.SysTimeCNT1ms;
				//接受到了数据
				if(RxBusyFlag == 0){
					if(res == UART_HEAR_RX){	//识别到头
						RxBusyFlag = 1;
						RxIndex = 0;
						UsartRxBuffer[RxIndex++] = res;
					}
				}
				else{
					UsartRxBuffer[RxIndex++] = res;
					//判断长度，接收完成
					if(RxIndex >= (UsartRxBuffer[2] + 5)){
						RxBusyFlag = 0;
						memcpy(UsartRxOut, UsartRxBuffer, RxIndex);
						RxLength = RxIndex;
						RxIndex = 0;
						CheckUartData();
					}
				}
			}
			break;
		
		case APP_UART_FIFO_ERROR:
			NRF_LOG_ERROR("APP_UART_FIFO_ERROR");
			APP_ERROR_HANDLER(p_event->data.error_code);
			break;
		
		case APP_UART_COMMUNICATION_ERROR:
			NRF_LOG_ERROR("APP_UART_COMMUNICATION_ERROR");
			//串口通信故障，不进行处理
//			APP_ERROR_HANDLER(p_event->data.error_communication);
			break;
		
		case APP_UART_TX_EMPTY:
			TxBusyFlag = 0;
			TxTime = SysTime.SysTimeCNT1ms;
			break;
		
		default:
			break;
	}
}

//发送一次串口数据
void UsartTxOnce(void){
	uint16_t temp;
	uint16_t Len;
	
	if(TxBusyFlag != 0)return;
	
	UsartTxBuffer[0] = UART_HEAR_TX;
	UsartTxBuffer[1] = UART_BLE_ADDR;
	memcpy(&UsartTxBuffer[3], BleState.all, sizeof(BleState.all));
	for(Len = 3 + sizeof(BleState.all); Len > 4 && UsartTxBuffer[Len-1] == 0; Len--);
	UsartTxBuffer[2] = Len-3;
	temp = CRCCheck(UsartTxBuffer, Len);
	UsartTxBuffer[Len++] = temp>>8;
	UsartTxBuffer[Len++] = temp;
	
	for(uint16_t i = 0; i < Len; i++){
		app_uart_put(UsartTxBuffer[i]);
	}
	TxBusyFlag = 1;
}


//发送一次键值
//因为hid最后一帧必须是单按键变化，需要处理一下
void SendHIDKeyboard(void){
	uint8_t ChangeKeyNumA = 0;
	uint8_t ChangeKeyNumB = 0;
	uint8_t ChangeKeyA = 0;
	uint8_t ChangeKeyB = 0;
	
	//获取被改变的值
	for(uint8_t i=0;i<8;i++){
		if((((key_buff[1]^key_buff_bak[1]) >> i) & 1) != 0){
			ChangeKeyNumA++;
			if(ChangeKeyA == 0)ChangeKeyA = 1 << i;
		}
	}
	for(uint8_t i=0;i<KeepKeyNum;i++){
		if(key_buff[2 + i] != key_buff_bak[2 + i]){
			ChangeKeyNumB++;
			ChangeKeyB = i;
		}
	}
	NRF_LOG_INFO("ChangeKey = %d %d",ChangeKeyNumA,ChangeKeyNumB);
	//复制键值
	HID_Input[0] = 1;
	HID_Input[1] = key_buff[1];
	HID_Input[2] = 0;
	memcpy(&HID_Input[3], &key_buff[2], KeepKeyNum);
	memcpy(&key_buff_bak[1], &key_buff[1], sizeof(key_buff)-1);
	
	//超过2个键值被改变，则需要清0一个值
	//这里清除末尾的那个值
	if(ChangeKeyNumB){
		if(ChangeKeyNumA + ChangeKeyNumB > 1){
			HID_Input[3 + ChangeKeyB] = 0;
			key_buff_bak[2 + ChangeKeyB] = 0;
		}
	}
	else{
		if(ChangeKeyNumA > 1){
			HID_Input[1] &= ~ChangeKeyA;
			key_buff_bak[1] &= ~ChangeKeyA;
		}
	}
	
	//发送键值
	if(hid_send_release(HID_Input, sizeof(HID_Input)) == NRF_SUCCESS){
		//发送成功
		if(ChangeKeyNumA + ChangeKeyNumB > 1){	//还未发送完成
			key_fresh |= 1;
		}
		else{
			key_fresh &= ~1;
		}
	}
	else{
		key_fresh |= 1;
	}
}

//串口获取一个键值
void GetUartData(void){
	
	BLE_CMD = UsartRxOut[3] & 0x0F;
	BatteryLevel = (UsartRxOut[3] >> 4) & 0x0F;
	
	for(uint8_t i=0;i<sizeof(key_buff);i++){
		if((UsartRxOut[2] - 1) > i)key_buff[i] = UsartRxOut[4+i];
		else key_buff[i] = 0;
	}
	
	//media Send
	if(key_buff[0] != key_buff_bak[0]){
		HID_Input2[0] = 2;
		HID_Input2[1] = key_buff[0];
		if(hid_send_release(HID_Input2, sizeof(HID_Input2)) != NRF_SUCCESS){
			key_fresh |= 2;
		}
		key_buff_bak[0] = key_buff[0];
	}
	
	//KeyBoard Send
	if(memcmp(&key_buff[1], &key_buff_bak[1], sizeof(key_buff) - 1)){
		SendHIDKeyboard();
	}
}

//校验数据帧
void CheckUartData(void){
	uint16_t temp;
	
	if(RxLength == (UsartRxOut[2] + 5)){  //帧头
		temp = CRCCheck(UsartRxOut, RxLength - 2);
		temp ^= ((uint16_t)UsartRxOut[RxLength - 2] << 8) | UsartRxOut[RxLength - 1];

		if(temp == 0){  //CRC Verify
			RxCpltTime = SysTime.SysTimeCNT100ms;	//校验通过
			LinkErr = 0;
			
			switch(UsartRxOut[1]){

				case UART_BLE_ADDR:
					
					//收到主机数据
					GetUartData();
				
					//回复数据
					UsartTxOnce();	
					break;

				default:
					RxOtherFlag = 1;
					RxOtherTime = SysTime.SysTimeCNT100ms;
					break;

			} //end of switch

		}//end of verify
		else
			RxErrNum++;
		
	}
	else
		RxErrNum++;
}

//串口常驻任务，用于接收异常中断
void UsartTask(void){
	//接收中
	if(RxBusyFlag != 0){
		//超过4ms没有新数据
		if(GetDtTime(RxTime,SysTime.SysTimeCNT1ms) >= 4){
			RxBusyFlag = 0;
			memcpy(UsartRxOut, UsartRxBuffer, RxIndex);
			RxLength = RxIndex;
			RxIndex = 0;
			CheckUartData();
		}
	}
}

#define BLEK_Idle                 0
#define BLEK_Disconnect           1
#define BLEK_ClearBind            2
#define BLEK_WhitelistOFF         3


//CMD处理函数
void CMDTask(void){
	static uint8_t CMD_Bak = 0;
	
	if(CMD_Bak != BLE_CMD){
		CMD_Bak = BLE_CMD;
		switch(CMD_Bak){
			
			case BLEK_Disconnect:
				Disconnect_BLE();
				break;
			
			case BLEK_ClearBind:
				delete_bonds();
				break;
			
			case BLEK_WhitelistOFF:
				WhitelistOFF_BLE();
				break;
			
			default:
				break;
		}
	}
}


void HidTask(void){
	//接收中
	if(key_fresh & 1){
		SendHIDKeyboard();
	}
	if(key_fresh & 2){
		if(hid_send_release(HID_Input2, sizeof(HID_Input2)) == NRF_SUCCESS){
			key_fresh &= ~2;
		}
	}
}

void LedTask(void){
	static uint8_t LED_Bak = 0;
	
	if(RxOtherFlag != 0 && GetDtTime(RxOtherTime, SysTime.SysTimeCNT100ms) >= 10){
		RxOtherFlag = 0;
	}
	
	if(RxOtherFlag == 0 && TxBusyFlag == 0 
	&& LED_Bak != BleState.bit.LEDState 
	&& GetDtTime(TxTime, SysTime.SysTimeCNT1ms) > 5){
		LED_Bak = BleState.bit.LEDState;
		UsartTxOnce();
	}
}


static void usart_init(void){
	ret_code_t err_code;
	
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
#if defined (UART_PRESENT)
          NRF_UART_BAUDRATE_115200
#else
          NRF_UARTE_BAUDRATE_115200
#endif
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);
}


static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
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


/**@brief Fetch the list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
    pm_peer_id_t peer_id;
    uint32_t     peers_to_copy;

    peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
                     *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    *p_size = 0;

    while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
    {
        p_peers[(*p_size)++] = peer_id;
        peer_id = pm_next_peer_id_get(peer_id);
    }
}



static void Disconnect_BLE(void){
    ret_code_t err_code;
	
	NRF_LOG_INFO("Disconnect_BLE");
	if (m_conn_handle != BLE_CONN_HANDLE_INVALID){
		err_code = sd_ble_gap_disconnect(m_conn_handle,
										 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		if (err_code != NRF_ERROR_INVALID_STATE)
		{
			APP_ERROR_CHECK(err_code);
		}
	}
}


static void WhitelistOFF_BLE(void){
    ret_code_t err_code;
	
	if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
	{
		err_code = ble_advertising_restart_without_whitelist(&m_advertising);
		if (err_code != NRF_ERROR_INVALID_STATE)
		{
			APP_ERROR_CHECK(err_code);
		}
	}
}


/**@brief Clear bond information from persistent storage.
 */

static void delete_bonds(void)
{
    ret_code_t err_code;
	Disconnect_BLE();
    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
	ret_code_t ret;

	memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
	m_whitelist_peer_cnt = (sizeof(m_whitelist_peers) / sizeof(pm_peer_id_t));

	peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);

	ret = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
	APP_ERROR_CHECK(ret);

	// Setup the device identies list.

	// Some SoftDevices do not support this feature.
	ret = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
	if (ret != NRF_ERROR_NOT_SUPPORTED)
	{
		APP_ERROR_CHECK(ret);
	}

	ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
	APP_ERROR_CHECK(ret);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);

            m_peer_id = p_evt->peer_id;
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
			NRF_LOG_INFO("PM_EVT_CONN_SEC_FAILED");
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
			NRF_LOG_INFO("Relink");
			
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
				delete_bonds();
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {	
			sleep_mode_enter();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        {
			uint8_t i;
			//检查white list是否已经包含这个ID
			BleState.bit.LinkID = m_peer_id;
			for(i = 0; i < BLE_GAP_WHITELIST_ADDR_MAX_COUNT; i++){
				if(m_peer_id == m_whitelist_peers[i])
					break;
			}
			if(i < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)break;
			
            if (     p_evt->params.peer_data_update_succeeded.flash_changed
                 && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
            {
				  NRF_LOG_INFO("New Bond %d",m_peer_id);
                // Note: You should check on what kind of white list policy your application should use.

                if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
                {
                    // Bonded to a new peer, add it to the whitelist.
                    m_whitelist_peers[m_whitelist_peer_cnt++] = m_peer_id;

                    // The whitelist has been modified, update it in the Peer Manager.
                    err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
                    if (err_code != NRF_ERROR_NOT_SUPPORTED)
                    {
                        APP_ERROR_CHECK(err_code);
                    }

                    err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // This can happen when the local DB has changed.
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    ret_code_t err_code;
	
	if(m_conn_handle != BLE_CONN_HANDLE_INVALID){
		err_code = ble_bas_battery_level_update(&m_bas, BatteryLevel * 10, BLE_CONN_HANDLE_ALL);
	}
	else{
		err_code = ble_bas_battery_level_update(&m_bas, 100, BLE_CONN_HANDLE_ALL);
	}
	
	if ((err_code != NRF_SUCCESS) &&
		(err_code != NRF_ERROR_BUSY) &&
		(err_code != NRF_ERROR_RESOURCES) &&
		(err_code != NRF_ERROR_FORBIDDEN) &&
		(err_code != NRF_ERROR_INVALID_STATE) &&
		(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
	   )
	{
		APP_ERROR_HANDLER(err_code);
	}
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}

/*
	发送响应函数
	10ms
	进行发送更新与保持
*/
	
static void app_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

	SysTimeCall();
	
	
	{
		UsartTask();
		HidTask();
		LedTask();
		CMDTask();
	}
	
	if(SysTime.SysTimeFLG10ms){
		SysTime.SysTimeFLG10ms = 0;
	}
	
	if(SysTime.SysTimeFLG100ms){
		SysTime.SysTimeFLG100ms = 0;
		
		//0.2秒复位
		if(BleState.bit.NeedReset == 1 
		&& GetDtTime(NeedResetTime, SysTime.SysTimeCNT10ms) >= 20){
			NVIC_SystemReset();
		}
		
		//断线处理
		{
			if(LinkErr == 0 && GetDtTime(RxCpltTime, SysTime.SysTimeCNT100ms) >= 20){
				LinkErr = 1;
			}
		}
	}
	
	if(SysTime.SysTimeFLG1s){
		SysTime.SysTimeFLG1s = 0;
	}
	
	if(SysTime.SysTimeFLG1min){
		SysTime.SysTimeFLG1min = 0;
		NRF_LOG_INFO("TickTime %d min",SysTime.SysTimeCNT1min);
	}
	
	if(SysTime.SysTimeFLG1h){
		SysTime.SysTimeFLG1h = 0;
	}
		
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create battery timer.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    // App timer
    err_code = app_timer_create(&m_app_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                app_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_KEYBOARD);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Queued Write Module.
 */
static void qwr_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init_obj = {0};

    qwr_init_obj.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    ret_code_t       err_code;
    ble_dis_init_t   dis_init_obj;
    ble_dis_pnp_id_t pnp_id;

    pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;
    pnp_id.vendor_id        = PNP_ID_VENDOR_ID;
    pnp_id.product_id       = PNP_ID_PRODUCT_ID;
    pnp_id.product_version  = PNP_ID_PRODUCT_VERSION;

    memset(&dis_init_obj, 0, sizeof(dis_init_obj));

    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
    dis_init_obj.p_pnp_id = &pnp_id;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&dis_init_obj.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init_obj.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Battery Service.
 */
static void bas_init(void)
{
    ret_code_t     err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = NULL;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init_obj.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_report_read_perm);

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing HID Service.
 */
static void hids_init(void)
{
    ret_code_t                 err_code;
    ble_hids_init_t            hids_init_obj;
    ble_hids_inp_rep_init_t  * p_input_report;
    ble_hids_outp_rep_init_t * p_output_report;
    uint8_t                    hid_info_flags;

    static ble_hids_inp_rep_init_t  input_report_array[1];
    static ble_hids_outp_rep_init_t output_report_array[1];
    static uint8_t                  report_map_data[] =
    {
	0x05, 0x01,// USAGE_PAGE (Generic Desktop)
	0x09, 0x06,// USAGE (Keyboard)
	0xa1, 0x01,// COLLECTION (Application)
 	0x85, 0x01,// ID = 1
	//GUI keys
	0x75, 0x01,// REPORT_SIZE (1)
	0x95, 0x08,// REPORT_COUNT (8)
	
	0x05, 0x07,// USAGE_PAGE (Keyboard)
	0x19, 0xE0,// USAGE_MINIMUM (Keyboard LeftControl)
	0x29, 0xE7,// USAGE_MAXIMUM (Keyboard Right GUI)
	0x15, 0x00,// LOGICAL_MINIMUM (0)
	0x25, 0x01,// LOGICAL_MAXIMUM (1)
	
	0x81, 0x02,// INPUT (Data,Var,Abs)
	//not use
	0x75, 0x08,// REPORT_SIZE (8)
	0x95, 0x01,// REPORT_COUNT (1)
	0x81, 0x03,// INPUT (Cnst,Var,Abs)
	//keyboard
	0x75, 0x08,// REPORT_SIZE (8)
	0x95, 0x0C,// REPORT_COUNT (12)
	
	0x05, 0x07,// USAGE_PAGE (Keyboard)
	0x19, 0x00,// USAGE_MINIMUM (Reserved (no event indicated))
	0x29, 0xE7,// USAGE_MAXIMUM (Keyboard Application)
	0x15, 0x00,// LOGICAL_MINIMUM (0)
	0x25, 0xFF,// LOGICAL_MAXIMUM (255)
	
	0x81, 0x00,// INPUT (Data,Ary,Abs)	
	//LED
	0x75, 0x01,// REPORT_SIZE (1)
	0x95, 0x05,// REPORT_COUNT (5)
	
	0x05, 0x08,// USAGE_PAGE (LEDs)
	0x19, 0x01,// USAGE_MINIMUM (Num Lock)
	0x29, 0x05,// USAGE_MAXIMUM (Kana)
	
	0x91, 0x02,// OUTPUT (Data,Var,Abs)
	//
	0x75, 0x03,// REPORT_SIZE (3)
	0x95, 0x01,// REPORT_COUNT (1)
	
	0x91, 0x03,// OUTPUT (Cnst,Var,Abs)	//ì?3???
	//end ID=1
	0xc0, 
	
	0x05, 0x0c,//con device
	0x09, 0x01,//con ctrl
	0xa1, 0x01,//COLLECTION (Application)
	0x85, 0x02,// ID = 2
	
	0x75, 0x01,
	0x95, 0x08,//8*1b
	
	//media ctrl
	0x09, 0xe2,	//mute
	0x09, 0xe9, //vol+
	0x09, 0xea, //vol-
	0x09, 0xcd, //pause/play
	0x09, 0xb7, //stop
	0x09, 0xb6, //Last
	0x09, 0xb5, //Next
	0x09, 0xe2,	//mute
	
	0x15, 0x00,
	0x25, 0x01,//0~1
	
	0x81, 0x62,//input
	
	0xc0,
    };

    memset((void *)input_report_array, 0, sizeof(ble_hids_inp_rep_init_t));
    memset((void *)output_report_array, 0, sizeof(ble_hids_outp_rep_init_t));

    // Initialize HID Service
    p_input_report                      = &input_report_array[INPUT_REPORT_KEYS_INDEX];
    p_input_report->max_len             = INPUT_REPORT_KEYS_MAX_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    p_output_report                      = &output_report_array[OUTPUT_REPORT_INDEX];
    p_output_report->max_len             = OUTPUT_REPORT_MAX_LEN;
    p_output_report->rep_ref.report_id   = OUTPUT_REP_REF_ID;
    p_output_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_OUTPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.write_perm);

    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    hids_init_obj.is_kb                          = true;
    hids_init_obj.is_mouse                       = false;
    hids_init_obj.inp_rep_count                  = 1;
    hids_init_obj.p_inp_rep_array                = input_report_array;
    hids_init_obj.outp_rep_count                 = 1;
    hids_init_obj.p_outp_rep_array               = output_report_array;
    hids_init_obj.feature_rep_count              = 0;
    hids_init_obj.p_feature_rep_array            = NULL;
    hids_init_obj.rep_map.data_len               = sizeof(report_map_data);
    hids_init_obj.rep_map.p_data                 = report_map_data;
    hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.rep_map.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.rep_map.security_mode.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.hid_information.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.hid_information.security_mode.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(
        &hids_init_obj.security_mode_boot_kb_inp_rep.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_inp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_boot_kb_inp_rep.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_ctrl_point.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_ctrl_point.write_perm);

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    qwr_init();
    dis_init();
    bas_init();
    hids_init();
}



/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void timers_start(void)
{
    ret_code_t err_code;

    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_start(m_app_timer_id, APP_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/*HID 发送数据*/
static uint32_t hid_send_release(uint8_t* pData, uint8_t Len)
{
	ret_code_t err_code;
	
	if (!m_in_boot_mode)
	{
		err_code = ble_hids_inp_rep_send(&m_hids,
										 0,
										 Len,
										 pData,
										 m_conn_handle);
	}
	
	else
	{
		err_code = ble_hids_boot_kb_inp_rep_send(&m_hids,
												 Len,
												 pData,
												 m_conn_handle);
	}
	
	NRF_LOG_INFO("Return:%d",err_code);
	
    return err_code;
}


/**@brief Function for handling the HID Report Characteristic Write event.
 *
 * @param[in]   p_evt   HID service event.
 */
static void on_hid_rep_char_write(ble_hids_evt_t * p_evt)
{
    if (p_evt->params.char_write.char_id.rep_type == BLE_HIDS_REP_TYPE_OUTPUT)
    {
        ret_code_t err_code;
        uint8_t  report_index = p_evt->params.char_write.char_id.rep_index;

        if (report_index == OUTPUT_REPORT_INDEX)
        {
            // This code assumes that the output report is one byte long. Hence the following
            // static assert is made.
			
            err_code = ble_hids_outp_rep_get(&m_hids,
                                             report_index,
                                             OUTPUT_REPORT_MAX_LEN,
                                             0,
                                             m_conn_handle,
                                             HID_Output);
            APP_ERROR_CHECK(err_code);
			
			//获取LED状态
			if(HID_Output[0] == 1){
				BleState.bit.LEDState = HID_Output[1];
			}
			
        }
    }
}



/**@brief Function for handling HID events.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service structure.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
            m_in_boot_mode = true;
            break;

        case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
            m_in_boot_mode = false;
            break;

        case BLE_HIDS_EVT_REP_CHAR_WRITE:
            on_hid_rep_char_write(p_evt);
            break;

        case BLE_HIDS_EVT_NOTIF_ENABLED:
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
            NRF_LOG_INFO("High Duty Directed advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_DIRECTED:
            NRF_LOG_INFO("Directed advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("Slow advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_INFO("Fast advertising with whitelist.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_INFO("Slow advertising with whitelist.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
			BleState.bit.Timeout = 1;
            sleep_mode_enter();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                        whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                          addr_cnt, irk_cnt);

            // Apply the whitelist.
            err_code = ble_advertising_whitelist_reply(&m_advertising,
                                                       whitelist_addrs,
                                                       addr_cnt,
                                                       whitelist_irks,
                                                       irk_cnt);
            APP_ERROR_CHECK(err_code);
        } break; //BLE_ADV_EVT_WHITELIST_REQUEST

        case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        {
            pm_peer_data_bonding_t peer_bonding_data;

            // Only Give peer address if we have a handle to the bonded peer.
            if (m_peer_id != PM_PEER_ID_INVALID)
            {
                err_code = pm_peer_data_bonding_load(m_peer_id, &peer_bonding_data);
                if (err_code != NRF_ERROR_NOT_FOUND)
                {
                    APP_ERROR_CHECK(err_code);

                    ble_gap_addr_t * p_peer_addr = &(peer_bonding_data.peer_ble_id.id_addr_info);
                    err_code = ble_advertising_peer_addr_reply(&m_advertising, p_peer_addr);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; //BLE_ADV_EVT_PEER_ADDR_REQUEST

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
			BleState.bit.LinkSta = 1;
			BleState.bit.LinkID = m_peer_id;
            NRF_LOG_INFO("Link ID = %d", BleState.bit.LinkID);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected %d min %d s", SysTime.SysTimeCNT1min%60, SysTime.SysTimeCNT1s%60);
            // Dequeue all keys without transmission.
			memset(HID_Input,0,sizeof(HID_Input));
			memset(HID_Output,0,sizeof(HID_Output));
			BleState.bit.LinkSta = 0;
			BleState.bit.LinkID = 0;
			BleState.bit.LEDState = 0;
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            // Reset m_caps_on variable. Upon reconnect, the HID host will re-send the Output
            // report containing the Caps lock state.
            // disabling alert 3. signal - used for capslock ON
            err_code = bsp_indication_set(BSP_INDICATE_ALERT_OFF);
            APP_ERROR_CHECK(err_code);

            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            // Send next key event
			// 需要发送继续
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}



/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    uint8_t                adv_flags;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    adv_flags                            = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = adv_flags;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_whitelist_enabled          = true;
    init.config.ble_adv_directed_high_duty_enabled = true;
    init.config.ble_adv_directed_enabled           = false;
    init.config.ble_adv_directed_interval          = 0;
    init.config.ble_adv_directed_timeout           = 0;
    init.config.ble_adv_fast_enabled               = true;
    init.config.ble_adv_fast_interval              = APP_ADV_FAST_INTERVAL;
    init.config.ble_adv_fast_timeout               = APP_ADV_FAST_DURATION;
    init.config.ble_adv_slow_enabled               = true;
    init.config.ble_adv_slow_interval              = APP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout               = APP_ADV_SLOW_DURATION;

    init.evt_handler   = on_adv_evt;
    init.error_handler = ble_advertising_error_handler;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    app_sched_execute();
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for application main entry.
 */
int main(void)
{

    // Initialize.
    log_init();
	
    timers_init();
    buttons_leds_init();
	usart_init();
	
    power_management_init();
    ble_stack_init();
    scheduler_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
    peer_manager_init();

    // Start execution.
    timers_start();
    advertising_start();
	
    NRF_LOG_INFO("INIT Complete!");
	
    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
	
}


/**
 * @}
 */
