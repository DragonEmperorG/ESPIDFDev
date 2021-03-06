// Copyright 2015-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "esp_types.h"
#include "esp_event.h"
#include "esp_err.h"
#include "driver/uart.h"

#define GPS_MAX_SATELLITES_IN_USE (12)
#define GPS_MAX_SATELLITES_IN_VIEW (16)

   /**
 * @brief Declare of NMEA Parser Event base
 *
 */
   ESP_EVENT_DECLARE_BASE(ESP_NMEA_EVENT)

   /**
 * @brief TD0D01 data type
 *
 */
   typedef enum
   {
      I8S,  /*!< 0 */
      I8U,  /*!< 1 */
      I16S, /*!< 2 */
      I16U, /*!< 3 */
      I32S, /*!< 4 */
      I32U, /*!< 5 */
      F32,  /*!< 6 */
      F64,  /*!< 7 */
      B8,   /*!< 8 */
      B16,  /*!< 9 */
      B32,  /*!< 10 */
      CH,   /*!< 11 */
   } td0d01_data_t;

   /**
 * @brief GPS fix type
 *
 */
   typedef enum
   {
      GPS_FIX_INVALID, /*!< Not fixed */
      GPS_FIX_GPS,     /*!< GPS */
      GPS_FIX_DGPS,    /*!< Differential GPS */
   } gps_fix_t;

   /**
 * @brief GPS fix mode
 *
 */
   typedef enum
   {
      GPS_MODE_INVALID = 1, /*!< Not fixed */
      GPS_MODE_2D,          /*!< 2D GPS */
      GPS_MODE_3D           /*!< 3D GPS */
   } gps_fix_mode_t;

   /**
 * @brief GPS satellite information
 *
 */
   typedef struct
   {
      uint8_t num;       /*!< Satellite number */
      uint8_t elevation; /*!< Satellite elevation */
      uint16_t azimuth;  /*!< Satellite azimuth */
      uint8_t snr;       /*!< Satellite signal noise ratio */
   } gps_satellite_t;

   /**
 * @brief GPS time
 *
 */
   typedef struct
   {
      uint8_t hour;      /*!< Hour */
      uint8_t minute;    /*!< Minute */
      uint8_t second;    /*!< Second */
      uint16_t thousand; /*!< Thousand */
   } gps_time_t;

   /**
 * @brief GPS date
 *
 */
   typedef struct
   {
      uint8_t day;   /*!< Day (start from 1) */
      uint8_t month; /*!< Month (start from 1) */
      uint16_t year; /*!< Year (start from 2000) */
   } gps_date_t;

   /**
 * @brief Calendar time
 *
 */
   typedef struct
   {
      uint8_t second; /*!< Second */
      uint8_t minute; /*!< Minute */
      uint8_t hour;   /*!< Hour */
      uint8_t day;    /*!< Day */
      uint8_t month;  /*!< Month */
      uint16_t year;  /*!< Year */
   } calendar_time_t;

   /**
 * @brief NMEA Statement
 *
 */
   typedef enum
   {
      STATEMENT_UNKNOWN = 0, /*!< Unknown statement */
      STATEMENT_GGA,         /*!< GGA */
      STATEMENT_GSA,         /*!< GSA */
      STATEMENT_RMC,         /*!< RMC */
      STATEMENT_GSV,         /*!< GSV */
      STATEMENT_GLL,         /*!< GLL */
      STATEMENT_VTG          /*!< VTG */
   } nmea_statement_t;

   /**
 * @brief GPS object
 *
 */
   typedef struct
   {
      float latitude;                                                /*!< Latitude (degrees) */
      float longitude;                                               /*!< Longitude (degrees) */
      float altitude;                                                /*!< Altitude (meters) */
      gps_fix_t fix;                                                 /*!< Fix status */
      uint8_t sats_in_use;                                           /*!< Number of satellites in use */
      gps_time_t tim;                                                /*!< time in UTC */
      gps_fix_mode_t fix_mode;                                       /*!< Fix mode */
      uint8_t sats_id_in_use[GPS_MAX_SATELLITES_IN_USE];             /*!< ID list of satellite in use */
      float dop_h;                                                   /*!< Horizontal dilution of precision */
      float dop_p;                                                   /*!< Position dilution of precision  */
      float dop_v;                                                   /*!< Vertical dilution of precision  */
      uint8_t sats_in_view;                                          /*!< Number of satellites in view */
      gps_satellite_t sats_desc_in_view[GPS_MAX_SATELLITES_IN_VIEW]; /*!< Information of satellites in view */
      gps_date_t date;                                               /*!< Fix date */
      bool valid;                                                    /*!< GPS validity */
      float speed;                                                   /*!< Ground speed, unit: m/s */
      float cog;                                                     /*!< Course over ground */
      float variation;                                               /*!< Magnetic variation */
   } gps_t;

   /**
 * @brief MEAS object
 *
 */
   typedef struct
   {
      uint8_t gnss_category;               /*!< GNSS类别 */
      uint8_t satellite_id;                /*!< 卫星ID */
      uint8_t cn0;                         /*!< CN0 */
      double pseudorange;                        /*!< 伪距(m) */
      double carrier_phase;                      /*!< 载波相位(cycles) */
      float doppler;                             /*!< 多普勒(Hz) */
      uint16_t carrier_phase_locking_time; /*!< 载波相位锁定时间 */
      uint8_t sign;                              /*!< 标志 */
   } gnss_meas_repeat_data;

   typedef struct
   {
      double obs_second;                             /*!< 观测时刻TOW */
      uint16_t obs_week;                       /*!< 观测时刻WN */
      int8_t utc_leap_second;                   /*!< UTC闰秒 */
      uint16_t obs_num;                         /*!< 观测量个数 */
      gnss_meas_repeat_data observations[36]; /*!< 重复观测量数据 */
   } gnss_meas_data_t;

   /**
 * @brief Configuration of NMEA Parser
 *
 */
   typedef struct
   {
      struct
      {
         uart_port_t uart_port;            /*!< UART port number */
         uint32_t rx_pin;                  /*!< UART Rx Pin number */
         uint32_t baud_rate;               /*!< UART baud rate */
         uart_word_length_t data_bits;     /*!< UART data bits length */
         uart_parity_t parity;             /*!< UART parity */
         uart_stop_bits_t stop_bits;       /*!< UART stop bits length */
         uart_hw_flowcontrol_t flow_ctrl;  /*!< UART HW flow control mode (cts/rts) */
         uint8_t rx_flow_ctrl_thresh;      /*!< UART HW RTS threshold */
         uint32_t event_queue_size;        /*!< UART event queue size */
         uint32_t intr_enable_mask;        /*!< UART interrupt enable mask */
         uint8_t rx_timeout_thresh;        /*!< UART timeout interrupt threshold  */
         uint8_t txfifo_empty_intr_thresh; /*!< UART TX empty interrupt threshold */
         uint8_t rxfifo_full_thresh;       /*!< UART RX full interrupt threshold */
      } uart;                              /*!< UART specific configuration */
   } nmea_parser_config_t;

   /**
 * @brief NMEA Parser Handle
 *
 */
   typedef void *nmea_parser_handle_t;

/**
 * @brief Default configuration for NMEA Parser
 * 依据《TD0D01协议说明》本系统暂不支持数据位、停止位、校验位的调整，所有配置都被忽略。
 * 系统默认配置如下：
 *
 * @param .uart_port 
 * @param .rx_pin
 * @param .baud_rate 波特率9600bps
 * @param .data_bits 数据位8
 * @param .parity 校验位无
 * @param .stop_bits 停止位1
 * @param .flow_ctrl
 * @param .rx_flow_ctrl_thresh 数据帧最大长度为960字节，超过最大长度的都将视为无效数据。
 * @param .event_queue_size 
 * @param .intr_enable_mask
 * @param .rxfifo_full_thresh #define UART_FULL_THRESH_DEFAULT (120)
 * @param .rx_timeout_thresh #define UART_TOUT_THRESH_DEFAULT  (10)
 * @param .txfifo_empty_intr_thresh #define UART_EMPTY_THRESH_DEFAULT   (10)
 */
#define NMEA_PARSER_CONFIG_DEFAULT()                                                                                                                                                            \
   {                                                                                                                                                                                            \
      .uart = {                                                                                                                                                                                 \
         .uart_port = UART_NUM_1,                                                                                                                                                               \
         .rx_pin = 5,                                                                                                                                                                           \
         .baud_rate = 9600,                                                                                                                                                                     \
         .data_bits = UART_DATA_8_BITS,                                                                                                                                                         \
         .parity = UART_PARITY_DISABLE,                                                                                                                                                         \
         .stop_bits = UART_STOP_BITS_1,                                                                                                                                                         \
         .flow_ctrl = UART_HW_FLOWCTRL_RTS,                                                                                                                                                     \
         .rx_flow_ctrl_thresh = 120,                                                                                                                                                            \
         .event_queue_size = 16,                                                                                                                                                                \
         .intr_enable_mask = UART_RXFIFO_FULL_INT_ENA_M | UART_RXFIFO_TOUT_INT_ENA_M | UART_FRM_ERR_INT_ENA_M | UART_RXFIFO_OVF_INT_ENA_M | UART_BRK_DET_INT_ENA_M | UART_PARITY_ERR_INT_ENA_M, \
         .rxfifo_full_thresh = 128,                                                                                                                                                             \
         .rx_timeout_thresh = 10,                                                                                                                                                               \
         .txfifo_empty_intr_thresh = 10                                                                                                                                                         \
      }                                                                                                                                                                                         \
   }

   /**
 * @brief NMEA Parser Event ID
 *
 */
   typedef enum
   {
      GPS_UPDATE, /*!< GPS information has been updated */
      GPS_UNKNOWN /*!< Unknown statements detected */
   } nmea_event_id_t;

   /**
 * @brief Init NMEA Parser
 *
 * @param config Configuration of NMEA Parser
 * @return nmea_parser_handle_t handle of NMEA parser
 */
   nmea_parser_handle_t nmea_parser_init(const nmea_parser_config_t *config);

   /**
 * @brief Deinit NMEA Parser
 *
 * @param nmea_hdl handle of NMEA parser
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
   esp_err_t nmea_parser_deinit(nmea_parser_handle_t nmea_hdl);

   /**
 * @brief Add user defined handler for NMEA parser
 *
 * @param nmea_hdl handle of NMEA parser
 * @param event_handler user defined event handler
 * @param handler_args handler specific arguments
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_NO_MEM: Cannot allocate memory for the handler
 *  - ESP_ERR_INVALIG_ARG: Invalid combination of event base and event id
 *  - Others: Fail
 */
   esp_err_t nmea_parser_add_handler(nmea_parser_handle_t nmea_hdl, esp_event_handler_t event_handler, void *handler_args);

   /**
 * @brief Remove user defined handler for NMEA parser
 *
 * @param nmea_hdl handle of NMEA parser
 * @param event_handler user defined event handler
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALIG_ARG: Invalid combination of event base and event id
 *  - Others: Fail
 */
   esp_err_t nmea_parser_remove_handler(nmea_parser_handle_t nmea_hdl, esp_event_handler_t event_handler);

#ifdef __cplusplus
}
#endif
