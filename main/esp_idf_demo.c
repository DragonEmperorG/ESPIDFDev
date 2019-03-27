/*
  *Copyright(C),2019-2022, LIESMARS
  *FileName:             // 文件名
  *Author: QIAN LONG     // 作者
  *Version: 1.0.0        // 版本
  *Date: 2019/3/23       // 完成日期
  *Description:          //用于主要说明此程序文件完成的主要功能与其他模块或函数的接口、输出值、取值范围、含义及参数间的控制、顺序、独立及依赖关系
  *Others:               //其他内容说明
     1.泰斗D303-3模块需要通过通信接口选择开关选择Micro USB接口或RS232接口与上位机通信，
  UART_SEL开关拨向左侧为RS232接口通信，拨向右侧为Micro USB接口通信，但是实验显示显示该开关不影响模块UART口发送数据。  
  *Function List:        //主要函数列表，每条记录应包含函数名及功能简要说明
     1.…………
     2.…………
  *History:  //修改历史记录列表，每条修改记录应包含修改日期、修改者及修改内容简介
     1.Date:
       Author:
       Modification:
     2.…………
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nmea_parser.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/uart_struct.h"

// UART PIN
#define TXD_TD_PIN (GPIO_NUM_4)
#define RXD_TD_PIN (GPIO_NUM_5)
// RTS(Request To Send)
#define RTS_TD_PIN (UART_PIN_NO_CHANGE)
// CTS(Clear To Send)
#define CTS_TD_PIN (UART_PIN_NO_CHANGE)

#define BUF_SIZE (960)
#define TX_BUF_SIZE (BUF_SIZE)
// 泰斗TD0D01固件数据帧最大长度为960字节，超过最大长度的都将视为无效数据。
#define RX_BUF_SIZE (BUF_SIZE)
// 泰斗D303-3模块默认波特率为9600bps
#define BAUD_RATE (9600)

#define CTRL_UART_PORT (UART_NUM_1)
#define ECHO_UART_PORT (UART_NUM_2)

#define TIME_ZONE (+8)   //Beijing Time
#define YEAR_BASE (2000) //date in GPS starts from 2000

static const char *TAG = "EPS32_D303-3_UART_APP";
static const char *RX_TD_TASK_TAG = "RX_TD_TASK";

// void echo_platform_infomation()
// {
//     /* Print chip information */
//     esp_chip_info_t chip_info;
//     esp_chip_info(&chip_info);

//     printf("Chip Information\n");
//     printf("Chip Models: %s\n", (chip_info.model == 1) ? "ESP32" : "");
//     printf("Embedded Flash Memory: %s\n", (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
//     printf("2.4GHz WiFi: %s\n", (chip_info.features & CHIP_FEATURE_BT) ? "BGN" : "");
//     printf("Bluetooth LE: %s\n", (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "");
//     printf("Bluetooth Classic: %s\n", (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "");
//     printf("CPU Cores: %d\n", chip_info.cores);
//     printf("Silicon Revision Number: %d\n", chip_info.revision);

//     printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
//            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

//     for (int i = 10; i >= 0; i--)
//     {
//         printf("Restarting in %d seconds...\n", i);
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
//     printf("Restarting now.\n");
//     fflush(stdout);
//     esp_restart();
// }

void init()
{
    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "Start EPS-WROOM-32-D303-3 Application.");
    ESP_LOGI(TAG, "Configure UART.");

    // Setting Communication Parameters
    // Structure
    // uart_config_t
    // Descriptions
    // UART configuration parameters for uart_param_config function.
    // Public Members
    // baud_rate : int
    // data_bits : UART_DATA_5_BITS | UART_DATA_6_BITS | UART_DATA_7_BITS | UART_DATA_8_BITS | UART_DATA_BITS_MAX
    // parity : UART_PARITY_DISABLE | UART_PARITY_EVEN | UART_PARITY_ODD
    // stop_bits : UART_STOP_BITS_1 | UART_STOP_BITS_1_5 | UART_STOP_BITS_2 | UART_STOP_BITS_MAX
    // flow_ctrl : UART_HW_FLOWCTRL_DISABLE | UART_HW_FLOWCTRL_RTS | UART_HW_FLOWCTRL_CTS | UART_HW_FLOWCTRL_CTS_RTS | UART_HW_FLOWCTRL_MAX
    // rx_flow_ctrl_thresh : uint8_t
    // use_ref_tick : bool
    const uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(CTRL_UART_PORT, &uart_config);

    // Setting Communication Pins
    // Funtion
    // uart_set_pin()
    // Parameters
    // uart_num : UART_NUM_0 | UART_NUM_1 | UART_NUM_2
    // tx_io_num :
    // rx_io_num :
    // rts_io_num :
    // cts_io_num :
    ESP_LOGI(TAG, "UART set pins.");
    ESP_ERROR_CHECK(uart_set_pin(CTRL_UART_PORT, TXD_TD_PIN, RXD_TD_PIN, RTS_TD_PIN, CTS_TD_PIN));
    // Driver Installation
    // Function
    // uart_driver_install()
    // Parameters
    // uart_num : UART_NUM_0 | UART_NUM_1 | UART_NUM_2
    // rx_buffer_size :
    // tx_buffer_size :
    // queue_size :
    // uart_queue :
    // intr_alloc_flags :
    ESP_LOGI(TAG, "UART install driver.");
    ESP_ERROR_CHECK(uart_driver_install(CTRL_UART_PORT, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
}

static void rx_td_task()
{
    esp_log_level_set(RX_TD_TASK_TAG, ESP_LOG_INFO);
    ESP_LOGI(RX_TD_TASK_TAG, "UART start receiving data ...");
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    while (1)
    {
        // Read data from UART.
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(CTRL_UART_PORT, (size_t *)&length));

        // Function
        // uart_read_bytes()
        // Parameters
        // uart_num : UART_NUM_0 | UART_NUM_1 | UART_NUM_2
        // buf :
        // length :
        // ticks_to_wait :
        ESP_LOGI(RX_TD_TASK_TAG, "Buffered data length: %d", length);
        const int rxBytes = uart_read_bytes(CTRL_UART_PORT, data, length, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TD_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TD_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}

/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "gps_event_handler begin");
    gps_t *gps = NULL;
    switch (event_id)
    {
    case GPS_UPDATE:
        gps = (gps_t *)event_data;
        /* print information parsed from GPS statements */
        ESP_LOGI(TAG, "%d/%d/%d %d:%d:%d => \r\n"
                      "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                      "\t\t\t\t\t\tlongtitude = %.05f°E\r\n"
                      "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                      "\t\t\t\t\t\tspeed      = %fm/s",
                 gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
                 gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
                 gps->latitude, gps->longitude, gps->altitude, gps->speed);
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(TAG, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        ESP_LOGI(TAG, "gps_event_handler default output");
        break;
    }
}

void app_main()
{
    // init();

    // Function
    // xTaskCreate()
    // Parameters
    // pvTaskCode :
    // pcName :
    // usStackDepth :
    // pvParameters :
    // uxPriority :
    // pvCreatedTask :
    // xTaskCreate(rx_td_task, "uart_rx_td_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);

    /* NMEA parser configuration */
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    /* init NMEA parser library */
    nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
    /* register event handler for NMEA parser library */
    // ESP_LOGI(TAG, "Execute nmea_parser_add_handler()");
    esp_err_t nmea_parser_add_handler_err_code = nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);

    // ESP_LOGI(TAG, "Execute nmea_parser_add_handler() return: %d", nmea_parser_add_handler_err_code);

    vTaskDelay(10000000 / portTICK_PERIOD_MS);

    /* unregister event handler */
    nmea_parser_remove_handler(nmea_hdl, gps_event_handler);
    /* deinit NMEA parser library */
    nmea_parser_deinit(nmea_hdl);
}
