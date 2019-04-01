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

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nmea_parser.h"

/**
 * @brief NMEA Parser runtime buffer size
 *
 */
#define NMEA_PARSER_RUNTIME_BUFFER_SIZE (CONFIG_NMEA_PARSER_RING_BUFFER_SIZE / 2)
#define NMEA_MAX_STATEMENT_ITEM_LENGTH (16)
#define NMEA_EVENT_LOOP_QUEUE_SIZE (16)

/**
 * @brief TD0D01 DATA FRAME buffer size
 *
 */
#define TD0D01_DATA_FRAME_MAX_BUFFER_SIZE (960)

/**
 * @brief TD0D01 DATA FRAME Structure
 *
 */
static const uint8_t DATA_FRAME_HEADER[] = {0x23, 0x3E};
#define DATA_FRAME_HEADER_SIZE (2)
#define DATA_FRAME_IDCODE_SIZE (2)
#define DATA_FRAME_LENGTH_SIZE (2)
#define DATA_FRAME_LENGTH_INDEX (DATA_FRAME_HEADER_SIZE + DATA_FRAME_IDCODE_SIZE)
#define DATA_FRAME_CHKSUM_SIZE (2)

/* DRAM_ATTR is required to avoid UART array placed in flash, due to accessed from ISR */
static DRAM_ATTR uart_dev_t *const UART[UART_NUM_MAX] = {&UART0, &UART1, &UART2};

/**
 * @brief Define of NMEA Parser Event base
 *
 */
ESP_EVENT_DEFINE_BASE(ESP_NMEA_EVENT)

static const char *GPS_TAG = "nmea_parser";

static int event_count = 0;

/**
 * @brief GPS parser library runtime structure
 *
 */
typedef struct
{
    uint8_t item_pos;                              /*!< Current position in item */
    uint8_t item_num;                              /*!< Current item number */
    uint8_t asterisk;                              /*!< Asterisk detected flag */
    uint8_t crc;                                   /*!< Calculated CRC value */
    uint8_t parsed_statement;                      /*!< OR'd of statements that have been parsed */
    uint8_t sat_num;                               /*!< Satellite number */
    uint8_t sat_count;                             /*!< Satellite count */
    uint8_t cur_statement;                         /*!< Current statement ID */
    uint32_t all_statements;                       /*!< All statements mask */
    char item_str[NMEA_MAX_STATEMENT_ITEM_LENGTH]; /*!< Current item */
    gps_t parent;                                  /*!< Parent class */
    uart_port_t uart_port;                         /*!< Uart port number */
    uint8_t *buffer;                               /*!< Runtime buffer */
    uint16_t data_length;                          /*!< Data length*/
    uint8_t *data_frame_buffer;                    /*!< Data frame buffer */
    uint8_t *data_frame_buffer_index;              /*!< Data frame buffer index*/
    uint16_t data_frame_buffer_length;             /*!< Data frame buffer length*/
    esp_event_loop_handle_t event_loop_hdl;        /*!< Event loop handle */
    TaskHandle_t tsk_hdl;                          /*!< NMEA Parser task handle */
    QueueHandle_t event_queue;                     /*!< UART event queue handle */
} esp_gps_t;

/**
 * @brief parse latitude or longitude
 *              format of latitude in NMEA is ddmm.sss and longitude is dddmm.sss
 * @param esp_gps esp_gps_t type object
 * @return float Latitude or Longitude value (unit: degree)
 */
static float parse_lat_long(esp_gps_t *esp_gps)
{
    float ll = strtof(esp_gps->item_str, NULL);
    int deg = ((int)ll) / 100;
    float min = ll - (deg * 100);
    ll = deg + min / 60.0f;
    return ll;
}

/**
 * @brief Converter two continuous numeric character into a uint8_t number
 *
 * @param digit_char numeric character
 * @return uint8_t result of converting
 */
static inline uint8_t convert_two_digit2number(const char *digit_char)
{
    return 10 * (digit_char[0] - '0') + (digit_char[1] - '0');
}

/**
 * @brief Parse UTC time in GPS statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_utc_time(esp_gps_t *esp_gps)
{
    esp_gps->parent.tim.hour = convert_two_digit2number(esp_gps->item_str + 0);
    esp_gps->parent.tim.minute = convert_two_digit2number(esp_gps->item_str + 2);
    esp_gps->parent.tim.second = convert_two_digit2number(esp_gps->item_str + 4);
    if (esp_gps->item_str[6] == '.')
    {
        uint16_t tmp = 0;
        uint8_t i = 7;
        while (esp_gps->item_str[i])
        {
            tmp = 10 * tmp + esp_gps->item_str[i] - '0';
            i++;
        }
        esp_gps->parent.tim.thousand = tmp;
    }
}

#if CONFIG_NMEA_STATEMENT_GGA
/**
 * @brief Parse GGA statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_gga(esp_gps_t *esp_gps)
{
    /* Process GGA statement */
    switch (esp_gps->item_num)
    {
    case 1: /* Process UTC time */
        parse_utc_time(esp_gps);
        break;
    case 2: /* Latitude */
        esp_gps->parent.latitude = parse_lat_long(esp_gps);
        break;
    case 3: /* Latitude north(1)/south(-1) information */
        if (esp_gps->item_str[0] == 'S' || esp_gps->item_str[0] == 's')
        {
            esp_gps->parent.latitude *= -1;
        }
        break;
    case 4: /* Longitude */
        esp_gps->parent.longitude = parse_lat_long(esp_gps);
        break;
    case 5: /* Longitude east(1)/west(-1) information */
        if (esp_gps->item_str[0] == 'W' || esp_gps->item_str[0] == 'w')
        {
            esp_gps->parent.longitude *= -1;
        }
        break;
    case 6: /* Fix status */
        esp_gps->parent.fix = (gps_fix_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    case 7: /* Satellites in use */
        esp_gps->parent.sats_in_use = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    case 8: /* HDOP */
        esp_gps->parent.dop_h = strtof(esp_gps->item_str, NULL);
        break;
    case 9: /* Altitude */
        esp_gps->parent.altitude = strtof(esp_gps->item_str, NULL);
        break;
    case 11: /* Altitude above ellipsoid */
        esp_gps->parent.altitude += strtof(esp_gps->item_str, NULL);
        break;
    default:
        break;
    }
}
#endif

#if CONFIG_NMEA_STATEMENT_GSA
/**
 * @brief Parse GSA statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_gsa(esp_gps_t *esp_gps)
{
    /* Process GSA statement */
    switch (esp_gps->item_num)
    {
    case 2: /* Process fix mode */
        esp_gps->parent.fix_mode = (gps_fix_mode_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    case 15: /* Process PDOP */
        esp_gps->parent.dop_p = strtof(esp_gps->item_str, NULL);
        break;
    case 16: /* Process HDOP */
        esp_gps->parent.dop_h = strtof(esp_gps->item_str, NULL);
        break;
    case 17: /* Process VDOP */
        esp_gps->parent.dop_v = strtof(esp_gps->item_str, NULL);
        break;
    default:
        /* Parse satellite IDs */
        if (esp_gps->item_num >= 3 && esp_gps->item_num <= 14)
        {
            esp_gps->parent.sats_id_in_use[esp_gps->item_num - 3] = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        }
        break;
    }
}
#endif

#if CONFIG_NMEA_STATEMENT_GSV
/**
 * @brief Parse GSV statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_gsv(esp_gps_t *esp_gps)
{
    /* Process GSV statement */
    switch (esp_gps->item_num)
    {
    case 1: /* total GSV numbers */
        esp_gps->sat_count = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    case 2: /* Current GSV statement number */
        esp_gps->sat_num = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    case 3: /* Process satellites in view */
        esp_gps->parent.sats_in_view = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    default:
        if (esp_gps->item_num >= 4 && esp_gps->item_num <= 19)
        {
            uint8_t item_num = esp_gps->item_num - 4; /* Normalize item number from 4-19 to 0-15 */
            uint8_t index;
            uint32_t value;
            index = 4 * (esp_gps->sat_num - 1) + item_num / 4; /* Get array index */
            if (index < GPS_MAX_SATELLITES_IN_VIEW)
            {
                value = strtol(esp_gps->item_str, NULL, 10);
                switch (item_num % 4)
                {
                case 0:
                    esp_gps->parent.sats_desc_in_view[index].num = (uint8_t)value;
                    break;
                case 1:
                    esp_gps->parent.sats_desc_in_view[index].elevation = (uint8_t)value;
                    break;
                case 2:
                    esp_gps->parent.sats_desc_in_view[index].azimuth = (uint16_t)value;
                    break;
                case 3:
                    esp_gps->parent.sats_desc_in_view[index].snr = (uint8_t)value;
                    break;
                default:
                    break;
                }
            }
        }
        break;
    }
}
#endif

#if CONFIG_NMEA_STATEMENT_RMC
/**
 * @brief Parse RMC statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_rmc(esp_gps_t *esp_gps)
{
    /* Process GPRMC statement */
    switch (esp_gps->item_num)
    {
    case 1: /* Process UTC time */
        parse_utc_time(esp_gps);
        break;
    case 2: /* Process valid status */
        esp_gps->parent.valid = (esp_gps->item_str[0] == 'A');
        break;
    case 3: /* Latitude */
        esp_gps->parent.latitude = parse_lat_long(esp_gps);
        break;
    case 4: /* Latitude north(1)/south(-1) information */
        if (esp_gps->item_str[0] == 'S' || esp_gps->item_str[0] == 's')
        {
            esp_gps->parent.latitude *= -1;
        }
        break;
    case 5: /* Longitude */
        esp_gps->parent.longitude = parse_lat_long(esp_gps);
        break;
    case 6: /* Longitude east(1)/west(-1) information */
        if (esp_gps->item_str[0] == 'W' || esp_gps->item_str[0] == 'w')
        {
            esp_gps->parent.longitude *= -1;
        }
        break;
    case 7: /* Process ground speed in unit m/s */
        esp_gps->parent.speed = strtof(esp_gps->item_str, NULL) * 1.852;
        break;
    case 8: /* Process true course over ground */
        esp_gps->parent.cog = strtof(esp_gps->item_str, NULL);
        break;
    case 9: /* Process date */
        esp_gps->parent.date.day = convert_two_digit2number(esp_gps->item_str + 0);
        esp_gps->parent.date.month = convert_two_digit2number(esp_gps->item_str + 2);
        esp_gps->parent.date.year = convert_two_digit2number(esp_gps->item_str + 4);
        break;
    case 10: /* Process magnetic variation */
        esp_gps->parent.variation = strtof(esp_gps->item_str, NULL);
        break;
    default:
        break;
    }
}
#endif

#if CONFIG_NMEA_STATEMENT_GLL
/**
 * @brief Parse GLL statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_gll(esp_gps_t *esp_gps)
{
    /* Process GPGLL statement */
    switch (esp_gps->item_num)
    {
    case 1: /* Latitude */
        esp_gps->parent.latitude = parse_lat_long(esp_gps);
        break;
    case 2: /* Latitude north(1)/south(-1) information */
        if (esp_gps->item_str[0] == 'S' || esp_gps->item_str[0] == 's')
        {
            esp_gps->parent.latitude *= -1;
        }
        break;
    case 3: /* Longitude */
        esp_gps->parent.longitude = parse_lat_long(esp_gps);
        break;
    case 4: /* Longitude east(1)/west(-1) information */
        if (esp_gps->item_str[0] == 'W' || esp_gps->item_str[0] == 'w')
        {
            esp_gps->parent.longitude *= -1;
        }
        break;
    case 5: /* Process UTC time */
        parse_utc_time(esp_gps);
        break;
    case 6: /* Process valid status */
        esp_gps->parent.valid = (esp_gps->item_str[0] == 'A');
        break;
    default:
        break;
    }
}
#endif

#if CONFIG_NMEA_STATEMENT_VTG
/**
 * @brief Parse VTG statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_vtg(esp_gps_t *esp_gps)
{
    /* Process GPVGT statement */
    switch (esp_gps->item_num)
    {
    case 1: /* Process true course over ground */
        esp_gps->parent.cog = strtof(esp_gps->item_str, NULL);
        break;
    case 3: /* Process magnetic variation */
        esp_gps->parent.variation = strtof(esp_gps->item_str, NULL);
        break;
    case 5:                                                              /* Process ground speed in unit m/s */
        esp_gps->parent.speed = strtof(esp_gps->item_str, NULL) * 1.852; //knots to m/s
        break;
    case 7:                                                            /* Process ground speed in unit m/s */
        esp_gps->parent.speed = strtof(esp_gps->item_str, NULL) / 3.6; //km/h to m/s
        break;
    default:
        break;
    }
}
#endif

/**
 * @brief Parse received item
 *
 * @param esp_gps esp_gps_t type object
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t parse_item(esp_gps_t *esp_gps)
{
    esp_err_t err = ESP_OK;
    /* start of a statement */
    if (esp_gps->item_num == 0 && esp_gps->item_str[0] == '$')
    {
        if (0)
        {
        }
#if CONFIG_NMEA_STATEMENT_GGA
        else if (strstr(esp_gps->item_str, "GGA"))
        {
            esp_gps->cur_statement = STATEMENT_GGA;
        }
#endif
#if CONFIG_NMEA_STATEMENT_GSA
        else if (strstr(esp_gps->item_str, "GSA"))
        {
            esp_gps->cur_statement = STATEMENT_GSA;
        }
#endif
#if CONFIG_NMEA_STATEMENT_RMC
        else if (strstr(esp_gps->item_str, "RMC"))
        {
            esp_gps->cur_statement = STATEMENT_RMC;
        }
#endif
#if CONFIG_NMEA_STATEMENT_GSV
        else if (strstr(esp_gps->item_str, "GSV"))
        {
            esp_gps->cur_statement = STATEMENT_GSV;
        }
#endif
#if CONFIG_NMEA_STATEMENT_GLL
        else if (strstr(esp_gps->item_str, "GLL"))
        {
            esp_gps->cur_statement = STATEMENT_GLL;
        }
#endif
#if CONFIG_NMEA_STATEMENT_VTG
        else if (strstr(esp_gps->item_str, "VTG"))
        {
            esp_gps->cur_statement = STATEMENT_VTG;
        }
#endif
        else
        {
            esp_gps->cur_statement = STATEMENT_UNKNOWN;
        }
        goto out;
    }
    /* Parse each item, depend on the type of the statement */
    if (esp_gps->cur_statement == STATEMENT_UNKNOWN)
    {
        goto out;
    }
#if CONFIG_NMEA_STATEMENT_GGA
    else if (esp_gps->cur_statement == STATEMENT_GGA)
    {
        parse_gga(esp_gps);
    }
#endif
#if CONFIG_NMEA_STATEMENT_GSA
    else if (esp_gps->cur_statement == STATEMENT_GSA)
    {
        parse_gsa(esp_gps);
    }
#endif
#if CONFIG_NMEA_STATEMENT_GSV
    else if (esp_gps->cur_statement == STATEMENT_GSV)
    {
        parse_gsv(esp_gps);
    }
#endif
#if CONFIG_NMEA_STATEMENT_RMC
    else if (esp_gps->cur_statement == STATEMENT_RMC)
    {
        parse_rmc(esp_gps);
    }
#endif
#if CONFIG_NMEA_STATEMENT_GLL
    else if (esp_gps->cur_statement == STATEMENT_GLL)
    {
        parse_gll(esp_gps);
    }
#endif
#if CONFIG_NMEA_STATEMENT_VTG
    else if (esp_gps->cur_statement == STATEMENT_VTG)
    {
        parse_vtg(esp_gps);
    }
#endif
    else
    {
        err = ESP_FAIL;
    }
out:
    return err;
}

/**
 * @brief Parse NMEA statements from GPS receiver
 *
 * @param esp_gps esp_gps_t type object
 * @param len number of bytes to decode
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t gps_decode(esp_gps_t *esp_gps, size_t len)
{
    const uint8_t *d = esp_gps->buffer;
    while (*d)
    {
        /* Start of a statement */
        if (*d == '$')
        {
            /* Reset runtime information */
            esp_gps->asterisk = 0;
            esp_gps->item_num = 0;
            esp_gps->item_pos = 0;
            esp_gps->cur_statement = 0;
            esp_gps->crc = 0;
            esp_gps->sat_count = 0;
            esp_gps->sat_num = 0;
            /* Add character to item */
            esp_gps->item_str[esp_gps->item_pos++] = *d;
            esp_gps->item_str[esp_gps->item_pos] = '\0';
        }
        /* Detect item separator character */
        else if (*d == ',')
        {
            /* Parse current item */
            parse_item(esp_gps);
            /* Add character to CRC computation */
            esp_gps->crc ^= (uint8_t)(*d);
            /* Start with next item */
            esp_gps->item_pos = 0;
            esp_gps->item_str[0] = '\0';
            esp_gps->item_num++;
        }
        /* End of CRC computation */
        else if (*d == '*')
        {
            /* Parse current item */
            parse_item(esp_gps);
            /* Asterisk detected */
            esp_gps->asterisk = 1;
            /* Start with next item */
            esp_gps->item_pos = 0;
            esp_gps->item_str[0] = '\0';
            esp_gps->item_num++;
        }
        /* End of statement */
        else if (*d == '\r')
        {
            /* Convert received CRC from string (hex) to number */
            uint8_t crc = (uint8_t)strtol(esp_gps->item_str, NULL, 16);
            /* CRC passed */
            if (esp_gps->crc == crc)
            {
                switch (esp_gps->cur_statement)
                {
#if CONFIG_NMEA_STATEMENT_GGA
                case STATEMENT_GGA:
                    esp_gps->parsed_statement |= 1 << STATEMENT_GGA;
                    break;
#endif
#if CONFIG_NMEA_STATEMENT_GSA
                case STATEMENT_GSA:
                    esp_gps->parsed_statement |= 1 << STATEMENT_GSA;
                    break;
#endif
#if CONFIG_NMEA_STATEMENT_RMC
                case STATEMENT_RMC:
                    esp_gps->parsed_statement |= 1 << STATEMENT_RMC;
                    break;
#endif
#if CONFIG_NMEA_STATEMENT_GSV
                case STATEMENT_GSV:
                    if (esp_gps->sat_num == esp_gps->sat_count)
                    {
                        esp_gps->parsed_statement |= 1 << STATEMENT_GSV;
                    }
                    break;
#endif
#if CONFIG_NMEA_STATEMENT_GLL
                case STATEMENT_GLL:
                    esp_gps->parsed_statement |= 1 << STATEMENT_GLL;
                    break;
#endif
#if CONFIG_NMEA_STATEMENT_VTG
                case STATEMENT_VTG:
                    esp_gps->parsed_statement |= 1 << STATEMENT_VTG;
                    break;
#endif
                default:
                    break;
                }
                /* Check if all statements have been parsed */
                if (((esp_gps->parsed_statement) & esp_gps->all_statements) == esp_gps->all_statements)
                {
                    esp_gps->parsed_statement = 0;
                    /* Send signal to notify that GPS information has been updated */
                    esp_event_post_to(esp_gps->event_loop_hdl, ESP_NMEA_EVENT, GPS_UPDATE,
                                      &(esp_gps->parent), sizeof(gps_t), 100 / portTICK_PERIOD_MS);
                }
            }
            else
            {
                ESP_LOGD(GPS_TAG, "CRC Error for statement:%s", esp_gps->buffer);
            }
            if (esp_gps->cur_statement == STATEMENT_UNKNOWN)
            {
                /* Send signal to notify that one unknown statement has been met */
                esp_event_post_to(esp_gps->event_loop_hdl, ESP_NMEA_EVENT, GPS_UNKNOWN,
                                  esp_gps->buffer, len, 100 / portTICK_PERIOD_MS);
            }
        }
        /* Other non-space character */
        else
        {
            if (!(esp_gps->asterisk))
            {
                /* Add to CRC */
                esp_gps->crc ^= (uint8_t)(*d);
            }
            /* Add character to item */
            esp_gps->item_str[esp_gps->item_pos++] = *d;
            esp_gps->item_str[esp_gps->item_pos] = '\0';
        }
        /* Process next character */
        d++;
    }
    return ESP_OK;
}

/**
 * @brief Parse MEAS Data from GPS receiver
 *
 * @param data_frame uint8_t
 * @param len number of bytes to decode
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t meas_decode(uint8_t *data_frame, uint16_t len)
{
    ESP_LOGI(GPS_TAG, "ONE COMPLETE DATA FRAME");
    return ESP_OK;
}

/**
 * @brief Handle when a pattern has been detected by uart
 *
 * @param esp_gps esp_gps_t type object
 */
static void esp_handle_uart_pattern(esp_gps_t *esp_gps)
{
    ESP_LOGI(GPS_TAG, "Start execute esp_handle_uart_pattern() ...");
    int pos = uart_pattern_pop_pos(esp_gps->uart_port);

    ESP_LOGI(GPS_TAG, "Get the nearest detected pattern position: %d", pos);

    if (pos != -1)
    {
        /* read one line(include '\n') */
        int read_len = uart_read_bytes(esp_gps->uart_port, esp_gps->buffer, pos, 100 / portTICK_PERIOD_MS);

        /* make sure the line is a standard string */
        esp_gps->buffer[read_len] = '\0';

        if (read_len > 0)
        {
            ESP_LOGI(GPS_TAG, "%s", esp_gps->buffer);
            ESP_LOGI(GPS_TAG, "Read %d bytes: '%s'", read_len, esp_gps->buffer);
            ESP_LOG_BUFFER_HEXDUMP(GPS_TAG, esp_gps->buffer, read_len, ESP_LOG_INFO);
        }

        /* Send new line to handle */
        if (gps_decode(esp_gps, read_len + 1) != ESP_OK)
        {
            ESP_LOGW(GPS_TAG, "GPS decode line failed");
        }
    }
    else
    {
        ESP_LOGW(GPS_TAG, "Pattern Queue Size too small");
        uart_flush_input(esp_gps->uart_port);
    }
    ESP_LOGI(GPS_TAG, "End execute esp_handle_uart_pattern() ...");
}

/**
 * @brief NMEA Parser Task Entry
 *
 * @param arg argument
 */
static void nmea_parser_task_entry(void *arg)
{
    esp_gps_t *esp_gps = (esp_gps_t *)arg;
    uart_event_t event;

    /* 处理完整数据帧流程 */

    while (1)
    {
        if (xQueueReceive(esp_gps->event_queue, &event, pdMS_TO_TICKS(200)))
        {
            event_count = event_count + 1;
            ESP_LOGI(GPS_TAG, "Dealing with %d event in queue ...", event_count);
            switch (event.type)
            {
            case UART_DATA:
                ESP_LOGI(GPS_TAG, "Dealing with UART_DATA event");

                int length = 0;
                ESP_ERROR_CHECK(uart_get_buffered_data_len(esp_gps->uart_port, (size_t *)&length));
                length = uart_read_bytes(esp_gps->uart_port, esp_gps->buffer, length, 1000 / portTICK_RATE_MS);

                if (length > 0)
                {
                    esp_gps->buffer[length] = 0;
                    ESP_LOGI(GPS_TAG, "Read %d bytes: '%s'", length, esp_gps->buffer);
                    ESP_LOG_BUFFER_HEXDUMP(GPS_TAG, esp_gps->buffer, length, ESP_LOG_INFO);
                }

                /* 判断本次读取的数据以何种方式进行处理 */
                if (esp_gps->data_frame_buffer_length == 0)
                {
                    bool has_frame_header = true;
                    for (int i = 0; i < DATA_FRAME_HEADER_SIZE; i++)
                    {
                        if (has_frame_header)
                        {
                            if (esp_gps->buffer[i] != DATA_FRAME_HEADER[i])
                            {
                                has_frame_header = false;
                            }
                        }
                        else
                        {
                            break;
                        }
                    }
                    if (has_frame_header)
                    {
                        memcpy(esp_gps->data_frame_buffer_index, esp_gps->buffer, length);
                        esp_gps->data_frame_buffer_index = esp_gps->data_frame_buffer_index + length;
                        esp_gps->data_frame_buffer_length = esp_gps->data_frame_buffer_length + length;
                        // ESP_LOGI(GPS_TAG, "Data Frame Date Length: %d %d %d", esp_gps->data_frame_buffer[DATA_FRAME_LENGTH_INDEX], esp_gps->data_frame_buffer[DATA_FRAME_LENGTH_INDEX + 1], esp_gps->data_frame_buffer[DATA_FRAME_LENGTH_INDEX] + esp_gps->data_frame_buffer[DATA_FRAME_LENGTH_INDEX + 1] * 256);
                        esp_gps->data_length = esp_gps->data_frame_buffer[DATA_FRAME_LENGTH_INDEX] + esp_gps->data_frame_buffer[DATA_FRAME_LENGTH_INDEX + 1] * 256;
                    }
                }
                else if (esp_gps->data_frame_buffer_length > 0 && esp_gps->data_frame_buffer_length <= TD0D01_DATA_FRAME_MAX_BUFFER_SIZE)
                {
                    memcpy(esp_gps->data_frame_buffer_index, esp_gps->buffer, length);
                    esp_gps->data_frame_buffer_index = esp_gps->data_frame_buffer_index + length;
                    esp_gps->data_frame_buffer_length = esp_gps->data_frame_buffer_length + length;
                    ESP_LOGI(GPS_TAG, "data_frame_buffer_length: %d", esp_gps->data_frame_buffer_length);
                    ESP_LOG_BUFFER_HEXDUMP(GPS_TAG, esp_gps->data_frame_buffer, esp_gps->data_frame_buffer_length, ESP_LOG_INFO);

                    if (esp_gps->data_frame_buffer_length == esp_gps->data_length + 8)
                    {
                        meas_decode(esp_gps->data_frame_buffer, esp_gps->data_frame_buffer_length);
                        memset(esp_gps->data_frame_buffer, 0, sizeof(uint8_t) * TD0D01_DATA_FRAME_MAX_BUFFER_SIZE);
                        esp_gps->data_frame_buffer_index = esp_gps->data_frame_buffer;
                        esp_gps->data_frame_buffer_length = 0;
                    }
                }
                break;
            case UART_FIFO_OVF:
                ESP_LOGW(GPS_TAG, "HW FIFO Overflow");
                if (uart_flush(esp_gps->uart_port) != ESP_OK)
                {
                    ESP_LOGE(GPS_TAG, "UART ring buffer flush failed");
                }
                xQueueReset(esp_gps->event_queue);
                break;
            case UART_BUFFER_FULL:
                ESP_LOGW(GPS_TAG, "Ring Buffer Full");
                uart_flush(esp_gps->uart_port);
                xQueueReset(esp_gps->event_queue);
                break;
            case UART_BREAK:
                ESP_LOGW(GPS_TAG, "Rx Break");
                break;
            case UART_PARITY_ERR:
                ESP_LOGE(GPS_TAG, "Parity Error");
                break;
            case UART_FRAME_ERR:
                ESP_LOGE(GPS_TAG, "Frame Error");
                break;
            case UART_PATTERN_DET:
                ESP_LOGI(GPS_TAG, "Dealing with UART_PATTERN_DET event");
                esp_handle_uart_pattern(esp_gps);
                break;
            default:
                ESP_LOGW(GPS_TAG, "unknown uart event type: %d", event.type);
                break;
            }
        }
        /* Drive the event loop */
        esp_event_loop_run(esp_gps->event_loop_hdl, pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
}

/**
 * @brief Customed Interrupts
 *
 * @param arg Configuration of Customed Interrupts
 */
//中断服务函数放在IRAM中执行，所以在这里定义的时候要添加IRAM_ATTR
// static void IRAM_ATTR customed_uart_intr_handler(void *arg)
// {
//     esp_gps_t *esp_gps = (esp_gps_t *)arg;
//     uint8_t uart_num = esp_gps->uart_port;
//     volatile uart_dev_t *uart = UART[uart_num];
//     uint8_t recSize = 0;
//     uart->int_clr.rxfifo_full = 1;
//     uart->int_clr.frm_err = 1;
// }

/**
 * @brief Init NMEA Parser
 *
 * @param config Configuration of NMEA Parser
 * @return nmea_parser_handle_t handle of nmea_parser
 */
nmea_parser_handle_t nmea_parser_init(const nmea_parser_config_t *config)
{
    esp_gps_t *esp_gps = calloc(1, sizeof(esp_gps_t));
    if (!esp_gps)
    {
        ESP_LOGE(GPS_TAG, "calloc memory for esp_fps failed");
        goto err_gps;
    }
    esp_gps->buffer = calloc(NMEA_PARSER_RUNTIME_BUFFER_SIZE, sizeof(uint8_t));
    if (!esp_gps->buffer)
    {
        ESP_LOGE(GPS_TAG, "calloc memory for runtime buffer failed");
        goto err_buffer;
    }
    esp_gps->data_frame_buffer = calloc(TD0D01_DATA_FRAME_MAX_BUFFER_SIZE, sizeof(uint8_t));
    if (!esp_gps->data_frame_buffer)
    {
        ESP_LOGE(GPS_TAG, "calloc memory for data frame buffer failed");
        goto err_data_frame;
    }
    esp_gps->data_frame_buffer_index = esp_gps->data_frame_buffer;
    esp_gps->data_frame_buffer_length = 0;
#if CONFIG_NMEA_STATEMENT_GSA
    esp_gps->all_statements |= (1 << STATEMENT_GSA);
#endif
#if CONFIG_NMEA_STATEMENT_GSV
    esp_gps->all_statements |= (1 << STATEMENT_GSV);
#endif
#if CONFIG_NMEA_STATEMENT_GGA
    esp_gps->all_statements |= (1 << STATEMENT_GGA);
#endif
#if CONFIG_NMEA_STATEMENT_RMC
    esp_gps->all_statements |= (1 << STATEMENT_RMC);
#endif
#if CONFIG_NMEA_STATEMENT_GLL
    esp_gps->all_statements |= (1 << STATEMENT_GLL);
#endif
#if CONFIG_NMEA_STATEMENT_VTG
    esp_gps->all_statements |= (1 << STATEMENT_VTG);
#endif
    /* Set attributes */
    esp_gps->uart_port = config->uart.uart_port;
    esp_gps->all_statements &= 0xFE;
    /* Install UART driver */
    uart_config_t uart_config = {
        .baud_rate = config->uart.baud_rate,
        .data_bits = config->uart.data_bits,
        .parity = config->uart.parity,
        .stop_bits = config->uart.stop_bits,
        .flow_ctrl = config->uart.flow_ctrl,
        .rx_flow_ctrl_thresh = config->uart.rx_flow_ctrl_thresh};
    if (uart_param_config(esp_gps->uart_port, &uart_config) != ESP_OK)
    {
        ESP_LOGE(GPS_TAG, "config uart parameter failed");
        goto err_uart_config;
    }

    if (uart_set_pin(esp_gps->uart_port, UART_PIN_NO_CHANGE, config->uart.rx_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
    {
        ESP_LOGE(GPS_TAG, "config uart gpio failed");
        goto err_uart_config;
    }
    if (uart_driver_install(esp_gps->uart_port, CONFIG_NMEA_PARSER_RING_BUFFER_SIZE, 0,
                            config->uart.event_queue_size, &esp_gps->event_queue, 0) != ESP_OK)
    {
        ESP_LOGE(GPS_TAG, "install uart driver failed");
        goto err_uart_install;
    }
    /* Configure UART interrupts */
    /* 配置UART中断需要在uart_driver_install()函数执行后设置，否则在安装驱动时配置的中断会被默认值替换掉 */
    // uart_intr_config_t uart_intr = {
    //     .intr_enable_mask = config->uart.intr_enable_mask,
    //     .rxfifo_full_thresh = config->uart.rxfifo_full_thresh,
    //     .rx_timeout_thresh = config->uart.rx_timeout_thresh,
    //     .txfifo_empty_intr_thresh = config->uart.txfifo_empty_intr_thresh};
    // if (uart_intr_config(esp_gps->uart_port, &uart_intr) != ESP_OK)
    // {
    //     ESP_LOGE(GPS_TAG, "config uart interrupts failed");
    //     goto err_uart_config;
    // }
    // if (uart_enable_rx_intr(esp_gps->uart_port) != ESP_OK)
    // {
    //     ESP_LOGE(GPS_TAG, "enable uart rx interrupts failed");
    //     goto err_uart_config;
    // }
    /* Set pattern interrupt, used to detect the end of a line */
    // char uart_enable_pattern_det_intr_pattern_chr = '\n';
    // uart_enable_pattern_det_intr(esp_gps->uart_port, uart_enable_pattern_det_intr_pattern_chr, 1, 10000, 10, 10);
    /* Set pattern queue size */
    // uart_pattern_queue_reset(esp_gps->uart_port, config->uart.event_queue_size);
    // uart_flush(esp_gps->uart_port);

    /* Configure customed UART interrupts */
    // 这里是关键点，必须要先uart_driver_install安装驱动，再把中断服务给释放掉
    // if (uart_isr_free(esp_gps->uart_port) != ESP_OK)
    // {
    //     ESP_LOGE(GPS_TAG, "free an handler to service interrupts failed");
    //     goto err_uart_isr_free;
    // }
    // 重新注册中断服务函数
    // uart_isr_handle_t handle;
    // uart_isr_register(esp_gps->uart_port, customed_uart_intr_handler, &esp_gps, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, &handle);
    // 使能串口接收中断
    // uart_enable_rx_intr(esp_gps->uart_port);

    /* Create Event loop */
    esp_event_loop_args_t loop_args = {
        .queue_size = NMEA_EVENT_LOOP_QUEUE_SIZE,
        .task_name = NULL};
    if (esp_event_loop_create(&loop_args, &esp_gps->event_loop_hdl) != ESP_OK)
    {
        ESP_LOGE(GPS_TAG, "create event loop faild");
        goto err_eloop;
    }
    /* Create NMEA Parser task */
    BaseType_t err = xTaskCreate(
        nmea_parser_task_entry,
        "nmea_parser",
        CONFIG_NMEA_PARSER_TASK_STACK_SIZE,
        esp_gps,
        CONFIG_NMEA_PARSER_TASK_PRIORITY,
        &esp_gps->tsk_hdl);
    if (err != pdTRUE)
    {
        ESP_LOGE(GPS_TAG, "create NMEA Parser task failed");
        goto err_task_create;
    }
    ESP_LOGI(GPS_TAG, "NMEA Parser init OK");
    return esp_gps;
    /*Error Handling*/
err_task_create:
    esp_event_loop_delete(esp_gps->event_loop_hdl);
err_eloop:
// err_uart_isr_free:
err_uart_install:
    uart_driver_delete(esp_gps->uart_port);
err_uart_config:
err_data_frame:
    free(esp_gps->data_frame_buffer);
err_buffer:
    free(esp_gps->buffer);
err_gps:
    free(esp_gps);
    return NULL;
}

/**
 * @brief Deinit NMEA Parser
 *
 * @param nmea_hdl handle of NMEA parser
 * @return esp_err_t ESP_OK on success,ESP_FAIL on error
 */
esp_err_t nmea_parser_deinit(nmea_parser_handle_t nmea_hdl)
{
    esp_gps_t *esp_gps = (esp_gps_t *)nmea_hdl;
    vTaskDelete(esp_gps->tsk_hdl);
    esp_event_loop_delete(esp_gps->event_loop_hdl);
    esp_err_t err = uart_driver_delete(esp_gps->uart_port);
    free(esp_gps->buffer);
    free(esp_gps);
    return err;
}

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
esp_err_t nmea_parser_add_handler(nmea_parser_handle_t nmea_hdl, esp_event_handler_t event_handler, void *handler_args)
{
    esp_gps_t *esp_gps = (esp_gps_t *)nmea_hdl;
    return esp_event_handler_register_with(esp_gps->event_loop_hdl, ESP_NMEA_EVENT, ESP_EVENT_ANY_ID,
                                           event_handler, handler_args);
}

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
esp_err_t nmea_parser_remove_handler(nmea_parser_handle_t nmea_hdl, esp_event_handler_t event_handler)
{
    esp_gps_t *esp_gps = (esp_gps_t *)nmea_hdl;
    return esp_event_handler_unregister_with(esp_gps->event_loop_hdl, ESP_NMEA_EVENT, ESP_EVENT_ANY_ID, event_handler);
}
