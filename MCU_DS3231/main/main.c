#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "C:\Users\Thang\esp\esp-idf\examples\common_components\protocol_examples_common\include\protocol_examples_common.h"
#include "DS3231.h"
#include "HD44780.h"
#include "adv_button.h"
#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lora.h"
#include "nvs_flash.h"

#define SDA_PIN         21
#define SCL_PIN         22
#define BUTTON_MENU     34
#define BUTTON_MODE     35
#define BUTTON_UP       33
#define BUTTON_DOWN     32
#define BUTTON_RELAY1   25
#define BUTTON_RELAY1   26
#define BUTTON_RELAY1   27
#define CONFIG_TIMEZONE 7

#define LCD_MENU_BIT    BIT1
#define UP_BIT          BIT2
#define DOWN_BIT        BIT3

#define NTP_SERVER      CONFIG_NTP_SERVER

static const char TAG[] = "DS3231";
uint32_t txBuf[128];
uint8_t buf[128];
char timeBuf[512];
uint8_t buffer[512];
uint8_t receive[3];

RTC_DATA_ATTR static int boot_count = 0;
gpio_num_t button;
adv_button_event_id_t event;
TaskHandle_t xHandle1 = NULL;
TaskHandle_t xHandle2 = NULL;
TaskHandle_t xHandle3 = NULL;
adv_button_event_id_t event1;
adv_button_config_t button_cfg = ADVANCED_BUTTON_CONFIG_DEFAULT();
EventGroupHandle_t xLCDButtonHandle;
typedef enum {
    MENU_LCD_1 = 0,
    MENU_LCD_2,
    MENU_LCD_3,
    MENU_LCD_4,
    MENU_LCD_MAX,
} menu_lcd_t;
menu_lcd_t menu_lcd = 0;

// union {
//     uint8_t raw[12];

//     struct {
//         float temp;
//         float humid;
//         float lux;
//     }
// } sensor1_data_t;

// typedef union {
//     uint8_t raw[5];

//     struct __attribute__((packed)) {
//         float ground;
//         uint8_t rain;
//     }
// } sensor2_data_t;

struct {
    char* ssid;
    bool wifi_connect;
    struct tm time;
} menu1_data;

// struct {
//     sensor1_data_t data1;
//     sensor2_data_t data2;
// } menu2_data;

typedef struct {
    float max_val;
    float min_val;
} sensor_param_data_t;

struct {
    sensor_param_data_t temp;
    sensor_param_data_t humid;
    sensor_param_data_t lux;
    sensor_param_data_t ground;
    enum {
        TEMP_MIN = 0,
        TEMP_MAX,
        HUMID_MIN,
        HUMID_MAX,
        LUX_MIN,
        LUX_MAX,
        GROUND_MIN,
        GROUND_MAX,
        CURSOR_MAX,
    } cursor;

    bool select;
    bool edit_param;
} menu3_data;

// declare function

void time_sync_notification_cb(struct timeval* tv);
static esp_err_t i2c_master_init(void);
static void initialize_sntp(void);
void getClock(void* pvParameters);
void menu_lcd4(void* pvParameters);
void getLoRa(void* pvParameters);

void menubutton_callback(adv_button_event_id_t event, void* params);
void up_press();
void down_press();
void upbutton_callback(adv_button_event_id_t event, void* params);
void downbutton_callback(adv_button_event_id_t event, void* params);
void button_hold(adv_button_event_id_t event, void* params);

// void handle_edit();
void handle_cursor();
void handle_lcd_menu();
void mode_hold();
void handledown_edit();
void handleup_edit();
// Start program
// void down_button(void) {
//     if (menu_lcd == MENU_LCD_3 && menu3_data.select)
//         if (menu3_data.cursor < CURSOR_MAX - 1) menu3_data.cursor++;
// }

// void up_button(void) {
//     if (menu_lcd == MENU_LCD_3 && menu3_data.select)
//         if (menu3_data.cursor > 0) menu3_data.cursor--;
// }

void mode_hold() {
    printf("Mode hold-----------------\n");
    if (menu_lcd == MENU_LCD_4) {
        printf("Mode hold----------------- lcd4\n");
        menu3_data.select ^= 1;
        if (menu3_data.select) {
            menu3_data.cursor = 0;
            LCD_setCursor(6, 0);
            blink();
        } else noBlink();
    }
}

void mode_single() {
    if (menu_lcd == MENU_LCD_4 && menu3_data.select) {
        menu3_data.edit_param ^= 1;
    }
}

/*-------------------------------------Button MODE
 * Callback------------------------------------*/
void button_hold(adv_button_event_id_t event, void* params) {
    printf("MODE Hold Callback\n");
    gpio_num_t button = (gpio_num_t)params;
    printf("%d\n", button);
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:
            mode_single();
            printf("MODE call back single \n");
            break;
        case ADV_BUTTON_EVENT_HOLD_PRESS:
            printf("MODE call back holddddddd \n");
            mode_hold();
            break;
        default:
            printf("mode bi loi -------\n");
            break;
    }
}

/*--------------------------------CALLBACK______________BUTTON_______________DOWN---------------------------*/
void downbutton_callback(adv_button_event_id_t event, void* params) {
    printf("DOWN Button Callback\n");
    gpio_num_t button = (gpio_num_t)params;
    printf("%d\n", button);
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:

            xEventGroupSetBits(xLCDButtonHandle, DOWN_BIT);
            printf("Vo ham adv button\n");
            break;

        default:
            printf("even: %d\n", event);
            break;
    }
}

/*-----------------------------------TASK______________BUTTON_______________DOWN-----------------------------*/
void downbutton_task(void* pvParameters) {
    while (1) {
        if (xEventGroupWaitBits(xLCDButtonHandle, DOWN_BIT, true, false,
                                portMAX_DELAY)) {
            printf("Vo ham adv up press button\n");
            down_press();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/*--------------------------------HANDLE______________BUTTON_______________DOWN---------------------------*/
void down_press() {
    if (menu3_data.cursor <= 0) menu3_data.cursor = TEMP_MIN;
    if (menu_lcd != MENU_LCD_4) {
        printf("Xin chao111111\n");
        return;
    }
    if (menu3_data.select == false) {
        printf("Xin chaoo22222222\n");
        return;
    }
    if (menu3_data.edit_param == true && menu3_data.select == true) {
        handledown_edit();
        printf("Handle down Edit\n");
    } else if (menu3_data.select == true) {
        menu3_data.cursor--;
        printf("Xin chao\n");
        handle_cursor();
    }
}

/*--------------------------------Callback_UpButton
 * ---------------------------*/
void upbutton_callback(adv_button_event_id_t event, void* params) {
    printf("Up Button Callback\n");
    gpio_num_t button = (gpio_num_t)params;
    printf("%d\n", button);
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:

            xEventGroupSetBits(xLCDButtonHandle, UP_BIT);
            printf("Vo ham adv button\n");
            break;

        default:
            printf("even: %d\n", event);
            break;
    }
}

/*-----------------------------------Task_BUTTONUP-----------------------------*/
void upbutton_task(void* pvParameters) {
    while (1) {
        if (xEventGroupWaitBits(xLCDButtonHandle, UP_BIT, true, false,
                                portMAX_DELAY)) {
            printf("Vo ham adv up press button\n");
            up_press();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/*--------------------------------Handle_UpButton
 * ---------------------------*/
void up_press() {
    if (menu3_data.cursor >= CURSOR_MAX - 1) menu3_data.cursor = GROUND_MAX;
    if (menu_lcd != MENU_LCD_4) {
        printf("Xin chao111111\n");
        return;
    }
    if (menu3_data.select == false) {
        printf("Xin chaoo22222222\n");
        return;
    }
    if (menu3_data.edit_param && menu3_data.select == true) {
        handleup_edit();
        printf("Handle Up Edit\n");
    } else if (menu3_data.select == true) {
        menu3_data.cursor++;
        printf("Xin chao\n");
        handle_cursor();
    }
}

/*--------------------------------Handle_Cursor
 * ---------------------------*/
void handle_cursor(void) {
    if (menu3_data.cursor == TEMP_MIN) {
        LCD_setCursor(6, 0);
        blink();
    } else if (menu3_data.cursor == TEMP_MAX) {
        LCD_setCursor(15, 0);
        blink();
    } else if (menu3_data.cursor == HUMID_MIN) {
        LCD_setCursor(6, 1);
        blink();
    } else if (menu3_data.cursor == HUMID_MAX) {
        LCD_setCursor(14, 1);
        blink();
    } else if (menu3_data.cursor == LUX_MIN) {
        LCD_setCursor(6, 2);
        blink();
    } else if (menu3_data.cursor == LUX_MAX) {
        LCD_setCursor(14, 2);
        blink();
    } else if (menu3_data.cursor == GROUND_MIN) {
        LCD_setCursor(6, 3);
        blink();
    } else if (menu3_data.cursor == GROUND_MAX) {
        LCD_setCursor(14, 3);
        blink();
    }
}
/*-------------------------------------Button Menu
 * Callback------------------------------------*/

void menubutton_callback(adv_button_event_id_t event, void* params) {
    printf("Menu Button Callback\n");
    gpio_num_t button = (gpio_num_t)params;
    printf("%d\n", button);
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:
            menu_lcd++;
            if (menu_lcd >= MENU_LCD_MAX) menu_lcd = 0;
            printf("menu :%d\n", menu_lcd);
            xEventGroupSetBits(xLCDButtonHandle, LCD_MENU_BIT);

            break;
        case ADV_BUTTON_EVENT_DOUBLE_PRESS:
            printf("Double press\n");
            break;
        default:
            printf("even: %d\n", event);
            break;
    }
}

/*-----------------------------------Task_LCD1-----------------------------*/
void lcd_task1(void* pvParameters) {
    while (1) {
        if (xEventGroupWaitBits(xLCDButtonHandle, LCD_MENU_BIT, true, false,
                                portMAX_DELAY)) {
            handle_lcd_menu();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/*-------------------------------------Handle_Task_LCD------------------------------*/
void handle_lcd_menu() {
    if (menu_lcd == MENU_LCD_2) {
        if (xHandle3 != NULL) {
            vTaskDelete(xHandle3);
        }
        xTaskCreate(&getClock, "getClock", 1024 * 4, NULL, 5, &xHandle1);
    }

    else if (menu_lcd == MENU_LCD_3) {
        if (xHandle1 != NULL) {
            vTaskDelete(xHandle1);
        }
        xTaskCreate(&getLoRa, "getLORA", 1024 * 4, NULL, 6, &xHandle2);

    }

    else if (menu_lcd == MENU_LCD_4) {
        if (xHandle2 != NULL) {
            vTaskDelete(xHandle2);
        }
        xTaskCreate(&menu_lcd4, "MENU_LCD4", 1024 * 4, NULL, 5, &xHandle3);
    }
}

void getLoRa(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // uint8_t receive[3];
        LCD_clearScreen();
        lora_receive();
        printf("Xin chao TMT\n");
        while (lora_received()) {
            printf("Xin chao VN123443455\n");
            lora_receive_packet(receive, sizeof(receive));
            buffer[0] = receive[0];
            buffer[1] = receive[1];
            buffer[2] = receive[2];

            printf("Xin chao VNdsdsdsdsdsd\n");
            for (int i = 0; i < 3; i++) {
                printf("Received: %d\n", buffer[i]);
            }

            lora_receive();
        }
        // vTaskDelay(1000/portMAX_DELAY);
        // Display on temperature
        LCD_setCursor(0, 0);
        sprintf(timeBuf, "Temperature: %02d*C", buffer[0]);
        LCD_writeStr(timeBuf);

        // Display on humidity
        LCD_setCursor(0, 1);
        sprintf(timeBuf, "Humidity: %02d", buffer[1]);
        LCD_writeStr(timeBuf);

        // // Display on temperature
        // LCD_setCursor(0, 2);
        // sprintf(timeBuf, "Temperature: %.2f C", temp);
        // LCD_writeStr(timeBuf);
        vTaskDelayUntil(&xLastWakeTime, 2000 / portTICK_RATE_MS);
    }
}

void menu_lcd4(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Get RTC date and time
    while (1) {
        // LCD_clearScreen();
        // Display on temperature
        LCD_setCursor(0, 0);
        sprintf(timeBuf, "Temp: %0.2fC -- %0.2fC", menu3_data.temp.min_val,
                menu3_data.temp.max_val);
        LCD_writeStr(timeBuf);

        // Display on humidity
        LCD_setCursor(0, 1);
        sprintf(timeBuf, "Humi: %0.2f -- %0.2f", menu3_data.humid.min_val,
                menu3_data.humid.max_val);
        LCD_writeStr(timeBuf);

        // // Display on soid sensor
        LCD_setCursor(0, 2);
        sprintf(timeBuf, "Soil: %0.2f -- %0.2f", menu3_data.ground.min_val,
                menu3_data.ground.max_val);
        LCD_writeStr(timeBuf);

        // Display on light intensiy
        LCD_setCursor(0, 3);
        sprintf(timeBuf, "Ligh: %0.2f -- %0.2f", menu3_data.lux.min_val,
                menu3_data.lux.max_val);
        LCD_writeStr(timeBuf);

        vTaskDelayUntil(&xLastWakeTime, 2000 / portTICK_RATE_MS);
    }
}
/*Handle Edit*/
void handleup_edit() {
    if (menu3_data.cursor == 0) {
        menu3_data.temp.min_val++;
        if (menu3_data.temp.min_val > 100) menu3_data.temp.min_val = 100;
    } else if (menu3_data.cursor == 1) {
        menu3_data.temp.max_val++;
        if (menu3_data.temp.max_val > 100) menu3_data.temp.min_val = 100;
    } else if (menu3_data.cursor == 2) {
        menu3_data.humid.min_val++;
        if (menu3_data.humid.min_val > 100) menu3_data.humid.min_val = 100;
    } else if (menu3_data.cursor == 3) {
        menu3_data.humid.max_val++;
        if (menu3_data.humid.max_val > 100) menu3_data.humid.min_val = 100;
    } else if (menu3_data.cursor == 4) {
        menu3_data.ground.min_val++;
        if (menu3_data.ground.min_val > 100) menu3_data.ground.min_val = 100;
    } else if (menu3_data.cursor == 5) {
        menu3_data.ground.max_val++;
        if (menu3_data.ground.max_val > 100) menu3_data.ground.min_val = 100;
    } else if (menu3_data.cursor == 6) {
        menu3_data.lux.min_val++;
        if (menu3_data.lux.min_val > 100) menu3_data.lux.min_val = 100;
    } else if (menu3_data.cursor == 7) {
        menu3_data.lux.max_val++;
        if (menu3_data.lux.max_val > 100) menu3_data.lux.min_val = 100;
    }
}

void handledown_edit() {
    if (menu3_data.cursor == 0) {
        menu3_data.temp.min_val--;
        if (menu3_data.temp.min_val < 0) menu3_data.temp.min_val = 0;
    } else if (menu3_data.cursor == 1) {
        menu3_data.temp.max_val--;
        if (menu3_data.temp.max_val < 0) menu3_data.temp.min_val = 0;
    } else if (menu3_data.cursor == 2) {
        menu3_data.humid.min_val--;
        if (menu3_data.humid.min_val < 0) menu3_data.humid.min_val = 0;
    } else if (menu3_data.cursor == 3) {
        menu3_data.humid.max_val--;
        if (menu3_data.humid.max_val < 0) menu3_data.humid.min_val = 0;
    } else if (menu3_data.cursor == 4) {
        menu3_data.ground.min_val--;
        if (menu3_data.ground.min_val < 0) menu3_data.ground.min_val = 0;
    } else if (menu3_data.cursor == 5) {
        menu3_data.ground.max_val--;
        if (menu3_data.ground.max_val < 0) menu3_data.ground.min_val = 0;
    } else if (menu3_data.cursor == 6) {
        menu3_data.lux.min_val--;
        if (menu3_data.lux.min_val < 0) menu3_data.lux.min_val = 0;
    } else if (menu3_data.cursor == 7) {
        menu3_data.lux.max_val--;
        if (menu3_data.lux.max_val < 0) menu3_data.lux.min_val = 0;
    }
}

/*------------------------------------------Set man hinh menu
 * 2:--------------------------------------*/
// void menu2(void* pvParameters) {
//     char timeBuf[128];
//     uint8_t buffer[512];
//     uint8_t receive[3];
//     TickType_t xLastWakeTime = xTaskGetTickCount();
//     // Get RTC date and time
//     while (1) {
//         LCD_home();
//         LCD_clearScreen();
//         lora_receive();
//         printf("Xin chao TMT\n");
//         while (lora_received()) {
//             printf("Xin chao VN123443455\n");
//             lora_receive_packet(receive, sizeof(receive));
//             buffer[0] = receive[0];
//             buffer[1] = receive[1];
//             buffer[2] = receive[2];
//             printf("Xin chao VNdsdsdsdsdsd\n");
//             for (int i = 0; i < 3; i++) {
//                 printf("Received: %d\n", buffer[i]);
//             }
//             lora_receive();
//         }
//         // Display on temperature
//         LCD_setCursor(0, 0);
//         sprintf(timeBuf, "Temperature: %02d*C", buffer[0]);
//         LCD_writeStr(timeBuf);

//         // Display on humidity
//         LCD_setCursor(0, 1);
//         sprintf(timeBuf, "Humidity: %02d", buffer[1]);
//         LCD_writeStr(timeBuf);

//         // // Display on temperature
//         // LCD_setCursor(0, 2);
//         // sprintf(timeBuf, "Temperature: %.2f C", temp);
//         // LCD_writeStr(timeBuf);
//         vTaskDelayUntil(&xLastWakeTime, 1000);
//     }
// }

/*------------------------------------------------------------Time Sync
 * Notify
 * callback--------------------------------------------------------------------------------------------*/
void time_sync_notification_cb(struct timeval* tv) {
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

/*------------------------------------------------------------Init
 * SNTP--------------------------------------------------------------------------------------------*/
static void initialize_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    // sntp_setservername(0, "pool.ntp.org");
    ESP_LOGI(TAG, "Your NTP Server is %s", NTP_SERVER);
    sntp_setservername(0, NTP_SERVER);
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();
}

/*------------------------------------------------------------Init I2c
 * Master
 * --------------------------------------------------------------------------------------------*/
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = SDA_PIN,
        .scl_io_num       = SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100 * 1000,
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    return ESP_OK;
}

/*------------------------------------------------------------Obtain
 * Time--------------------------------------------------------------------------------------------*/
static bool obtain_time(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    // tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in
     * menuconfig. Read "Establishing Wi-Fi or Ethernet Connection"
     * section in examples/protocols/README.md for more information
     * about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    initialize_sntp();

    // wait for time to be set
    int retry             = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET &&
           ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry,
                 retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    ESP_ERROR_CHECK(example_disconnect());
    if (retry == retry_count) return false;
    return true;
}

// Set clock
void setClock(void* pvParameters) {
    // obtain time over NTP
    ESP_LOGI(pcTaskGetTaskName(0),
             "Connecting to WiFi and getting time over NTP.");
    if (!obtain_time()) {
        ESP_LOGE(pcTaskGetTaskName(0), "Fail to getting time over NTP.");
        while (1) {
            vTaskDelay(1);
        }
    }

    // update 'now' variable with current time
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];
    time(&now);
    now = now + (CONFIG_TIMEZONE * 60 * 60);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(pcTaskGetTaskName(0), "The current date/time is: %s",
             strftime_buf);

    ESP_LOGD(pcTaskGetTaskName(0), "timeinfo.tm_sec=%d", timeinfo.tm_sec);
    ESP_LOGD(pcTaskGetTaskName(0), "timeinfo.tm_min=%d", timeinfo.tm_min);
    ESP_LOGD(pcTaskGetTaskName(0), "timeinfo.tm_hour=%d", timeinfo.tm_hour);
    ESP_LOGD(pcTaskGetTaskName(0), "timeinfo.tm_wday=%d", timeinfo.tm_wday);
    ESP_LOGD(pcTaskGetTaskName(0), "timeinfo.tm_mday=%d", timeinfo.tm_mday);
    ESP_LOGD(pcTaskGetTaskName(0), "timeinfo.tm_mon=%d", timeinfo.tm_mon);
    ESP_LOGD(pcTaskGetTaskName(0), "timeinfo.tm_year=%d", timeinfo.tm_year);

    struct tm time = {.tm_year = timeinfo.tm_year + 1900,
                      .tm_mon  = timeinfo.tm_mon,  // 0-based
                      .tm_mday = timeinfo.tm_mday,
                      .tm_hour = timeinfo.tm_hour,
                      .tm_min  = timeinfo.tm_min,
                      .tm_sec  = timeinfo.tm_sec};

    if (ds3231_set_time(&time) != ESP_OK) {
        ESP_LOGE(pcTaskGetTaskName(0), "Could not set time.");
        while (1) {
            vTaskDelay(1);
        }
    }
    ESP_LOGI(pcTaskGetTaskName(0), "Set initial date time done");

    // goto deep sleep
    const int deep_sleep_sec = 10;
    ESP_LOGI(pcTaskGetTaskName(0), "Entering deep sleep for %d seconds",
             deep_sleep_sec);
    esp_deep_sleep(1000000LL * deep_sleep_sec);
}

/*------------------------------------------------------------Get
 * Clock--------------------------------------------------------------------------------------------*/

void getClock(void* pvParameters) {
    char txtBuf[128];

    if (ds3231_init(I2C_NUM_0) != ESP_OK) {
        ESP_LOGE(pcTaskGetTaskName(0), "Could not init device decriptior. ");
        while (1)
            vTaskDelay(10);
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Get RTC date and time
    while (1) {
        float temp;
        struct tm rtcinfo;
        LCD_home();
        LCD_clearScreen();

        if (ds3231_get_temp_float(&temp) != ESP_OK) {
            ESP_LOGE(pcTaskGetTaskName(0), "Could not get temperature.");
            while (1) {
                vTaskDelay(1);
            }
        }
        if (ds3231_get_time(&rtcinfo) != ESP_OK) {
            ESP_LOGE(pcTaskGetTaskName(0), "Could not get time. ");
            while (1) {
                vTaskDelay(1);
            }
        }
        ESP_LOGI(pcTaskGetTaskName(0),
                 "%04d-%02d-%02d %02d:%02d:%02d, %.2f deg Cel", rtcinfo.tm_year,
                 rtcinfo.tm_mon + 1, rtcinfo.tm_mday, rtcinfo.tm_hour,
                 rtcinfo.tm_min, rtcinfo.tm_sec, temp);

        // Display on time
        LCD_setCursor(0, 0);
        sprintf(txtBuf, "Time: %02d:%02d:%02d", rtcinfo.tm_hour, rtcinfo.tm_min,
                rtcinfo.tm_sec);
        LCD_writeStr(txtBuf);

        // Display on date
        LCD_setCursor(0, 1);
        sprintf(txtBuf, "Date: %02d-%02d-%04d", rtcinfo.tm_mday,
                rtcinfo.tm_mon + 1, rtcinfo.tm_year);
        LCD_writeStr(txtBuf);

        // Display on temperature
        LCD_setCursor(0, 2);
        sprintf(txtBuf, "Temperature: %.2f C", temp);
        LCD_writeStr(txtBuf);

        vTaskDelayUntil(&xLastWakeTime, 1000);
    }
}

/*--------------------------------------------------------------Update
 * Time
 * now--------------------------------------------------------------*/

void diffClock(void* pvParameters) {
    // obtain time over NTP
    ESP_LOGI(pcTaskGetTaskName(0),
             "Connecting to WiFi and getting time over NTP.");
    if (!obtain_time()) {
        ESP_LOGE(pcTaskGetTaskName(0), "Fail to getting time over NTP.");
        while (1) {
            vTaskDelay(1);
        }
    }

    // update 'now' variable with current time
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];
    time(&now);
    now = now + (CONFIG_TIMEZONE * 60 * 60);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%m-%d-%y %H:%M:%S",
             &timeinfo);
    ESP_LOGI(pcTaskGetTaskName(0), "NTP date/time is: %s", strftime_buf);

    // Initialize RTC
    // i2c_dev_t dev;
    // if (ds3231_init_desc(&dev, I2C_NUM_0, CONFIG_SDA_GPIO,
    // CONFIG_SCL_GPIO)
    // != ESP_OK) {
    //     ESP_LOGE(pcTaskGetTaskName(0), "Could not init device
    //     descriptor."); while (1) { vTaskDelay(1); }
    // }

    // Get RTC date and time
    struct tm rtcinfo;
    if (ds3231_get_time(&rtcinfo) != ESP_OK) {
        ESP_LOGE(pcTaskGetTaskName(0), "Could not get time.");
        while (1) {
            vTaskDelay(1);
        }
    }
    rtcinfo.tm_year  = rtcinfo.tm_year - 1900;
    rtcinfo.tm_isdst = -1;
    ESP_LOGD(pcTaskGetTaskName(0), "%04d-%02d-%02d %02d:%02d:%02d",
             rtcinfo.tm_year, rtcinfo.tm_mon + 1, rtcinfo.tm_mday,
             rtcinfo.tm_hour, rtcinfo.tm_min, rtcinfo.tm_sec);

    // update 'rtcnow' variable with current time
    time_t rtcnow = mktime(&rtcinfo);
    localtime_r(&rtcnow, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%m-%d-%y %H:%M:%S",
             &timeinfo);
    ESP_LOGI(pcTaskGetTaskName(0), "RTC date/time is: %s", strftime_buf);

    // Get the time difference
    double x = difftime(rtcnow, now);
    ESP_LOGI(pcTaskGetTaskName(0), "Time difference is: %f", x);

    while (1) {
        vTaskDelay(1000);
    }
}
        struct tm rtc;
/*-----------------------Send_Data_LoRa---------------------- */
void task_tx(void* pvParameters) {

    uint8_t send[5];
    while (1) {
        send[0] = rtc.tm_hour;
        send[1] = rtc.tm_min;
        send[2] = rtc.tm_sec;
        printf("Xin chao Viet Nam SEND DATA LORA\n");
        lora_send_packet((uint8_t*)send, 5);

        vTaskDelay(1000);
    }
}

/*------------------------------------------------------------Get Data
 * From
 * LoRa-------------------------------------------------------------------------------*/
// void task_rx(void* pvParameters) {
//     uint8_t buffer[512];
//     uint8_t receive[3];
//     int x;
//     while (1) {
//         printf("Xin chao VN\n");
//         lora_receive();
//         printf("Xin chao TMT\n");
//         while (lora_received()) {
//             printf("Xin chao VN123443455\n");
//             lora_receive_packet(receive, sizeof(receive));
//             buffer[0] = receive[0];
//             buffer[1] = receive[1];
//             buffer[2] = receive[2];

//             printf("Xin chao VNdsdsdsdsdsd\n");
//             for (int i = 0; i < 3; i++) {
//                 printf("Received: %d\n", buffer[i]);
//             }

//             lora_receive();
//         }
//         vTaskDelay(1000);
//     }
// }

/*------------------------------------------------------------App
 * Main--------------------------------------------------------------------------------------------*/

void app_main(void) {
    xLCDButtonHandle               = xEventGroupCreate();

    adv_button_config_t button_cfg = ADVANCED_BUTTON_CONFIG_DEFAULT();
    ++boot_count;
    ESP_LOGI(TAG, "CONFIG_SCL_GPIO = %d", SCL_PIN);
    ESP_LOGI(TAG, "CONFIG_SDA_GPIO = %d", SDA_PIN);
    ESP_LOGI(TAG, "CONFIG_TIMEZONE= %d", CONFIG_TIMEZONE);
    ESP_LOGI(TAG, "Boot count: %d", boot_count);
    i2c_master_init();
    lcd_init(I2C_NUM_0);
    ds3231_init(I2C_NUM_0);
    // noBlink();

    /*      ---- Init LoRa  ------*/
    lora_init();
    lora_initialized();
    lora_set_frequency(433E6);
    lora_set_spreading_factor(7);
    lora_set_tx_power(17);
    lora_set_bandwidth(125E3);
    lora_set_coding_rate(5);
    lora_set_preamble_length(10);
    lora_set_sync_word(0xF3);
    lora_enable_crc();
    // // xTaskCreate(&task_rx, "task_rx", 2048, NULL, 5, NULL);

    // /*---- Init Button----*/
    adv_button_create(BUTTON_MENU, &button_cfg, menubutton_callback,
                      (void*)BUTTON_MENU);
    adv_button_create(BUTTON_UP, &button_cfg, upbutton_callback,
                      (void*)BUTTON_UP);
    adv_button_create(BUTTON_DOWN, &button_cfg, downbutton_callback,
                      (void*)BUTTON_DOWN);
    adv_button_create(BUTTON_MODE, &button_cfg, button_hold,
                      (void*)BUTTON_MODE);

    xTaskCreate(&lcd_task1, "LCD TASK", 1024 * 4, NULL, 6, NULL);
    xTaskCreate(&upbutton_task, "UP BUTTON", 1024 * 4, NULL, 6, NULL);
    xTaskCreate(&downbutton_task, "UP BUTTON", 1024 * 4, NULL, 6, NULL);
    xTaskCreate(&task_tx, "SEND DATA LORA", 1024 * 4, NULL, 7, NULL);

#if CONFIG_SET_CLOCK
    // Set clock & Get clock
    if (boot_count == 1) {
        xTaskCreate(setClock, "setClock", 1024 * 4, NULL, 2, NULL);
    }
    // else {
    //     xTaskCreate(getClock, "getClock", 1024 * 4, NULL, 2, NULL);
    // }
#endif

    // #if CONFIG_GET_CLOCK
    //     // Get clock
    //     xTaskCreate(getClock, "getClock", 1024 * 4, NULL, 2, NULL);
    // #endif

#if CONFIG_DIFF_CLOCK
    // Diff clock
    xTaskCreate(diffClock, "diffClock", 1024 * 4, NULL, 2, NULL);

#endif
}