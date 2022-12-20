
#include <iostream>

#include "DS3231.h"
#include "HD44780.h"
#include "adv_button.h"
#include "esp_firebase/app.h"
#include "esp_firebase/rtdb.h"
#include "firebase_config.h"
#include "jsoncpp/json.h"
#include "jsoncpp/value.h"
#include "lora.h"
#include "wifi_utils.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <cstdlib>
#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sleep.h"
#include "esp_sntp.h"

#include "esp_system.h"
#include "nvs_flash.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SDA_PIN         GPIO_NUM_21
#define SCL_PIN         GPIO_NUM_22
#define BUTTON_MENU     GPIO_NUM_34
#define BUTTON_MODE     GPIO_NUM_35
#define BUTTON_UP       GPIO_NUM_33
#define BUTTON_DOWN     GPIO_NUM_32
#define BUTTON_RELAY1   GPIO_NUM_25
#define BUTTON_RELAY2   GPIO_NUM_26
#define BUTTON_RELAY3   GPIO_NUM_27
#define CONFIG_TIMEZONE 7

#define LCD_MENU_BIT    BIT7
#define UP_BIT          BIT2
#define DOWN_BIT        BIT3
#define RELAY1_BIT      BIT4
#define RELAY2_BIT      BIT5
#define RELAY3_BIT      BIT6

#define NTP_SERVER      CONFIG_NTP_SERVER

using namespace ESPFirebase;
FirebaseApp app         = FirebaseApp(API_KEY);
RTDB db                 = RTDB(&app, DATABASE_URL);
RTDB dbbb               = RTDB(&app, DATABASE_URL);

static const char TAG[] = "DS3231";
uint32_t txBuf[128];
uint8_t buf[128];
char timeBuf[128];
uint8_t bufferRx[20];
uint8_t receive[20];
uint8_t send_lora[20];
b

RTC_DATA_ATTR static int boot_count = 0;
EventGroupHandle_t xLCDButtonHandle;
EventGroupHandle_t xUpdateFirebase;
static SemaphoreHandle_t xLoraData;

TaskHandle_t xHandle1 = NULL;
TaskHandle_t xHandle2 = NULL;
TaskHandle_t xHandle3 = NULL;
TaskHandle_t xHandle4 = NULL;
TaskHandle_t xHandle5 = NULL;

adv_button_event_id_t event;
adv_button_event_id_t event1;
adv_button_config_t button_cfg = ADVANCED_BUTTON_CONFIG_DEFAULT();

typedef enum {
    MENU_LCD_1 = 0,
    MENU_LCD_2,
    MENU_LCD_3,
    MENU_LCD_4,
    MENU_LCD_MAX,
} menu_lcd_t;
menu_lcd_t menu_lcd = MENU_LCD_1;

typedef struct {
    int max_val;
    int min_val;
} sensor_param_data_t;

static int temperature, hum, soil, lights;

typedef struct {
    int hour;
    int minutes;
    int statu;
} alarm_t;

struct {
    alarm_t lamp1;
    alarm_t lamp2;
    alarm_t fan1;
    alarm_t fan2;
    alarm_t pump1;
    alarm_t pump2;
    alarm_t humidifier1;
    alarm_t humidifier2;

    bool sta;
} firebase_data;

typedef enum {
    TEMP_MAX = 0,
    HUMID_MIN,
    GROUND_MIN,
    LUX_MIN,
    CURSOR_MAX,
} cursor_enum;

struct {
    sensor_param_data_t temp;
    sensor_param_data_t humid;
    sensor_param_data_t lux;
    sensor_param_data_t ground;
    cursor_enum cursor;

    bool select;
    bool edit_param;
} menu4_data;

struct {
    bool relay1;
    bool relay2;
    bool relay3;
} status_relay;

Json::Value data;
Json::Value data1;
Json::Reader reader;
// declare function

void time_sync_notification_cb(struct timeval* tv);
static esp_err_t i2c_master_init(void);
static void initialize_sntp(void);
void task1(void* pvParameters);
void getClock(void* pvParameters);
void menu_lcd4(void* pvParameters);
void getLoRa(void* pvParameters);
// void task_relay1(void* pvParameters);
void task_relay2(void* pvParameters);
void task_tx(void* pvParameters);

void menubutton_callback(adv_button_event_id_t event, void* params);
void up_press();
void down_press();
void lcd_display(void);
void upbutton_callback(adv_button_event_id_t event, void* params);
void downbutton_callback(adv_button_event_id_t event, void* params);
void button_hold(adv_button_event_id_t event, void* params);
void button_relay1(adv_button_event_id_t event, void* params);
void button_relay2(adv_button_event_id_t event, void* params);
// void init_firebase(void);
// void handle_firebase(RTDB db);
// void init_lora(void);

// void handle_edit();
void handle_cursor();
void handle_lcd_menu();
void mode_hold();
void handledown_edit();
void handleup_edit();
void lcd_menu2(void);
void menu_lcd3(void* pvParameters);

/*-------------------------------------Button_RELAY1_Callback------------------------------------*/
void button_relay1(adv_button_event_id_t event, void* params) {
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:
            status_relay.relay1 ^= 1;
            // xEventGroupSetBits(xLCDButtonHandle, RELAY1_BIT);
            if (status_relay.relay1 == true) {
                // send_lora[0]    = 1;
                data["S1_Lamp"] = "1";
                db.patchData("RELAY1", data);
                printf("status = 1\n");
            } else if (status_relay.relay1 == false) {
                // send_lora[0]    = 2;
                data["S1_Lamp"] = "0";
                db.patchData("RELAY1", data);
                printf("status = 1\n");
            }

            break;
        default:
            break;
    }
}
/*
void task_relay1(void* pvParameters) {
    while (1) {
        if (xEventGroupWaitBits(xLCDButtonHandle, RELAY1_BIT, true, false,
                                portMAX_DELAY)) {
            if (status_relay.relay1 == true) {
                data["S1_Lamp"] = "1";
                db.patchData("RELAY1", data);
                printf("status = 1\n");
            } else if (status_relay.relay1 == false) {
                data["S1_Lamp"] = "0";
                db.patchData("RELAY1", data);
            }
        }
    }
}
*/

/*-------------------------------------Button_RELAY2_Callback------------------------------------*/
void button_relay2(adv_button_event_id_t event, void* params) {
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:
            status_relay.relay2 ^= 1;
            xEventGroupSetBits(xLCDButtonHandle, RELAY2_BIT);

            break;
        default:
            break;
    }
}

void task_relay2(void* pvParameters) {
    while (1) {
        if (xEventGroupWaitBits(xLCDButtonHandle, RELAY2_BIT, true, false,
                                portMAX_DELAY)) {
            if (status_relay.relay2 == true) {
                data["S2_Fan"] = "1";
                //   vTaskDelay(10/portTICK_RATE_MS);
                db.patchData("RELAY2", data);

            } else if (status_relay.relay2 == false) {
                data["S2_Fan"] = "0";
                // vTaskDelay(10/portTICK_RATE_MS);
                db.patchData("RELAY2", data);
            }
        }
    }
}

/*-------------------------------------Button_MODE_Callback------------------------------------*/
void button_hold(adv_button_event_id_t event, void* params) {
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:
            if (menu_lcd == MENU_LCD_4 && menu4_data.select == true) {
                menu4_data.edit_param ^= 1;
            }
            break;
        case ADV_BUTTON_EVENT_HOLD_PRESS:
            if (menu_lcd == MENU_LCD_4) {
                menu4_data.select ^= 1;
                if (menu4_data.select) {
                    menu4_data.cursor = TEMP_MAX;
                    LCD_setCursor(11, 0);
                    blink();
                } else noBlink();
            }
            break;
        default:
            break;
    }
}

/*--------------------------------CALLBACK______________BUTTON_______________DOWN---------------------------*/
void downbutton_callback(adv_button_event_id_t event, void* params) {
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:
            // xEventGroupSetBits(xLCDButtonHandle, DOWN_BIT);
            if (menu4_data.cursor <= TEMP_MAX) menu4_data.cursor = TEMP_MAX;
            if (menu_lcd != MENU_LCD_4) {
                return;
            }
            if (menu4_data.select == false) {
                return;
            }
            if (menu4_data.edit_param == true && menu4_data.select == true) {
                handledown_edit();
            } else if (menu4_data.select == true) {
                menu4_data.cursor = (cursor_enum)((int)menu4_data.cursor - 1);
                handle_cursor();
            }
            break;

        default:
            break;
    }
}

/*--------------------------------Callback_UpButton---------------------------*/
void upbutton_callback(adv_button_event_id_t event, void* params) {
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:
            // xEventGroupSetBits(xLCDButtonHandle, UP_BIT);
            if (menu4_data.cursor >= CURSOR_MAX - 1)
                menu4_data.cursor = LUX_MIN;
            if (menu_lcd != MENU_LCD_4) {
                return;
            }
            if (menu4_data.select == false) {
                return;
            }
            if (menu4_data.edit_param && menu4_data.select == true) {
                handleup_edit();
            } else if (menu4_data.select == true) {
                menu4_data.cursor = (cursor_enum)((int)menu4_data.cursor + 1);
                handle_cursor();
            }
            break;
        default:
            break;
    }
}

/*--------------------------------Handle_Cursor---------------------------*/
void handle_cursor(void) {
    switch (menu4_data.cursor) {
        case TEMP_MAX:
            LCD_setCursor(11, 0);
            blink();
            break;
        case HUMID_MIN:
            LCD_setCursor(11, 1);
            blink();
            break;
        case GROUND_MIN:
            LCD_setCursor(11, 2);
            blink();
            break;
        case LUX_MIN:
            LCD_setCursor(11, 3);
            blink();
            break;
        default:
            break;
    }
}
/*-------------------------------------Button
 * MenuCallback------------------------------------*/

void menubutton_callback(adv_button_event_id_t event, void* params) {
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:
            menu_lcd = (menu_lcd_t)(int)(menu_lcd + 1);
            if (menu_lcd >= MENU_LCD_MAX) menu_lcd = MENU_LCD_2;
            xEventGroupSetBits(xLCDButtonHandle, LCD_MENU_BIT);
            break;
        case ADV_BUTTON_EVENT_DOUBLE_PRESS:
            break;
        default:
            break;
    }
}

/*-----------------------------------Task_LCD1-----------------------------*/
void lcd_task1(void* pvParameters) {
    while (1) {
        if (xEventGroupWaitBits(xLCDButtonHandle, LCD_MENU_BIT, true, false,
                                portMAX_DELAY)) {
            switch (menu_lcd) {
                case MENU_LCD_2:
                    if (xHandle3 != NULL) {
                        vTaskDelete(xHandle3);
                    }
                    xTaskCreate(&getClock, "getClock", 1024 * 4, NULL, 6,
                                &xHandle1);
                    break;
                case MENU_LCD_3:
                    if (xHandle1 != NULL) {
                        vTaskDelete(xHandle1);
                    }
                    // vTaskSuspend(xHandle4);

                    xTaskCreate(&menu_lcd3, "MenuLCD 3", 1024 * 4, NULL, 6,
                                &xHandle2);

                    break;
                case MENU_LCD_4:
                    if (xHandle2 != NULL) {
                        vTaskDelete(xHandle2);
                    }
                    // vTaskResume(xHandle4);
                    xTaskCreate(&menu_lcd4, "MENU_LCD4", 1024 * 4, NULL, 6,
                                &xHandle3);
                    break;
                default:
                    break;
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/*----------------------------GET_LORA-----------------------------*/

void menu_lcd3(void* pvParameters) {
    while (1) {
        LCD_clearScreen();
        // Display on temperature
        LCD_setCursor(0, 0);
        sprintf(timeBuf, "Temperature: %02doC", bufferRx[0]);
        LCD_writeStr(timeBuf);

        // Display on humidity
        LCD_setCursor(0, 1);
        sprintf(timeBuf, "Humidity: %02d", bufferRx[1]);
        LCD_writeStr(timeBuf);

        LCD_setCursor(0, 2);
        sprintf(timeBuf, "Lights: %02dlux", bufferRx[2]);
        LCD_writeStr(timeBuf);

        // data1["sensor/dht22/temp"] = "bufferRx[0]";
        // db.patchData("TGarden", data1);
        // data1["sensor/dht22/hum"] = "bufferRx[1]";
        // db.patchData("TGarden", data1);
        // data1["sensor/lights"] = "bufferRx[2]";
        // db.patchData("TGarden", data1);

        vTaskDelay(30000 / portTICK_RATE_MS);
    }
}

void menu_lcd4(void* pvParameters) {
    // TickType_t xLastWakeTime = xTaskGetTickCount();

    // Get RTC date and time
    while (1) {
        // LCD_clearScreen();
        // Display on temperature
        LCD_clearScreen();
        LCD_setCursor(0, 0);
        sprintf(timeBuf, "Max_Temp: %doC", temperature);
        LCD_writeStr(timeBuf);

        // Display on humidity
        LCD_setCursor(0, 1);
        sprintf(timeBuf, "Min_Humi: %d", hum);
        LCD_writeStr(timeBuf);

        // // Display on soid sensor
        LCD_setCursor(0, 2);
        sprintf(timeBuf, "Min_Soil: %d", soil);
        LCD_writeStr(timeBuf);

        // Display on light intensiy
        LCD_setCursor(0, 3);
        sprintf(timeBuf, "Min_Ligh: %dlux", lights);
        LCD_writeStr(timeBuf);

        // data1["Auto/Temp_high"] = "menu4_data.temp.max_val";
        // db.patchData("TGarden", data1);

        // data1["Auto/Hum"] = "menu4_data.humid.min_val";
        // db.patchData("TGarden", data1);

        // data1["Auto/Soil"] = "menu4_data.ground.min_val";
        // db.patchData("TGarden", data1);

        // data1["Auto/Lights"] = "menu4_data.lux.min_val";
        // db.patchData("TGarden", data1);
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}
/*Handle Edit*/
void handleup_edit() {
    if (menu4_data.cursor == 0) {
        temperature++;
        if (temperature > 100) temperature = 100;
    } else if (menu4_data.cursor == 1) {
        hum++;
        if (hum > 100) hum = 100;
    } else if (menu4_data.cursor == 2) {
        soil++;
        if (soil > 100) soil = 100;
    } else if (menu4_data.cursor == 3) {
        lights++;
        if (lights > 100) lights = 100;
    }
}

void handledown_edit() {
    if (menu4_data.cursor == 0) {
        temperature--;
        if (temperature < 0) temperature = 0;
    } else if (menu4_data.cursor == 1) {
        hum--;
        if (hum < 0) hum = 0;
    } else if (menu4_data.cursor == 2) {
        soil--;
        if (soil < 0) soil = 0;
    } else if (menu4_data.cursor == 3) {
        lights--;
        if (lights < 0) lights = 0;
    }
}

/*------------------------------------------------------------Time_Sync_Notify_callback--------------------------------------------------------------------------------------------*/
void time_sync_notification_cb(struct timeval* tv) {
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

/*------------------------------------------------------------Init_SNTP--------------------------------------------------------------------------------------------*/
static void initialize_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    // sntp_setservername(0, "pool.ntp.org");
    ESP_LOGI(TAG, "Your NTP Server is %s", NTP_SERVER);
    sntp_setservername(0, NTP_SERVER);
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();
}

/*------------------------------------------------------------Init_I2c_Master
 * --------------------------------------------------------------------------------------------*/
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode          = I2C_MODE_MASTER,
        .sda_io_num    = SDA_PIN,
        .scl_io_num    = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    conf.master.clk_speed = 100 * 1000;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    return ESP_OK;
}

// Set clock
void setClock(void* pvParameters) {
    // obtain time over NTP
    ESP_LOGI(pcTaskGetTaskName(0),
             "Connecting to WiFi and getting time over NTP.");
    // if (!obtain_time()) {
    //     ESP_LOGE(pcTaskGetTaskName(0), "Fail to getting time over NTP.");
    //     while (1) {
    //         vTaskDelay(1);
    //     }
    // }

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

    struct tm time = {.tm_sec  = timeinfo.tm_sec,
                      .tm_min  = timeinfo.tm_min,
                      .tm_hour = timeinfo.tm_hour,
                      .tm_mday = timeinfo.tm_mday,
                      .tm_mon  = timeinfo.tm_mon,
                      .tm_year = (timeinfo.tm_year + 1900)};

    if (ds3231_set_time(&time) != ESP_OK) {
        ESP_LOGE(pcTaskGetTaskName(0), "Could not set time.");
        while (1) {
            vTaskDelay(1);
        }
    }
    ESP_LOGI(pcTaskGetTaskName(0), "Set initial date time done");

    // goto deep sleep
    const int deep_sleep_sec = 2;
    ESP_LOGI(pcTaskGetTaskName(0), "Entering deep sleep for %d seconds",
             deep_sleep_sec);
    esp_deep_sleep(1000000LL * deep_sleep_sec);
}

/*------------------------------------------------------------Get_Clock--------------------------------------------------------------------------------------------*/

void getClock(void* pvParameters) {
    char txtBuf[128];

    if (ds3231_init(I2C_NUM_0) != ESP_OK) {
        ESP_LOGE(pcTaskGetTaskName(0), "Could not init device decriptior. ");
        while (1)
            vTaskDelay(10);
    }

    // TickType_t xLastWakeTime = xTaskGetTickCount();

    // Get RTC date and time
    while (1) {
        float temp;
        struct tm rtcinfo;
        EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
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

        // Display staus wifi
        if (bits & WIFI_CONNECTED_BIT) {
            LCD_setCursor(0, 2);
            sprintf(txtBuf, "WIFI is connected");
            LCD_writeStr(txtBuf);
        } else if (bits & WIFI_FAIL_BIT) {
            LCD_setCursor(0, 2);
            sprintf(txtBuf, "Fail connected to WIFI");
            LCD_writeStr(txtBuf);
        }
        // Display on temperature
        // LCD_setCursor(0, 2);
        // sprintf(txtBuf, "Temperature: %.2f C", temp);
        // LCD_writeStr(txtBuf);

        vTaskDelay(10000 / portTICK_RATE_MS);
        // vTaskDelayUntil(&xLastWakeTime, 100);
    }
}

/*--------------------------------------------------------------Update_Time_now--------------------------------------------------------------*/

/*-----------------------Send_Data_LoRa---------------------- */
void task_tx(void* pvParameters) {
    while (1) {
        printf("gia tri resp%d\n", send_lora[1]);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        lora_send_packet((uint8_t*)send_lora, 20);
    }
}

void update_firebase_button(void* pvParameters) {
    while (1) {
        if (xLoraData != NULL) {
            if (xSemaphoreTake(xLoraData, portMAX_DELAY) == pdTRUE) {
                Json::Value S1_Lamp = db.getData("RELAY1/S1_Lamp");
                if (S1_Lamp == "1") {
                    send_lora[0] = 1;
                    lora_send_packet((uint8_t*)send_lora, 20);
                } else if (S1_Lamp == "0") {
                    send_lora[0] = 2;
                    lora_send_packet((uint8_t*)send_lora, 20);
                }
                lora_send_packet((uint8_t*)send_lora, 20);

                Json::Value S2_Fan = db.getData("RELAY2/S2_Fan");
                if (S2_Fan == "1") {
                    send_lora[1] = 1;
                    lora_send_packet((uint8_t*)send_lora, 20);
                    status_relay.relay2 = true;
                } else if (S2_Fan == "0") {
                    send_lora[1] = 2;
                    lora_send_packet((uint8_t*)send_lora, 20);
                    status_relay.relay2 = false;
                }
                lora_send_packet((uint8_t*)send_lora, 20);
                Json::Value S3_Pump = db.getData("RELAY3/S3_Pump");
                if (S3_Pump == "1") {
                    send_lora[2] = 1;
                    lora_send_packet((uint8_t*)send_lora, 20);
                } else if (S1_Lamp == "0") {
                    send_lora[2] = 2;
                    lora_send_packet((uint8_t*)send_lora, 20);
                }
                lora_send_packet((uint8_t*)send_lora, 20);

                Json::Value S4_Hum = db.getData("RELAY4/S4_Hum");
                if (S3_Pump == "1") {
                    send_lora[3] = 1;
                    lora_send_packet((uint8_t*)send_lora, 20);
                } else if (S1_Lamp == "0") {
                    send_lora[3] = 2;
                    lora_send_packet((uint8_t*)send_lora, 20);
                }
                lora_send_packet((uint8_t*)send_lora, 20);
                xSemaphoreGive(xLoraData);

            } else {
                printf("bbbbbbbbbbbbbbbbb\n");
            }
        }
        vTaskDelay(500 / portTICK_RATE_MS);

        // Json::Value S1_Auto   = db.getData("S1_Auto");
        // Json::Value S2_Manual = db.getData("S2_Manual");

        // Json::Value S1_Auto   = db.getData("S1_Auto");
        // Json::Value S1_Manual = db.getData("S1_Manual");

        // Json::FastWriter writer;
        // std::string autoHum = writer.write(S1_Auto);
        // printf("Gia tri cua ban la: %s\n", autoHum.c_str());

        // ESP_LOGI("MAIN", "person3 as json string: \n%s",
        // autoHum.c_str());
    }
}

void update_dataFirebase(void* pvParameters) {
    while (1) {
        if (xLoraData != NULL) {
            if (xSemaphoreTake(xLoraData, portMAX_DELAY) == pdTRUE) {
                // Temp
                if (menu4_data.temp.max_val != temperature) {
                    menu4_data.temp.max_val = temperature;
                    data1["Auto/Temp_high"] = menu4_data.temp.max_val;
                    db.patchData("TGarden", data1);
                } else {
                    std::string Temp_max =
                        dbbb.getData("TGarden/Auto/Temp_high").asString();
                    menu4_data.temp.max_val = atoi(Temp_max.c_str());
                    temperature             = atoi(Temp_max.c_str());
                }
                // send_lora[21] = menu4_data.temp.max_val;
                // Hum
                if (menu4_data.humid.min_val != hum) {
                    menu4_data.humid.min_val = hum;
                    data1["Auto/Hum"]        = menu4_data.humid.min_val;
                    db.patchData("TGarden", data1);
                } else {
                    std::string Hum_min =
                        dbbb.getData("TGarden/Auto/Hum").asString();
                    menu4_data.humid.min_val = atoi(Hum_min.c_str());
                    hum                      = atoi(Hum_min.c_str());
                }
                // send_lora[22] = menu4_data.humid.min_val;

                // Lights
                if (menu4_data.lux.min_val != lights) {
                    menu4_data.lux.min_val = lights;
                    data1["Auto/Lights"]   = menu4_data.lux.min_val;
                    db.patchData("TGarden", data1);
                } else {
                    std::string Lights_min =
                        dbbb.getData("TGarden/Auto/Lights").asString();
                    menu4_data.lux.min_val = atoi(Lights_min.c_str());
                    lights                 = atoi(Lights_min.c_str());
                }
                // send_lora[23] = menu4_data.lux.min_val;
                // Ground
                if (menu4_data.ground.min_val != soil) {
                    menu4_data.ground.min_val = soil;
                    data1["Auto/Soil"]        = menu4_data.ground.min_val;
                    db.patchData("TGarden", data1);
                } else {
                    std::string Soil_min =
                        dbbb.getData("TGarden/Auto/Soil").asString();
                    menu4_data.ground.min_val = atoi(Soil_min.c_str());
                    soil                      = atoi(Soil_min.c_str());
                }
                // send_lora[24] = menu4_data.ground.min_val;

                printf("Gia tri nhiet do: %d\n", menu4_data.ground.min_val);
                // Doc gia tri hen gio cua quat
                std::string Alarm_Fan1_Hour =
                    db.getData("TGarden/Alarm/Fan/AmFan1/Hour").asString();
                firebase_data.fan1.hour = atoi(Alarm_Fan1_Hour.c_str());

                std::string Alarm_Fan1_Minu =
                    db.getData("TGarden/Alarm/Fan/AmFan1/Minutes").asString();
                firebase_data.fan1.minutes = atoi(Alarm_Fan1_Minu.c_str());

                std::string Alarm_Fan1_Sta =
                    db.getData("TGarden/Alarm/Fan/AmFan1/Status").asString();
                firebase_data.fan1.statu = atoi(Alarm_Fan1_Sta.c_str());

                std::string Alarm_Fan2_Hour =
                    db.getData("TGarden/Alarm/Fan/AmFan2/Hour").asString();
                firebase_data.fan2.hour = atoi(Alarm_Fan2_Hour.c_str());

                std::string Alarm_Fan2_Minu =
                    db.getData("TGarden/Alarm/Fan/AmFan2/Minutes").asString();
                firebase_data.fan2.minutes = atoi(Alarm_Fan2_Minu.c_str());

                std::string Alarm_Fan2_Sta =
                    db.getData("TGarden/Alarm/Fan/AmFan2/Status").asString();
                firebase_data.fan2.statu = atoi(Alarm_Fan2_Sta.c_str());

                // Doc gia tri hen gio cua den
                std::string Alarm_Lamp1_Hour =
                    db.getData("TGarden/Alarm/Lamp/AmLamp1/Hour").asString();
                firebase_data.lamp1.hour = atoi(Alarm_Lamp1_Hour.c_str());

                std::string Alarm_Lamp1_Minu =
                    db.getData("TGarden/Alarm/Lamp/AmLamp1/Minutes").asString();
                firebase_data.lamp1.minutes = atoi(Alarm_Lamp1_Minu.c_str());

                std::string Alarm_Lamp1_Sta =
                    db.getData("TGarden/Alarm/Lamp/AmLamp1/Status").asString();
                firebase_data.lamp1.statu = atoi(Alarm_Lamp1_Sta.c_str());

                std::string Alarm_Lamp2_Hour =
                    db.getData("TGarden/Alarm/Lamp/AmLamp2/Hour").asString();
                firebase_data.lamp2.hour = atoi(Alarm_Lamp2_Hour.c_str());

                std::string Alarm_Lamp2_Minu =
                    db.getData("TGarden/Alarm/Lamp/AmLamp2/Minutes").asString();
                firebase_data.lamp2.minutes = atoi(Alarm_Lamp2_Minu.c_str());

                std::string Alarm_Lamp2_Sta =
                    db.getData("TGarden/Alarm/Lamp/AmLamp2/Status").asString();
                firebase_data.lamp2.statu = atoi(Alarm_Lamp2_Sta.c_str());

                // Doc gia tri hen gio cua bom nuoc
                std::string Alarm_Pump1_Hour =
                    db.getData("TGarden/Alarm/Pump/AmPump1/Hour").asString();
                firebase_data.pump1.hour = atoi(Alarm_Pump1_Hour.c_str());

                std::string Alarm_Pump1_Minu =
                    db.getData("TGarden/Alarm/Pump/AmPump1/Minutes").asString();
                firebase_data.pump1.minutes = atoi(Alarm_Pump1_Minu.c_str());

                std::string Alarm_Pump1_Sta =
                    db.getData("TGarden/Alarm/Pump/AmPump1/Status").asString();
                firebase_data.pump1.statu = atoi(Alarm_Pump1_Sta.c_str());

                std::string Alarm_Pump2_Hour =
                    db.getData("TGarden/Alarm/Pump/AmPump2/Hour").asString();
                firebase_data.pump2.hour = atoi(Alarm_Pump2_Hour.c_str());

                std::string Alarm_Pump2_Minu =
                    db.getData("TGarden/Alarm/Pump/AmPump2/Minutes").asString();
                firebase_data.pump2.minutes = atoi(Alarm_Pump2_Minu.c_str());

                std::string Alarm_Pump2_Sta =
                    db.getData("TGarden/Alarm/Pump/AmPump2/Status").asString();
                firebase_data.pump2.statu = atoi(Alarm_Pump2_Sta.c_str());

                // Doc gia tri hen gio cua may phun suong
                std::string Alarm_Hum1_Hour =
                    db.getData("TGarden/Alarm/Humidifier/AmHum1/Hour")
                        .asString();
                firebase_data.humidifier1.hour = atoi(Alarm_Hum1_Hour.c_str());

                std::string Alarm_Hum1_Minu =
                    db.getData("TGarden/Alarm/Humidifier/AmHum1/Minutes")
                        .asString();
                firebase_data.humidifier1.minutes =
                    atoi(Alarm_Hum1_Minu.c_str());

                std::string Alarm_Hum1_Sta =
                    db.getData("TGarden/Alarm/Humidifier/AmHum1/Status")
                        .asString();
                firebase_data.humidifier1.statu = atoi(Alarm_Hum1_Sta.c_str());

                std::string Alarm_Hum2_Hour =
                    db.getData("TGarden/Alarm/Humidifier/AmHum2/Hour")
                        .asString();
                firebase_data.humidifier2.hour = atoi(Alarm_Hum2_Hour.c_str());

                std::string Alarm_Hum2_Minu =
                    db.getData("TGarden/Alarm/Humidifier/AmHum2/Minutes")
                        .asString();
                firebase_data.humidifier2.minutes =
                    atoi(Alarm_Hum2_Minu.c_str());

                std::string Alarm_Hum2_Sta =
                    db.getData("TGarden/Alarm/Humidifier/AmHum2/Status")
                        .asString();
                firebase_data.humidifier2.statu = atoi(Alarm_Hum2_Sta.c_str());

                printf("Gia tri min hum %d\n", menu4_data.humid.min_val);

                xSemaphoreGive(xLoraData);
            } else {
                printf("jaaaaaaaaaaaaaaaaaaaaaa\n");
            }

            // lora_send_packet((uint8_t*)send_lora, 20);
        }
        vTaskDelay(60000 / portTICK_RATE_MS);
    }
}

void getLoRa(void* pvParameters) {
    while (1) {
        if (xLoraData != NULL) {
            if (xSemaphoreTake(xLoraData, portMAX_DELAY) == pdTRUE) {
                lora_receive();
                printf("Xin chao TMT\n");

                while (lora_received()) {

                lora_receive_packet(receive, 20);
                lora_receive_packet(receive, 20);
                lora_receive_packet(receive, 20);
                bufferRx[0] = receive[10];
                bufferRx[1] = receive[11];
                bufferRx[2] = receive[12];
                printf("Kich thuoc cua size of: %d\n", sizeof(receive));
                for (int i = 10; i < 13; i++) {
                    printf("Received: %d\n", receive[i]);
                }

                lora_receive();
                data1["sensor/dht22/temp"] = bufferRx[0];
                db.patchData("TGarden", data1);
                data1["sensor/dht22/hum"] = bufferRx[1];
                db.patchData("TGarden", data1);
                data1["sensor/lights"] = bufferRx[2];
                db.patchData("TGarden", data1);
                }
                xSemaphoreGive(xLoraData);
            } else {
                printf("jaaaaaaaaaaaaaaaaaaaaaa\n");
            }
        }
        vTaskDelay(5000 / portTICK_RATE_MS);
    }
}

void lcd_display(void) {
    LCD_clearScreen();
    LCD_home();
    LCD_setCursor(0, 0);
    sprintf(timeBuf, "LUAN VAN TOT NGHIEP");
    LCD_writeStr(timeBuf);

    LCD_setCursor(0, 1);
    sprintf(timeBuf, "-------****--------");
    LCD_writeStr(timeBuf);
    // Display on humidity
    LCD_setCursor(1, 2);
    sprintf(timeBuf, "CHAM SOC CAY TRONG");
    LCD_writeStr(timeBuf);

    // Display on temperature
    LCD_setCursor(7, 3);
    sprintf(timeBuf, "TU DONG");
    LCD_writeStr(timeBuf);
}

void app_main(void) {
    ++boot_count;
    xLCDButtonHandle               = xEventGroupCreate();
    xLoraData                      = xSemaphoreCreateMutex();
    adv_button_config_t button_cfg = ADVANCED_BUTTON_CONFIG_DEFAULT();

    lora_init();
    lora_set_frequency(433e6);
    lora_enable_crc();
    lora_set_coding_rate(CONFIG_CODING_RATE);
    lora_set_bandwidth(CONFIG_BANDWIDTH);
    lora_set_spreading_factor(CONFIG_SF_RATE);

    //     /*Init Firebase*/
    wifiInit(SSID, PASSWORD);
    user_account_t account = {USER_EMAIL, USER_PASSWORD};
    // blocking until it connects
    initialize_sntp();
    // Config and Authentication
    app.loginUserAccount(account);

    i2c_master_init();
    lcd_init(I2C_NUM_0);
    ds3231_init(I2C_NUM_0);

#if CONFIG_SET_CLOCK
    if (boot_count == 1) {
        xTaskCreate(setClock, "setClock", 1024 * 4, NULL, 2, NULL);
    }
#endif

    // #if CONFIG_GET_CLOCK
    //     xTaskCreate(getClock, "getClock", 1024 * 4, NULL, 2, NULL);
    // #endif

    adv_button_create(BUTTON_MENU, &button_cfg, menubutton_callback,
                      (void*)BUTTON_MENU);
    adv_button_create(BUTTON_UP, &button_cfg, upbutton_callback,
                      (void*)BUTTON_UP);
    adv_button_create(BUTTON_DOWN, &button_cfg, downbutton_callback,
                      (void*)BUTTON_DOWN);
    adv_button_create(BUTTON_MODE, &button_cfg, button_hold,
                      (void*)BUTTON_MODE);

    adv_button_create(BUTTON_RELAY1, &button_cfg, button_relay1,
                      (void*)BUTTON_RELAY1);

    adv_button_create(BUTTON_RELAY2, &button_cfg, button_relay2,
                      (void*)BUTTON_RELAY2);

    xTaskCreate(&lcd_task1, "LCD TASK", 1024 * 4, NULL, 6, NULL);
    // xTaskCreate(&task_relay1, "RELAY1 BUTTON", 1024 * 4, NULL, 3, NULL);
    xTaskCreate(&task_relay2, "RELAY2 BUTTON", 1024 * 4, NULL, 6, NULL);
    xTaskCreate(&update_firebase_button, "UPDATE FIREBASE BUTTON", 1024 * 6,
                NULL, 6, &xHandle4);

    xTaskCreate(&getLoRa, "getLORA", 1024 * 4, NULL, 6, NULL);

    xTaskCreate(&update_dataFirebase, "updata", 1024 * 8, NULL, 6, NULL);

    lcd_display();
}
 
#ifdef __cplusplus
}
#endif
