#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "temp_sensor_driver.h"
#include "switch_driver.h"
#include <stdio.h>
#include "driver/gpio.h"
#include <stdlib.h>
#include <string.h>
#include "soc/soc_caps.h"
#include "esp_rom_sys.h"
#include "esp_zb_PZEM_004t.h"
#include "temp_sensor_driver.h"
#include "switch_driver.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile sensor (End Device) source code.
#endif

/*----------------------------------Macros----------------------------------*/
static const char *TAG = "ESP_ZB_SENSOR";

#define LIGHT_PIN GPIO_NUM_0
#define MOTION_PIN GPIO_NUM_10 
#define DHT11_PIN GPIO_NUM_11 

uint16_t Movement = 0;
uint16_t Humidity = 0;
uint16_t Temperatura = 0;
uint16_t Illuminance = 0;
uint16_t countdown = 60;

static int16_t zb_temperature_to_s16(float temp)
{
    return (int16_t)(temp * 100);
}

void delay_us(uint32_t us) {    
    esp_rom_delay_us(us);// include [us] for DHT data capture
}

void dht11_start_signal() {
    gpio_set_direction(DHT11_PIN, GPIO_MODE_OUTPUT);// sets pin as output
    gpio_set_level(DHT11_PIN, 0);
    vTaskDelay(20 / portTICK_PERIOD_MS);// sends a low for at least 18 ms
    gpio_set_level(DHT11_PIN, 1);
    delay_us(30);// sends a high for 20-40us
    gpio_set_direction(DHT11_PIN, GPIO_MODE_INPUT);// sets pin as input
}

int dht11_check_response() {
    int response = 0;
    delay_us(40);
    if (!gpio_get_level(DHT11_PIN)) {
        delay_us(80);
    if (gpio_get_level(DHT11_PIN)) response = 1;
        delay_us(40);
    }
    return response;
}

uint8_t dht11_read_byte() {
    uint8_t result = 0;
    for (int i = 0; i < 8; i++) {
        while (!gpio_get_level(DHT11_PIN));  //wait for signal to be High
        delay_us(30);
        if (gpio_get_level(DHT11_PIN)) result |= (1 << (7 - i));
        while (gpio_get_level(DHT11_PIN));  //wait for signal to be Low
}
return result;
}

void read_dht11_data(uint16_t *Humidity, uint16_t *Temperatura) {
    uint16_t hum_int, hum_dec, temp_int, temp_dec, checksum;
    dht11_start_signal();

    if (dht11_check_response()) {
        hum_int = dht11_read_byte();
        hum_dec = dht11_read_byte();
        temp_int = dht11_read_byte();
        temp_dec = dht11_read_byte();
        checksum = dht11_read_byte();

    if (hum_int + hum_dec + temp_int + temp_dec == checksum) {
        *Humidity = hum_int*100;
        *Temperatura = temp_int*100;
    } else {
        printf("Checksum error\n");
        }
    } else {
        printf("No response from DHT11\n");
        }
    }

static void esp_app_temp_sensor_handler(float temperature)
{
    
    /* Update temperature sensor measured value */

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &Temperatura, false);

    /* Update Humedad sensor measured value */
    esp_zb_zcl_set_attribute_val(HA_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &Humidity, false);

    /* Update movement sensor measured value */
    esp_zb_zcl_set_attribute_val(HA_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, &Movement, false); // OCCUPANCY SENSING 

    /* Update light sensor measured value */
/*    esp_zb_zcl_set_attribute_val(HA_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID, &Illuminance, false); //ILLUMINANCE 
*/

    esp_zb_lock_release();
}
/********************* Define functions **************************/
static esp_err_t deferred_driver_init(void)
{
    temperature_sensor_config_t temp_sensor_config =TEMPERATURE_SENSOR_CONFIG_DEFAULT(ESP_TEMP_SENSOR_MIN_VALUE, ESP_TEMP_SENSOR_MAX_VALUE);
    ESP_RETURN_ON_ERROR(temp_sensor_driver_init(&temp_sensor_config, ESP_TEMP_SENSOR_UPDATE_INTERVAL, esp_app_temp_sensor_handler), TAG,
                        "Failed to initialize temperature sensor");
    return ESP_OK;
}
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            } else {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_configuration_tool_cfg_t sensor_cfg =ESP_ZB_DEFAULT_CONFIGURATION_TOOL_CONFIG();

    /* endpoint configuration */

    esp_zb_ep_list_t *esp_zb_sensor_ep = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_SENSOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_COMBINED_INTERFACE_DEVICE_ID,
        .app_device_version = 0 
    };

    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(sensor_cfg.basic_cfg));

    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(sensor_cfg.identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

    
    /* creating clusters and adding attributes */

    uint16_t undefined_value;
    undefined_value = 0x0000;

     /* Temperature cluster */
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    esp_zb_temperature_meas_cluster_add_attr ( esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr ( esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr ( esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /* Humidity cluster */
    esp_zb_attribute_list_t *esp_zb_humidity_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    esp_zb_humidity_meas_cluster_add_attr ( esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_humidity_meas_cluster_add_attr ( esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID, &undefined_value);
    esp_zb_humidity_meas_cluster_add_attr ( esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    

    /* Occupancy cluster */
    esp_zb_attribute_list_t *esp_zb_occupancy_sensing_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING);
    esp_zb_occupancy_sensing_cluster_add_attr ( esp_zb_occupancy_sensing_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, &undefined_value); 
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_occupancy_sensing_cluster(cluster_list, esp_zb_occupancy_sensing_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /* Illuminance cluster */
/*  esp_zb_attribute_list_t *esp_zb_illuminance_meas_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT);
    esp_zb_illuminance_meas_cluster_add_attr( esp_zb_illuminance_meas_cluster, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID, &undefined_value);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_illuminance_meas_cluster(cluster_list, esp_zb_illuminance_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
*/
    esp_zb_ep_list_add_ep(esp_zb_sensor_ep, cluster_list, endpoint_config); // adding the created endpoint to the list 
    esp_zb_device_register(esp_zb_sensor_ep);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}

/*-----------------------------------------------------------------Sensor Functions-----------------------------------------------------------------*/
/*------------------------------------------------------------HC-SR501------------------------------------------------------------*/
static void sensor1_task(void *pvParameter) {

    esp_rom_gpio_pad_select_gpio(MOTION_PIN);
    gpio_set_direction(MOTION_PIN, GPIO_MODE_INPUT);//sets pin as input
    int previous_state = gpio_get_level(MOTION_PIN); // Initial state of the pin
    while(1){

        int current_state = gpio_get_level(MOTION_PIN); // Read the current state of the pin
        if (current_state == 1 && previous_state == 0) { // Detect rising edge
            Movement++;
            printf("Movimiento detectado en la zona\n"); // Correct the count display    
            vTaskDelay(3000 / portTICK_PERIOD_MS); // Delay to debounce
        }
    previous_state = current_state; // Update the previous state
    vTaskDelay(300 / portTICK_PERIOD_MS); // Delay to debounce
    Movement=0;    
    }
}
/*---------------------------------------------------------------LDR---------------------------------------------------------------
static void sensor3_task(void *pvParameters){
   
    esp_rom_gpio_pad_select_gpio(LIGHT_PIN);
    gpio_set_direction(LIGHT_PIN, GPIO_MODE_INPUT);
    int light_level = gpio_get_level(LIGHT_PIN); // Initial state of the pin
    while(1){

        if (light_level == 0) { // high light level
            printf("NIVEL DE LUZ ADECUADO\n");
            Light=1; 
        } else if (light_level == 1) { // low light level
            printf("NIVEL DE LUZ BAJO, ENCENDER LUCES\n");
            Light=0;
        }    
    }
  vTaskDelay(pdMS_TO_TICKS(600000)); //Takes a sample every 10 minutes  
}
*/
/*--------------------------------------------------------------DHT11--------------------------------------------------------------*/
static void sensor2_task(void *pvParameters){

    while(1){
        read_dht11_data(&Humidity, &Temperatura);
        printf("Humedad: %d%% Temperatura: %dC\n", Humidity/100, Temperatura/100); 
        vTaskDelay(pdMS_TO_TICKS(900000)); //Takes a sample every 15 minutes

    }
}

void app_main(void) {    

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);// Start Zigbee stack task

    while (countdown > 0) {
        printf("Calibracion del sensor\n"); //Sensors need at least 1 minute to calibrate
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        countdown--;
    }
    xTaskCreate(sensor1_task, "Sensor HC-SR501", 2048, NULL, 3, NULL);
    xTaskCreate(sensor2_task, "Sensor DHT11", 4096, NULL, 3, NULL);
//    xTaskCreate(sensor3_task, "Sensor LDR", 2048, NULL, 3, NULL);
}