#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdlib.h>
#include <string.h>
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

/*----------------------------------Macros----------------------------------*/
static const char *TAG = "ESP_ZB_SENSOR";

#define LIGHT_PIN GPIO_NUM_0
#define MOTION_PIN GPIO_NUM_10 
#define DHT11_PIN GPIO_NUM_11 

uint8_t Movement = 0;
uint8_t humidity = 0;
uint8_t temperature = 0;
uint8_t Light = 0;
uint8_t countdown = 60;
/*----------------------------------Sensor Functions----------------------------------*/

static void sensor1_task(void *pvParameter) {
/*-----------------------------------HC-SR501-----------------------------------*/

        esp_rom_gpio_pad_select_gpio(MOTION_PIN);
        gpio_set_direction(MOTION_PIN, GPIO_MODE_INPUT);//sets pin as input

        int previous_state = gpio_get_level(MOTION_PIN); // Initial state of the pin
        
        while(1){
            int current_state = gpio_get_level(MOTION_PIN); // Read the current state of the pin
            
            if (current_state == 1 && previous_state == 0) { // Detect rising edge
                Movement++;
                printf("Movimiento detectado en la zona\n"); // Correct the count display
                Movement=0;
            }
        previous_state = current_state; // Update the previous state
        vTaskDelay(300 / portTICK_PERIOD_MS); // Delay to debounce
        }
    }

static void sensor2_task(void *pvParameters)
{
    /*--------------------------------------LDR--------------------------------------*/
    //void light_detect(void *pvParameter) {
        esp_rom_gpio_pad_select_gpio(LIGHT_PIN);
        gpio_set_direction(LIGHT_PIN, GPIO_MODE_INPUT);
        int light_level = gpio_get_level(LIGHT_PIN); // Initial state of the pin
        
    //}

    //void dht_detect(void *pvParameter){
    /*---------------------------------------DHT---------------------------------------*/
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
        void read_dht11_data(uint8_t *humidity, uint8_t *temperature) {
            uint8_t hum_int, hum_dec, temp_int, temp_dec, checksum;
            dht11_start_signal();
            if (dht11_check_response()) {
                hum_int = dht11_read_byte();
                hum_dec = dht11_read_byte();
                temp_int = dht11_read_byte();
                temp_dec = dht11_read_byte();
                checksum = dht11_read_byte();

                if (hum_int + hum_dec + temp_int + temp_dec == checksum) {
                    *humidity = hum_int;
                    *temperature = temp_int;
                } else {
                    printf("Checksum error\n");
                }
            } else {
                printf("No response from DHT11\n");
            }
        }
    //}  

    while(1){
/*--------------------------------------LDR--------------------------------------*/
        if (light_level == 0) { // high light level
            printf("NIVEL DE LUZ ADECUADO\n");
            Light=1; 
        } else if (light_level == 1) { // low light level
            printf("NIVEL DE LUZ BAJO, ENCENDER LUCES\n");
            Light=0;
        }
/*-------------------------------------DHT11-------------------------------------*/
        read_dht11_data(&humidity, &temperature);
        printf("Humedad: %d%% Temperatura: %dC\n", humidity, temperature); 
        vTaskDelay(pdMS_TO_TICKS(300000)); //Takes a sample every 5 minutes
    }
}

/*------------------------------------------Zigbee Stack Functions-----------------------------------------*/

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void app_main(void) {    
    while (countdown > 0) {
        printf("Calibracion del sensor\n"); //Sensors need at least 1 minute to calibrate
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
        countdown--; // Decrease the countdown value by 1
    }
    xTaskCreate(sensor1_task, "Sensor HC-SR501", 2048, NULL, 2, NULL);
    xTaskCreate(sensor2_task, "Sensor DHT11 & LDR", 2048, NULL, 2, NULL);
}
