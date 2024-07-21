#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdlib.h>
#include <string.h>
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

/*---------------------------------------------------------------
        General Macros
---------------------------------------------------------------*/
#define LIGHT_PIN GPIO_NUM_0
#define MOTION_PIN GPIO_NUM_10 
#define DHT11_PIN GPIO_NUM_11 

uint8_t Movement = 0; //bandera, este dato se va a enviar al Controlador domotico. 
uint8_t humidity = 0;
uint8_t temperature = 0;
uint8_t Light = 0;// Bandera para el nivel de luz. 
uint8_t countdown = 60; // Set the initial countdown value to 60

void motion_detect(void *pvParameter) {
    esp_rom_gpio_pad_select_gpio(MOTION_PIN);
    gpio_set_direction(MOTION_PIN, GPIO_MODE_INPUT);

    int previous_state = gpio_get_level(MOTION_PIN); // Initial state of the pin

    while (1) {
        int current_state = gpio_get_level(MOTION_PIN); // Read the current state of the pin

        if (current_state == 1 && previous_state == 0) { // Detect rising edge
            Movement++;
            printf("Movimiento detectado en la zona\n"); // Correct the count display
            Movement=0;
        }
        previous_state = current_state; // Update the previous state
        vTaskDelay(200 / portTICK_PERIOD_MS); // Delay to debounce
    }
}

void light_detect(void *pvParameter) {
    esp_rom_gpio_pad_select_gpio(LIGHT_PIN);
    gpio_set_direction(LIGHT_PIN, GPIO_MODE_INPUT);

    int light_level = gpio_get_level(LIGHT_PIN); // Initial state of the pin

    while (1) {
       if (light_level == 0) { // Nivel de luz adecuado
            printf("NIVEL DE LUZ ADECUADO\n");
            Light=1; 
        } else if (light_level == 1) { // Nivel de luz bajo
            printf("NIVEL DE LUZ BAJO, ENCENDER LUCES\n");
            Light=0;
        }
        vTaskDelay(500000 / portTICK_PERIOD_MS); // Delay to debounce
    }
}


void delay_us(uint32_t us) {
    // Implementa un delay en microsegundos usando esp_rom_delay_us
    esp_rom_delay_us(us);
}

void dht11_start_signal() {
    // Configura el pin como salida
    gpio_set_direction(DHT11_PIN, GPIO_MODE_OUTPUT);
    // Envía señal baja por al menos 18 ms
    gpio_set_level(DHT11_PIN, 0);
    vTaskDelay(20 / portTICK_PERIOD_MS);  // 20 ms delay
    // Envía señal alta por 20-40 us
    gpio_set_level(DHT11_PIN, 1);
    delay_us(30);
    // Configura el pin como entrada
    gpio_set_direction(DHT11_PIN, GPIO_MODE_INPUT);
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
        while (!gpio_get_level(DHT11_PIN));  // Espera a que el pin se ponga alto
        delay_us(30);
        if (gpio_get_level(DHT11_PIN)) result |= (1 << (7 - i));
        while (gpio_get_level(DHT11_PIN));  // Espera a que el pin se ponga bajo
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


void app_main(void) {

    while (countdown > 0) {
        printf("Calibracion del sensor:%d segundos restantes\n", countdown);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
        countdown--; // Decrease the countdown value by 1
    }

   xTaskCreate(motion_detect, "motion detect", 2048, NULL, 5, NULL);
   xTaskCreate(light_detect, "light detect", 2048, NULL, 5, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500000)); //Realiza una muestra de datos cada 5 minutos de la temperatura, humedad y nivel de luz cada 5 minutos
        read_dht11_data(&humidity, &temperature);
        printf("Humedad: %d%% Temperatura: %dC\n", humidity, temperature);    
    }

}