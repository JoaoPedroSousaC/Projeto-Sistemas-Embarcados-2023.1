#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "i2c-lcd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
#include "sys/time.h"
#include "driver/gpio.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/uart.h"
#include <ds18x20.h>

#define UART_NUM UART_NUM_0
#define RX_PIN 3
#define BUF_SIZE (1024)
#define RELE1 GPIO_NUM_19
#define RELE2 GPIO_NUM_18
char buffer[18];
float alturaTotal = 300; // Altura total  (em mm)
float tempEscolhida = 26;
static const char *TAG = "PROJ_EMBARCADOS";
uint16_t dist = 0;
float temp = 0;

void uart_task(void *pvParameters)
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    // Configurar a porta UART com as configurações definidas acima
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Instalar o driver UART
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Buffer para armazenar os dados recebidos
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    uint16_t sumDist = 0;
    uint8_t distCount = 0;

    while (1)
    {
        // Ler dados da porta UART
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0)
        {
            uint8_t startByte = data[0];
            uint16_t mmDist = 0;
            if (startByte == 0xFF)
            {
                uint8_t msByte = data[1];
                uint8_t lsByte = data[2];
                mmDist = (msByte * 256) + lsByte;
            }
            if (mmDist != 0)
            {
                sumDist += mmDist;
                distCount++;

                if (distCount >= 10)
                {
                    uint16_t avgDist = sumDist / distCount;

                    // lcd_init();
                    // lcd_clear();
                    // sprintf(buffer, "%d", avgDist / 10);
                    // lcd_put_cur(5, 0);
                    // lcd_send_string(buffer);
                    // ESP_LOGI(TAG, "distance: %d\n", avgDist);
                    dist = avgDist;
                    sumDist = 0;
                    distCount = 0;
                }
            }
        }
    }

    free(data);
    vTaskDelete(NULL);
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_NUM_0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void atualizarDist()
{
    int acionaBomba = 0;
    while (1)
    {
        if (dist != 0)
        {
            ESP_LOGI("Sem compensacao", "distance: %d\n", dist);
            if (dist >= 250)
            {
                dist = dist - 250;
            }
            else
            {
                dist = 0;
            }

            if (dist > alturaTotal)
            {
                dist = alturaTotal;
            }
            float porcentagem = ((alturaTotal - dist) / alturaTotal) * 100;
            char teste2[30];
            sprintf(teste2, "dist: %.2f", porcentagem);
            lcd_init();
            lcd_clear();
            lcd_put_cur(0, 0);
            lcd_send_string(teste2);
            ESP_LOGI(TAG, "distance: %d\n", dist);
            lcd_put_cur(1, 0);
            char teste3[30];
            sprintf(teste3, "temp: %.2f ", temp);
            lcd_send_string(teste3);
            if (porcentagem < 10)
            {
                acionaBomba = 1;
            }
            if (porcentagem >= 90)
            {
                acionaBomba = 0;
            }
            if (acionaBomba == 1)
            {
                gpio_set_level(RELE1, 0);
            }
            else
            {
                gpio_set_level(RELE1, 1);
            }
            if (temp < tempEscolhida)
            {
                gpio_set_level(RELE2, 0);
            }
            if (temp >= tempEscolhida)
            {
                gpio_set_level(RELE2, 1);
            }

            vTaskDelay(100);
        }
    }
}
#define SENSOR_GPIO GPIO_NUM_5
static const int MAX_SENSORS = 1;
static const int RESCAN_INTERVAL = 8;
static const uint32_t LOOP_DELAY_MS = 500;
void measure_temperature(void *pvParameter)
{
    ds18x20_addr_t addrs[MAX_SENSORS];
    float temps[MAX_SENSORS];
    size_t sensor_count = 0;

    gpio_set_pull_mode(SENSOR_GPIO, GPIO_PULLUP_ONLY);

    esp_err_t res;
    while (1)
    {
        res = ds18x20_scan_devices(SENSOR_GPIO, addrs, MAX_SENSORS, &sensor_count);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "Sensors scan error %d (%s)", res, esp_err_to_name(res));
            continue;
        }

        if (!sensor_count)
        {
            ESP_LOGW(TAG, "No sensors detected!");
            continue;
        }

        ESP_LOGI(TAG, "%d sensors detected", sensor_count);

        if (sensor_count > MAX_SENSORS)
            sensor_count = MAX_SENSORS;

        for (int i = 0; i < RESCAN_INTERVAL; i++)
        {

            res = ds18x20_measure_and_read_multi(SENSOR_GPIO, addrs, sensor_count, temps);
            if (res != ESP_OK)
            {
                ESP_LOGE(TAG, "Sensors read error %d (%s)", res, esp_err_to_name(res));
                continue;
            }

            for (int j = 0; j < sensor_count; j++)
            {
                float temp_c = temps[j];
                ds18x20_addr_t sensor_address = addrs[j];

                ESP_LOGI(TAG, "Temperatura = %.2f", temp_c);
                temp = temp_c;
            }

            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
        }
    }
}

void app_main(void)
{
    gpio_set_direction(RELE1, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELE2, GPIO_MODE_OUTPUT);
    lcd_init();
    lcd_clear();
    ESP_ERROR_CHECK(i2c_master_init());
    lcd_put_cur(0, 0);
    lcd_send_string("Dist:");
    ESP_LOGI(TAG, "I2C initialized successfully");
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);
    xTaskCreate(atualizarDist, "atualizarDist", 4096, NULL, 10, NULL);
    xTaskCreate(measure_temperature, "measure_temperature", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
}
