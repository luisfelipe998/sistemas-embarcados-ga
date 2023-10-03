#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "string.h"
#include "driver/uart.h"
#include "DHT22.h"

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)


#define PINO_LED 22
#define PIN_LED_UMIDADE 23
#define PIN_LED_TEMPERATURA 21

#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 50  // Multisampling

 
// Configuração do ADC
static const adc_channel_t channel = ADC_CHANNEL_6; // GPIO34 CHANNEL_6
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11; // ver https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/api-reference/peripherals/adc.html#_CPPv425adc1_config_channel_atten14adc1_channel_t11adc_atten_t

// Configuração do LED
bool estado_led = 0;
int adc_reading = 0;
float umidade =0.0;
float temperatura=0.0;
char *test_str = "\n";
char umidade_str[20];
char temperatura_str[20];
 
//Configurações iniciais da porta UART 

void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
    return txBytes;
}
static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    while (1) {
                   
            // Envie as strings pela UART
            sendData(TX_TASK_TAG, umidade_str);
            sendData(TX_TASK_TAG, temperatura_str);

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
void tarefaADC(void *parametros)
{
    // Configure ADC
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);
    for (;;)
    {
       
        // Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        }
        adc_reading /= NO_OF_SAMPLES;
        //printf("ADC: %d\n", adc_reading);

        // caso o LDR identifique que está escuro, ele aciona o LED 
        if (adc_reading > 3000){
             estado_led =1;
        }
        else 
        {
             estado_led =0;
        }
    
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void tarefaLED(void *parametros)
{
 
    for (;;)
    {
        gpio_set_level(PINO_LED, estado_led);
        vTaskDelay(10 / portTICK_PERIOD_MS);

        //Caso a umidade esteja muito baixa, sinalizo nível crítico com LED VERMELHO
        if (umidade < 50.00)
        {
            gpio_set_level(PIN_LED_UMIDADE, 1);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }else 
        {
            gpio_set_level(PIN_LED_UMIDADE, 0);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

           //Caso a temperatura esteja muito alta, sinalizo com um LED AZUL para simular o acionamento de um dispositivo para reduzir a temperatura. 
        if (temperatura > 25.00)
        {
            gpio_set_level(PIN_LED_TEMPERATURA, 1);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }else {
            gpio_set_level(PIN_LED_TEMPERATURA, 0);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    

    }
}

void tarefaDHT(void *parametros)
{
    //Realizar a leitura do sensor de temperatura e humidade. 


	setDHTgpio(GPIO_NUM_27);

	while(1) {
	
		//printf("DHT Sensor Readings\n" );
		int ret = readDHT();
		
        if (ret == DHT_OK){
             umidade = getHumidity();
             temperatura = getTemperature();

            // Formate os valores de umidade e temperatura como strings
 
            snprintf(umidade_str, sizeof(umidade_str), "Umidade: %.2f %%\n", umidade);
            snprintf(temperatura_str, sizeof(temperatura_str), "Temperatura: %.2f", temperatura);



        }
        else 
		    errorHandler(ret);

        /*printf("Humidity %.2f %%\n", getHumidity());
		printf("Temperature %.2f degC\n\n", getTemperature());*/
		
		vTaskDelay(3000 /portTICK_PERIOD_MS);
	}
    
}


void app_main(void)
{
    gpio_reset_pin(PINO_LED);
    gpio_set_direction(PINO_LED,GPIO_MODE_OUTPUT);
    
    gpio_reset_pin(PIN_LED_UMIDADE);
    gpio_set_direction(PIN_LED_UMIDADE,GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_LED_TEMPERATURA);
    gpio_set_direction(PIN_LED_TEMPERATURA,GPIO_MODE_OUTPUT);
    bool estado_anterior = estado_led;


    //Configurações da UART 
    init();
    
    xTaskCreate(tarefaADC, "adc", 2048, NULL, 0, NULL);
    xTaskCreate(tarefaLED, "led", 2048, NULL, 0, NULL);
    xTaskCreate(tarefaDHT, "dht", 2048, NULL, 0, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);


        for (;;)
    {
        
        if (estado_led != estado_anterior){
            estado_anterior = estado_led;
            if (estado_led == 0)
                printf("ADC: %d  LED DESLIGADO\n", adc_reading);
            else 
                printf("ADC: %d  LED LIGADO \n", adc_reading);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
}

