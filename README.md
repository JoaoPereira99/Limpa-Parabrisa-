//********************************************************************************Sistemas Eletricos e eletroncios do Veiculo *******************************************
//*****Joao Pereira Nº2213082/****************
//*****Rodrigo Oliveira Nº2213084************

#include <Arduino.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_system.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <DHT11.h>
#include <ESP32Servo.h>

// Definições de pinos
#define DHTPIN 15           
#define LDRPIN 2            
#define FLUXOPIN 35
#define LED_DESEMB_PIN 4
#define SERVOPIN 33
#define LED_LUZ_PIN 21

#define LED_INT_PIN 27
#define BOT_INT_PIN 32

//Configuração do ADC
#define ADC_RESOLUTION 12              
#define VREF_PLUS 3.3                     
#define VREF_MINUS 0.0                  

//Definição do PWM
#define PWM_CHANNEL 0
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8

// Terminais do LCD
#define TFT_CS 5
#define TFT_DC 26
#define TFT_MOSI 23
#define TFT_MISO 19
#define TFT_SCLK 18
#define TFT_RST -1
#define DEBOUNCE_ATRASO 200

Servo servo;

// Variáveis globais
uint32_t tempoAtual = 0, ultimoBotao = 0;
bool luzes = false;
bool estado_fluxo = false;
bool desembaciador = false;
float luminosidade = 0.0;
float fluxo = 0.0;
bool estadoLED = LOW;

// Queues e semáforos
QueueHandle_t xQueueTemperature, xQueueHumidity, xQueueLDR, xQueueFluxo; 
SemaphoreHandle_t fluxoSemaphore; 
SemaphoreHandle_t ledintSemaphore;

// declaração das tarefas utilizadas
void readSensorsTask(void *pvParameters);
void controlServoTask(void *pvParameters);
void controlLEDTask(void *pvParameters);
void displayTask(void *pvParameters);
void checkLDRTask(void *pvParameters);
void checkFluxoTask(void *pvParameters);
//void TaskLEDINT(void *pvParameters);

void IRAM_ATTR handleLEDINTnterrupt() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	tempoAtual = millis();
	if (tempoAtual - ultimoBotao > DEBOUNCE_ATRASO) {
		estadoLED = !estadoLED;
		digitalWrite(LED_INT_PIN, estadoLED);
		ultimoBotao = millis();

	}
}

void setup() {

	Serial.begin(115200);            

	pinMode(FLUXOPIN, INPUT);              
	pinMode(LED_DESEMB_PIN, OUTPUT);
	pinMode(LED_LUZ_PIN, OUTPUT);

	pinMode(LED_INT_PIN, OUTPUT);
	pinMode(BOT_INT_PIN, INPUT_PULLUP);
	attachInterrupt(BOT_INT_PIN, handleLEDINTnterrupt, RISING);

	// Criar filas e semáforos
	xQueueTemperature = xQueueCreate(1, sizeof(int)); 
	xQueueHumidity = xQueueCreate(1, sizeof(int));
	xQueueLDR = xQueueCreate(1, sizeof(float));
	xQueueFluxo = xQueueCreate(1, sizeof(float));
	fluxoSemaphore = xSemaphoreCreateBinary();             
	ledintSemaphore = xSemaphoreCreateBinary();

	// Criar tarefas e Priroridades das tarefas
	xTaskCreatePinnedToCore(readSensorsTask, "ReadSensors", 4096, NULL, 3, NULL, 1);

	xTaskCreatePinnedToCore(checkLDRTask, "CheckLDR", 4096, NULL, 2, NULL, 1); 

	xTaskCreatePinnedToCore(checkFluxoTask, "CheckFluxo", 4096, NULL, 2, NULL, 1); 

	xTaskCreatePinnedToCore(controlServoTask, "ControlServo", 4096, NULL, 2,NULL, 1); 

	xTaskCreatePinnedToCore(controlLEDTask, "ControlLED", 4096, NULL, 2, NULL, 1); 

	xTaskCreatePinnedToCore(displayTask, "Display", 4096, NULL, 1, NULL, 1); 

}

void loop() {
	vTaskDelete(NULL); 
}

// Tarefa para leitura de sensores
void readSensorsTask(void *pvParameters) {
	DHT11 dht11(DHTPIN);                        

	while (true) { 
		int temperature = 0, humidity = 0;
		int result = dht11.readTemperatureHumidity(temperature, humidity); 
		if (result == 0) {
			Serial.printf("Temperature: %d °C, Humidity: %d %%\n", temperature, humidity); 
			xQueueOverwrite(xQueueTemperature, &temperature); 
			xQueueOverwrite(xQueueHumidity, &humidity); 
		} else {                     // Segurança, caso não consiga ler o sensor
			Serial.println(DHT11::getErrorString(result)); 
		}
		//Serial.printf("DHT\n"); 
		vTaskDelay(pdMS_TO_TICKS(1000)); 
	}
}

// Tarefa para verificar a LDR
void checkLDRTask(void *pvParameters) {                               
	analogReadResolution(ADC_RESOLUTION); 
	pinMode(LED_LUZ_PIN, OUTPUT); 

	while (true) { 
		float rawValue = analogRead(LDRPIN); 
		luminosidade = (rawValue * (VREF_PLUS - VREF_MINUS) / (1 << ADC_RESOLUTION)) + VREF_MINUS; 

		int ledBrightness = map(luminosidade * 1000, 700, 1200, 0, 255); 

		ledBrightness = constrain(ledBrightness, 0, 255); 

		analogWrite(LED_LUZ_PIN, ledBrightness); 

		// Atualiza o estado das luzes
		luzes = (luminosidade > 0.7); 
		Serial.printf("LUX: %.2f V, LED Brightness: %d\n", luminosidade, ledBrightness); 
		xQueueOverwrite(xQueueLDR, &luminosidade); 
		vTaskDelay(pdMS_TO_TICKS(200)); 
	}
}

void checkFluxoTask(void *pvParameters) {
	analogReadResolution(ADC_RESOLUTION); 
	servo.attach(SERVOPIN); 

	while (true) {
		float rawValue = analogRead(FLUXOPIN); variavel Rawvalue
		fluxo = (rawValue * (VREF_PLUS - VREF_MINUS) / (1 << ADC_RESOLUTION)) + VREF_MINUS; // conversão do valor digital em  V

		// se o valor estiver acima 3.2
		if (fluxo < 3.2) {
			xSemaphoreGive(fluxoSemaphore); 
			estado_fluxo = true; 
		} else {
			servo.write(90);      
			estado_fluxo = false;      
		}

		Serial.printf("FLUXO: %.2f V\n", fluxo);  
		xQueueOverwrite(xQueueFluxo, &fluxo); 
		vTaskDelay(pdMS_TO_TICKS(25)); 
	}
}

// Tarefa para controle do servo motor
void controlServoTask(void *pvParameters) {
	unsigned portBASE_TYPE uxPriority; 
	uxPriority = uxTaskPriorityGet( NULL); 
	servo.attach(SERVOPIN); 
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(500));
		if (estado_fluxo) { 
			vTaskPrioritySet( NULL, ( configMAX_PRIORITIES - 1)); 
			if (xSemaphoreTake(fluxoSemaphore, portMAX_DELAY) == pdPASS) { 
				servo.write(90);         
				vTaskDelay(pdMS_TO_TICKS(1000)); 
				servo.write(0);			
			}
			vTaskPrioritySet( NULL, (uxPriority)); 
		}
		Serial.printf("SERVO\n"); 
	}
}

// Tarefa para controle do LED (desembaçador)...HUMIDADE
void controlLEDTask(void *pvParameters) {
	int humidity = 0; 
	while (true) {
		if (xQueuePeek(xQueueHumidity, &humidity, 0) == pdPASS) { 
			digitalWrite(LED_DESEMB_PIN, (humidity > 80) ? HIGH : LOW); 
			if (humidity > 80) {
				desembaciador = true; 
			} else {
				desembaciador = false; 
			}
		}
		Serial.printf("LED\n"); 
		vTaskDelay(pdMS_TO_TICKS(200)); 
	}
}

// Tarefa para exibição no display
void displayTask(void *pvParameters) {
	//Configurar o display
	Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK,
	TFT_RST, TFT_MISO);
	int temperature = 0, humidity = 0;
	float ldrValue = 0.0;
	int lastTemperature = -999, lastHumidity = -999;
	float lastLdrValue = -999.0;
	bool lastLuzes = true, lastDesembaciador = true, lastFluxo = true;
	char buffer[20];

	tft.begin(); 

	tft.setRotation(0); 

	// Configuração inicial do display (desenha layout fixo)
	tft.fillScreen(ILI9341_BLACK); 

	// Header
	tft.setCursor(5, 5); 
	tft.setTextColor(ILI9341_WHITE); 
	tft.setTextSize(2); 
	tft.println("Sistema Para-brisa"); 
	tft.drawRect(5, 5, 230, 30, ILI9341_RED); 

	// Linhas fixas separadoras
	tft.drawLine(5, 60, 230, 60, ILI9341_WHITE);
	tft.drawLine(5, 90, 230, 90, ILI9341_WHITE);
	tft.drawLine(5, 120, 230, 120, ILI9341_WHITE);
	tft.drawLine(5, 150, 230, 150, ILI9341_WHITE);
	tft.drawLine(5, 180, 230, 180, ILI9341_WHITE);

	while (true) {
		// Recebe dados das filas
		xQueuePeek(xQueueTemperature, &temperature, 0);
		xQueuePeek(xQueueHumidity, &humidity, 0);
		xQueuePeek(xQueueLDR, &ldrValue, 0);

		// Atualiza apenas se o valor da temperatura mudou
		if (temperature != lastTemperature) { 
			tft.setCursor(10, 40); 
			tft.setTextSize(2); 
			tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK); 
			tft.print("Temp: "); 
			sprintf(buffer, "%5d C", temperature); 
			tft.println(buffer); 
			lastTemperature = temperature; 
		}

		// Atualiza apenas se o valor da humidade mudou
		if (humidity != lastHumidity) { 
			tft.setCursor(10, 70); 
			tft.setTextSize(2); 
			tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK); 
			sprintf(buffer, "Hum: %5d %%", humidity); 
			tft.println(buffer); 
			lastHumidity = humidity; 
		}

		// Atualiza apenas se o valor da luminosidade mudou
		if (ldrValue != lastLdrValue) { 
			tft.setCursor(10, 100); 
			tft.setTextSize(2); 
			tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK); 
			sprintf(buffer, "LDR: %.2f V", ldrValue); 
			tft.println(buffer); 
			lastLdrValue = ldrValue; 
		}

		// Atualiza estado das luzes apenas se mudou
		if (luzes != lastLuzes) { 
			tft.setCursor(10, 130); 
			tft.setTextSize(2); 
			if (luzes) { 
				tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK); 
				tft.println("Luzes: Ligadas   "); 
			} else {
				tft.setTextColor(ILI9341_RED, ILI9341_BLACK); 
				tft.println("Luzes: Desligadas"); 
			}
			lastLuzes = luzes; 
		}

		// Atualiza estado do desembaciador apenas se mudou
		if (desembaciador != lastDesembaciador) { 
			tft.setCursor(10, 160); 
			tft.setTextSize(2); 
			if (desembaciador) { 
				tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK); 
				tft.println("Desemba: Ligado   "); 
			} else {
				tft.setTextColor(ILI9341_RED, ILI9341_BLACK); 
				tft.println("Desemba: Desligado"); 
			}
			lastDesembaciador = desembaciador; 
		}

		if (fluxo == fluxo) { 
			tft.setCursor(10, 190); 
			tft.setTextSize(2); 
			if (estado_fluxo) { 
				tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK); 
				tft.println("Escovas: Ligado   "); 
			} else {
				tft.setTextColor(ILI9341_RED, ILI9341_BLACK); 
				tft.println("Escovas: Desligado"); 
			}
			lastFluxo = fluxo; 
		}

		vTaskDelay(pdMS_TO_TICKS(250)); 
	}
}
