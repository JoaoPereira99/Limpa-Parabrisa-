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
#define DHTPIN 15           //Fio Branco
#define LDRPIN 2            //Fio laranja
#define FLUXOPIN 35
#define LED_DESEMB_PIN 4
#define SERVOPIN 33
#define LED_LUZ_PIN 21

#define LED_INT_PIN 27
#define BOT_INT_PIN 32

//Configuração do ADC
#define ADC_RESOLUTION 12              // Escala do ADC
#define VREF_PLUS 3.3                     // defenição do maior  valor que vou querer ler
#define VREF_MINUS 0.0                  // defenição do menor valor que vou querer ler

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

//Tempo que o botão demora a ler , para proteger caso exista uam leitura falsa provocada pelo mecaniismo do mesmo
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
QueueHandle_t xQueueTemperature, xQueueHumidity, xQueueLDR, xQueueFluxo; //  declaração das queue que utilizei para guardar o valor dos sensores d
SemaphoreHandle_t fluxoSemaphore; // decalração do semafro como professor obrigou a usar semafros , utilizei um semafro , para ler o sensor de fluxo , apenas quando quero é que ( o semafro é chamado e evita estar sempre a correr)
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

	Serial.begin(115200);            //leitura da porta seri

	pinMode(FLUXOPIN, INPUT);              // Configuração dos Pinos I/O
	pinMode(LED_DESEMB_PIN, OUTPUT);
	pinMode(LED_LUZ_PIN, OUTPUT);

	pinMode(LED_INT_PIN, OUTPUT);
	pinMode(BOT_INT_PIN, INPUT_PULLUP);
	attachInterrupt(BOT_INT_PIN, handleLEDINTnterrupt, RISING);

	// Criar filas e semáforos
	xQueueTemperature = xQueueCreate(1, sizeof(int)); // crianção das Queue onde serão guardos os valores lidos atravez dos sensores  , a configuração so aceita 1 valor de cada vez
	xQueueHumidity = xQueueCreate(1, sizeof(int));
	xQueueLDR = xQueueCreate(1, sizeof(float));
	xQueueFluxo = xQueueCreate(1, sizeof(float));
	fluxoSemaphore = xSemaphoreCreateBinary();             // criação do semafro
	ledintSemaphore = xSemaphoreCreateBinary();

	// Criar tarefas e Priroridades das tarefas
	xTaskCreatePinnedToCore(readSensorsTask, "ReadSensors", 4096, NULL, 3, NULL, 1); // tarefas ler sensor de temperatura e Humidade

	xTaskCreatePinnedToCore(checkLDRTask, "CheckLDR", 4096, NULL, 2, NULL, 1); // Tarefa ler LDR  e a sua prioridade

	xTaskCreatePinnedToCore(checkFluxoTask, "CheckFluxo", 4096, NULL, 2, NULL, 1); // tarefa ler sensor de Fluxo de agua e a sua prioridade

	xTaskCreatePinnedToCore(controlServoTask, "ControlServo", 4096, NULL, 2,NULL, 1); //Tarefa qque controla o servo motor

	xTaskCreatePinnedToCore(controlLEDTask, "ControlLED", 4096, NULL, 2, NULL, 1); // tarefa que controla o LED

	xTaskCreatePinnedToCore(displayTask, "Display", 4096, NULL, 1, NULL, 1); //tarefa do display

}

void loop() {
	vTaskDelete(NULL); // Desativa o loop principal , o loop é uma task , como não a quero utilizar , ela corra para não entrar em conflito comm  o resto do programa
}

// Tarefa para leitura de sensores
void readSensorsTask(void *pvParameters) {
	DHT11 dht11(DHTPIN);                        //decalração do senso

	while (true) { // utilizamos um ciclo While para leiruta dos sensores , para estar sempre a correr a adquerir novos valores atravez do sensor de temperatura
		int temperature = 0, humidity = 0;
		int result = dht11.readTemperatureHumidity(temperature, humidity); //leitura do sensor , int result serve de segurança para ver se o sensor leu ou não um valor
		if (result == 0) {
			Serial.printf("Temperature: %d °C, Humidity: %d %%\n", temperature, humidity); // Escreve para consola
			xQueueOverwrite(xQueueTemperature, &temperature); // envia para Queue o valor da temperatura
			xQueueOverwrite(xQueueHumidity, &humidity); // envia para Queue o valor da Humidade
		} else {                     // Segurança, caso não consiga ler o sensor
			Serial.println(DHT11::getErrorString(result)); // Imprimi este erro na consola
		}
		//Serial.printf("DHT\n"); // sempre que a task corre imprime DHT na consola
		vTaskDelay(pdMS_TO_TICKS(1000)); //  esta tarefa corre periodicamente de 1s em 1s
	}
}

// Tarefa para verificar a LDR.................LUZES Ligadas/Desligadas
void checkLDRTask(void *pvParameters) {                                //
	analogReadResolution(ADC_RESOLUTION); // definir a resolução de leitura do ADC
	pinMode(LED_LUZ_PIN, OUTPUT); // Certifique-se de que o pino do LED está configurado como saída

	while (true) { // utilizamos um ciclo While para leiruta dos sensores , para estar sempre a correr a adquerir novos valores atravez do sensor  , caso não tivesse um ciclo while ou algo do genero só iria correr uma vez
		float rawValue = analogRead(LDRPIN); //le o valor da Ldr e guardo no rawvalue
		luminosidade = (rawValue * (VREF_PLUS - VREF_MINUS) / (1 << ADC_RESOLUTION)) + VREF_MINUS; // converter tensão em digital

		int ledBrightness = map(luminosidade * 1000, 700, 1200, 0, 255); //

		ledBrightness = constrain(ledBrightness, 0, 255); //

		analogWrite(LED_LUZ_PIN, ledBrightness); // envia o pwm a led para variar a luminusidade

		// Atualiza o estado das luzes
		luzes = (luminosidade > 0.7); //FAz com que o led seja ligado com  um valor 0.7V lidos atravez da LDR
		Serial.printf("LUX: %.2f V, LED Brightness: %d\n", luminosidade, ledBrightness); //  Imprime na consola o valor lido pela LDr , a liminusidade que ira ser enviada.
		xQueueOverwrite(xQueueLDR, &luminosidade); // guarda o valor na queue da luminusidade
		vTaskDelay(pdMS_TO_TICKS(200)); //Tempo periodico que a task corre
	}
}

void checkFluxoTask(void *pvParameters) {
	analogReadResolution(ADC_RESOLUTION); // definir a resolução de leitura do ADC
	servo.attach(SERVOPIN); // reconhecimento do servo motor por parte da biblioteca

	while (true) {
		float rawValue = analogRead(FLUXOPIN); // Le o valor sesnor de fluxo e guarda na variavel Rawvalue
		fluxo = (rawValue * (VREF_PLUS - VREF_MINUS) / (1 << ADC_RESOLUTION)) + VREF_MINUS; // conversão do valor digital em  V

		// se o valor estiver acima 3.2
		if (fluxo < 3.2) {
			xSemaphoreGive(fluxoSemaphore); // da permissão para a task do servo correr o função do limpa vidros
			estado_fluxo = true; // esta varivale guarda o estado atual das escovas para escrever no display
		} else {
			servo.write(90);      //voltar a posição inicial
			estado_fluxo = false;      //imprime que as escovas estão desligadas
		}

		Serial.printf("FLUXO: %.2f V\n", fluxo);  // escreve o estado na consola
		xQueueOverwrite(xQueueFluxo, &fluxo); // guarda na queque o valor do fluxo
		vTaskDelay(pdMS_TO_TICKS(25)); //Tempo periodico que a task corre
	}
}

// Tarefa para controle do servo motor.......ATravez do FLuxo de Agua
void controlServoTask(void *pvParameters) {
	unsigned portBASE_TYPE uxPriority; //Definir a variavel da prioridade
	uxPriority = uxTaskPriorityGet( NULL); //Buscar a prioridade normal da task (definida no setup())
	servo.attach(SERVOPIN); // reconhecimento do servo motor por parte da biblioteca
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(500));
		if (estado_fluxo) { //Se o semafro estiver disponivel e detectar agua no sensor
			vTaskPrioritySet( NULL, ( configMAX_PRIORITIES - 1)); //Colocar a task (NULL = atual) com a máxima prioridade, sem ela, o servo não aciona
			if (xSemaphoreTake(fluxoSemaphore, portMAX_DELAY) == pdPASS) { //Obter o semafro pq foi detectada agua no sensor
				servo.write(90);         // Mete o servomotor na posicao inicial
				vTaskDelay(pdMS_TO_TICKS(1000)); // para fazer o movimento das escovas atraves dá pausa  da tarefa
				servo.write(0);			// roda o servomotor 90º
			}
			vTaskPrioritySet( NULL, (uxPriority)); // Voltar à prioridade normal
		}
		Serial.printf("SERVO\n"); // Imprime na porta série q a task controlServoTask correu
	}
}

// Tarefa para controle do LED (desembaçador)...HUMIDADE
void controlLEDTask(void *pvParameters) {
	int humidity = 0; //variavel apenas  par esta task
	while (true) {
		if (xQueuePeek(xQueueHumidity, &humidity, 0) == pdPASS) { //Se conseguir obter o valorda queue
			digitalWrite(LED_DESEMB_PIN, (humidity > 80) ? HIGH : LOW); //Mete o pino de output a high ou low dependendo do valor da humidade
			if (humidity > 80) {
				desembaciador = true; // mostrar no display
			} else {
				desembaciador = false; //l para mostrar no display
			}
		}
		Serial.printf("LED\n"); // Imprime na porta série q a task controlLEDTask correu
		vTaskDelay(pdMS_TO_TICKS(200)); // Atualiza a cada 500ms
	}
}

// Tarefa para exibição no display
void displayTask(void *pvParameters) {
	//Configurar o display
	Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK,
	TFT_RST, TFT_MISO);
	// Declaracao das  variaveis utilizada nesta task
	int temperature = 0, humidity = 0;
	float ldrValue = 0.0;
	int lastTemperature = -999, lastHumidity = -999;
	float lastLdrValue = -999.0;
	bool lastLuzes = true, lastDesembaciador = true, lastFluxo = true;
	char buffer[20];

	tft.begin(); // Iniciar o display

	tft.setRotation(0); //Orientacao do display

	// Configuração inicial do display (desenha layout fixo)
	tft.fillScreen(ILI9341_BLACK); //Colocaro ecra do display a preto

	// Header
	tft.setCursor(5, 5); //comecar a escrever/desenhar nestas coordenadas
	tft.setTextColor(ILI9341_WHITE); //Escrever o texto numa cor especifica
	tft.setTextSize(2); //Definir tamanho do texto
	tft.println("Sistema Para-brisa"); //Escrever texto
	tft.drawRect(5, 5, 230, 30, ILI9341_RED); // Moldura ao redor do header

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
		if (temperature != lastTemperature) { //Se o valor for diferente ao anterior, atualiza
			tft.setCursor(10, 40); //comecar a escrever/desenhar nestas coordenadas
			tft.setTextSize(2); //Definir tamanho do texto
			tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK); //Escrever o texto numa cor especifica
			tft.print("Temp: "); //Escrever texto
			sprintf(buffer, "%5d C", temperature); //Colocar o valor lido num buffer temporario de chars
			tft.println(buffer); //Escrever o que está no buffer
			lastTemperature = temperature; //Atualizar o ultimo valor escrito no display
		}

		// Atualiza apenas se o valor da humidade mudou
		if (humidity != lastHumidity) { //Se o valor for diferente ao anterior, atualiza
			tft.setCursor(10, 70); //comecar a escrever/desenhar nestas coordenadas
			tft.setTextSize(2); //Definir tamanho do texto
			tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK); //Escrever o texto numa cor especifica com fundo preto
			sprintf(buffer, "Hum: %5d %%", humidity); //Colocar o valor lido num buffer temporario de chars
			tft.println(buffer); //Escrever o que está no buffer
			lastHumidity = humidity; //Atualizar o ultimo valor escrito no display
		}

		// Atualiza apenas se o valor da luminosidade mudou
		if (ldrValue != lastLdrValue) { //Se o valor for diferente ao anterior, atualiza
			tft.setCursor(10, 100); //comecar a escrever/desenhar nestas coordenadas
			tft.setTextSize(2); //Definir tamanho do texto
			tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK); //Escrever o texto numa cor especifica com fundo preto
			sprintf(buffer, "LDR: %.2f V", ldrValue); //Colocar o valor lido num buffer temporario de chars
			tft.println(buffer); //Escrever o que está no buffer
			lastLdrValue = ldrValue; //Atualizar o ultimo valor escrito no display
		}

		// Atualiza estado das luzes apenas se mudou
		if (luzes != lastLuzes) { //Se o valor for diferente ao anterior, atualiza
			tft.setCursor(10, 130); //comecar a escrever/desenhar nestas coordenadas
			tft.setTextSize(2); //Definir tamanho do texto

			if (luzes) { // Se asluzes estiver a true, logo estao ligadas
				tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK); //Escrever o texto numa cor especifica com fundo preto
				tft.println("Luzes: Ligadas   "); //Escrever texto
			} else {
				tft.setTextColor(ILI9341_RED, ILI9341_BLACK); //Escrever o texto numa cor especifica com fundo preto
				tft.println("Luzes: Desligadas"); //Escrever texto
			}
			lastLuzes = luzes; //Atualizar o ultimo valor escrito no display
		}

		// Atualiza estado do desembaciador apenas se mudou
		if (desembaciador != lastDesembaciador) { //Se o valor for diferente ao anterior, atualiza
			tft.setCursor(10, 160); //comecar a escrever/desenhar nestas coordenadas
			tft.setTextSize(2); //Definir tamanho do texto
			if (desembaciador) { // Se o bool desembaciador estiver a true, logo o dembaciador deve de ligar, mostrando isto no display
				tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK); //Escrever o texto numa cor especifica com fundo preto
				tft.println("Desemba: Ligado   "); //Escrever texto
			} else {
				tft.setTextColor(ILI9341_RED, ILI9341_BLACK); //Escrever o texto numa cor especifica com fundo preto
				tft.println("Desemba: Desligado"); //Escrever texto
			}
			lastDesembaciador = desembaciador; //Atualizar o ultimo valor escrito no display
		}

		if (fluxo == fluxo) { //Se o valor for diferente ao anterior, atualiza
			tft.setCursor(10, 190); //comecar a escrever/desenhar nestas coordenadas
			tft.setTextSize(2); //Definir tamanho do texto
			if (estado_fluxo) { // Se o estado_fluxo estiver a true, é pq o sensor detectou agua, logo as escovas devem de estar ligadas
				tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK); //Escrever o texto numa cor especifica com fundo preto
				tft.println("Escovas: Ligado   "); //Escrever texto
			} else {
				tft.setTextColor(ILI9341_RED, ILI9341_BLACK); //Escrever o texto numa cor especifica com fundo preto
				tft.println("Escovas: Desligado"); //Escrever texto
			}
			lastFluxo = fluxo; //Atualizar o ultimo valor escrito no display
		}

		vTaskDelay(pdMS_TO_TICKS(250)); // Atualiza a cada 200ms
	}
}
