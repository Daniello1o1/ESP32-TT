#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <esp_gatt_common_api.h>

const int RAW_PIN = 36;
const int DIN_PIN = 34;
const int SAMPLE_RATE = 1000;
const int PACKET_SAMPLES = 4;

// BLE
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer *pServer;
BLECharacteristic *pCharacteristic;
bool clienteConectado = false;

// Cola FreeRTOS para pasar paquetes del loop a la tarea BLE
QueueHandle_t colaEnvio;

// Estructura de un paquete
struct Paquete {
    uint8_t data[4 + PACKET_SAMPLES * 4];
    size_t  size;
};

hw_timer_t *timer = NULL;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t pendingSamples = 0;
volatile uint32_t sampleCounter  = 0;

uint16_t emgBuffer[PACKET_SAMPLES];
uint16_t dinBuffer[PACKET_SAMPLES];
int bufferCount = 0;

// ISR
void IRAM_ATTR onTimer() {
    pendingSamples++;
}

// Callbacks BLE
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        clienteConectado = true;
        Serial.println("Cliente conectado");
    }
    void onDisconnect(BLEServer* pServer) {
        clienteConectado = false;
        Serial.println("Cliente desconectado");
        BLEDevice::startAdvertising();
    }
};

// Tarea FreeRTOS — solo se encarga de enviar por BLE
void tareaEnvioBLE(void *param) {
    Paquete paquete;
    while (true) {
        // Espera hasta que haya un paquete en la cola
        if (xQueueReceive(colaEnvio, &paquete, portMAX_DELAY) == pdTRUE) {
            if (clienteConectado) {
                pCharacteristic->setValue(paquete.data, paquete.size);
                pCharacteristic->notify();
            }
        }
    }
}

void setup() {
    Serial.begin(9600);

    analogReadResolution(12);
    analogSetPinAttenuation(RAW_PIN, ADC_11db);
    analogSetPinAttenuation(DIN_PIN, ADC_11db);

    // Cola con capacidad para 10 paquetes
    colaEnvio = xQueueCreate(10, sizeof(Paquete));

    // Tarea BLE en core 0 — el loop corre en core 1
    xTaskCreatePinnedToCore(
        tareaEnvioBLE,
        "BLE_Send",
        4096,
        NULL,
        1,
        NULL,
        0  // core 0
    );

    // Timer a 1000 Hz
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000000 / SAMPLE_RATE, true);
    timerAlarmEnable(timer);

    // BLE
    BLEDevice::init("MicroC");
    esp_ble_gatt_set_local_mtu(256);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setValue("Iniciando EMG RAW");
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("BLE listo.");
}

void loop() {
    if (pendingSamples > 0) {
        portENTER_CRITICAL(&mux);
        pendingSamples--;
        uint32_t idx = sampleCounter++;
        portEXIT_CRITICAL(&mux);

        uint16_t val1 = analogRead(RAW_PIN);
        uint16_t val2 = analogRead(DIN_PIN);

        emgBuffer[bufferCount] = val1;
        dinBuffer[bufferCount] = val2;
        bufferCount++;

        if (bufferCount == PACKET_SAMPLES) {
            uint32_t firstIndex = idx - PACKET_SAMPLES + 1;

            Paquete paquete;
            paquete.size = 4 + PACKET_SAMPLES * 4;

            paquete.data[0] = (firstIndex >> 24) & 0xFF;
            paquete.data[1] = (firstIndex >> 16) & 0xFF;
            paquete.data[2] = (firstIndex >> 8)  & 0xFF;
            paquete.data[3] =  firstIndex        & 0xFF;

            for (int i = 0; i < PACKET_SAMPLES; i++) {
                paquete.data[4 + i*4]     = (emgBuffer[i] >> 8) & 0xFF;
                paquete.data[4 + i*4 + 1] =  emgBuffer[i]       & 0xFF;
                paquete.data[4 + i*4 + 2] = (dinBuffer[i] >> 8) & 0xFF;
                paquete.data[4 + i*4 + 3] =  dinBuffer[i]       & 0xFF;
            }

            // Encolar sin bloquear — si la cola está llena se descarta
            xQueueSend(colaEnvio, &paquete, 0);
            bufferCount = 0;
        }
    }
}