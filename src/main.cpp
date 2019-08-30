#include <Arduino.h>
#include <LoRaMesh.h>
// Variáveis a serem preenchidas com o recebido da mensagem
uint16_t id;
uint8_t payload[31], payloadSize;

/* Local device ID */
uint16_t localId;

/* Remote device ID */
uint16_t remoteId;

/* Local device Unique ID */
uint32_t localUniqueId;

/* Local device Net */
uint16_t localNet;

/* Remote device ID to communicate with */
uint16_t ID = 10;

/* Payload buffer */
uint8_t bufferPayload[MAX_PAYLOAD_SIZE] = {0};

/* Values read on each analog inputs */
uint16_t AdcIn[2];

/* SoftwareSerial handles */
SoftwareSerial* hSerialCommands = NULL;
SoftwareSerial* hSerialTransparent = NULL;

void setup() {
    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN,
                 HIGH);  // turn the LED on (HIGH is the voltage level)
    // Configurando o módulo LoRaMesh
    delay(1000);
    Serial.begin(9600); /* Initialize monitor serial */

    /* Initialize SoftwareSerial */
    // Pinos:
    // 7-> Lora TX
    // 6-> Lora RX
    hSerialCommands = SerialCommandsInit(7, 6, 9600);

    // 4-> Lora TX
    // 5-> Lora RX
    hSerialTransparent = SerialTranspInit(4, 5, 9600);

    /* Gets the local device ID*/
    if(LocalRead(&localId, &localNet, &localUniqueId) != MESH_OK)
      Serial.print("Couldn't read local ID\n\n");
    else
    {
      Serial.print("Local ID: ");
      Serial.println(localId);
      Serial.print("Local NET: ");
      Serial.println(localNet);
      Serial.print("Local Unique ID: ");
      Serial.println(localUniqueId, HEX);
      Serial.print("\n");
    }

    delay(500);
}

void loop() {
    if (ReceivePacketTransp(&id, &payload[0], &payloadSize, 10000) == MESH_OK) {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("RECEBIDO:");
        Serial.print("tamanho: ");
        Serial.println(payloadSize);
        for (int i = 0; i < payloadSize; i++) Serial.println(payload[i], HEX);
    } else {
        Serial.println("ERRO AO RECEBER PACOTE");
    }
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}