#include <Arduino.h>
#include <LoRaMesh.h>
#include <Wire.h>

#define pinLed 2
#define sampleSize 500
//Loop counting and led
bool fastloop = false;
bool pinOn = false;

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

/* Received command */
uint8_t command;

long lastMsg = 0, loopTimer = 0;
uint16_t value = 0, readings = 0, peaksx = 0, peaksy = 0, peaksz = 0;
float signalsx[3] = {0, 0, 0}, signalsy[3] = {0, 0, 0}, signalsz[3] = {0, 0, 0}, threshold=1.10;
uint16_t rhox, rhoy, rhoz;
int16_t acc_x, acc_y, acc_z, temperature, gyro_x, gyro_y, gyro_z;

MeshStatus_Typedef LocalWriteConfig(uint16_t id, uint16_t net,
                                    uint32_t uniqueId) {
    uint8_t crc = 0;
    uint8_t bufferPayload[31];
    uint8_t payloadSize;
    uint8_t i = 0;
    uint8_t command;
    uint16_t idRead = 0;

    /* Asserts parameters */
    if (uniqueId == 0) {
        Serial.println("ERRO UniqueID");
        return MESH_ERROR;
    }

    if (hSerialCommands == NULL) {
        Serial.println("ERRO hSerialCommands");
        return MESH_ERROR;
    }

    /*Creating the payload with
    the passed NET and UniqueID and then,
    call the function with the ID passed*/

    bufferPayload[0] = net & 0xFF;
    bufferPayload[1] = (net >> 8) & 0xFF;
    bufferPayload[2] = uniqueId & 0xFF;
    bufferPayload[3] = (uniqueId >> 8) & 0xFF;
    bufferPayload[4] = (uniqueId >> 16) & 0xFF;
    bufferPayload[5] = (uniqueId >> 24) & 0xFF;

    /* Loads dummy bytes */
    for (i = 6; i < 11; i++) bufferPayload[i] = 0x00;

    PrepareFrameCommand(id, CMD_WRITECONFIG, &bufferPayload[0], i);

    /* Sends packet */
    SendPacket();

    /* Flush serial input buffer */
    // SerialFlush(hSerialCommand);
    // To not make modifications on the original library
    // I copied the content of the SerialFlush function
    while (hSerialCommands->available() > 0) {
        hSerialCommands->read();
    }

    /* Waits for response */
    if (ReceivePacketCommand(&idRead, &command, &bufferPayload[0], &payloadSize,
                             5000) != MESH_OK) {
        Serial.println("Erro de pacote recebido");
        return MESH_ERROR;
    }

    /* Checks command */
    if (command != CMD_WRITECONFIG) {
        Serial.print("Erro! Commando recebido: ");
        Serial.println(command, HEX);
        return MESH_ERROR;
    }else{
        Serial.print("ID: ");
        Serial.print("NET: ");
        Serial.println((uint32_t)bufferPayload[0] | ((uint32_t)(bufferPayload[1]) << 8));
        Serial.print("UID: ");
        Serial.println((uint32_t)bufferPayload[2] | ((uint32_t)(bufferPayload[3]) << 8) | ((uint32_t)(bufferPayload[4]) << 16) | ((uint32_t)(bufferPayload[5]) << 24));
        Serial.print("Versão de HW: ");
        Serial.println((uint32_t)bufferPayload[6]);
        Serial.print("RSD: ");
        Serial.println((uint32_t)bufferPayload[7]);
        Serial.print("Rev de FW: ");
        Serial.println((uint32_t)bufferPayload[8]);
        Serial.print("RSD: ");
        Serial.println((uint32_t)bufferPayload[9] | ((uint32_t)(bufferPayload[10]) << 8) | ((uint32_t)(bufferPayload[11]) << 16));
    }

    return MESH_OK;
}




MeshStatus_Typedef SendDatatest() {
    uint8_t i = 0, command, payloadSize;
    uint16_t idRead = 0;
    uint8_t bufferPayload[31];

    /*Creating the payload with
    the passed NET and UniqueID and then,
    call the function with the ID passed*/

    bufferPayload[0] = 0x23;
    bufferPayload[1] =  rhox & 0xFF;
    bufferPayload[2] =  (rhox >> 8) & 0xFF;
    bufferPayload[3] =  rhoy & 0xFF;
    bufferPayload[4] =  (rhoy >> 8) & 0xFF;
    bufferPayload[5] =  rhoz & 0xFF;
    bufferPayload[6] =  (rhoz >> 8) & 0xFF;

    if (PrepareFrameCommand(0x14, 0x11, &bufferPayload[0], 7) != MESH_OK) {
        Serial.println("Erro de mensagem enviada");
        return MESH_ERROR;
    }

    /* Sends packet */
    SendPacket();

    return MESH_OK;
}

void read_mpu_6050_data() {        // Subroutine for reading the raw gyro and
                                   // accelerometer data
    Wire.beginTransmission(0x68);  // Start communicating with the MPU-6050
    Wire.write(0x3B);              // Send the requested starting register
    Wire.endTransmission();        // End the transmission
    Wire.requestFrom(0x68, 14);    // Request 14 bytes from the MPU-6050
    while (Wire.available() < 14)
        ;  // Wait until all the bytes are received
    acc_x = Wire.read() << 8 |
            Wire.read();  // Add the low and high byte to the acc_x variable
    acc_y = Wire.read() << 8 |
            Wire.read();  // Add the low and high byte to the acc_y variable
    acc_z = Wire.read() << 8 |
            Wire.read();  // Add the low and high byte to the acc_z variable
    temperature =
        Wire.read() << 8 |
        Wire.read();  // Add the low and high byte to the temperature variable
    gyro_x = Wire.read() << 8 |
             Wire.read();  // Add the low and high byte to the gyro_x variable
    gyro_y = Wire.read() << 8 |
             Wire.read();  // Add the low and high byte to the gyro_y variable
    gyro_z = Wire.read() << 8 |
             Wire.read();  // Add the low and high byte to the gyro_z variable
}


void setup_mpu_6050_registers() {
    // Activate the MPU-6050
    //uint8_t data[4] = {0x07, 0x00, 0x10, 0x08};
    uint8_t data[2] = {0x08, 0x10};
    //wake up
    Wire.beginTransmission(0x68);  // Start communicating with the MPU-6050
    Wire.write(0x6B);              // Send the requested starting register
    Wire.write(0x00);              // Set the value to the requested starting register
    Wire.endTransmission();        // End the transmission
    //set the four registers
    Wire.beginTransmission(0x68);  // Start communicating with the MPU-6050
    Wire.write(0x1B);              // Send the requested starting register
    Wire.write(data, 2);              // Set the value to the requested starting register
    Wire.endTransmission();        // End the transmission
}

void setup() {
    // initialize digital pin LED_BUILTIN as an output.
    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);  // turn the LED on (HIGH is the voltage level)
    // Configurando o módulo LoRaMesh
    delay(1000);
    Serial.begin(9600);  /* Initialize monitor serial */
    Wire.begin();  // Start I2C as master
    setup_mpu_6050_registers();

    /* Initialize SoftwareSerial */
    // Pinos:
    // 7-> Lora TX
    // 6-> Lora RX
    hSerialCommands = SerialCommandsInit(7, 6, 9600);

    // 4-> Lora TX
    // 5-> Lora RX
    // hSerialTransparent = SerialTranspInit(4, 5, 9600);

    /* Gets the local device ID */
    if (LocalRead(&localId, &localNet, &localUniqueId) != MESH_OK)
        Serial.print("Couldn't read local ID\n\n");
    else {
        Serial.print("Local ID: ");
        Serial.println(localId);
        Serial.print("Local NET: ");
        Serial.println(localNet);
        Serial.print("Local Unique ID: ");
        Serial.println(localUniqueId, HEX);
        Serial.print("\n");
    }

    // ID = 0 means that is a master node
    /*if (localNet == 0) {
        Serial.println("Sending master configurations...");
        LocalWriteConfig(0x00, 0x25, localUniqueId);
    } else {
        Serial.println("The node is a master!");
    }

    if (LocalRead(&localId, &localNet, &localUniqueId) != MESH_OK)
        Serial.print("Couldn't read local ID\n\n");
    else {
        Serial.print("Local ID: ");
        Serial.println(localId);
        Serial.print("Local NET: ");
        Serial.println(localNet);
        Serial.print("Local Unique ID: ");
        Serial.println(localUniqueId, HEX);
        Serial.print("\n");
    }*/
    digitalWrite(pinLed, LOW);
    loopTimer = micros();
}

void loop() {
    /*if (ReceivePacketCommand(&id, &command, &bufferPayload[0], &payloadSize, 10000) == MESH_OK) {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("RECEBIDO:");
        Serial.print("tamanho: ");
        Serial.println(payloadSize);
        for (int i = 0; i < payloadSize; i++) Serial.println(bufferPayload[i], HEX);
    } else {*/
    //delay(5000);
    //SendDatatest();
    //counter++;
    //Serial.println("Pacote enviado");
    //}
    read_mpu_6050_data();
    signalsx[0] = signalsx[1];
    signalsy[0] = signalsy[1];
    signalsz[0] = signalsz[1];
    signalsx[1] = signalsx[2];
    signalsy[1] = signalsy[2];
    signalsz[1] = signalsz[2];
    signalsx[2] = acc_x/4096;
    signalsy[2] = acc_y/4096;
    signalsz[2] = acc_z/4096;
    readings++;
    if (readings > 2) {
        if (signalsx[0]*threshold < signalsx[1] && signalsx[1] > signalsx[2]*threshold) peaksx++;
        if (signalsy[0]*threshold < signalsy[1] && signalsy[1] > signalsy[2]*threshold) peaksy++;
        if (signalsz[0]*threshold < signalsz[1] && signalsz[1] > signalsz[2]*threshold) peaksz++;
    }
    if (readings == sampleSize) {
        rhox = (peaksx *1000) / sampleSize;
        rhoy = (peaksy *1000) / sampleSize;
        rhoz = (peaksz *1000) / sampleSize;
        SendDatatest();
        //Serial.print(rhox);
        //Serial.print(":");
        //Serial.print(rhoy);
        //Serial.print(":");
        //Serial.println(rhoz);
        readings = 1;
        peaksx = 0;
        peaksy = 0;
        peaksz = 0;
        if (fastloop){
            pinOn = !pinOn;
            digitalWrite(pinLed, pinOn);
            fastloop = false;
        }
        
    }
    //while (micros() - loopTimer < 1000) fastloop = true;
    while (micros() - loopTimer < 2000) fastloop = true;
    loopTimer = micros();
}