#include <HardwareSerial.h>

// Pines de control RS485
#define DE_RE_PIN 18  // DE & RE unidos controlados por GPIO18

// Usamos Serial2 para RS485 (GPIO16 como RX2, GPIO17 como TX2)
#define RS485_SERIAL Serial2

// Comandos básicos
byte start_cmd[] = {0x01, 0x03, 0x01, 0x01}; // RUN forward
byte stop_cmd[]  = {0x01, 0x03, 0x01, 0x08}; // STOP

const int MAX_RPM = 24000; // Máximo RPM de tu motor

// ====== OPCIÓN DEBUG ======
bool debug = true; // <-- Cambia a true si quieres ver la trama HEX

void setup() {
  Serial.begin(115200);
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW); // Modo recepción por defecto
  
  // Inicializar Serial2 para RS485
  RS485_SERIAL.begin(9600, SERIAL_8N1, 16, 17); // RX2=GPIO16, TX2=GPIO17
  
  Serial.println("Comandos disponibles:");
  Serial.println("ON   -> Encender motor");
  Serial.println("OFF  -> Apagar motor");
  Serial.println("RPM XXX -> Establecer revoluciones");
  Serial.print("RPM máximo: ");
  Serial.println(MAX_RPM);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.equalsIgnoreCase("ON")) {
      sendRTUCommand(start_cmd, sizeof(start_cmd));
      Serial.println("Encendiendo motor...");
    }
    else if (input.equalsIgnoreCase("OFF")) {
      sendRTUCommand(stop_cmd, sizeof(stop_cmd));
      Serial.println("Apagando motor...");
    }
    else if (input.startsWith("RPM")) {
      int spaceIndex = input.indexOf(' ');
      if (spaceIndex != -1) {
        int rpmValue = input.substring(spaceIndex + 1).toInt();
        if (rpmValue > MAX_RPM) {
          Serial.print("Error: RPM máximo permitido es ");
          Serial.println(MAX_RPM);
        } else {
          setRPM(rpmValue);
          Serial.print("Configurando RPM a: ");
          Serial.println(rpmValue);
        }
      }
    }
  }
}

void setRPM(int rpm) {
  float frequencyHz = rpm / 60.0;
  uint16_t value = frequencyHz * 100;

  byte freqCommand[5] = {0x01, 0x05, 0x02, highByte(value), lowByte(value)};
  sendRTUCommand(freqCommand, 5);
}

void sendRTUCommand(byte* data, int length) {
  uint16_t crc = calculateCRC(data, length);

  // Activar modo transmisión
  digitalWrite(DE_RE_PIN, HIGH);
  delayMicroseconds(500);

  // Enviar datos
  for(int i = 0; i < length; i++) {
    RS485_SERIAL.write(data[i]);
  }
  RS485_SERIAL.write(lowByte(crc));
  RS485_SERIAL.write(highByte(crc));

  RS485_SERIAL.flush(); // Esperar a que se envíen todos los datos
  delayMicroseconds(500);
  
  // Volver a modo recepción
  digitalWrite(DE_RE_PIN, LOW);
}

uint16_t calculateCRC(byte* data, int length) {
  uint16_t crc = 0xFFFF;
  for(int pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)data[pos];
    for(int i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}
