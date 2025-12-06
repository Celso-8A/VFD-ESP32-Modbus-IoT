#include <HardwareSerial.h>

// Pines de control RS485
#define DE_RE_PIN 18  // DE & RE unidos controlados por GPIO18

// Usamos Serial2 para RS485 (GPIO16 como RX2, GPIO17 como TX2)
#define RS485_SERIAL Serial2

// Comandos básicos
byte start_cmd[] = {0x01, 0x03, 0x01, 0x01}; // RUN forward
byte stop_cmd[]  = {0x01, 0x03, 0x01, 0x08}; // STOP

const int MAX_RPM = 24000; // Máximo RPM de tu motor

// Comandos de lectura
byte readOutF[]   = {0x01, 0x04, 0x01, 0x01}; // Output Frequency
byte readOutA[]   = {0x01, 0x04, 0x01, 0x02}; // Output Current
byte readOutRPM[] = {0x01, 0x04, 0x01, 0x03}; // Output RPM

unsigned long lastReadTime = 0;

// ====== OPCIÓN DEBUG ======
bool debug = false; // <-- Cambia a true si quieres ver la trama HEX

// ====== CONTROL DE LECTURAS ======
bool lecturasHabilitadas = true; // Lecturas del VFD (Hz, A, RPM)

void setup() {
  Serial.begin(115200);

  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW); // Modo recepción por defecto
  
  // Inicializar Serial2 para RS485
  RS485_SERIAL.begin(9600, SERIAL_8N1, 16, 17); // RX2=GPIO16, TX2=GPIO17
  
  Serial.println("Comandos disponibles:");
  Serial.println("ON          -> Encender motor");
  Serial.println("OFF         -> Apagar motor");
  Serial.println("RPM XXX     -> Establecer revoluciones (ej: RPM 12000)");
  Serial.println("READ ON     -> Habilitar lecturas VFD (Hz, A, RPM)");
  Serial.println("READ OFF    -> Deshabilitar lecturas VFD");
  Serial.println("DEBUG ON    -> Mostrar tramas HEX");
  Serial.println("DEBUG OFF   -> Ocultar tramas HEX");
  Serial.print("RPM máximo: ");
  Serial.println(MAX_RPM);
}

void loop() {
  // --- Lectura de comandos desde el monitor serie ---
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
        }
      }
    }
    else if (input.equalsIgnoreCase("READ ON")) {
      lecturasHabilitadas = true;
      Serial.println("Lecturas periódicas del VFD HABILITADAS.");
    }
    else if (input.equalsIgnoreCase("READ OFF")) {
      lecturasHabilitadas = false;
      Serial.println("Lecturas periódicas del VFD DESHABILITADAS.");
    }
    else if (input.equalsIgnoreCase("DEBUG ON")) {
      debug = true;
      Serial.println("DEBUG habilitado (se mostrarán tramas HEX).");
    }
    else if (input.equalsIgnoreCase("DEBUG OFF")) {
      debug = false;
      Serial.println("DEBUG deshabilitado.");
    }
  }

  // --- Lecturas automáticas del VFD cada 2s (si están habilitadas) ---
  if (lecturasHabilitadas && (millis() - lastReadTime > 2000)) {
    lastReadTime = millis();
    readAndPrint(readOutF,   "Frecuencia salida (Hz): ", false, false);
    delay(50);
    readAndPrint(readOutA,   "Corriente salida (A): ",  true,  false);
    delay(50);
    readAndPrint(readOutRPM, "RPM salida: ",             false, true);
    delay(50);
    Serial.println("-----------------------------");
  }
}

// ====== Lectura de registros del VFD ======
void readAndPrint(byte* cmd, const char* label, bool isCurrent, bool isRPM) {
  while (RS485_SERIAL.available()) RS485_SERIAL.read(); // limpiar buffer
  sendRTUCommand(cmd, 4);

  unsigned long start = millis();
  byte resp[8];
  int idx = 0;

  while ((millis() - start) < 200) {
    if (RS485_SERIAL.available()) {
      byte b = RS485_SERIAL.read();
      if (idx == 0 && b != 0x01) continue; // esperar inicio
      resp[idx++] = b;
      if (idx >= 7) break;
    }
  }

  if (idx >= 7) {
    uint16_t value = (resp[4] << 8) | resp[5];
    float result;
    if (isCurrent) result = value / 10.0;
    else if (isRPM) result = value; // ya está en RPM
    else result = value / 100.0;

    Serial.print(label);
    Serial.println(result, 2);

    if (debug) {
      Serial.print("Respuesta HEX: ");
      for (int i = 0; i < idx; i++) {
        Serial.print(resp[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  } else {
    Serial.println("Sin respuesta o incompleta");
  }
}

// ====== Setear RPM ======
void setRPM(int rpm) {
  float frequencyHz = rpm / 60.0;
  uint16_t value = frequencyHz * 100;

  byte freqCommand[5] = {0x01, 0x05, 0x02, highByte(value), lowByte(value)};
  sendRTUCommand(freqCommand, 5);

  Serial.print("RPM objetivo: ");
  Serial.println(rpm);
}

// ====== Enviar comando MODBUS RTU ======
void sendRTUCommand(byte* data, int length) {
  uint16_t crc = calculateCRC(data, length);

  // Activar modo transmisión
  digitalWrite(DE_RE_PIN, HIGH);
  delayMicroseconds(500);

  // Enviar datos
  for (int i = 0; i < length; i++) {
    RS485_SERIAL.write(data[i]);
  }
  RS485_SERIAL.write(lowByte(crc));
  RS485_SERIAL.write(highByte(crc));

  RS485_SERIAL.flush(); // Esperar a que se envíen todos los datos
  delayMicroseconds(500);
  
  // Volver a modo recepción
  digitalWrite(DE_RE_PIN, LOW);
}

// ====== Cálculo de CRC ======
uint16_t calculateCRC(byte* data, int length) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)data[pos];
    for (int i = 0; i < 8; i++) {
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
