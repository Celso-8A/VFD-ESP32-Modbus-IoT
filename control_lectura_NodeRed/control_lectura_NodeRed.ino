#include <HardwareSerial.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ====== CONFIG DHT11 (Temperatura simulada del motor) ======
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

const float TEMP_MAX_MOTOR = 30.0;  // Umbral de protección (°C)

// Pines de control RS485
#define DE_RE_PIN 18  // DE & RE unidos controlados por GPIO18

// Usamos Serial2 para RS485 (GPIO16 como RX2, GPIO17 como TX2)
#define RS485_SERIAL Serial2

// Comandos básicos VFD
byte start_cmd[] = {0x01, 0x03, 0x01, 0x01}; // RUN forward
byte stop_cmd[]  = {0x01, 0x03, 0x01, 0x08}; // STOP

const int MAX_RPM = 24000; // Máximo RPM de tu motor

// Comandos de lectura VFD
byte readOutF[]   = {0x01, 0x04, 0x01, 0x01}; // Output Frequency
byte readOutA[]   = {0x01, 0x04, 0x01, 0x02}; // Output Current
byte readOutRPM[] = {0x01, 0x04, 0x01, 0x03}; // Output RPM

unsigned long lastReadTime = 0;

// Intervalo DHT
unsigned long lastDhtRead = 0;
const unsigned long DHT_INTERVAL = 2000;  // 2 segundos

// ====== OPCIÓN DEBUG ======
bool debug = false; // true para ver tramas HEX

// ====== CONTROL DE LECTURAS ======
bool lecturasHabilitadas   = false;   // Lecturas del VFD (Hz, A, RPM)
bool tempLecturaHabilitada = true;    // Mostrar o no la lectura de temperatura

// Estado del motor
bool motorEncendido = false;          // Para saber si debemos detenerlo por sobretemperatura

// ====== WIFI + MQTT ======
const char* WIFI_SSID = "HUAWEI-rR7E";        // <-- CAMBIA AQUÍ
const char* WIFI_PASS = "s3SY9Gc8";    // <-- CAMBIA AQUÍ

// Si usas broker en la Raspberry, pon la IP, ej. "192.168.1.50"
// Si usas EMQX en la nube, pon "broker.emqx.io"
const char* MQTT_HOST = "broker.emqx.io";   // <-- CAMBIA AQUÍ
const uint16_t MQTT_PORT = 1883;

// Topics MQTT
const char* TOPIC_CMD    = "cnc/motor/cmd";      // Comandos de control
const char* TOPIC_FREQ   = "cnc/motor/freq";     // Frecuencia (Hz)
const char* TOPIC_CURR   = "cnc/motor/current";  // Corriente (A)
const char* TOPIC_RPM    = "cnc/motor/rpm";      // RPM
const char* TOPIC_TEMP   = "cnc/motor/temp";     // Temperatura (°C)
const char* TOPIC_STATUS = "cnc/motor/status";   // Estado del sistema

// Variables para guardar últimas lecturas
float ultimaFrecuenciaHz = 0.0;
float ultimaCorrienteA   = 0.0;
float ultimaRpm          = 0.0;
float ultimaTempC        = 0.0;

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ---------- Prototipos ----------
void procesarComando(String input);
void manejarTemperaturaMotor();
void readAndPrint(byte* cmd, const char* label, bool isCurrent, bool isRPM);
void setRPM(int rpm);
void sendRTUCommand(byte* data, int length);
uint16_t calculateCRC(byte* data, int length);
void publishStatus(const char* msg);

// ---------- Publicar estado del sistema ----------
void publishStatus(const char* msg) {
  Serial.print("ESTADO: ");
  Serial.println(msg);

  if (mqtt.connected()) {
    // Retain = true para que siempre se vea el último estado
    mqtt.publish(TOPIC_STATUS, msg, true);
  }
}

// ---------- Conectar WiFi ----------
void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// ---------- Callback MQTT (cuando llega un mensaje de Node-RED) ----------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String cmd;
  for (unsigned int i = 0; i < length; i++) {
    cmd += (char)payload[i];
  }
  cmd.trim();

  Serial.print("MQTT cmd [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(cmd);

  // Reutilizamos la misma lógica que el monitor serie:
  procesarComando(cmd);
}

// ---------- Asegurar conexión MQTT ----------
void ensureMqtt() {
  while (!mqtt.connected()) {
    Serial.print("Conectando a MQTT...");
    String cid = "ESP32-CNC-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(cid.c_str())) {
      Serial.println("conectado.");
      publishStatus("Sistema conectado a MQTT, motor apagado");
      mqtt.subscribe(TOPIC_CMD);  // Escuchar comandos de Node-RED
      Serial.print("Suscrito a: ");
      Serial.println(TOPIC_CMD);
    } else {
      Serial.print(" fallo, rc=");
      Serial.print(mqtt.state());
      Serial.println(" reintento en 2s");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW); // Modo recepción por defecto
  
  // Inicializar Serial2 para RS485
  RS485_SERIAL.begin(9600, SERIAL_8N1, 16, 17); // RX2=GPIO16, TX2=GPIO17

  // Inicializar DHT11
  dht.begin();

  // Conectar WiFi + MQTT
  connectWifi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  Serial.println("Comandos disponibles (Serial y MQTT):");
  Serial.println("ON          -> Encender motor");
  Serial.println("OFF         -> Apagar motor");
  Serial.println("RPM XXX     -> Establecer revoluciones (ej: RPM 12000)");
  Serial.println("READ ON     -> Habilitar lecturas VFD (Hz, A, RPM)");
  Serial.println("READ OFF    -> Deshabilitar lecturas VFD");
  Serial.println("TEMP ON     -> Mostrar temperatura DHT11");
  Serial.println("TEMP OFF    -> Ocultar temperatura DHT11 (la protección sigue activa)");
  Serial.println("DEBUG ON    -> Mostrar tramas HEX");
  Serial.println("DEBUG OFF   -> Ocultar tramas HEX");
  Serial.print("RPM máximo: ");
  Serial.println(MAX_RPM);
  Serial.print("Umbral temperatura motor: ");
  Serial.print(TEMP_MAX_MOTOR);
  Serial.println(" °C");
}

void loop() {
  // Mantener MQTT
  if (!mqtt.connected()) {
    ensureMqtt();
  }
  mqtt.loop();

  // --- Lectura de comandos desde el monitor serie ---
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    procesarComando(input);
  }

  // --- Lecturas automáticas del VFD cada 2s (si están habilitadas) ---
  if (lecturasHabilitadas && (millis() - lastReadTime > 2000)) {
    lastReadTime = millis();
    // Frecuencia
    readAndPrint(readOutF,   "Frecuencia salida (Hz): ", false, false);
    delay(50);
    // Corriente
    readAndPrint(readOutA,   "Corriente salida (A): ",  true,  false);
    delay(50);
    // RPM
    readAndPrint(readOutRPM, "RPM salida: ",             false, true);
    delay(50);
    Serial.println("-----------------------------");
  }

  // --- Manejo de temperatura del "motor" (DHT11) ---
  if (millis() - lastDhtRead > DHT_INTERVAL) {
    lastDhtRead = millis();
    manejarTemperaturaMotor();
  }
}

// ====== PROCESAR COMANDO (Serial + MQTT) ======
void procesarComando(String input) {
  if (input.length() == 0) return;

  if (input.equalsIgnoreCase("ON")) {
    sendRTUCommand(start_cmd, sizeof(start_cmd));
    motorEncendido = true;
    Serial.println("Encendiendo motor...");
    publishStatus("Motor encendido");
  }
  else if (input.equalsIgnoreCase("OFF")) {
    sendRTUCommand(stop_cmd, sizeof(stop_cmd));
    motorEncendido = false;
    Serial.println("Apagando motor...");
    publishStatus("Motor apagado");
  }
  else if (input.startsWith("RPM")) {
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex != -1) {
      int rpmValue = input.substring(spaceIndex + 1).toInt();
      if (rpmValue > MAX_RPM) {
        Serial.print("Error: RPM máximo permitido es ");
        Serial.println(MAX_RPM);
        publishStatus("Error: RPM excede el máximo permitido");
      } else {
        setRPM(rpmValue);
        publishStatus("RPM objetivo ajustado");
      }
    }
  }
  else if (input.equalsIgnoreCase("READ ON")) {
    lecturasHabilitadas = true;
    Serial.println("Lecturas periódicas del VFD HABILITADAS.");
    publishStatus("Lecturas del VFD habilitadas");
  }
  else if (input.equalsIgnoreCase("READ OFF")) {
    lecturasHabilitadas = false;
    Serial.println("Lecturas periódicas del VFD DESHABILITADAS.");
    publishStatus("Lecturas del VFD deshabilitadas");
  }
  else if (input.equalsIgnoreCase("TEMP ON")) {
    tempLecturaHabilitada = true;
    Serial.println("Lectura de temperatura DHT11 HABILITADA.");
    publishStatus("Monitor de temperatura habilitado");
  }
  else if (input.equalsIgnoreCase("TEMP OFF")) {
    tempLecturaHabilitada = false;
    Serial.println("Lectura de temperatura DHT11 DESHABILITADA (la protección sigue activa).");
    publishStatus("Monitor de temperatura oculto (protección activa)");
  }
  else if (input.equalsIgnoreCase("DEBUG ON")) {
    debug = true;
    Serial.println("DEBUG habilitado (se mostrarán tramas HEX).");
    publishStatus("DEBUG habilitado");
  }
  else if (input.equalsIgnoreCase("DEBUG OFF")) {
    debug = false;
    Serial.println("DEBUG deshabilitado.");
    publishStatus("DEBUG deshabilitado");
  }
}

// ====== Función de manejo de temperatura y protección ======
void manejarTemperaturaMotor() {
  float temp = dht.readTemperature(); // °C

  if (isnan(temp)) {
    if (tempLecturaHabilitada) {
      Serial.println("Error al leer el DHT11 (temperatura no válida).");
    }
    // Podrías publicar estado de error si quieres:
    // publishStatus("Error al leer temperatura DHT11");
    return;
  }

  ultimaTempC = temp;

  // Mostrar lectura sólo si está habilitada
  if (tempLecturaHabilitada) {
    Serial.print("Temperatura motor (DHT11): ");
    Serial.print(temp);
    Serial.println(" °C");
  }

  // Publicar temperatura por MQTT
  if (mqtt.connected()) {
    char buf[16];
    dtostrf(ultimaTempC, 0, 1, buf); // 1 decimal
    mqtt.publish(TOPIC_TEMP, buf, false);
  }

  // Protección por sobretemperatura
  if (temp > TEMP_MAX_MOTOR) {
    if (motorEncendido) {
      // Detener motor
      sendRTUCommand(stop_cmd, sizeof(stop_cmd));
      motorEncendido = false;

      Serial.print("ALERTA: Motor detenido. Temperatura excedida: ");
      Serial.print(temp);
      Serial.println(" °C");

      publishStatus("ALERTA: Motor detenido por sobretemperatura");
    } else {
      Serial.print("ALERTA: Temperatura excedida (motor ya apagado): ");
      Serial.print(temp);
      Serial.println(" °C");

      publishStatus("ALERTA: Sobretemperatura con motor apagado");
    }
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
    if (isCurrent) {
      result = value / 10.0;      // Corriente en A
    }
    else if (isRPM) {
      result = value;             // ya está en RPM
    }
    else {
      result = value / 100.0;     // Frecuencia en Hz
    }

    Serial.print(label);
    Serial.println(result, 2);

    // ==== PUBLICAR SEGÚN TIPO ====
    if (!isCurrent && !isRPM) {
      // Frecuencia
      ultimaFrecuenciaHz = result;
      if (mqtt.connected()) {
        char buf[16];
        dtostrf(ultimaFrecuenciaHz, 0, 2, buf); // ej. "50.00"
        mqtt.publish(TOPIC_FREQ, buf, false);
      }
    } else if (isCurrent) {
      // Corriente
      ultimaCorrienteA = result;
      if (mqtt.connected()) {
        char buf[16];
        dtostrf(ultimaCorrienteA, 0, 2, buf);   // ej. "2.35"
        mqtt.publish(TOPIC_CURR, buf, false);
      }
    } else if (isRPM) {
      // RPM
      ultimaRpm = result;
      if (mqtt.connected()) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%.0f", ultimaRpm); // entero
        mqtt.publish(TOPIC_RPM, buf, false);
      }
    }
    // ==============================

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
