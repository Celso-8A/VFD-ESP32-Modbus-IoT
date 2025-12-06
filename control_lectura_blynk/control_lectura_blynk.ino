#define BLYNK_TEMPLATE_ID "TMPL2olaNJZQQ"
#define BLYNK_TEMPLATE_NAME "Monitoreo y control VFD"
#define BLYNK_AUTH_TOKEN "xI0sep6ubtMi6S735OHKaM-i9Nj6_bj_"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <HardwareSerial.h>
#include <DHT.h>

// ------------------ WiFi ------------------
char auth[] = BLYNK_AUTH_TOKEN;
const char* ssid = "HUAWEI-rR7E";
const char* pass = "s3SY9Gc8";
BlynkTimer timer;

// ================== CONFIG DHT11 (Temperatura simulada del motor) ==================
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

const float TEMP_MAX_MOTOR = 30.0;  // Umbral de protección

// Intervalo de lectura del DHT11 (1 segundo)
unsigned long lastDhtRead = 0;
const unsigned long DHT_INTERVAL = 1000;  // 1 segundo

// ================== RS485 / VFD ==================
// Pin de control RS485
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

// Intervalo de lectura del VFD (1 segundo)
unsigned long lastReadTime = 0;

// ================== FLAGS / ESTADO ==================
bool debug = false;                 // Mostrar tramas HEX
bool lecturasHabilitadas   = true; // Lecturas del VFD (Hz, A, RPM)
bool tempLecturaHabilitada = true;  // Mostrar lectura de temperatura

bool motorEncendido = false;        // Para protección por sobretemperatura

// ================== VARIABLES PARA BLYNK ==================
float ultimaFrecuenciaHz = 0.0;
float ultimaCorrienteA   = 0.0;
float ultimaRPM          = 0.0;
float ultimaTempC        = 0.0;
String estadoSistema     = "Inicializando...";

// ================== PROTOTIPOS ==================
void procesarComando(String input);
void manejarTemperaturaMotor();
void readAndPrint(byte* cmd, const char* label, bool isCurrent, bool isRPM);
void setRPM(int rpm);
void sendRTUCommand(byte* data, int length);
uint16_t calculateCRC(byte* data, int length);
void enviarLecturasABlynk();

// ================== BLYNK: Text Input (V10) ==================
BLYNK_WRITE(V10) {
  String cmd = param.asStr();
  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.print("Comando desde Blynk: ");
  Serial.println(cmd);

  procesarComando(cmd);

  estadoSistema = "Último cmd: " + cmd;
  Blynk.virtualWrite(V20, estadoSistema);
}

// ================== BLYNK: Conectado ==================
BLYNK_CONNECTED() {
  estadoSistema = "Conectado a Blynk";
  Blynk.virtualWrite(V20, estadoSistema);
}

// ================== ENVÍO PERIÓDICO A BLYNK ==================
void enviarLecturasABlynk() {
  // Gauges
  Blynk.virtualWrite(V0, ultimaCorrienteA);   // Corriente (Double)
  Blynk.virtualWrite(V1, (int)ultimaRPM);     // RPM (Integer)
  Blynk.virtualWrite(V2, ultimaTempC);        // Temperatura (Double)
  Blynk.virtualWrite(V3, ultimaFrecuenciaHz); // Frecuencia (Double)

  // Label de estado
  Blynk.virtualWrite(V20, estadoSistema);
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);

  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW); // Modo recepción por defecto

  // Inicializar Serial2 para RS485
  RS485_SERIAL.begin(9600, SERIAL_8N1, 16, 17); // RX2=GPIO16, TX2=GPIO17

  // Inicializar DHT11
  dht.begin();

  // Conectar a WiFi + Blynk
  Blynk.begin(auth, ssid, pass);

  // Timer: enviar lecturas a Blynk cada 1 segundo
  timer.setInterval(1000L, enviarLecturasABlynk);

  Serial.println("Comandos disponibles (Serial o Blynk V10):");
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

  estadoSistema = "Sistema iniciado";
}

// ================== LOOP ==================
void loop() {
  Blynk.run();
  timer.run();

  // --- Comandos desde el monitor serie ---
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    procesarComando(input);
  }

  // --- Lecturas automáticas del VFD cada 1s (si están habilitadas) ---
  if (lecturasHabilitadas && (millis() - lastReadTime > 1000)) { // 1 segundo
    lastReadTime = millis();
    readAndPrint(readOutF,   "Frecuencia salida (Hz): ", false, false);
    delay(10);
    readAndPrint(readOutA,   "Corriente salida (A): ",  true,  false);
    delay(10);
    readAndPrint(readOutRPM, "RPM salida: ",             false, true);
    delay(10);
    Serial.println("-----------------------------");
  }

  // --- Manejo de temperatura del motor cada 1s ---
  if (millis() - lastDhtRead > DHT_INTERVAL) {  // 1 segundo
    lastDhtRead = millis();
    manejarTemperaturaMotor();
  }
}

// ================== PROCESAR COMANDO (Serial + Blynk) ==================
void procesarComando(String input) {
  if (input.equalsIgnoreCase("ON")) {
    sendRTUCommand(start_cmd, sizeof(start_cmd));
    motorEncendido = true;
    Serial.println("Encendiendo motor...");
    estadoSistema = "Motor encendido";
  }
  else if (input.equalsIgnoreCase("OFF")) {
    sendRTUCommand(stop_cmd, sizeof(stop_cmd));
    motorEncendido = false;
    Serial.println("Apagando motor...");
    estadoSistema = "Motor apagado";
  }
  else if (input.startsWith("RPM")) {
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex != -1) {
      int rpmValue = input.substring(spaceIndex + 1).toInt();
      if (rpmValue > MAX_RPM) {
        Serial.print("Error: RPM máximo permitido es ");
        Serial.println(MAX_RPM);
        estadoSistema = "Error: RPM > MAX";
      } else {
        setRPM(rpmValue);
        estadoSistema = "RPM objetivo: " + String(rpmValue);
      }
    }
  }
  else if (input.equalsIgnoreCase("READ ON")) {
    lecturasHabilitadas = true;
    Serial.println("Lecturas periódicas del VFD HABILITADAS.");
    estadoSistema = "Lecturas VFD ON";
  }
  else if (input.equalsIgnoreCase("READ OFF")) {
    lecturasHabilitadas = false;
    Serial.println("Lecturas periódicas del VFD DESHABILITADAS.");
    estadoSistema = "Lecturas VFD OFF";
  }
  else if (input.equalsIgnoreCase("TEMP ON")) {
    tempLecturaHabilitada = true;
    Serial.println("Lectura de temperatura DHT11 HABILITADA.");
    estadoSistema = "Temp ON";
  }
  else if (input.equalsIgnoreCase("TEMP OFF")) {
    tempLecturaHabilitada = false;
    Serial.println("Lectura de temperatura DHT11 DESHABILITADA (la protección sigue activa).");
    estadoSistema = "Temp OFF (protección activa)";
  }
  else if (input.equalsIgnoreCase("DEBUG ON")) {
    debug = true;
    Serial.println("DEBUG habilitado (se mostrarán tramas HEX).");
    estadoSistema = "DEBUG ON";
  }
  else if (input.equalsIgnoreCase("DEBUG OFF")) {
    debug = false;
    Serial.println("DEBUG deshabilitado.");
    estadoSistema = "DEBUG OFF";
  }
}

// ================== Manejo de temperatura y protección ==================
void manejarTemperaturaMotor() {
  float temp = dht.readTemperature(); // °C

  if (isnan(temp)) {
    if (tempLecturaHabilitada) {
      Serial.println("Error al leer el DHT11 (temperatura no válida).");
    }
    return;
  }

  ultimaTempC = temp;  // Para Blynk

  if (tempLecturaHabilitada) {
    Serial.print("Temperatura motor (DHT11): ");
    Serial.print(temp);
    Serial.println(" °C");
  }

  // Protección por sobretemperatura
  if (temp > TEMP_MAX_MOTOR) {
    if (motorEncendido) {
      sendRTUCommand(stop_cmd, sizeof(stop_cmd));
      motorEncendido = false;

      Serial.print("ALERTA: Motor detenido. Temperatura excedida: ");
      Serial.print(temp);
      Serial.println(" °C");

      estadoSistema = "ALERTA: Motor detenido por alta temp";
    } else {
      Serial.print("ALERTA: Temperatura excedida (motor ya apagado): ");
      Serial.print(temp);
      Serial.println(" °C");
      estadoSistema = "ALERTA: Alta temp (motor OFF)";
    }
  }
}

// ================== Lectura de registros del VFD ==================
void readAndPrint(byte* cmd, const char* label, bool isCurrent, bool isRPM) {
  while (RS485_SERIAL.available()) RS485_SERIAL.read(); // limpiar buffer
  sendRTUCommand(cmd, 4);

  unsigned long start = millis();
  byte resp[8];
  int idx = 0;

  // Timeout reducido: 100 ms en vez de 200 ms
  while ((millis() - start) < 100) {
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
    else if (isRPM) result = value;      // ya está en RPM
    else           result = value / 100.0; // frecuencia en Hz

    Serial.print(label);
    Serial.println(result, 2);

    // Guardar para Blynk
    if (isCurrent)      ultimaCorrienteA   = result;
    else if (isRPM)     ultimaRPM          = result;
    else                ultimaFrecuenciaHz = result;

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

// ================== Setear RPM (cambia frecuencia) ==================
void setRPM(int rpm) {
  float frequencyHz = rpm / 60.0;
  uint16_t value = frequencyHz * 100;   // según escala de tu VFD

  byte freqCommand[5] = {0x01, 0x05, 0x02, highByte(value), lowByte(value)};
  sendRTUCommand(freqCommand, 5);

  Serial.print("RPM objetivo: ");
  Serial.println(rpm);
}

// ================== Enviar comando MODBUS RTU ==================
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

// ================== Cálculo de CRC ==================
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
