# Sistema de Monitoreo y Control de un Variador de Frecuencia (VFD) con ESP32
Este repositorio contiene el desarrollo completo del proyecto final de ingeniería, cuyo objetivo fue el diseño e implementación de un sistema de monitoreo y control remoto de un variador de frecuencia (VFD) utilizando un ESP32, comunicación RS485 bajo protocolo Modbus RTU, integración con Blynk y visualización avanzada mediante MQTT + Node-RED.

El proyecto fue desarrollado en cuatro etapas progresivas, permitiendo validar el funcionamiento desde el nivel más básico (control serial) hasta una plataforma IoT completa con dashboards remotos.
# Objetivos
Desarrollar un sistema capaz de:
Controlar un variador de frecuencia de forma local y remota.
Ajustar la velocidad del motor mediante RPM.
Monitorear en tiempo real:
Frecuencia de salida (Hz)
Corriente de salida (A)
Velocidad del motor (RPM)
Visualizar y controlar el sistema desde:
Monitor Serial
Aplicación Blynk
Dashboard Node-RED vía MQTT
# Tecnologías Utilizadas
ESP32
Convertidor TTL ↔ RS485 (MAX485)
Variador de Frecuencia (VFD)
Protocolo Modbus RTU
Plataforma Blynk IoT 
MQTT (MQTTX)
Node-RED
Arduino IDE
# Estructura del Repositorio
📂 control_modbus_serial.ino
📂 control_lectura_modbus_serial.ino
📂 control_lectura_blynk.ino
📂 control_lectura_NodeRed.ino
📂 flow1.json   (flujo Node-RED)
