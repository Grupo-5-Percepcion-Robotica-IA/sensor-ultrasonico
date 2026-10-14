# Avances TP Distancia - Percepción Robótica

## Ultrasonido HC-SR04
- ESP32 con ESP-IDF 5.5
- RMT TX para TRIG y RMT RX para ECHO
- micro-ROS mediante Wi-Fi
- Frecuencia aproximada: 14 Hz
- Tópicos: /ultrasonic/distance y /ultrasonic/range
- Estado: probado y funcionando

## Láser VL53L0X
- Comunicación I2C
- API VL53L0X
- micro-ROS mediante Wi-Fi
- Tópicos: /laser/distance y /laser/range
- Estado: compilado, pendiente de prueba física

Se incluyen los archivos fuente y CMake necesarios
para que el equipo pueda integrarlos a su workspace.
Las dependencias externas deben configurarse aparte.
