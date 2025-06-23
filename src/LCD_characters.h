#include <Arduino.h>

struct SensorDataValues {
  const float min;
  const float max;
  float value;
};

struct SensorData {
  struct SensorDataValues temperature{-40.0, 80.0, 0.0};
  struct SensorDataValues humidity{0.0, 100.0, 0.0};
  struct SensorDataValues lux{0.0, 100000.0, 0.0 };
  struct SensorDataValues kmh{0.0, 120.0, 0.0,};
  String airQuality;
  String rosetaPosition;
};

const byte fullChar[] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

const byte termometro[8] = {
  0b00100,
  0b01010,
  0b01010,
  0b01110,
  0b01110,
  0b11111,
  0b11111,
  0b01110
};

const byte gota[8] = {
  0b00100,
  0b00100,
  0b01010,
  0b01010,
  0b10001,
  0b10001,
  0b10001,
  0b01110
};

const byte sol[8] = {
  0b00100,
  0b10101,
  0b01110,
  0b11111,
  0b01110,
  0b10101,
  0b00100,
  0b00000
};

const byte calidadAire[8] = {
  0b00000,
  0b01110,
  0b11111,
  0b11111,
  0b11111,
  0b10101,
  0b00100,
  0b00000
};

const byte viento[8] = {
  0b00100,
  0b01010,
  0b10001,
  0b00100,
  0b00100,
  0b10001,
  0b01010,
  0b00100
};

const byte veleta[8] = {
  0b00100,
  0b01110,
  0b10101,
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b00000
};
