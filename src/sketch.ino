#include <Arduino.h>

//Actividad 3: Sistema de control y actuación en función del clima
/* Se completan las actividades 1 y 2 añadiendo técnicas de comunicaciones */
// Equipos e Instrumentación electrónica
//Autores: José Luis Darias Perdomo & Carlos Barrera Utrera

#include <SimpleDHT.h> // Libreria del Sensor Humedad y Temperatura | DHT22
#include <LCD.cpp> // Libreia propia para la pantalla LCD I2C
#include <Servo.h> // Libreria para el control de servos
#include <IRremote.hpp>

#define DHTPIN 2 // Pin digital 2 como entrada DHT22
#define LCD_COLUMNS 16 // Cantidad de columnas del LCD I2C
#define LCD_LINES   2 // Cantidad de filas del LCD I2C
#define ENCODER_CLK 7 // Pin digital 7 como entrada CLK del encoder
#define ENCODER_DT  3 // Pin digital 3 como entrada DT del encoder

#define SERVO_CALOR  9 // Definimos pin para el control del servo caliente
#define SERVO_FRIO  10  // Definimos pin para el control del servo frio
#define LED_PIN     6
#define IRPIN       4
#define SUPPRESS_ERROR_MESSAGE_FOR_BEGIN

struct SensorData sensorData_t;

int CALIDAD_AIRE_PIN = A0; // Pin A0 como entrada potenciometro CALIDAD_AIRE
int VELOCIDAD_VIENTO_PIN = A1; // Pin A1 como entrada potenciómetro VEL_VIENTO 
int LDR_PIN = A2; // Pin A2 como entrada LDR
int MAX_WIND_SPEED  = 120; // Velocidad máxima del viento en km/h

double defaultPotentiometerValue = 1023.0; //Valor máximo por defecto del potenciómetro
bool lcdOverride = false;

SimpleDHT22 dht22(DHTPIN); // Se crea objeto dht22 y le asigna como parámetro DHTPIN = pin digital donde está conectado el sensor. 
LCD lcd(LCD_COLUMNS, LCD_LINES, &lcdOverride, &sensorData_t); // Se crea el objeto lcd y se le asignan como parámetros IC2_ADDR, LCD_COLUMNS, LCD_LINES
//IrReceiver;
// Definimos el valor medio de la veleta (SUR)
int anterior = 10;
volatile int pos = 10;
int maxpos = 19;

// Definimos la dirección de la veleta
String roseta[20] = {"Norte","Noreste", "Noreste", "Noreste","Noreste","Este", "Sureste","Sureste", "Sureste","Sureste","Sur", "Suroeste","Suroeste","Suroeste","Suroeste", "Oeste",  "Noroeste","Noroeste","Noroeste", "Noroeste" };

Servo servoFrio, servoCalor; // Creamos variables para manejar servos
bool counterState = false;
int tempThreshold = 3;
int maxServoLimit = 180; // Limite máximo en pasos de los servos
uint8_t normalStateTempControl[2] = {25, 80}; // Temperatura normal de control del servo frio (25ºC) y humedad(80%) 

void translateIR();
String setCalidadAire(int value);
void setLuxValue(float value);
//Callback de interrupciones del encoder
void OnEncoderChange() {
  static unsigned long ultimaInter= 0;
  unsigned long tiempoInterrup = millis();
  
  if(tiempoInterrup - ultimaInter > 2) {
    pos = digitalRead(ENCODER_CLK) == HIGH ? pos+1 : pos-1;   
    
    if(pos > maxpos) {
      pos = 0;
    } else if (pos < 0) {
      pos = maxpos;
    }

    pos = min(maxpos, max(0, pos));    
    ultimaInter = tiempoInterrup;
  }
}

void setup() {  
  Serial.begin(115200); //Se inicia la comunicación serie a una velocidad de 115200 baudios (bits por segundo)

  lcd.init(); //Se inicia la pantalla LC I2C   
  
  //Configuramos los pines como entrada para las lecturas del sensor LDR y encoder. 
  pinMode(LDR_PIN, INPUT);
  pinMode(ENCODER_CLK, INPUT);
  pinMode(ENCODER_DT, INPUT);

  //Configuramos los pines PWM para controlar la intensidad del led. 
  pinMode(LED_PIN, OUTPUT);

  IrReceiver.setReceivePin(IRPIN);
  IrReceiver.enableIRIn();

  //Configuramos los pines de los servos
  servoFrio.attach(SERVO_FRIO); // Pin 10 como salida servo (válvula frio)
  servoCalor.attach(SERVO_CALOR); // Pin 9 como salida servo (válvula calor)

  //Iniciamos callback de interrupciones del encoder 
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT), OnEncoderChange, LOW);
}

void controlServoState() { // Se controla el estado del servo frio y calor
  int tempActual = (int) sensorData_t.temperature.value;

  bool tempIndicator = tempActual > normalStateTempControl[0] ? true : false;

  if(tempIndicator) {
    long servoFrioAngulo = map(tempActual, normalStateTempControl[0]+tempThreshold, 50, 0, 180);
    servoFrio.write(servoFrioAngulo);
    servoCalor.write(0); // Cerrar servo calor mientras actua el frío
  } else {
    long servoCalorAngulo = map(tempActual, normalStateTempControl[0]-tempThreshold, 0, 0, 180); 
    servoCalor.write(servoCalorAngulo);
    servoFrio.write(0); // Cerrar el servo frio mientras actua el calor
  }
}

int ledValueState = 0;
void aumentarIntensidadLed(int max) { // Se aumenta el valor del led
  for(int bright = ledValueState; bright <= max; bright++) {
    analogWrite(LED_PIN, bright); // Se enciende el led
    
    delay(10);
  }

  ledValueState = max;
}

void disminuirIntensidadLed(int min) { // Se disminuye el valor del led
  for(int bright = ledValueState; bright >= min; bright--) {
    analogWrite(LED_PIN, bright); // Se apaga el led 
    delay(10);
  }

  ledValueState = min; 
}

void controlLedState() {
  float state = sensorData_t.lux.value;

  if(state > 1400){
    disminuirIntensidadLed(0);
  }
 
  if(state > 1000 && state < 1400)
  {
    disminuirIntensidadLed(55);
  }
 
  if(state > 400 && state < 1000)
  {
    disminuirIntensidadLed(83);
  }

  if(state > 100 && state < 400)
  {
    aumentarIntensidadLed(127);
  }

  if(state > 10 && state < 100)
  {
    aumentarIntensidadLed(200);
  }

  if(state < 10)
  {
    aumentarIntensidadLed(255);
  }
}


void loop() {
  if (IrReceiver.decode()) {
    translateIR();
    IrReceiver.resume();  // Receive the next value
  }

  if(lcdOverride) {
    delay(100);
    return;
  }

  //"================ HUMEDAD & TEMPERATURA =================>>";
  int err = SimpleDHTErrSuccess; // Se inicializa la variable la variable err ¨sin error¨
  if((err = dht22.read2(&sensorData_t.temperature.value, &sensorData_t.humidity.value, NULL)) != SimpleDHTErrSuccess) {    
    lcd.SetError(err);
    return;
  }

  controlServoState(); // Se controla el estado del servo frio y calor

  //"================ LUMINOSIDAD  & CALIDAD DE AIRE =================>>"
  const float GAMMA = 0.7; // coeficiente que determina pendiente curva logarítmica. 
  const float RL10 = 50; // resistencia del LDR a 10 lux (por defecto, 50 kΩ) 
  int ldrValue = analogRead(LDR_PIN); // Lee LDR_PIN y devuelve un valor de 0 a 1023 (resolución de 10 bits) correspondiente a una tensión entre 0 y 5 V 
  float voltage = ldrValue / 1024.0 * 5.0; // Convierte val analógico leído a voltaje 0-5V 
  float resistance = 2000 * voltage / (1 - voltage / 5.0); // Calcula la resistencia del LDR utilizando la fórmula del divisor de tensión 
  setLuxValue(pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA))); // Calcula la iluminación en lux utilizando la relación logarítmica entre resistencia e iluminación 


  int calidadAire = analogRead(CALIDAD_AIRE_PIN); // Lee CALIDAD_AIRE_PIN y devuelve un valor de 0 a 1023
  sensorData_t.airQuality = setCalidadAire(calidadAire);

  //"================ VELOCIDAD  & DIRECCION DE VIENTO =================>>";
  //Proceso de velocidad del viento
  int readViento = analogRead(VELOCIDAD_VIENTO_PIN); // Lee VELOCIDAD_VIENTO_PIN y devuelve un valor de 0 a 1023
  int offset = (float(readViento) / defaultPotentiometerValue)  * 100; // Convertir a % 
  int kmh = (offset * MAX_WIND_SPEED) / 100;  // Convertir a km/h

  sensorData_t.kmh.value = kmh;
  sensorData_t.rosetaPosition = roseta[pos];

  if(pos != anterior) {       
    anterior = pos; // Se actualiza la variable anterior si pos cambia
  }

  lcd.initDisplayValues();

  delay(500);
}

void setLuxValue(float value) {
  sensorData_t.lux.value = value;
  controlLedState();
}

String setCalidadAire(int value) {
  //Proceso de calidad del aire
  String ca; // declaración variable ca tipo string
  if (value >= 0 && value <= 341) {
    ca = "Buena";
  } else if (value > 341 && value <= 682) {
    ca = "Normal";
  } else {
    ca = "Mala";
  }
  
  return ca;
}

void translateIR()
{  
  switch (IrReceiver.decodedIRData.command) {
    case 226: // MENU  
      lcdOverride = !lcdOverride;
      lcd.showMenu();
      break;    
    case 2: // PLUS KEY
      lcd.incrementSensorValue();  
      break;
    case 152: // MINUS KEY
      lcd.decrementSensorValue(); 
      break;
    case 194: // BACK
      if(lcdOverride) {
        lcdOverride = false;        
      }
      break;    
    case 48: //NUM KEY 1
      lcd.showDataMenu(&sensorData_t.temperature, "TEMPERATURA", 1, &controlServoState);       
      break;
    case 24: //NUM KEY 2
      lcd.showDataMenu(&sensorData_t.lux, "ILUMINACION", 100, &controlLedState);
      break;    
  }
}