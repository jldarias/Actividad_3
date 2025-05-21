#include <Arduino.h>

//Actividad 2: Sistema de control y actuación en función del clima
/* Este código realiza la monitorización del clima con un sistema basado en Arduino Uno
que mide temperatura, humedad, luminosidad, calidad del aire, velocidad y dirección 
del viento usando varios sensores y una pantalla LCD */
// Equipos e Instrumentación electrónica
//Autor: José Luis Darias Perdomo

#define _TASK_SLEEP_ON_IDLE_RUN // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass
#define _TASK_STATUS_REQUEST

#include <SimpleDHT.h> // Libreria del Sensor Humedad y Temperatura | DHT22
#include <LCD.cpp> // Libreia propia para la pantalla LCD I2C
#include <TaskScheduler.h> // Libreria para la programación de tareas
#include <Servo.h> // Libreria para el control de servos


#define DHTPIN 2 // Pin digital 2 como entrada DHT22
#define LCD_COLUMNS 16 // Cantidad de columnas del LCD I2C
#define LCD_LINES   2 // Cantidad de filas del LCD I2C
#define ENCODER_CLK 7 // Pin digital 7 como entrada CLK del encoder
#define ENCODER_DT  3 // Pin digital 3 como entrada DT del encoder

#define SERVO_CALOR  9 // Definimos pin para el control del servo caliente
#define SERVO_FRIO  10  // Definimos pin para el control del servo frio

int CALIDAD_AIRE_PIN = A0; // Pin A0 como entrada potenciometro CALIDAD_AIRE
int VELOCIDAD_VIENTO_PIN = A1; // Pin A1 como entrada potenciómetro VEL_VIENTO 
int LDR_PIN = A2; // Pin A2 como entrada LDR
int MAX_WIND_SPEED  = 120; // Velocidad máxima del viento en km/h

double defaultPotentiometerValue = 1023.0; //Valor máximo por defecto del potenciómetro

SimpleDHT22 dht22(DHTPIN); // Se crea objeto dht22 y le asigna como parámetro DHTPIN = pin digital donde está conectado el sensor. 
//LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);  //Se crea el objeto lcd y se le asignan como parámetros IC2_ADDR, LCD_COLUMNS, LCD_LINES
LCD lcd(LCD_COLUMNS, LCD_LINES); // Se crea el objeto lcd y se le asignan como parámetros IC2_ADDR, LCD_COLUMNS, LCD_LINES

void displaySensorValues();
Scheduler ts; // Se crea el objeto ts 
Task DisplayInLCD(lcd.displayTimeSpeed, TASK_FOREVER, &displaySensorValues, &ts, true); // Se crea la tarea DisplayInLCD que se ejecuta indefinidamente

// Definimos el valor medio de la veleta (SUR)
int anterior = 10;
volatile int pos = 10;
int maxpos = 19;

// Definimos la dirección de la veleta
String roseta[20] = {"Norte","Noreste", "Noreste", "Noreste","Noreste","Este", "Sureste","Sureste", "Sureste","Sureste","Sur", "Suroeste","Suroeste","Suroeste","Suroeste", "Oeste",  "Noroeste","Noroeste","Noroeste", "Noroeste" };


Servo servoFrio, servoCalor; // Creamos variables para manejar servos
bool counterState = false;
int tempThreshold = 3;
int maxServoLimit = 190; // Limite máximo en pasos de los servos
uint8_t normalStateTempControl[2] = {25, 80}; // Temperatura normal de control del servo frio (25ºC) y humedad(80%) 

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

  //Configuramos los pines de los servos
  servoFrio.attach(SERVO_FRIO); // Pin 9 como salida servo (válvula frio)
  servoCalor.attach(SERVO_CALOR); // Pin 10 como salida servo (válvula calor)

  //Iniciamos callback de interrupciones del encoder 
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT), OnEncoderChange, LOW);
}

void controlServoState(float *temp, float *hum) { // Se controla el estado del servo frio y calor
  //uint8_t tempPosMedia = 25;  
  bool tempState = true; // Estado del frio 1 = frio, 0 = calor
  int temperaturaActual = (int)*temp; // Se convierte la variable tipo float a int
  
  tempState = temperaturaActual > normalStateTempControl[0] ? true : false; // Se determina el estado del servo frio (1) o calor (0)  

  if(tempState && (temperaturaActual >= normalStateTempControl[0] && temperaturaActual <= normalStateTempControl[0] + tempThreshold) ) {
    servoFrio.write(0); // Se cierra la válvula de frio
  }else if(tempState && (temperaturaActual >= normalStateTempControl[0] - tempThreshold && temperaturaActual <= normalStateTempControl[0])) {
    servoCalor.write(0); // Se cierra la válvula de calor 
  }

  if(tempState && temperaturaActual > normalStateTempControl[0] + tempThreshold && !counterState) { // Si el estado es frio y la temperatura es mayor a 25ºC + 3ºC
    servoFrio.write(20); // Se abre la válvula de frio un 1%
  }else if(!tempState && temperaturaActual < normalStateTempControl[0] - tempThreshold && !counterState) { // Si el estado es calor y la temperatura es menor a 25ºC - 3ºC
    servoCalor.write(20); // Se abre la válvula de calor un 1%    
  }



  /*int localPercent = 0;
  int servoValue = 0;
  if(counterState) {    
    localPercent = temperaturaActual - tempThreshold - normalStateTempControl[0];
    servoValue = servoFrio.read();
  } else {
    localPercent = abs(temperaturaActual + tempThreshold - normalStateTempControl[0]);
    servoValue = servoCalor.read();
  }*/
  


  //counterState = true;
  //temperaturaActual = abs(temperaturaActual);    
  Serial.print("Temperatura: ");
  Serial.println(temperaturaActual);
}



void loop() {

  //"================ HUMEDAD & TEMPERATURA =================>>");
  float temperature = 0; // declaramos variables locales temperatura y humidity e inicializamos a 0
  float humidity = 0;
  int err = SimpleDHTErrSuccess; // Se inicializa la variable la variable err ¨sin error¨
  if((err = dht22.read2(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {    
    lcd.SetError(err);
    return;
  }

  controlServoState(&temperature, &humidity); // Se controla el estado del servo frio y calor
  lcd.DHTValues(temperature, humidity); // Se envían los valores de temperatura y humedad a la función DHTValues de la clase LCD

  //"================ LUMINOSIDAD  & CALIDAD DE AIRE =================>>");  
  const float GAMMA = 0.7; // coeficiente que determina pendiente curva logarítmica. 
  const float RL10 = 50; // resistencia del LDR a 10 lux (por defecto, 50 kΩ) 
  int ldrValue = analogRead(LDR_PIN); // Lee LDR_PIN y devuelve un valor de 0 a 1023 (resolución de 10 bits) correspondiente a una tensión entre 0 y 5 V 
  float voltage = ldrValue / 1024.0 * 5.0; // Convierte val analógico leído a voltaje 0-5V 
  float resistance = 2000 * voltage / (1 - voltage / 5.0); // Calcula la resistencia del LDR utilizando la fórmula del divisor de tensión 
  float lux = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA)); // Calcula la iluminación en lux utilizando la relación logarítmica entre resistencia e iluminación 

    
  int calidadAire = analogRead(CALIDAD_AIRE_PIN); // Lee CALIDAD_AIRE_PIN y devuelve un valor de 0 a 1023

  //Proceso de calidad del aire
  String ca; // declaración variable ca tipo string
  if (calidadAire >= 0 && calidadAire <= 341) {
    ca = "Buena";
  } else if (calidadAire > 341 && calidadAire <= 682) {
    ca = "Normal";
  } else {
    ca = "Mala";
  }
  
  lcd.LDRValues(lux); // Se envían los valores de luminosidad a la función LDRValues de la clase LCD
  lcd.AirQualityValues(ca); // Se envían los valores de calidad de aire a la función AirQualityValues de la clase LCD

  //"================ VELOCIDAD  & DIRECCION DE VIENTO =================>>");
  //Proceso de velocidad del viento
  int readViento = analogRead(VELOCIDAD_VIENTO_PIN); // Lee VELOCIDAD_VIENTO_PIN y devuelve un valor de 0 a 1023
  int offset = (float(readViento) / defaultPotentiometerValue)  * 100; // Convertir a % 
  int kmh = (offset * MAX_WIND_SPEED) / 100;  // Convertir a km/h

  lcd.WindSpeedValues(kmh); // Se envían los valores de velocidad del viento a la función WindSpeedValues de la clase LCD
  lcd.RosetaValues(roseta[pos]);  // Se envían los valores de dirección del viento a la función RosetaValues de la clase LCD

  if(pos != anterior) {       
    anterior = pos; // Se actualiza la variable anterior si pos cambia
  }

  ts.execute(); // Se ejecuta el scheduler
  delay(500);
}

void displaySensorValues(){ // Se ejecuta la tarea DisplayInLCD
  lcd.initDisplayValues(); // Se muestran los valores en la pantalla LCD
}