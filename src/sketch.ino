#include <Arduino.h>

//Actividad 2: Sistema de control y actuación en función del clima
/* Este código realiza la monitorización del clima con un sistema basado en Arduino Uno
que mide temperatura, humedad, luminosidad, calidad del aire, velocidad y dirección 
del viento usando varios sensores y una pantalla LCD */
// Equipos e Instrumentación electrónica
//Autor: José Luis Darias Perdomo

#include <SimpleDHT.h> // Libreria del Sensor Humedad y Temperatura | DHT22
#include <LiquidCrystal_I2C.h> // Libreria LCD I2C
#include <Servo.h>

#define DHTPIN 2 // Pin digital 2 como entrada DHT22
#define I2C_ADDR    0x27 // Dirección I2C del LCD I2C
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
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);  //Se crea el objeto lcd y se le asignan como parámetros IC2_ADDR, LCD_COLUMNS, LCD_LINES

byte termometro[8] = {
  0b00100,
  0b01010,
  0b01010,
  0b01110,
  0b01110,
  0b11111,
  0b11111,
  0b01110
};

byte gota[8] = {
  0b00100,
  0b00100,
  0b01010,
  0b01010,
  0b10001,
  0b10001,
  0b10001,
  0b01110
};

byte sol[8] = {
  0b00100,
  0b10101,
  0b01110,
  0b11111,
  0b01110,
  0b10101,
  0b00100,
  0b00000
};

byte calidadAire[8] = {
  0b00000,
  0b01110,
  0b11111,
  0b11111,
  0b11111,
  0b10101,
  0b00100,
  0b00000
};

byte viento[8] = {
  0b00100,
  0b01010,
  0b10001,
  0b00100,
  0b00100,
  0b10001,
  0b01010,
  0b00100
};

byte veleta[8] = {
  0b00100,
  0b01110,
  0b10101,
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b00000
};


// Definimos el valor medio de la veleta (SUR)
int anterior = 10;
volatile int pos = 10;
int maxpos = 19;

// Definimos la dirección de la veleta
String roseta[20] = {"Norte","Noreste", "Noreste", "Noreste","Noreste","Este", "Sureste","Sureste", "Sureste","Sureste","Sur", "Suroeste","Suroeste","Suroeste","Suroeste", "Oeste",  "Noroeste","Noroeste","Noroeste", "Noroeste" };


Servo servoFrio, servoCalor; // Creamos variables para manejar servos

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
  lcd.backlight(); //encendemos la luz de fondo (backlight) de la pantalla LCD 

 //Iniciamos caracteres personalizados 

  //Iniciamos caracteres personalizados
  lcd.createChar(0, termometro); 
  lcd.createChar(1, gota); 
  lcd.createChar(2, sol); 
  lcd.createChar(3, calidadAire); 
  lcd.createChar(4, viento); 
  lcd.createChar(5, veleta); 
  
  //Configuramos los pines como entrada para las lecturas del sensor LDR y encoder. 
  pinMode(LDR_PIN, INPUT);
  pinMode(ENCODER_CLK, INPUT);
  pinMode(ENCODER_DT, INPUT);

  //Configuramos los pines de los servos
  servoFrio.attach(SERVO_FRIO);
  servoCalor.attach(SERVO_CALOR);


  //Iniciamos callback de interrupciones del encoder 
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT), OnEncoderChange, LOW);
}

void loop() {
  
  //"================ HUMEDAD & TEMPERATURA =================>>");
  lcd.clear(); // borra el contenido de la pantalla LCD
  lcd.setCursor(0, 0); // Posicionamos el cursor en la pantalla LCD (columna:0 | fila:0)

  byte temperature = 0; // declaramos variables locales temperatura y humidity
  byte humidity = 0;
  int err = SimpleDHTErrSuccess; // Se inicializa la variable la variable err ¨sin error¨
  if ((err = dht22.read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    lcd.clear();
    lcd.println("Error DHT22 ="); // Muestra el mensaje de error
    lcd.println(SimpleDHTErrCode(err)); // Muestra el código de error
    lcd.print(","); 
    lcd.println(SimpleDHTErrDuration(err)); // Muestra la duración del intento de lectura

    delay(2000); // Retardo de 2 segundos
    return; // Sale de la función
  }  
  
  lcd.write(byte(0)); // Muestra carácter (termómetro) que está en posición mem (0)
  lcd.setCursor(2, 0); // Posicionamos el cursor en la pantalla LCD (columna:2 | fila:0) 
  lcd.print("TEMP: "); // Mostramos el texto TEMP:
  lcd.print((int)temperature); // Convierte la variable tipo byte a int y la muestra 
  lcd.print((char)223); // Muestra el símbolo º
  lcd.println("C      "); // Muestra el carácter C

  lcd.setCursor(0, 1);  // Posicionamos cursor en la segunda fila
  lcd.write(byte(1)); // Muestra carácter (gota) que está en posición mem (1)
  lcd.setCursor(2, 1); // Posicionamos el cursor en la pantalla LCD (columna:2 | fila:1)
  lcd.print("HUMEDAD: "); // Mostramos el texto HUMEDAD:
  lcd.print((int)humidity); // Convierte la variable tipo byte a int y la muestra
  lcd.print((char)37); // Muestra el símbolo %

  delay(2000); // Retardo de 2 segundos

  
  //"================ LUMINOSIDAD  & CALIDAD DE AIRE =================>>");
   
  const float GAMMA = 0.7; // coeficiente que determina pendiente curva logarítmica. 
  const float RL10 = 50; // resistencia del LDR a 10 lux (por defecto, 50 kΩ) 
  int ldrValue = analogRead(LDR_PIN); // Lee LDR_PIN y devuelve un valor de 0 a 1023 (resolución de 10 bits) correspondiente a una tensión entre 0 y 5 V 
  float voltage = ldrValue / 1024.0 * 5.0; // Convierte val analógico leído a voltaje 0-5V 
  float resistance = 2000 * voltage / (1 - voltage / 5.0); // Calcula la resistencia del LDR utilizando la fórmula del divisor de tensión 
  float lux = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA)); // Calcula la iluminación en lux utilizando la relación logarítmica entre resistencia e iluminación 

  lcd.clear();
  lcd.setCursor(0, 0); 
  lcd.write(byte(2)); // Muestra carácter (sol) que está en posición mem (2)
  lcd.setCursor(2, 0);
  lcd.print("LUX: ");
  lcd.print(lux);
  
  
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

  lcd.setCursor(0, 1);   
  lcd.write(byte(3)); // Muestra carácter (calidadAire) que está en posición mem (3)
  lcd.setCursor(2, 1);
  lcd.print("CA: ");
  lcd.print(ca);
  delay(2000); 
  

  //"================ VELOCIDAD  & DIRECCION DE VIENTO =================>>");
  //Proceso de velocidad del viento
  int readViento = analogRead(VELOCIDAD_VIENTO_PIN); // Lee VELOCIDAD_VIENTO_PIN y devuelve un valor de 0 a 1023
  int offset = (float(readViento) / defaultPotentiometerValue)  * 100; // Convertir a % 
  int kmh = (offset * MAX_WIND_SPEED) / 100;  // Convertir a km/h

  lcd.clear();
  lcd.setCursor(0, 0); 
  lcd.write(byte(4));// Muestra carácter (viento) que está en posición mem (4)
  lcd.setCursor(2, 0);
  lcd.print("VEL: ");
  lcd.print(kmh);  
  lcd.print("km/h");
  
  
  lcd.setCursor(0, 1); 
  lcd.write(byte(5)); // Muestra carácter (veleta) que está en posición mem (5)
  lcd.setCursor(2, 1);
  lcd.print("DIR: ");
  lcd.print(roseta[pos]); // Muestra la dirección del viento N, NE, SE, S, SO, O, NO, determinada por la posición del encoder

  if(pos != anterior) {       
    anterior = pos; // Se actualiza la variable anterior si pos cambia
  }

  delay(2000); 
}
