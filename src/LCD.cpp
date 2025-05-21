#include <Arduino.h>
#include <SimpleDHT.h>
#include <LiquidCrystal_I2C.h>
#include <LCD_characters.h>

#define I2C_ADDR    0x27

#define TERMOMETRO 

class LCD {        
    public:
        int displayTimeSpeed = 2000; // Tiempo de refresco de la pantalla LCD en milisegundos
        LCD(uint8_t columnas, uint8_t filas) : lcd(I2C_ADDR, columnas, filas){}
        ~LCD(){};

        void init() 
        {
            lcd.init(); //Se inicia la pantalla LC I2C 
            lcd.backlight();

            addSpecialCharacters();
        }

        void addSpecialCharacters() {
            lcd.createChar(0, (uint8_t*) termometro);
            lcd.createChar(1, (uint8_t*) gota); 
            lcd.createChar(2, (uint8_t*) sol); 
            lcd.createChar(3, (uint8_t*) calidadAire); 
            lcd.createChar(4, (uint8_t*) viento); 
            lcd.createChar(5, (uint8_t*) veleta);    
        }

        LiquidCrystal_I2C* getLCD()
        {
            return &lcd;
        }
    
        void DHTValues(float temperatura, float humedad) {
            Temperatura = temperatura;
            Humedad = humedad;
        }

        void LDRValues(float lux) {
            Lux = lux;
        }

        void WindSpeedValues(int kmh) {
            Kmh = kmh;
        }

        void AirQualityValues(String calidadAireStr) {            
            CalidadAireStr = calidadAireStr;
        }

        void RosetaValues(String rosetaPos) {
            RosetaPos = rosetaPos;
        }

        void SetError(int err) {
            err = err;
        }

        void setTimeSpeed(int timeSpeed) {
            displayTimeSpeed = timeSpeed;
        }

        void initDisplayValues() {
            if(err != SimpleDHTErrSuccess) {                
                lcd.clear();
                lcd.println("Error DHT22 ="); // Muestra el mensaje de error
                lcd.println(SimpleDHTErrCode(err)); // Muestra el código de error
                lcd.print(","); 
                lcd.println(SimpleDHTErrDuration(err)); // Muestra la duración del intento de lectura
                delay(2000); // Retardo de 2 segundos
                return;
            }

            displayTempAndHumValues();            

            displayLDRValues();            

            displayWindSpeedValues();            
        }
        
    private:
        LiquidCrystal_I2C lcd;
        float Temperatura, Humedad, Lux;
        int err, Kmh;        
        String CalidadAireStr, RosetaPos;

        void displayTempAndHumValues() {
            lcd.clear();
            lcd.setCursor(0, 0); 
            lcd.write(byte(0)); // Muestra carácter (termómetro) que está en posición mem (0)
            lcd.setCursor(2, 0); // Posicionamos el cursor en la pantalla LCD (columna:2 | fila:0) 
            lcd.print("TEMP: "); // Mostramos el texto TEMP:
            lcd.print((int)Temperatura); // Convierte la variable tipo byte a int y la muestra 
            lcd.print((char)223); // Muestra el símbolo º
            lcd.println("C      "); // Muestra el carácter C

            lcd.setCursor(0, 1);  // Posicionamos cursor en la segunda fila
            lcd.write(byte(1)); // Muestra carácter (gota) que está en posición mem (1)
            lcd.setCursor(2, 1); // Posicionamos el cursor en la pantalla LCD (columna:2 | fila:1)
            lcd.print("HUMEDAD: "); // Mostramos el texto HUMEDAD:
            lcd.print((int)Humedad); // Convierte la variable tipo byte a int y la muestra
            lcd.print((char)37); // Muestra el símbolo %

            delay(displayTimeSpeed);
        }

        void displayLDRValues() {
            lcd.clear();
            lcd.setCursor(0, 0); 
            lcd.write(byte(2)); // Muestra carácter (sol) que está en posición mem (2)
            lcd.setCursor(2, 0);
            lcd.print("LUX: ");
            lcd.print(Lux);

            lcd.setCursor(0, 1);   
            lcd.write(byte(3)); // Muestra carácter (calidadAire) que está en posición mem (3)
            lcd.setCursor(2, 1);
            lcd.print("CA: ");
            lcd.print(CalidadAireStr);

            delay(displayTimeSpeed);
        }

        void displayWindSpeedValues() {
            lcd.clear();
            lcd.setCursor(0, 0); 
            lcd.write(byte(4));// Muestra carácter (viento) que está en posición mem (4)
            lcd.setCursor(2, 0);
            lcd.print("VEL: ");
            lcd.print(Kmh);  
            lcd.print("km/h");

            lcd.setCursor(0, 1); 
            lcd.write(byte(5)); // Muestra carácter (veleta) que está en posición mem (5)
            lcd.setCursor(2, 1);
            lcd.print("DIR: ");
            lcd.print(RosetaPos);

            delay(displayTimeSpeed);
        }
};