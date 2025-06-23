#include <Arduino.h>
#include <SimpleDHT.h>
#include <LiquidCrystal_I2C.h>
#include <LCD_characters.h>

#define I2C_ADDR    0x27

class LCD {        
    public:
        unsigned long displayTimeSpeed = 2000; // Tiempo de refresco de la pantalla LCD en milisegundos
        LCD(uint8_t columnas, uint8_t filas, bool *isOverride, struct SensorData *sensorData_t) : 
            lcd(I2C_ADDR, columnas, filas), 
            _isOverride(isOverride),
            _sensorData_t(sensorData_t){}

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
            lcd.createChar(6, (uint8_t*) fullChar); 
        }

        void showDataMenu(SensorDataValues *sensor, String displayText, int incrementValue, void (*cb)()) {
            if(!*_isOverride) return;

            _menuTextDisplay = displayText;
            _curSensorDataOnLCD = sensor;
            _incrementValue = incrementValue;
            _sensorCB = cb;

            showMenuSensorDataValue();
        }

        void incrementSensorValue()
        {
            if(!*_isOverride) return;

            _curSensorDataOnLCD->value += _incrementValue;
            if(_curSensorDataOnLCD->value > _curSensorDataOnLCD->max) 
            {
                _curSensorDataOnLCD->value -= _incrementValue;
                return;
            }
            
            showMenuSensorDataValue();
            _sensorCB();
        }

        void decrementSensorValue()
        {
            if(!*_isOverride) return;

            _curSensorDataOnLCD->value -= _incrementValue;
            if(_curSensorDataOnLCD->value < _curSensorDataOnLCD->min) 
            {
                _curSensorDataOnLCD->value += _incrementValue;
                return;
            }
            
            showMenuSensorDataValue();
            _sensorCB();
        }

        LiquidCrystal_I2C* getLCD()
        {
            return &lcd;
        }
    
        void SetError(int err) {
            err = err;
        }

        void setTimeSpeed(int timeSpeed) {
            displayTimeSpeed = timeSpeed;
        }

        void showMenu() {
            if(!*_isOverride) return;

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.println("***** MENU *****");
            lcd.setCursor(0, 1);
            lcd.println("1.Temp <> 2.Ilum");
        }

        void initDisplayValues() {

            if(*_isOverride) return;

            unsigned long currentTime = millis();                           

            if(err != SimpleDHTErrSuccess) {                
                lcd.clear();
                lcd.println("Error DHT22 ="); // Muestra el mensaje de error
                lcd.println(SimpleDHTErrCode(err)); // Muestra el código de error
                lcd.print(","); 
                lcd.println(SimpleDHTErrDuration(err)); // Muestra la duración del intento de lectura
                return;
            }

            if(currentTime - task_time > displayTimeSpeed){
                if(count >= 3){
                    count = 0;
                }
                
                switch (count)
                {
                    case 0:
                        displayTempAndHumValues();
                        break;
                    case 1:
                        displayLDRValues();
                        break;
                    case 2:
                        displayWindSpeedValues();
                        break;
                }

                count++;
                task_time = currentTime;
            } 
        }
        
    private:
        LiquidCrystal_I2C lcd; // Instancia de la clase LiquidCrystal_I2C
        int err;
        unsigned long task_time = millis();
        uint8_t count = 0;
        bool *_isOverride;
        SensorData *_sensorData_t;
        SensorDataValues *_curSensorDataOnLCD;        
        void (*_sensorCB)();
        int _incrementValue;
        String _menuTextDisplay;

        void showMenuSensorDataValue()
        {
            lcd.clear();
            lcd.setCursor(2, 0);
            lcd.print(_menuTextDisplay);
            lcd.setCursor(0, 0);
            if(_menuTextDisplay.compareTo("TEMPERATURA") == 0)
            {                
                lcd.write(byte(0));                
                lcd.setCursor(6, 1);
                lcd.print((int)_curSensorDataOnLCD->value);
                lcd.print((char)223);
                lcd.print("C");

            } else {
                lcd.write(byte(2));
                lcd.setCursor(6, 1);
                lcd.print((int)_curSensorDataOnLCD->value);
            }
        }

        void displayTempAndHumValues() { // Función que muestra los valores de temperatura y humedad
            lcd.clear();
            lcd.setCursor(0, 0); 
            lcd.write(byte(0)); // Muestra carácter (termómetro) que está en posición mem (0)
            lcd.setCursor(2, 0); // Posicionamos el cursor en la pantalla LCD (columna:2 | fila:0) 
            lcd.print("TEMP: "); // Mostramos el texto TEMP:
            lcd.print((int)_sensorData_t->temperature.value); // Convierte la variable tipo byte a int y la muestra 
            lcd.print((char)223); // Muestra el símbolo º
            lcd.println("C      "); // Muestra el carácter C

            lcd.setCursor(0, 1);  // Posicionamos cursor en la segunda fila
            lcd.write(byte(1)); // Muestra carácter (gota) que está en posición mem (1)
            lcd.setCursor(2, 1); // Posicionamos el cursor en la pantalla LCD (columna:2 | fila:1)
            lcd.print("HUMEDAD: "); // Mostramos el texto HUMEDAD:
            lcd.print((int)_sensorData_t->humidity.value); // Convierte la variable tipo byte a int y la muestra
            lcd.print((char)37); // Muestra el símbolo %
        }

        void displayLDRValues() { // Función que muestra los valores de luminosidad y calidad de aire
            lcd.clear();
            lcd.setCursor(0, 0); 
            lcd.write(byte(2)); // Muestra carácter (sol) que está en posición mem (2)
            lcd.setCursor(2, 0);
            lcd.print("LUX: ");
            lcd.print(_sensorData_t->lux.value);

            lcd.setCursor(0, 1);   
            lcd.write(byte(3)); // Muestra carácter (calidadAire) que está en posición mem (3)
            lcd.setCursor(2, 1);
            lcd.print("CA: ");
            lcd.print(_sensorData_t->airQuality);
        }

        void displayWindSpeedValues() { // Función que muestra los valores de velocidad del viento y dirección
            lcd.clear();
            lcd.setCursor(0, 0); 
            lcd.write(byte(4));// Muestra carácter (viento) que está en posición mem (4)
            lcd.setCursor(2, 0);
            lcd.print("VEL: ");
            lcd.print((int)_sensorData_t->kmh.value);  
            lcd.print("km/h");

            lcd.setCursor(0, 1); 
            lcd.write(byte(5)); // Muestra carácter (veleta) que está en posición mem (5)
            lcd.setCursor(2, 1);
            lcd.print("DIR: ");
            lcd.print(_sensorData_t->rosetaPosition);
        }
};