# **Actividad 2: Sistema de control y actuación en función del clima**

<p align="center">
  <img src="images/Boya_1.jpg" alt="Boya climática" width="600">
</p>

## *Objetivo*

Partiendo de la actividad 1 que consistía en realizar una aplicación de medición y presentación del clima, en la que manejamos sensores de temperatura, humedad, iluminación, calidad de aire, velocidad/dirección del viento … de una estación meteorológica (boya climática), procesamos la medición con instrumentación programable (Arduino UNO) y presentamos los resultados en una HMI local (Display LCD I2C). En la presente actividad, que es una continuación de la anterior, añadiremos actuadores y algoritmos de control que, a partir de la medición de los sensores, mantendremos el sistema cercano a los valores deseados y presentaremos los resultados en la HMI local.

## *Descripción*

A partir los valores medidos de temperatura (ºC) e iluminación (lux), se determinan las acciones de control y actuación que garantizan que el sistema de baterías de la boya climática se mantenga alrededor de los parámetros deseados, esto es 25 grados centígrados, y se ilumine un LED para balizar la boya en función de la iluminación en lux del exterior, para ello se han establecido 8 niveles de iluminación (oscuridad < 10 lux, iluminación muy tenue: entre 10 y 100 lux, iluminación tenue: entre 100 y 400 lux, iluminación moderada: entre 400 y 1000 lux, iluminación intensa:  entre 1000 y 1400 lux e iluminación muy intensa > 1400 lux) que se corresponden de forma inversamente proporcional al brillo del diodo LED, es decir a más oscuridad, mayor brillo.

Para el control de temperatura hemos cosiderado dos servomotores que actuarán sobre válvulas de control continuo que permiten el paso de un agente refrigerante o de un fluido caliente por un dispositivo de intercambio térmico con el aire. Para la simulación del proceso térmico hemos considerado una relación proporcional de 0.5 entre la acción del elemento final y el proceso y un retardo de 500ms (es decir que incrementa o decrementa medio grado cada 500ms).

Se ha implementado un algoritmo de control continuo de tres posiciones con histéresis y zona muerta, es decir, con acciones continuas proporcionales al error de la medición con respecto al valor deseado. Por ejemplo, si el error es solo de 1 grado se abre solo el 1% de una válvula de control continua y así sucesivamente se incrementa un 1% de apertura de la válvula continua por cada grado de diferencia. En este caso sería un control tipo P con ganancia K=1 para cada acción de control en rampa, enfriar y calentar. Se selecciona este algoritmo porque permite acciones de control en ambos sentidos (calentar y enfriar) y la conmutación se hace en un rango de  +/-3 grados alrededor del valor deseado, que es nuestro caso son 25 ºC. En nuestro caso como son válvulas de control continuo se requiere su cierre o apertura por medio de actuadores eléctricos continuos (servo motores). 

Utilizamos una conexión directa de los servos con las salidas PWM del Arduino porque no se van a actuar sobre ninguna carga, y por tanto el consumo es mínimo. En el caso de que actuaran sobre una carga, se tendría que emplear una fuente de alimentación externa.





## *Autores*

- José Luis Darias Perdomo  
- Carlos Barrera Utrera
