1.1-	El robot es de tipo móvil debido a la estructura que presenta y debido a como se desplaza que es con el uso de ruedas. También es un robot autónomo debido a que sus acciones no son afectadas por un humano y sus acciones son basadas por el feedback que recibe.

1.2-	Incluirá 2 sensores:
Sensor ultrasónico: este sensor es un detector de proximidad, su funcionamiento se basa en que envía una onda sonora y se analiza la onda reflejada
Sensor RGB: sensor utilizado para poder distinguir los distintos cuadrados de colores presentados en el laberinto, esto lo realiza a través de la extracción de información de luz utilizando tres sensores acoplados que realizan la separación de luz en rojo, verde y azul

1.3-	El error puede representarse en: Una desviación entre la posición o el estado deseado y el real del robot. Puede expresarse en términos de distancia, ángulo y velocidad.
Para abordar la incertidumbre, se pueden emplear técnicas de estimación y control robustas como filtros de Kalmano filtros de partículas.

1.4-	 
En la imagen se puede apreciar el laberinto, el robot iniciara de la entrada y utilizara el sensor ultrasónico para orientarse por todo el laberinto, ira detectando los cuadrados presentes en el laberinto con el sensor RGB, cuando encuentre un cuadrado rojo o verde, lo detectara como una víctima, enviara una señal y seguirá el camino, si encuentra un cuadrado gris lo identificara como un obstáculo, el cual deberá evitar moviéndose en otra dirección. Una vez explorado todo el laberinto, el robot se devolverá hasta la entrada, saliendo de este

1.5-	El robot dispondrá de 5 grados de libertad, un grado de libertad se ubica en el “cuello”, que permite mover el sensor ultrasónico para la orientación,1 grado de libertad por cada rueda  

1.6-	Para poder controlar la velocidad de las ruedas, debemos aplicar cinemática diferencial, con cinemática podemos saber donde se encuentra la posición deseada.
Necesitamos varios datos respecto al robot necesitamos:
La localización del robot, esto se puede obtener sabiendo la distancia entre las dos ruedas que mueven al robot, el centro entre estas 2,representara la coordenada x e y del robot, se necesita también saber el ángulo respecto al plano proyectado
La velocidad de angular de las ruedas por separado
También necesitamos conocer el radio de las ruedas y la distancia entre estas dos
si la posición deseada es conocida y se requiere saber la velocidad que debe tener el robot para llegar a esa posición deseada se puede aplicar la siguiente formula:
Si se sabe la posición deseada, se debe ajustar la velocidad lineal instantánea del robot esta se representa por la siguiente formula 
 
Siendo V la velocidad lineal instantánea y Vr con VL las velocidades de las ruedas derecha e izquierda respectivamente.
 Debido a que sabemos la posición que queremos llegar podemos utilizar las siguientes fórmula para obtener la velocidad lineal instantánea:
 
Debido a que conocemos la posición deseada (X’ y Y’), nuestra posición actual (X y Y), tenemos el tiempo (t), el ángulo lo podemos obtener sabiendo la distancia recorrida y dividiéndola por el número de vueltas que el robot debe realizar. Una vez obtenidos los datos podemos despejar la velocidad lineal instantánea y así obtener esta. Si se desea saber la velocidad linear de cada rueda, se puede realizar con las siguientes formulas 
 
Siendo Rr  y Rl el radio de las ruedas derecha e izquierda respectivamente y la velocidad angular de las ruedas 

1.7-	Se utilizará Raspberry debido a que se necesita para poder procesar las emisiones de luz del sensor RGB

1.8-	El robot tendrá 2 retroalimentaciones:

Retroalimentación a distancia (sensor de ultrasonido): Se utilizara para ajustar la velocidad de las ruedas y evitar colisiones, si la distancia medida es menor que un umbral definido, el robot se detendrá o cambiara de dirección para evitar un obstáculo
Retroalimentación de color (sensor RGB):Se utilizara para detectar los objetos basados en su color, específicamente para diferenciar entre victimas (cuadrados verdes y rojos) y obstáculos (cuadrados grises).
