# Proyecto Final: Velocímetro para una bicicleta fija.
# Equipo 2: Bello Torres Miguel, Gonzalez Infante Cesar Augusto y Llampallas Vega Diego Alberto.

Este repositorio se muestra el funcionamiento de un velocimetro paso por paso programado desde una ESP32 con el sensor ultrasónico y de complemento una foto resistencia para cuando este muy obscuro iluminar el lugar por medio de un led, tras mostrar los resultados en la pantalla lcd y en el monitor serial se envian a una página para gráficarlos llamado node red.


## Introducción
A través de la página https://wokwi.com/  se puede hacer simulaciones de programas con arduino y sensores.
### Descripción

La ```Esp32``` la utilizamos en un entorno de adquision de datos, lo cual en esta practica ocuparemos un sensor (```Ultrasónico```) para por medio de la distancia que logra detectar usarlo para calcular velocidades, distancia, velocidad promedio y tiempo; a tráves de la página de node red se pueda visualizar los datos mostrados del sensor graficandolos usando gráficas interactivas que se mueven conforme sensea el sensor ultrasónico va mostrandolos en una página web creada por node red y los datos se almacenarán en una base de datos llamada phpMyAdmin; Cabe aclarar que esta practica se usara un simulador llamado [WOKWI](https://https://wokwi.com/).


## Material Necesario

Para realizar esta practica se usaran los siguientes elementos:

- [WOKWI](https://https://wokwi.com/)
- Tarjeta ESP 32
- Sensor Ultrasónico
- Pantalla lcd 4x20
- resistencia 220 ohms
- fuente de voltaje de 5v (para la pantalla y fotorresistencia)
- tierra (para la pantalla  y fotorresistencia)
- fotoresistencia
- [NODE RED](http://localhost:1880/)
- [XAMPP] phpMyAdmin

# Pasos previos


 ## Arranque de programa node-red

 1. Para arrancar el Programa  **Node-red** se usa el codigo :

 ```
node-red
```
2. Para abrir la aplicación nos vamos algun explorador y colocamos el siguente link:    ```localhost:1880```


## Instrucciones

### Requisitos previos

Para poder usar este repositorio necesitas entrar a la plataforma [WOKWI](https://https://wokwi.com/).

### Instrucciones de preparación de entorno 
1. Una vez dentro de wokwi seleccionar la tarjeta ESP32

![](https://github.com/DiegoLlampallas/Practica-DHT22/blob/main/6.png?raw=true)

2. Abrir la terminal de programación y colocar la siguente programación:

## Programación

```
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#define BUILTIN_LED 2

#include <LiquidCrystal_I2C.h>
#define I2C_ADDR    0x27
#define LCD_COLUMNS 20
#define LCD_LINES   4


// Update these with values suitable for your network.

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "44.195.202.69";
String username_mqtt="Proyecto";
String password_mqtt="12345678";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   
    // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  
    // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), username_mqtt.c_str() , password_mqtt.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


unsigned long elapsed;
 float h, m, s;
const int echo = 26;    //Pin digital 3 para el Echo del sensor
const int trig = 14;   //Pin digital 2 para el Trigger del sensor
float promedio=0;
float maximus=0;
float total=0;
float total1=0;
long duration;
int distance;
int valorSensor;// valores dados por el fotoresistor.
int luz = 9;//una luz ajustada de 12 v a 5 v gracias al relay
float valorMapeado;// lo que se va a mapear para conocer el valor de la fotoresistencia y los parámetros asignados.
long x;// se usará para compararlo con un valor.
bool y;//  se usara para saber si la condicion es falsa o verdadera
int z;//z= numero de pedaleadas dadas.
float tiempo;// tiempo que se asignará en milisegundos.
float t;// tiempo en una pedaleada.
float t1;// tiempo en una segunda pedaleada.
float v;// velocidad.
float d;//distancia recorrida.
int pulsor = 18;// el pin del push boton.
int w;// servirá para saber si se cumple la condicion del push boton.
int cont=0;
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
void setup() {// inicio de la parte principal del programa.

 pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
 
   delay(300);// tiempo de retraso.
   pinMode(trig, OUTPUT); // declaracion del pin trig como salida.
   pinMode(echo, INPUT);// declaracion del pin echo como entrada.
   pinMode (pulsor, INPUT);// declaracion del pin pulsor (push boton) como entrada.
   pinMode(luz, OUTPUT);// declaracion del pin que va desde la luz de 12v pasando al ajuste de 5v con el relay y llegando al microcontrolador arduino, siendo una salida de luz.
       lcd.init();
  lcd.backlight();
}





void loop() {// parte del programa que se sigue y sigue. no se ejecuta una sola vez si no que vuelve a hacerlo siempre que el arduino este encendido guardando un registro de la accion realizada anteriormente y sigue asi hasta apagar el arduino.
   // Clears the trigPin


if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    //++value;
    //snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);

    StaticJsonDocument<128> doc;

    doc["DEVICE"] = "ESP32";
    //doc["Anho"] = 2022;
    //doc["Empresa"] = "Educatronicos";
    doc["VELOCIDAD1"] = (v);
    doc["DISTANCIA1"] = (d);
    doc["PROMEDIO1"] = (promedio);
    //doc["T"] = ("T:" + String(h)+ "h"+ String(m)+ "m"+ String(s)+ "s");
    doc["VELOCIDAD2"] = (v*0.621371);
    doc["DISTANCIA2"] = (d*0.621371);
    doc["PROMEDIO2"] = (promedio*0.621371);

    String output;
    
    serializeJson(doc, output);

    Serial.print("Publish message: ");
    Serial.println(output);
    Serial.println(output.c_str());
    client.publish("ELEQUIPO2", output.c_str());
  }





digitalWrite(trig, LOW);
delayMicroseconds(2);

// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trig, HIGH);
delayMicroseconds(10);
digitalWrite(trig, LOW);

// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echo, HIGH);

// Calculating the distance
distance= duration*0.034/2;
x= distance;// funcion de distancia con respecto a los valores marcados por el sensor ultrasonico.
   
   valorSensor = analogRead(0);// funcion para leer los valores analogicos dados por la fotorresistencia como entrada analógica. 
   valorMapeado = map(valorSensor, 0, 1023, 0, 5000);// mapeo de los valores dados: valorMapeado = valor arrojado por la fotoresistencia. 1023, 0 = rango maximo de fotoresistencia en milivolts. 0 a 5000 = se mandarán 5000 milivoltios para el foco si se cumple la condicion de a 900 milivots si es superior a eso no encenderá.
   analogWrite(luz, valorMapeado);//entonces el foco se ensendera con respecto al valor mapeado.
   if(valorMapeado <=900){// condicion si el valor mapeado es menor o igual a 900 encenderá.
   digitalWrite (luz, HIGH);}// se mandará a encender el foco.
   else {// si la condicion no se cumple o sea si es mayor a 900 entonces se mantendrá apagado.
   digitalWrite (luz, LOW);}// no se mandará a encender el foco.
   w = digitalRead (pulsor);// se le asigna la lectura del push boton.
   tiempo = millis();// se le asigna a tiempo los milisegundos que dará el arduino.
 
   if (x <15 && y==false)// condicion si la distancia x es menor a 200(para el proteus necesita valores mayores para la resistencia variable, en fisico se le asignará 3 a 5 cm) y si b es falso entonces se hará la condición.
   { 
    z = z+1;//Incremento si se cumple la condicion gracias al loop se dará de manera continua siempre que se cumpla la condicion. Si se hace una pedaleada la contará y asi hasta el total de pedaleadas dadas.
    y =true;// y pasa a ser verdadero.
    velocimetro();// aparecerán los calculos del velocimetro y se calcularán.
    Pantalla();// mostrará los datos en la pantalla lcd.
      
   }
   if (x>=15){y=false;}// si "x" es igual o menor a 500 entonces "y" será falso. y no mostrará nada ni habrá incremento de "z" + 1 debido a que es mayor "x" o igual que 500. 



if (x < 15) { //valida si el pulsador ha cambiado de 1 a 0 ó si el contador esta en -1
    cont++; // suma 1 al valor de la variable contador
    
    if (cont >= 0){
      maximus=v;
    
    }
    
      else if(cont>=maximus){
        maximus= v;
        
      }
  
      total=total+v;
  
     }

      
  
     if (cont==0){
      promedio=0;
      }
      else { promedio=total/cont;
  
     
      }

unsigned long over;
elapsed = millis();
 h = int(elapsed / 3600000);
 over = elapsed % 3600000;
 m = int(over / 60000);
 over = over % 60000;
 s = int(over / 1000);
}

void velocimetro(){// funcion del velocimetro.

  t1 = tiempo-t;// intervalo de tiempo.
  t = tiempo;// "t" como primer tiempo.
  v = 23682.63744/t1; // calculo de la velocidad. la constante dada es la misma a la lo calculado en el reporte.
  d = 0.0065785104*z; //distancia recorrida de metros a kilometros y de kilometros a millas.
 
}
void Pantalla(){// funcion de la pantalla lcd.



   lcd.clear();// se limpia pantalla.
   if (w == LOW){// si w esta en bajo ( push boton abierto). entonces se mostrarán los valores en km y km/h.
   lcd.setCursor(0,0);// coordenada de la  velocidad.
   lcd.print("Vel:"+String(v)+"Km/h" );// mostrará velocidad en kilometros por hora.
   lcd.setCursor(0,1);// coordenada de la  distancia.
   lcd.print("Dist:"+ String(d)+"Km");//mostrará distancia en kilometros.
   lcd.setCursor(0,2);// coordenada de la  velocidad.
   lcd.print("Promedio:"+ String(promedio)+"Km/h");//mostrará distancia en kilometros.
   lcd.setCursor(0,3);// coordenada de la  distancia.
   lcd.print("T:" + String(h)+ "h"+ String(m)+ "m"+ String(s)+ "s");
   delay(100); }
    else if (w == HIGH){// si w esta en alto ( push boton cerrado). entonces se mostrarán los valores en mi y mi/h.
   lcd.setCursor(0,0);// coordenada de la  velocidad.
    lcd.print("Vel:" + String(v*0.621371) +"Mi/h");// mostrará velocidad en millas por hora.
   lcd.setCursor(0,1);// coordenada de la  distancia.
   lcd.print("Dist:" + String(d*0.621371) + "Mi");//mostrará dististancia en millas 
   lcd.setCursor(0,2);// coordenada de la  velocidad.
   lcd.print("Promedio:" + String(promedio*0.621371) + "Mi/h");//mostrará dististancia en millas 
   lcd.setCursor(0,3);// coordenada de la  distancia.
   lcd.print("Equipo 2");
    delay(100);}
   return;  // regresa la ejecución de la función y devuelve el control a la función que llama.
 }    



```
## Explicación de funcionamiento escencial

Primeramente el velocimetro es para una bicicleta fija que cuando el pedal pasa cerca del sensor (15 cm) este detecta movimiento y empieza a hacer cálculos estos cálculos estan determinados por lo mostrado en las siguientes imágenes:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/11.png?raw=true)

En la imagen de arriba sólo va a funcionar si cumple la condicion de x < 15 por lo qué si esto cumple procede a hacer los cálculos debidos:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/14.png?raw=true)

En la imagen anterior tiempo esta dado en la funcion millis() que es de contar en milisegundos a lo que t y t1 estan para determinar el intervalo de tiempo en el que ocurre el senseo, los demás datos como "v" y "d" estan dados por lo siguiente:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/12.png?raw=true)
![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/13.png?raw=true)

Una vez comprendida la escencia del dispositivo otras partes importantes:

### Mandar datos a node-red
![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/38.png?raw=true)


### Cálculo del promedio
![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/39.png?raw=true)

## Encender y apagar el led
![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/40.png?raw=true)

## Contador de tiempo del ejercicio
![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/41.png?raw=true)

## Condicion de cambio entre kilometros y millas a traves de un pulsador llamado "w"
![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/42.png?raw=true)

## Librerias
![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/15.png?raw=true)

## Conexión

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/16.png?raw=true)

# Configuración y conexión de node red

## Conexión de node red
 1. Colocar bloque ```mqqtt in```.

![](https://github.com/DiegoLlampallas/DHT22NODERED/blob/main/8.png?raw=true)

 2. Colocar bloque ```json```.

![](https://github.com/DiegoLlampallas/DHT22NODERED/blob/main/9.png?raw=true)

 3. Colocar 6 bloques ```function```.

![](https://github.com/DiegoLlampallas/DHT22NODERED/blob/main/10.png?raw=true)

 4. Colocar 6 bloques ```gauge```.

![](https://github.com/DiegoLlampallas/DHT22NODERED/blob/main/11.png?raw=true)

 5. Colocar 2 bloques ```chart```.

![](https://github.com/DiegoLlampallas/DHT22NODERED/blob/main/12.png?raw=true)

 6. Conectamos todos los componentes como se muestra en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/2.png?raw=true)

Tras conectar todo se pasa a la configuración.

## Configuración de node red

1. Vamos a la esquina superior derecha y en la "flechita que apunta  hacia abajo seleccionamos y buscamos "Dashboard".

![](https://github.com/DiegoLlampallas/DHT22NODERED/blob/main/21.png?raw=true)

2. Le damos de"+ tab".

3. Le damos en "+ group" cuatro veces para crear cuatro grupos.

4. Le damos en "edit" al "tab" creado.

5. Llenamos los datos con la información tal y como aparece en la siguiente imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/17.png?raw=true)

6. Le damos en "edit" al "grou 1" creado.

7. Llenamos los datos con la información tal y como aparece en la siguiente imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/18.png?raw=true)


8. Le damos en "edit" al "grou 2" creado.

9. Llenamos los datos con la información tal y como aparece en la siguiente imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/19.png?raw=true)

10. Le damos en "edit" al "grou 3" creado.

11. Llenamos los datos con la información tal y como aparece en la siguiente imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/20.png?raw=true)

12. Le damos en "edit" al "grou 4" creado.

13. Llenamos los datos con la información tal y como aparece en la siguiente imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/21.png?raw=true)

Tras completar todo esto vamos a ir seleccionando y dando doble click a cada bloque y llenarlo con la información necesaria, en este caso de izquierda a derecha:


1. Edit "mqtt" y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/22.png?raw=true)

2. Edit "json" y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/23.png?raw=true)

3. Edit "function" la de arriba y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/24.png?raw=true)

4. Edit "function" la de abajo y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/25.png?raw=true)

5. Edit "function" la de arriba y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/26.png?raw=true)

6. Edit "function" la de abajo y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/27.png?raw=true)

7. Edit "function" la de arriba y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/28.png?raw=true)

8. Edit "function" la de abajo y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/29.png?raw=true)

9. Edit "gauge" la de arriba y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/30.png?raw=true)

10. Edit "gauge" la de abajo y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/31.png?raw=true)

11. Edit "gauge" la de arriba y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/32.png?raw=true)

12. Edit "gauge" la de abajo y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/33.png?raw=true)

13. Edit "gauge" la de arriba y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/34.png?raw=true)

14. Edit "gauge" la de abajo y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/35.png?raw=true)

15. Edit "chart" y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/36.png?raw=true)

16. Edit "chart" y completar los datos mostrados en la imagen:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/37.png?raw=true)

Tras completar todo esto puede empezar la simulación de operación tanto en "wokwi" como en el "node red" como se mostrará acontinuación: 

### Instrucciónes de operación

1. Iniciar simulador "wokwi".

2. Una vez que conecte y empiece a dar datos en el monitor serial podemos dar en iniciar en "node red" como se muestra en la siguiente imagen:

![](https://github.com/DiegoLlampallas/DHT22NODERED/blob/main/30.png?raw=true)

3. Podemos ir haciendo detecciones en el simulador "wokwi" en distancias de 15 cm que es la detección con el sensor **Ultrasónico** 

4. Para acceder a la información mandada por el "ESP32" hay dos opciones para acceder a la página web creada:

Opción 1:

1. Escribiendo el código "localhost/1880/ic" en la imagen en una nueva página limpia:

![](https://github.com/DiegoLlampallas/DHT22NODERED/blob/main/31.png?raw=true)

Opción 2:

2. Dando click en la esquina superior derecha por donde esta dashboard:

![](https://github.com/DiegoLlampallas/DHT22NODERED/blob/main/32.png?raw=true)




## Ejecutar el programa XAMPP para usar el phpMyAdmin 

1. Debemos abrir el programa llamado XAMPP.
2. Dentro de la interfaz nos vamos a la fila llamada **Mysql**.
3. Le damos doble click al boton **Admin**.

![](https://github.com/DiegoLlampallas/Basededatos/blob/main/3.png?raw=true)

Tras esto le damos en crear base de datos en la sección de la izquierda y le ponemos el nombre que deseemos (en este caso bicicleta):

![](https://github.com/DiegoLlampallas/Basededatos/blob/main/4.png?raw=true)

Tras esto le damos en tabla en la sección de la izquierda y le ponemos el nombre que deseemos (en este caso tabla) con 9 columnas.

Despues llenamos los datos con la siguiente información:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/43.png?raw=true)


*Nota de ser necesario en la parte superior donde dice agregar 1 columnas al hacer click podemos agregar mas columnas.

Tras crear y guardar la tabla se irá en la seccion de insertar y se añaden los siguientes datos:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/44.png?raw=true)

Tras esto dará un código que se ocupará en la sección de function donde se acoplará y se programará en esa sección:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/45.png?raw=true)

## Usarlo en node-red

Se colocará el bloque de Mysql y un bloque de función para obtener los datos con el siguente codigo. 

![](https://github.com/DiegoLlampallas/DHT22NODERED/blob/main/10.png?raw=true)
![](https://github.com/DiegoLlampallas/Basededatos/blob/main/7.png?raw=true)

Se anexan un nuevo function y mysql en la conexión:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/46.png?raw=true)

Tras esto se configuran primero function añadiendo el código dado anteriormente por phpMyAdmin y demás datos:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/47.png?raw=true)

Mysql:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/48.png?raw=true)
![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/49.png?raw=true)

Al final se le dará en el boton rojo Deploy y se visualizaran los resultados.

## Resultados

Cuando haya funcionado, verás los valores dentro del monitor serial en "wokwi", en la página creada red-node y los datos en phpMyAdmin.

## Funcionamiento

1. Funcionamiento en "wokwi":

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/3.png?raw=true)

2. Funcionamiento en la página web creada en red-node:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/1.png?raw=true)
![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/4.png?raw=true)
![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/5.png?raw=true)
![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/6.png?raw=true)
![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/7.png?raw=true)
![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/8.png?raw=true)
![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/9.png?raw=true)
![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/10.png?raw=true)

3. Funcionamiento en la página web creada en phpMyAdmin:

![](https://github.com/DiegoLlampallas/ProyectoFinal/blob/main/52.png?raw=true)

## Evidencias

[Página](https://wokwi.com/projects/367838812052939777)


# Créditos

Desarrollado por: Ing. Diego Alberto Llampallas Vega, Ing. Cesar Gonzalez Infante e Ing. Miguel Bello Torres.

- [GitHub](https://github.com/DiegoLlampallas)
- [GitHub](https://github.com/CesarG16)
- [GitHub](https://github.com/Miguebt2707)