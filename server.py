
import socket
import threading

# Configuración del servidor
HOST = '0.0.0.0'  # Escucha en todas las interfaces de red disponibles
PORT = 8000  # Puerto de escucha

# Diccionario para almacenar las conexiones de los clientes
client_connections = {}

# Función para manejar la conexión con un cliente
def handle_client(client_socket, client_address):
    try:
        print(f'Conexión entrante de {client_address}')

        while True:
            # Leer los datos enviados por el cliente
            data = client_socket.recv(1024).decode().strip()
            if not data:
                print(f'Cliente {client_address} desconectado.')
                break
            
            print(f'Datos recibidos de {client_address}: {data}')

            # Interpretar los datos y tomar decisiones
            print(f'Datos interpretados de {client_address}: {data}')

            # Puedes agregar aquí la lógica para controlar el LED y enviar comandos a los clientes según sea necesario
            if data.startswith("TEMP:"):
                temperature = float(data.split(":")[1])
                if temperature > 30:
                    # Enviar comando para encender el LED al cliente .25
                    print("temperatura demasiado alta")
                    led_command = "ENCENDER_LED"
                    led_address = "192.168.0.25"  # Dirección IP del cliente
                    print("Enviando datos a: ")
                    print(led_address)
                    if led_address in client_connections:
                        print("Cliente LED ENCONTRADO")
                        print(led_address)
                        led_client_socket = client_connections[led_address]
                        led_client_socket.sendall(led_command.encode())
                    else:
                        print("Error: Cliente LED no encontrado.")
                else:
                    # Enviar comando para encender el LED al cliente .25
                    print("Temperatura aceptable. No se requiere acción.")
                    led_command = "APAGAR_LED"
                    led_address = "192.168.0.25"  # Dirección IP del cliente
                    if led_address in client_connections:
                        print("Cliente LED ENCONTRADO")
                        led_client_socket = client_connections[led_address]
                        led_client_socket.sendall(led_command.encode())
                    else: 
                        print("Error: Cliente LED no encontrado.")
                    

    except Exception as e:
        print(f'Error al manejar la conexión con {client_address}: {str(e)}')
    finally:
        # Cerrar la conexión con el cliente
        client_socket.close()
        del client_connections[client_address[0]]  # Eliminar la entrada del cliente del diccionario

# Función principal del servidor
def main():
    # Crear el socket del servidor
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)

    print(f'Servidor TCP escuchando en {HOST}:{PORT}')

    try:
        while True:
            # Esperar conexiones entrantes
            client_socket, client_address = server_socket.accept()

            # Almacenar la conexión del cliente
            client_connections[client_address[0]] = client_socket

            # Mostrar los clientes conectados
            print("Clientes conectados:")
            for address in client_connections:
                print(f"- {address}")

            # Iniciar un hilo para manejar la conexión con el cliente
            client_thread = threading.Thread(target=handle_client, args=(client_socket, client_address))
            client_thread.start()

    except KeyboardInterrupt:
        print('Deteniendo el servidor...')
    finally:
        # Cerrar todas las conexiones de clientes al detener el servidor
        for client_socket in client_connections.values():
            client_socket.close()
        server_socket.close()

if __name__ == '__main__':
    main()


'''
                CODIGO DEL ESP32 LED


#include <WiFi.h>
#include <WiFiClient.h>

class LED{ 
public: 
  LED(int pin): pin(pin){
    pinMode(pin, OUTPUT);
    estado = LOW;
  }
  
  void encenderLed(){
    digitalWrite(pin, HIGH);
    estado = HIGH;
  }
  
  void apagarLed(){
    digitalWrite(pin, LOW);
    estado = LOW;
  }
  
  void parpadearLed(int intervalo){
    encenderLed();
    delay(intervalo / 2);
    apagarLed();
    //delay(intervalo / 2);
  }
  
  int obtenerEstado(){
    return estado;
  }
  
private:
  int pin;
  int estado;
};


const char* ssid = "nombre red wifi"; // Nombre de tu red Wi-Fi
const char* password = "contraseña red wifi"; // Contraseña de tu red Wi-Fi

const char* serverIP = "ip de la pc con el server"; 
 // Dirección IP del servidor (PC)
const uint16_t serverPort = 8000;      // Puerto del servidor

WiFiClient client;

// Pin del LED
//const int ledPin = 23; // Cambia al pin que esté conectado el LED en tu ESP32
LED led(23);

void setup() {
  Serial.begin(9600);
  //pinMode(ledPin, OUTPUT);

  // Conectar a la red WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a la red WiFi...");
  }

  Serial.println("Conexión exitosa a la red WiFi");
  Serial.print("Dirección IP asignada: ");
  Serial.println(WiFi.localIP());

  // Iniciar conexión con el servidor
  if (!connectToServer()) {
    Serial.println("Error al conectar con el servidor.");
    return;
  }
}

void loop() {
  // Leer respuesta del servidor
  while (client.available()) {
    String mensaje = client.readStringUntil('\n');
    mensaje.trim();

    // Interpretar el mensaje recibido y controlar el LED
    if (mensaje == "ENCENDER_LED") {
      //digitalWrite(ledPin, HIGH);
      led.encenderLed();
      Serial.println("LED encendido");
    } else if (mensaje == "APAGAR_LED") {
      //digitalWrite(ledPin, LOW);
      led.apagarLed();
      Serial.println("LED apagado");
    }
  }

  // Verificar la conexión con el servidor o reconectar
  if (!client.connected()) {
    if (!connectToServer()) {
      Serial.println("Error al reconectar con el servidor.");
    }
  }

  // Esperar antes de tomar nuevas acciones
  delay(1000);
}

bool connectToServer() {
  if (client.connect(serverIP, serverPort)) {
    Serial.println("Conectado al servidor.");
    return true;
  } else {
    Serial.println("Error al conectar al servidor.");
    return false;
  }
}
'''

'''
                CODIGO DEL ESP32 SENSOR


#include <WiFi.h>
#include <DHT.h>
#include <WiFiClient.h>

const char* ssid = "nombre red wifi"; // Nombre de tu red Wi-Fi
const char* password = "contraseña red wifi"; // Contraseña de tu red Wi-Fi

const char* serverIP = "ip de la pc con el server"; 
 // Dirección IP del servidor (PC)
const uint16_t serverPort = 8000;      // Puerto del servidor

// Configuración del sensor de temperatura DHT11
#define DHTPIN 13
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

const char* serverIP = "ip de la pc con el server"; 
 // Dirección IP del servidor (PC)
const uint16_t serverPort = 8000;      // Puerto del servidor
// Variables globales
WiFiClient client;

void setup() {
  Serial.begin(9600);

  // Conexión a la red Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a la red WiFi...");
  }
  Serial.println("Conexión exitosa a la red WiFi");
  Serial.print("Dirección IP asignada: ");
  Serial.println(WiFi.localIP());

  // Inicialización del sensor DHT11
  dht.begin();
}

void loop() {
  // Lectura de la temperatura y humedad del sensor
  float temperature = dht.readTemperature();
  if (isnan(temperature)) {
    Serial.println("Error al leer la temperatura del sensor");
    delay(2000);
    return;
  }

  // Verificar la conexión con el servidor
  if (!client.connected()) {
    if (!client.connect(serverIP, serverPort)) {
      Serial.println("Error al conectar con el servidor");
      delay(2000);
      return;
    }
  }

  // Construir el mensaje a enviar al servidor
  String message = "TEMP:" + String(temperature);

  // Enviar el mensaje al servidor
  if (client.connected()) {
    client.println(message);
  } else {
    Serial.println("Error: Conexión perdida con el servidor");
  }

  // Esperar un intervalo antes de enviar el próximo dato
  delay(5000); // Intervalo aumentado a 15 segundos
}

'''