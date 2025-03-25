#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ModbusIP_ESP8266.h>

#define SLAVE_ADDRESS 0x02
#define RX_RS485  D7 
#define TX_RS485  D5
#define RE_RS485  D6

EspSoftwareSerial::UART testSerial;
ModbusIP mb;
IPAddress remote(192, 168, 0, 191);
const uint16_t serverPort = 502;

const uint16_t TOTAL_REGS = 400;
uint16_t bufferModbus[TOTAL_REGS];

struct ModbusRequest {
    uint16_t start;
    uint16_t count;
    bool leer;
};

ModbusRequest requests[] = {
    {0, 18,true}, {18, 18,true}, {52, 12,true}, {70, 12,true}, {200, 6,true}, {342, 4,true}
};
const uint8_t numRequests = sizeof(requests) / sizeof(requests[0]);

uint16_t calcularCRC16(const uint8_t* datos, uint16_t longitud);

void setup() {
    pinMode(RE_RS485, OUTPUT);
    digitalWrite(RE_RS485, LOW);
    testSerial.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, RX_RS485, TX_RS485);
    Serial.begin(115200);

    WiFi.begin("red", "password");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.println(WiFi.localIP());
    mb.client();
}

void loop() {
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 5000) {
        lastUpdate = millis();
        if (!mb.isConnected(remote)) {
            if (mb.connect(remote, serverPort)) {
                Serial.println("Conectado a Modbus TCP");
            } else {
                Serial.println("Fallo conexión Modbus TCP");
                ESP.restart();
                return;
            }
        }
    }
        static unsigned long lastUpdate_tcp = 0;
        if(millis() - lastUpdate_tcp > 1000) {
            lastUpdate_tcp = millis();
            for (uint8_t i = 0; i < numRequests; i++) {
                if (requests[i].leer) { // Solo se realiza la lectura si 'leer' es true
                    if (mb.readIreg(remote, requests[i].start, &bufferModbus[requests[i].start], requests[i].count)) {
                        Serial.printf("Leídos %d registros desde %d\n", requests[i].count, requests[i].start);
                    } else {
                        Serial.printf("Error leyendo desde %d\n", requests[i].start);
                    }
                    // Cambiar el valor de 'leer' a false para no repetir la lectura innecesariamente
                    requests[i].leer = false;
                }
            }
        }

    if (testSerial.available() > 0) {
        delay(10);
        uint8_t buffer[256];
        uint16_t length = 0;
        while (testSerial.available() > 0 && length < sizeof(buffer)) {
             buffer[length] = testSerial.read();
             //Serial.print(buffer[length], HEX);
            // Serial.print(" ");
             length++;
        }
      /*******************************************************************************/
      if (length >= 4) {
          uint16_t crcRecibido = (buffer[length - 1] << 8) | buffer[length - 2];
          uint16_t crcCalculado = calcularCRC16(buffer, length - 2);
      
          if (crcRecibido == crcCalculado && buffer[0] == SLAVE_ADDRESS) {
              uint16_t start = (buffer[2] << 8) | buffer[3];
              uint16_t count = (buffer[4] << 8) | buffer[5];
                if (buffer[1] != 0x04) {
                      Serial.printf("Función 0x%02X no soportada\n", buffer[1]);
                }
              if (start + count <= TOTAL_REGS) {
                  bool requestFound = false;
                  for (uint8_t i = 0; i < numRequests; i++) {
                      if (requests[i].start == start && requests[i].count == count) {
                          // Se encuentra la petición en la tabla, se activa la lectura
                          requests[i].leer = true;
                          requestFound = true;
                          break;
                      }
                  }
                  if (!requestFound) {
                      // Si el mensaje es para esclavo 2 pero no está en la tabla, se imprime el mensaje
                      Serial.printf("Mensaje para esclavo %d no registrado en tabla: start = %d, count = %d\n", SLAVE_ADDRESS, start, count);
                  }
                  
                  // Preparar y enviar la respuesta usando los datos en bufferModbus
                  uint8_t respuesta[256] = { SLAVE_ADDRESS, 0x04, (uint8_t)(count * 2) };
                  for (uint16_t i = 0; i < count; i++) {
                      respuesta[3 + i * 2] = bufferModbus[start + i] >> 8;
                      respuesta[4 + i * 2] = bufferModbus[start + i] & 0xFF;
                  }
                  uint16_t crc = calcularCRC16(respuesta, 3 + count * 2);
                  respuesta[3 + count * 2] = crc & 0xFF;
                  respuesta[4 + count * 2] = crc >> 8;
      
                  digitalWrite(RE_RS485, HIGH);
                  testSerial.write(respuesta, 5 + count * 2);
                  delay(10);
                  digitalWrite(RE_RS485, LOW);
              }
          }
      }
      /*********************************************************************************/

    }
    mb.task();
}

uint16_t calcularCRC16(const uint8_t* datos, uint16_t longitud) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < longitud; i++) {
        crc ^= datos[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}
