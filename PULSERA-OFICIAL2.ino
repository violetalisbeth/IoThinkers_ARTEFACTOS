//CONFIG BLYNK
#define BLYNK_TEMPLATE_ID "TMPL2cFzs-wki"
#define BLYNK_TEMPLATE_NAME "IoThinkers"
#define BLYNK_AUTH_TOKEN "SkUCvQNih18xomnK-Z2nVQsXmpTQQqaL"

//LIBRERIAS
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <UniversalTelegramBot.h>
#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// INSTANCIAS SENSORES
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
MAX30105 particleSensor;

// SENSOR DS18B20
#define ONE_WIRE_BUS 14
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

// CONFIG GPS
#define GPS_RX 16
#define GPS_TX 17
HardwareSerial neogps(1);

char ssid[] = "Fer";
char pass[] = "ferdiaz00";

// CONFIG TELEGRAM
const char* BOT_TOKEN = "8502499699:AAGk5OXIe53OCt2NZkVDeBppfdrZd94zTtg";
const String CHAT_ID = "6455597869"; 

WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);
BlynkTimer timer;

const float CAIDA_UMBRAL = 2.0f;
const float IMPACTO_UMBRAL = 8.0f;
const float ROTACION_UMBRAL = 150.0f;
const unsigned long TIEMPO_ESPERA_CAIDA = 2000;
const unsigned long TIEMPO_POST_IMPACTO = 1000;
unsigned long ultimaDeteccion = 0;
unsigned long tiempoImpacto = 0;
bool caidaDetectada = false;
bool posibleCaida = false;
float aceleracionAnterior[3] = {0, 0, 0};
float velocidadAngularAnterior[3] = {0, 0, 0};
const int HISTORIAL_SIZE = 5;
float historialAceleracion[HISTORIAL_SIZE];
int historialIndex = 0;
bool sistemaInmovilPostCaida = false;
bool alertaEnviada = false;

// VARIABLES MAX30102
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;
float spo2 = 0;
unsigned long lastMAX30102Read = 0;
const unsigned long MAX30102_INTERVAL = 20;

// VARIABLES TEMPERATURA
float temperatura = 0;
unsigned long lastTempRead = 0;
const unsigned long TEMP_INTERVAL = 2000;

// RANGOS NORMALES PARA ADULTOS MAYORES
const float SPO2_MIN = 92.0;
const float SPO2_MAX = 100.0;
const int BPM_MIN = 60;
const int BPM_MAX = 90;
const float TEMP_MIN = 35.2;
const float TEMP_MAX = 37.5;

// VARIABLES PARA CONTROL DE ALERTAS
bool sistemaListo = false;
unsigned long tiempoInicio = 0;
const unsigned long TIEMPO_INICIALIZACION = 90000;

unsigned long ultimaAlertaSPO2 = 0;
unsigned long ultimaAlertaBPM = 0;
unsigned long ultimaAlertaTemp = 0;
const unsigned long INTERVALO_ALERTAS = 40000;

struct Ubicacion {
  float latitud;
  float longitud;
  float altitud;
  int satelites;
  bool valida;
};

void configurarTelegram() {
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  Serial.println("Cliente seguro Telegram configurado");
}

void configurarMPU6050() {
  Serial.println("Inicializando MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Error: No se encuentra el chip MPU6050");
    while (1) delay(10);
  }
  Serial.println("MPU6050 encontrado!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void configurarGPS() {
  Serial.println("Inicializando GPS...");
  neogps.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(1000);
}

void configurarMAX30102() {
  Serial.println("Inicializando MAX30102...");
  delay(100);
  
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 no encontrado. Verificar conexiones I2C.");
    return;
  }
  Serial.println("MAX30102 inicializado");

  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 400;
  int pulseWidth = 411;
  int adcRange = 4096;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeRed(0x2F);
  particleSensor.setPulseAmplitudeIR(0x2F);
  particleSensor.clearFIFO();
  
  Serial.println("Configuración MAX30102 aplicada");
}

void configurarTemperatura() {
  Serial.println("Inicializando sensor de temperatura DS18B20...");
  delay(100);
  
  tempSensor.begin();
  
  DeviceAddress tempDeviceAddress;
  if (!tempSensor.getAddress(tempDeviceAddress, 0)) {
    Serial.println("DS18B20 no encontrado. Verificar conexión en pin 14.");
  } else {
    Serial.println("DS18B20 inicializado correctamente");
    tempSensor.setResolution(tempDeviceAddress, 12);
    Serial.println("Resolución configurada a 12 bits");
  }
}

Ubicacion obtenerUbicacion() {
  Ubicacion ubicacion = {0, 0, 0, 0, false};
  ubicacion.valida = gps.location.isValid();
  if (ubicacion.valida) {
    ubicacion.latitud = gps.location.lat();
    ubicacion.longitud = gps.location.lng();
    ubicacion.altitud = gps.altitude.isValid() ? gps.altitude.meters() : 0;
    ubicacion.satelites = gps.satellites.isValid() ? gps.satellites.value() : 0;
  }
  return ubicacion;
}

bool detectarCaidaSensible(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ) {
  static unsigned long ultimoTiempo = millis();
  unsigned long tiempoActual = millis();
  
  if (alertaEnviada && (tiempoActual - ultimaDeteccion) > 10000) {
    alertaEnviada = false;
  }
  
  if (tiempoActual - ultimaDeteccion < 500) {
    return false;
  }
  
  float magnitud = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  
  historialAceleracion[historialIndex] = magnitud;
  historialIndex = (historialIndex + 1) % HISTORIAL_SIZE;
  
  float velocidadAngularTotal = sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ);
  
  static float magnitudAnterior = 9.8;
  float cambioAceleracion = abs(magnitud - magnitudAnterior);
  magnitudAnterior = magnitud;
  
  bool caidaLibreLeve = (magnitud < CAIDA_UMBRAL);
  bool impactoLeve = (cambioAceleracion > IMPACTO_UMBRAL);
  bool rotacionLeve = (velocidadAngularTotal > ROTACION_UMBRAL);
  
  bool movimientoBrusco = false;
  float varianza = calcularVarianzaAceleracion();
  
  if (impactoLeve || rotacionLeve) {
    movimientoBrusco = true;
  }
  

  bool detectarAlerta = false;
  
  if (caidaLibreLeve) {
    Serial.println("FASE 1: Caída libre detectada");
    detectarAlerta = true;
  }
  
  if (impactoLeve && magnitud > 5.0f) {
    Serial.println("FASE 2: Impacto detectado");
    detectarAlerta = true;
  }
  
  if (rotacionLeve && magnitud > 3.0f) {
    Serial.println("FASE 3: Rotación brusca detectada");
    detectarAlerta = true;
  }
  
  if ((cambioAceleracion > 5.0f && velocidadAngularTotal > 80.0f) || 
      (magnitud > 12.0f && velocidadAngularTotal > 100.0f)) {
    Serial.println("FASE 4: Movimiento complejo detectado");
    detectarAlerta = true;
  }
  
  static unsigned long ultimoMovimientoBrusco = 0;
  if (velocidadAngularTotal > 200.0f || cambioAceleracion > 10.0f) {
    if (tiempoActual - ultimoMovimientoBrusco > 1000) {
      Serial.println("FASE 5: Movimiento muy brusco detectado");
      detectarAlerta = true;
      ultimoMovimientoBrusco = tiempoActual;
    }
  }
  
  if (detectarAlerta && !alertaEnviada) {
    alertaEnviada = true;
    ultimaDeteccion = tiempoActual;
    Serial.println("¡ALERTA ACTIVADA! Enviando notificación...");
    return true;
  }
  
  return false;
}

float calcularVarianzaAceleracion() {
  float suma = 0;
  float sumaCuadrados = 0;
  int count = 0;
  
  for (int i = 0; i < HISTORIAL_SIZE; i++) {
    if (historialAceleracion[i] > 0) {
      suma += historialAceleracion[i];
      sumaCuadrados += historialAceleracion[i] * historialAceleracion[i];
      count++;
    }
  }
  
  if (count == 0) return 0;
  
  float promedio = suma / count;
  float varianza = (sumaCuadrados / count) - (promedio * promedio);
  
  return varianza;
}

void procesarMAX30102() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastMAX30102Read >= MAX30102_INTERVAL) {
    lastMAX30102Read = currentTime;
    
    long irValue = particleSensor.getIR();
    
    if (irValue > 50000) {
      if (checkForBeat(irValue)) {
        long delta = millis() - lastBeat;
        lastBeat = millis();

        if (delta > 0) {
          beatsPerMinute = 60 / (delta / 1000.0);
          if (beatsPerMinute < 255 && beatsPerMinute > 20) {
            rates[rateSpot++] = (byte)beatsPerMinute;
            rateSpot %= RATE_SIZE;

            beatAvg = 0;
            for (byte x = 0; x < RATE_SIZE; x++) {
              beatAvg += rates[x];
            }
            beatAvg /= RATE_SIZE;
          }
        }
      }

      long redValue = particleSensor.getRed();
      if (redValue > 10000 && irValue > 10000) {
        float ratio = (float)redValue / (float)irValue;
        spo2 = 110.0 - (25.0 * ratio);
        spo2 = constrain(spo2, 0, 100);
      }
    } else {
      if (currentTime - lastBeat > 3000) {
        beatAvg = 0;
        spo2 = 0;
        for (byte i = 0; i < RATE_SIZE; i++) {
          rates[i] = 0;
        }
      }
    }
  }
}

void leerTemperatura() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastTempRead >= TEMP_INTERVAL) {
    lastTempRead = currentTime;
    
    tempSensor.requestTemperatures();
    delay(10);
    
    temperatura = tempSensor.getTempCByIndex(0);
    
    if (temperatura == DEVICE_DISCONNECTED_C) {
      Serial.println("Error leyendo temperatura DS18B20");
      temperatura = -127;
    }
  }
}

// SISTEMA DE ALERTAS TELEGRAM

void verificarSignosVitales() {
  if (!sistemaListo) return;
  
  unsigned long tiempoActual = millis();
  Ubicacion ubicacion = obtenerUbicacion();
  
  if (spo2 > 0 && (spo2 < SPO2_MIN || spo2 > SPO2_MAX)) {
    if (tiempoActual - ultimaAlertaSPO2 > INTERVALO_ALERTAS) {
      enviarAlertaTelegram("NIVEL DE OXÍGENO CRÍTICO", 
                          "Nivel de Oxigenación: " + String(spo2, 1) + "%\nRango normal: " + String(SPO2_MIN) + "-" + String(SPO2_MAX) + "%",
                          ubicacion);
      ultimaAlertaSPO2 = tiempoActual;
    }
  }
  
  if (beatAvg > 0 && (beatAvg < BPM_MIN || beatAvg > BPM_MAX)) {
    if (tiempoActual - ultimaAlertaBPM > INTERVALO_ALERTAS) {
      enviarAlertaTelegram("FRECUENCIA CARDÍACA ANORMAL", 
                          "LPM: " + String(beatAvg) + "\nRango normal: " + String(BPM_MIN) + "-" + String(BPM_MAX) + " lpm",
                          ubicacion);
      ultimaAlertaBPM = tiempoActual;
    }
  }
  
  if (temperatura != -127 && (temperatura < TEMP_MIN || temperatura > TEMP_MAX)) {
    if (tiempoActual - ultimaAlertaTemp > INTERVALO_ALERTAS) {
      enviarAlertaTelegram("TEMPERATURA CORPORAL ANORMAL", 
                          "Temperatura: " + String(temperatura, 1) + "°C\nRango normal: " + String(TEMP_MIN) + "-" + String(TEMP_MAX) + "°C",
                          ubicacion);
      ultimaAlertaTemp = tiempoActual;
    }
  }
}
//ALERTA TELEGRAM
void enviarAlertaTelegram(const String& tipoAlerta, const String& datos, Ubicacion ubicacion) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("No hay conexión WiFi para enviar alerta");
    return;
  }
  
  String mensaje = "🚨 *ALERTA: " + tipoAlerta + "* 🚨\n\n";
  mensaje += datos + "\n\n";
  
  if (ubicacion.valida) {
    mensaje += "📍 *Ubicación:* \n";
    mensaje += "Latitud: " + String(ubicacion.latitud, 6) + "\n";
    mensaje += "Longitud: " + String(ubicacion.longitud, 6) + "\n";
    
    String googleMapsLink = "https://maps.google.com/?q=" + 
                           String(ubicacion.latitud, 6) + "," + 
                           String(ubicacion.longitud, 6);
    mensaje += "🗺️ [Abrir en Maps](" + googleMapsLink + ")\n";
  } else {
    mensaje += "📍 *Ubicación:* Sin señal GPS\n";
  }
    
  Serial.println("Enviando alerta a Telegram: " + tipoAlerta);
  
  if (bot.sendMessage(CHAT_ID, mensaje, "Markdown")) {
    Serial.println("Alerta enviada correctamente");
  } else {
    Serial.println("Error enviando alerta");
  }
}

// FUNCIONES BLYNK
void enviarTemperaturaBlynk() {
  Blynk.virtualWrite(V0, temperatura);
}

void enviarDatosSaludBlynk() {
  Blynk.virtualWrite(V2, spo2);
  Blynk.virtualWrite(V3, beatAvg);
}

void enviarUbicacionBlynk() {
  Ubicacion ubicacion = obtenerUbicacion();
  
  if (ubicacion.valida) {
    String coordenadas = String(ubicacion.latitud, 6) + "," + String(ubicacion.longitud, 6);
    Blynk.virtualWrite(V1, coordenadas);
  } else {
    Blynk.virtualWrite(V1, "Buscando...");
  }
}

void mostrarDatosSerial() {
  static unsigned long lastSerialPrint = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastSerialPrint >= 2000) {
    Serial.println("\n     DATOS DE SENSORES    ");
    Serial.print(" Temperatura: "); Serial.print(temperatura); Serial.println(" °C");
    Serial.print(" LPM: "); Serial.print(beatAvg); 
    Serial.print(" | Nivel de Oxigenación: "); Serial.print(spo2, 1); Serial.println("%");
    
    if (!sistemaListo) {
      Serial.print("Sistema iniciándose: ");
      Serial.print((TIEMPO_INICIALIZACION - (currentTime - tiempoInicio)) / 1000);
      Serial.println(" segundos restantes");
    } else {
      Serial.println("Sistema listo - Alertas activas");
    }
    
    Ubicacion ubicacion = obtenerUbicacion();
    if (ubicacion.valida) {
      Serial.print("GPS: "); Serial.print(ubicacion.latitud, 6);
      Serial.print(", "); Serial.print(ubicacion.longitud, 6);
    } else {
      Serial.println("GPS: Sin señal");
    }
    Serial.println("\n");
    
    lastSerialPrint = currentTime;
  }
}

void procesarCaidaDetectada() {
  unsigned long tiempoActual = millis();
  
  caidaDetectada = true;
  ultimaDeteccion = tiempoActual;
  
  Ubicacion ubicacion = obtenerUbicacion();
  Serial.println("ALERTA: POSIBLE CAÍDA DETECTADA - Enviando notificaciones...");
  
  enviarAlertaExterna(ubicacion);
}

void enviarAlertaExterna(Ubicacion ubicacion) {
  enviarAlertaTelegram("POSIBLE CAÍDA DETECTADA", 
                      "Se ha detectado un movimiento que podría indicar una caída.\n\n" +
                      String("LPM: ") + String(beatAvg) + 
                      "\nNivel de Oxigenación: " + String(spo2, 1) + "%" +
                      "\nTemperatura: " + String(temperatura, 1) + "°C" +
                      "\n", 
                      ubicacion);
  
  enviarTemperaturaBlynk();
  enviarDatosSaludBlynk();
  enviarUbicacionBlynk();
  
  Blynk.virtualWrite(V4, "¡Caída detectada!");
  
  Serial.println("Notificaciones enviadas correctamente");
}

void verificarInicializacion() {
  if (!sistemaListo && millis() - tiempoInicio > TIEMPO_INICIALIZACION) {
    sistemaListo = true;
    
    if (WiFi.status() == WL_CONNECTED) {
      bot.sendMessage(CHAT_ID, "\nEl sistema está listo", "Markdown");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\nSISTEMA DE MONITOREO");
  
  tiempoInicio = millis();
  
  Wire.begin();
  delay(100);
  
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 20) {
    delay(500);
    Serial.print(".");
    intentos++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado!");
  } else {
    Serial.println("\nError WiFi - Modo offline");
  }
  
  configurarMPU6050();
  delay(100);
  
  configurarGPS();
  delay(100);
  
  configurarMAX30102();
  delay(100);
  
  configurarTemperatura();
  delay(100);
  
  configurarTelegram();
  
  for (int i = 0; i < HISTORIAL_SIZE; i++) {
    historialAceleracion[i] = 9.8f;
  }

  timer.setInterval(1000L, enviarTemperaturaBlynk);
  timer.setInterval(2000L, enviarDatosSaludBlynk);  
  timer.setInterval(3000L, enviarUbicacionBlynk);

  Serial.println("\nTodos los sensores inicializados");
 
}

void loop() {
  Blynk.run();
  timer.run();

  verificarInicializacion();

  while (neogps.available() > 0) {
    gps.encode(neogps.read());
  }
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  procesarMAX30102();
  leerTemperatura();
  
  if (sistemaListo) {
    verificarSignosVitales();
  }
  
  if (detectarCaidaSensible(a.acceleration.x, a.acceleration.y, a.acceleration.z, 
                           g.gyro.x, g.gyro.y, g.gyro.z)) {
    procesarCaidaDetectada();
  }
  
  mostrarDatosSerial();

  delay(10);
}