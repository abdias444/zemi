// ============================================================
//  ESP32 – Control de Motores DC vía MQTT (HiveMQ Cloud TLS)
//  Recibe comandos F/B/L/R/S en el topic "zemi/motores/cmd"
//  + DHT22 + MQ2 + HC-SR04 + Servo Radar
// ============================================================

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ESP32Servo.h>

// ─── WiFi ────────────────────────────────────────────────────
const char* WIFI_SSID     = "Karen";       // ← Cambia esto
const char* WIFI_PASSWORD = "Karen8318";    // ← Cambia esto

// ─── MQTT HiveMQ Cloud ──────────────────────────────────────
const char* MQTT_BROKER   = "b1439a9c1a134c51be1ef17aa3c3cd62.s1.eu.hivemq.cloud";
const int   MQTT_PORT     = 8883;
const char* MQTT_USER     = "public";
const char* MQTT_PASS     = "Admin123";
// Client ID se genera en setup() para ser único
const char* TOPIC_CMD     = "zemi/motores/cmd";
const char* TOPIC_STATUS  = "zemi/motores/status";
const char* TOPIC_BATTERY = "zemi/motores/bateria";
const char* TOPIC_DHT     = "zemi/sensores/dht";
const char* TOPIC_GAS     = "zemi/sensores/gas";
const char* TOPIC_ULTRA   = "zemi/sensores/ultrasonico";

// ─── Sensor DHT22 ───────────────────────────────────────────
constexpr uint8_t DHT_PIN  = 26;       // GPIO26
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

constexpr unsigned long DHT_INTERVAL = 5000;  // leer cada 5 segundos
unsigned long lastDhtRead = 0;

// ─── Sensor MQ2 (Gas) ───────────────────────────────────────
constexpr uint8_t MQ2_PIN = 25;        // GPIO25 (ADC2_CH8)
constexpr unsigned long GAS_INTERVAL = 3000;  // leer cada 3 segundos
unsigned long lastGasRead = 0;

// ─── Sensor Ultrasonico HC-SR04 ─────────────────────────────
constexpr uint8_t TRIG_PIN = 32;       // GPIO32
constexpr uint8_t ECHO_PIN = 33;       // GPIO33
constexpr float MAX_DISTANCE = 400.0;  // distancia máxima en cm

// ─── Servo Motor (Radar Paneo) ──────────────────────────────
constexpr uint8_t SERVO_PIN = 34;      // GPIO34 – Señal del servo
Servo servoRadar;

int servoAngle = 0;                     // Ángulo actual del servo
int servoDirection = 1;                 // 1 = subiendo (0→180), -1 = bajando (180→0)
constexpr int SERVO_STEP = 2;           // Grados por paso
constexpr unsigned long SERVO_DELAY = 30; // ms entre pasos
unsigned long lastServoStep = 0;

// ─── Batería (ADC) ──────────────────────────────────────────
// Pin ADC para leer voltaje de batería (usa divisor de voltaje)
constexpr uint8_t BAT_PIN = 35;        // GPIO35 (ADC1_CH7, solo lectura)

// Parámetros del divisor de voltaje
// Si usas R1=100kΩ y R2=100kΩ → factor = 2.0
// Vbat_max ≈ 8.4V (2S LiPo) → al ADC llegan ~4.2V (dentro del rango 3.3V con atenuación 11dB)
constexpr float DIVISOR_FACTOR  = 2.0;   // (R1 + R2) / R2
constexpr float VREF            = 3.3;   // Voltaje de referencia ADC
constexpr int   ADC_RESOLUTION  = 4095;  // 12 bits

// Umbrales de batería (ajustar según tu batería)
constexpr float BAT_FULL        = 8.40;  // 100% – 2S LiPo completamente cargada
constexpr float BAT_EMPTY       = 6.00;  // 0%   – 2S LiPo descargada (corte seguro)

// Intervalo de lectura de batería (ms)
constexpr unsigned long BAT_INTERVAL = 10000;  // cada 10 segundos
unsigned long lastBatRead = 0;



// ─── Pines del puente H ─────────────────────────────────────
constexpr uint8_t IN1 = 13;   // Motor A+
constexpr uint8_t IN2 = 12;   // Motor A−
constexpr uint8_t IN3 = 14;   // Motor B+
constexpr uint8_t IN4 = 27;   // Motor B−

// ─── Certificado raíz HiveMQ Cloud (ISRG Root X1) ──────────
static const char* root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

// ─── Objetos globales ───────────────────────────────────────
WiFiClientSecure espClient;
PubSubClient     mqtt(espClient);
char clientId[40];

// ─── Control de motores ─────────────────────────────────────
void aplicarMotores(bool a1, bool a2, bool b1, bool b2) {
  digitalWrite(IN1, a1);
  digitalWrite(IN2, a2);
  digitalWrite(IN3, b1);
  digitalWrite(IN4, b2);
}

void adelante()   { aplicarMotores(HIGH, LOW,  LOW,  HIGH); }  // Motor A adelante, Motor B atrás → avanza
void atras()      { aplicarMotores(LOW,  HIGH, HIGH, LOW);  }  // Motor A atrás,    Motor B adelante → retrocede
void izquierda()  { aplicarMotores(LOW,  HIGH, LOW,  HIGH); }  // Ambos atrás → gira izquierda
void derecha()    { aplicarMotores(HIGH, LOW,  HIGH, LOW);  }  // Ambos adelante → gira derecha
void detener()    { aplicarMotores(LOW,  LOW,  LOW,  LOW);  }

// ─── Lectura de batería ─────────────────────────────────────
float leerVoltajeBateria() {
  // Promedio de 10 lecturas para mayor estabilidad
  long suma = 0;
  for (int i = 0; i < 10; i++) {
    suma += analogRead(BAT_PIN);
    delay(2);
  }
  float promedioADC = suma / 10.0;

  // Convertir lectura ADC a voltaje real de la batería
  float voltajeADC = (promedioADC / ADC_RESOLUTION) * VREF;
  float voltajeBat = voltajeADC * DIVISOR_FACTOR;

  return voltajeBat;
}

int calcularPorcentaje(float voltaje) {
  if (voltaje >= BAT_FULL) return 100;
  if (voltaje <= BAT_EMPTY) return 0;
  return (int)(((voltaje - BAT_EMPTY) / (BAT_FULL - BAT_EMPTY)) * 100.0);
}

void reportarBateria() {
  float voltaje = leerVoltajeBateria();
  int porcentaje = calcularPorcentaje(voltaje);

  // ── Reporte por Serial ──
  Serial.println("──────────────────────────────────");
  Serial.printf("🔋 Batería: %.2f V  |  %d%%\n", voltaje, porcentaje);

  if (porcentaje <= 10) {
    Serial.println("⚠️  ¡BATERÍA CRÍTICA! Recarga pronto.");
  } else if (porcentaje <= 25) {
    Serial.println("🟡 Batería baja.");
  } else {
    Serial.println("🟢 Batería OK.");
  }
  Serial.println("──────────────────────────────────");

  // ── Publicar por MQTT ──
  if (mqtt.connected()) {
    char payload[96];
    snprintf(payload, sizeof(payload),
             "{\"voltaje\":%.2f,\"porcentaje\":%d,\"estado\":\"%s\"}",
             voltaje, porcentaje,
             porcentaje <= 10 ? "critico" : (porcentaje <= 25 ? "bajo" : "ok"));
    mqtt.publish(TOPIC_BATTERY, payload);
  }
}

// ─── Lectura de DHT22 ───────────────────────────────────────
void reportarDHT() {
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();

  if (isnan(temp) || isnan(hum)) {
    Serial.println("[DHT22] ⚠️ Error al leer el sensor.");
    return;
  }

  // Índice de calor
  float hic = dht.computeHeatIndex(temp, hum, false);

  // ── Reporte por Serial ──
  Serial.println("──────────────────────────────────");
  Serial.printf("🌡️  Temperatura: %.1f °C\n", temp);
  Serial.printf("💧 Humedad:     %.1f %%\n", hum);
  Serial.printf("🔥 Sensación:   %.1f °C\n", hic);
  Serial.println("──────────────────────────────────");

  // ── Publicar por MQTT ──
  if (mqtt.connected()) {
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"temperatura\":%.1f,\"humedad\":%.1f,\"sensacion\":%.1f}",
             temp, hum, hic);
    mqtt.publish(TOPIC_DHT, payload);
  }
}

// ─── Lectura de MQ2 (Gas) ───────────────────────────────────
void reportarGas() {
  // Promedio de 5 lecturas
  long suma = 0;
  for (int i = 0; i < 5; i++) {
    suma += analogRead(MQ2_PIN);
    delay(2);
  }
  int valorRaw = suma / 5;

  // Mapear a porcentaje (0-4095 → 0-100%)
  int porcentaje = map(valorRaw, 0, 4095, 0, 100);
  porcentaje = constrain(porcentaje, 0, 100);

  // Determinar estado de alerta
  const char* estado;
  if (porcentaje >= 70) {
    estado = "peligro";
  } else if (porcentaje >= 40) {
    estado = "alerta";
  } else {
    estado = "normal";
  }

  // ── Reporte por Serial ──
  Serial.println("──────────────────────────────────");
  Serial.printf("💨 Gas MQ2: %d raw | %d%% | %s\n", valorRaw, porcentaje, estado);
  Serial.println("──────────────────────────────────");

  // ── Publicar por MQTT ──
  if (mqtt.connected()) {
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"raw\":%d,\"porcentaje\":%d,\"estado\":\"%s\"}",
             valorRaw, porcentaje, estado);
    mqtt.publish(TOPIC_GAS, payload);
  }
}

// ─── Lectura de Ultrasonico HC-SR04 (con ángulo del servo) ────
float leerDistancia() {
  // Enviar pulso de trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Medir duración del eco
  long duracion = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30ms

  // Calcular distancia en cm
  float distancia;
  if (duracion == 0) {
    distancia = MAX_DISTANCE; // Sin eco = fuera de rango
  } else {
    distancia = (duracion * 0.0343) / 2.0;
    if (distancia > MAX_DISTANCE) distancia = MAX_DISTANCE;
  }
  return distancia;
}

void reportarRadar(int angulo, float distancia) {
  // ── Reporte por Serial ──
  Serial.printf("📡 Radar: %d° → %.1f cm\n", angulo, distancia);

  // ── Publicar por MQTT ──
  if (mqtt.connected()) {
    char payload[96];
    snprintf(payload, sizeof(payload),
             "{\"angulo\":%d,\"distancia\":%.1f,\"max\":%.1f}",
             angulo, distancia, MAX_DISTANCE);
    mqtt.publish(TOPIC_ULTRA, payload);
  }
}

// ─── Callback MQTT ──────────────────────────────────────────
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (length == 0) return;

  char cmd = (char)payload[0];
  Serial.printf("[MQTT] Comando recibido: %c\n", cmd);

  switch (cmd) {
    case 'F': adelante();  break;
    case 'B': atras();     break;
    case 'L': izquierda(); break;
    case 'R': derecha();   break;
    case 'S': detener();   break;
    default:
      Serial.printf("[MQTT] Comando desconocido: %c\n", cmd);
      return;
  }

  // Publicar estado de vuelta
  char estado[32];
  snprintf(estado, sizeof(estado), "{\"cmd\":\"%c\",\"ok\":true}", cmd);
  mqtt.publish(TOPIC_STATUS, estado);
}

// ─── Conexión WiFi ──────────────────────────────────────────
void conectarWiFi() {
  Serial.printf("[WiFi] Conectando a %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.printf("\n[WiFi] Conectado — IP: %s\n", WiFi.localIP().toString().c_str());
}

// ─── Conexión MQTT ──────────────────────────────────────────
void conectarMQTT() {
  while (!mqtt.connected()) {
    Serial.printf("[MQTT] Conectando como '%s'...\n", clientId);
    Serial.printf("[MQTT] Broker: %s:%d\n", MQTT_BROKER, MQTT_PORT);
    Serial.printf("[MQTT] User: %s\n", MQTT_USER);
    Serial.printf("[MQTT] Pass length: %d\n", strlen(MQTT_PASS));

    if (mqtt.connect(clientId, MQTT_USER, MQTT_PASS)) {
      Serial.println("[MQTT] ¡Conectado!");
      mqtt.subscribe(TOPIC_CMD);
      Serial.printf("[MQTT] Suscrito a: %s\n", TOPIC_CMD);
      mqtt.publish(TOPIC_STATUS, "{\"status\":\"online\"}");
    } else {
      int rc = mqtt.state();
      Serial.printf("[MQTT] Error rc=%d\n", rc);
      Serial.println("[MQTT] Códigos: -4=timeout, -2=fail, 4=bad_cred, 5=no_auth");
      Serial.println("[MQTT] Reintentando en 5s...");
      delay(5000);
    }
  }
}

// ─── Setup ──────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n========================================");
  Serial.println("  ESP32 Motor Controller — MQTT + TLS  ");
  Serial.println("  DHT22+MQ2+HC-SR04+Servo Radar+Bat    ");
  Serial.println("========================================");

  // Configurar pines de motor como salida
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  detener();

  // Configurar ADC para batería
  analogReadResolution(12);           // 12 bits (0-4095)
  analogSetAttenuation(ADC_11db);     // Rango hasta ~3.3V
  pinMode(BAT_PIN, INPUT);

  // Inicializar DHT22
  dht.begin();
  Serial.println("[DHT22] Sensor inicializado en GPIO26.");

  // Inicializar MQ2 (Gas)
  pinMode(MQ2_PIN, INPUT);
  Serial.println("[MQ2] Sensor de gas inicializado en GPIO25.");

  // Inicializar Ultrasonico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("[HC-SR04] Ultrasonico inicializado (TRIG=32, ECHO=33).");

  // Inicializar Servo Radar
  servoRadar.attach(SERVO_PIN, 500, 2400); // pulso min/max en µs
  servoRadar.write(0);                     // posición inicial 0°
  servoAngle = 0;
  servoDirection = 1;
  Serial.println("[SERVO] Radar servo inicializado en GPIO34 (0-180°).");

  // Lectura inicial de batería
  Serial.println("[BAT] Leyendo nivel de batería inicial...");
  reportarBateria();

  // Conectar WiFi
  conectarWiFi();

  // Generar Client ID único
  snprintf(clientId, sizeof(clientId), "ESP32_%04X", (uint16_t)random(0xFFFF));
  Serial.printf("[MQTT] Client ID: %s\n", clientId);

  // Configurar TLS — usar setInsecure() para evitar problemas de certificado
  // espClient.setCACert(root_ca);   // Comentado para debug
  espClient.setInsecure();            // Acepta cualquier certificado

  // Configurar MQTT
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setBufferSize(1024);
  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(10);
  mqtt.setCallback(mqttCallback);

  conectarMQTT();
}

// ─── Loop ───────────────────────────────────────────────────
void loop() {
  // Reconectar si se pierde la conexión
  if (WiFi.status() != WL_CONNECTED) {
    conectarWiFi();
  }

  if (!mqtt.connected()) {
    detener();  // Seguridad: parar motores si se pierde conexión
    conectarMQTT();
  }

  mqtt.loop();

  // ── Lecturas periódicas ──
  unsigned long ahora = millis();

  // Servo Radar: barrido continuo 0°→1800°→0° con lectura ultrasónica
  if (ahora - lastServoStep >= SERVO_DELAY) {
    lastServoStep = ahora;

    // Mover servo al ángulo actual
    servoRadar.write(servoAngle);
    delay(15);  // breve pausa para que el servo se estabilice

    // Leer distancia en esta posición
    float dist = leerDistancia();
    reportarRadar(servoAngle, dist);

    // Avanzar al siguiente ángulo
    servoAngle += SERVO_STEP * servoDirection;

    // Invertir dirección en los límites
    if (servoAngle >= 180) {
      servoAngle = 180;
      servoDirection = -1;
    } else if (servoAngle <= 0) {
      servoAngle = 0;
      servoDirection = 1;
    }
  }

  // Gas MQ2 cada 3 segundos
  if (ahora - lastGasRead >= GAS_INTERVAL) {
    lastGasRead = ahora;
    reportarGas();
  }

  // DHT22 cada 5 segundos
  if (ahora - lastDhtRead >= DHT_INTERVAL) {
    lastDhtRead = ahora;
    reportarDHT();
  }

  // Batería cada 10 segundos
  if (ahora - lastBatRead >= BAT_INTERVAL) {
    lastBatRead = ahora;
    reportarBateria();
  }
}
