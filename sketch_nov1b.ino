// ============================
// 📘 Includes
// ============================
#include <Arduino.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <time.h>
#include "esp_mesh.h" 
#include "esp_log.h"
#include <HardwareSerial.h>
#include <esp_task_wdt.h>
#include <WiFiClientSecure.h>
#include <zoho-iot-client.h>
#include "certificate.h"
#include "esp_system.h"
#include <DHT.h>
#include <vector>
#include <algorithm>
#include <driver/adc.h>
#include <driver/dac.h>
#include <set>

// ============================
// 📘 OTA Includes (Moved after Zoho)
// ============================
#include <HTTPClient.h>
#include <HTTPUpdate.h>

// ============================
// 🔧 Debug Configuration
// ============================
#define SERIAL_DEBUG  // Comment this line to disable all serial prints

std::set<String> processedCommands; // Tracks executed commands

// ============================
// ⚙ Constants & Configuration
// ============================

// Configuration Mode
const char* CONFIG_AP_SSID = "WaterSystem-Config";
const char* CONFIG_AP_PASSWORD = "configure123";
#define CONFIG_MODE_DURATION 900000  // 15 minutes

// Web Server
WebServer server(80);

// Configuration structure
struct DeviceConfig {
  char wifiSSID[32];
  char wifiPassword[32];
  char mqttUsername[64];
  char mqttPassword[64];
  bool configured;
};

// WiFi Connection Timeout
#define WIFI_CONNECT_TIMEOUT       20000   // 20 seconds
#define WIFI_RETRY_INTERVAL        30000     // 3 minutes

// Mesh Network
#define CONFIG_MESH_MAX_LAYER      6
const String mesh_ssid            = "AK";
const String mesh_password        = "9016455633";
const uint8_t mesh_id[6]          = { 0x78, 0x42, 0x1C, 0xA2, 0xBF, 0x9C };

// ============================
// ⚙ OTA Configuration
// ============================
#define FIRMWARE_VERSION "1.0.1"
#define VERSION_FILE_URL "https://raw.githubusercontent.com/akshit-a11y/water-system-firmware/main/version.txt"
#define GITHUB_REPO "akshit-a11y/water-system-firmware"
#define OTA_CHECK_INTERVAL  300000 // 24 hours in milliseconds

// Time
#define NTP_SERVER                 "pool.ntp.org"
#define GMT_OFFSET_SEC             19800   // IST (UTC+5:30)
#define DAYLIGHT_OFFSET_SEC        0

// Timing & Sampling
#define SAMPLING_INTERVAL          20
#define WDT_TIMEOUT                120000   // Watchdog Timeout
const long SENSOR_READ_INTERVAL   = 10000;
const int THRESHOLD_COUNT         = 5;

// ADC Configuration
const float adcRefVoltage         = 3.3;
const int adcResolution           = 4096;

// pH Sensor
#define PH_SENSOR_PIN              34
#define PH_OFFSET                  0.00  
#define PH_OFFSET_VOLTAGE          2.5
#define PH_SLOPE                   0.18
#define PH_ARRAY_LENGTH            40
#define VREF                       5.0

// Pressure Sensor
#define PRESSURE_SENSOR_PIN        35
#define DAC_OUTPUT_PIN 25 // DAC1 to VFD analog input
uint8_t target_mac[] = { 0x78, 0x42, 0x1c, 0xa2, 0xbf, 0x9c };
const float resistor = 165.0;
const float adcRef = 3.3;
const float sensorMinCurrent = 0.004;  // 4 mA0
const float sensorMaxCurrent = 0.020;  // 20 mA
const float pressureMinBar = 0.0;
const float pressureMaxBar = 10.0;
unsigned long lastPressureSendTime = 0;
const unsigned long PRESSURE_SEND_INTERVAL = 5000; // Send every 5 seconds
float setpoint = 1.3; 
int dacValue = 0;

// TDS Sensor
#define TDS_SENSOR_PIN             36
#define SAMPLE_SIZE                30

// EEPROM
#define EEPROM_SIZE                1024  // Increased for sensor thresholds
#define EEPROM_CONFIG_ADDR         0
#define EEPROM_MIN_ADDR            100
#define EEPROM_MAX_ADDR            104
#define EEPROM_STATE_ADDR          108
#define EEPROM_AUTOMATION_CONFIG_ADDR 200  
#define EEPROM_SENSOR_THRESHOLDS_ADDR 300  // For sensor thresholds

// Offline Data Buffer
#define MAX_BUFFERED_ENTRIES       50

// ========================
// 🔧 Flow Sensor Constants
// ========================
#define FLOW_SENSOR_PIN 19
const float pulsesPerLiter = 2200.0;
volatile uint32_t pulseCount = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 1000;
volatile unsigned long lastPulseTime = 0;
const int SMOOTHING_WINDOW = 10;
float totalLiters = 0;

float flowRates[SMOOTHING_WINDOW] = {0};
int currentIndex = 0;

struct FlowData {
  float flow_rate;  // LPM
  float total_liters;
  unsigned long read_time; // Time taken to read flow sensor
};

void IRAM_ATTR countPulse() {
  static unsigned long lastTime = 0;
  pulseCount++;
}

// DHT Sensor
#define DHTPIN                     23
#define DHTTYPE                    DHT22

// Serial Communication
#define TRIG_PIN 21
#define ECHO_PIN 22 

// Output Pins
#define IP                    14
#define IV                    27
#define sv_1                  32
#define SV_4                  33
#define DP_2                  18
#define DP_1                  26
#define SV_2                  12   
#define SV_3                  13    
#define sv_8                  16  

// Power Management
#define GRID_POWER_PIN 37
#define GENERATOR_POWER_PIN 38
#define GRID_RELAY_PIN 15
#define GENERATOR_RELAY_PIN 4
#define VOLTAGE_THRESHOLD 200.0
#define CHECK_INTERVAL 5000
#define GRID_CUTOFF_VOLTAGE 180.0
#define ADC_MIN_THRESHOLD 1940
#define VOLTAGE_MIN_THRESHOLD 180.0

// ============================
// ⏰ Automation Configuration
// ============================
#define AUTOMATION_START_HOUR      8     // 8 AM start (default)
#define AUTOMATION_END_HOUR        18    // 6 PM end (default)
#define DEFAULT_FILL_LEVEL         10.0  // Default fill level in liters
#define DEFAULT_IV_TIMEOUT         (10 * 60 * 1000) // 10 minutes default IV timeout
#define DEFAULT_DP_DURATION        (5 * 60 * 1000)  // 5 minutes for DP1+DP2
#define DEFAULT_IP_SV1_DURATION    (5 * 60 * 1000)  // 5 minutes for IP+SV1
#define DEFAULT_IP_SV234_DURATION  (10 * 60 * 1000) // 10 minutes for IP+SV2,3,4
#define DEFAULT_VALVE1_TARGET      25.0  // liters
#define DEFAULT_VALVE2_TARGET      35.0  // liters  
#define DEFAULT_VALVE3_TARGET      40.0  // liters

// Automation states
#define AUTOMATION_IDLE 0
#define AUTOMATION_DRAINING 1
#define AUTOMATION_FILLING 2
#define AUTOMATION_DP_OPERATION 3
#define AUTOMATION_IP_SV1_OPERATION 4
#define AUTOMATION_IP_SV234_OPERATION 5
#define AUTOMATION_COMPLETE 6
#define AUTOMATION_ERROR 7

// Automation configuration from Zoho (with defaults)
struct AutomationConfig {
  int startHour = AUTOMATION_START_HOUR;
  int endHour = AUTOMATION_END_HOUR;
  float targetFillLevel = DEFAULT_FILL_LEVEL;
  unsigned long ivTimeout = DEFAULT_IV_TIMEOUT;
  unsigned long dpDuration = DEFAULT_DP_DURATION;
  unsigned long ipSv1Duration = DEFAULT_IP_SV1_DURATION;
  unsigned long ipSv234Duration = DEFAULT_IP_SV234_DURATION;
  bool automationEnabled = true;
  
  // Valve consumption targets
  float valve1Target = DEFAULT_VALVE1_TARGET;
  float valve2Target = DEFAULT_VALVE2_TARGET;
  float valve3Target = DEFAULT_VALVE3_TARGET;
};

// Sensor Thresholds structure
struct SensorThresholds {
  float min_temp = 0.0;
  float max_temp = 50.0;
  float min_hum = 0.0;
  float max_hum = 90.0;
  float min_level = 0.0;
  float max_level = 125.0;
  float min_pressure = 0.0;
  float max_pressure = 10.0;
  float min_flow = 0.0;
  float max_flow = 100.0;
  float min_ph = 0.0;
  float max_ph = 14.0;
  float min_tds = 0.0;
  float max_tds = 2000.0;
};

AutomationConfig automationConfig;
DeviceConfig deviceConfig;
SensorThresholds sensorThresholds;
 
bool configMode = true;
unsigned long configModeStartTime = 0;

// Add valve tracking structure
struct ValveConsumption {
  float valve1Consumed = 0.0;
  float valve2Consumed = 0.0;
  float valve3Consumed = 0.0;
  float initialLevel = 0.0;
  bool valve1Active = true;
  bool valve2Active = true;
  bool valve3Active = true;
};

ValveConsumption valveTracking;

// ============================
// 📦 Global Objects & Variables
// ============================

DHT dht(DHTPIN, DHTTYPE);
WiFiClientSecure espClient;
ZohoIOTClient zClient(&espClient, true);
ZohoIOTClient::commandAckResponseCodes success_response_code = ZohoIOTClient::SUCCESSFULLY_EXECUTED;

int phArray[PH_ARRAY_LENGTH];
int phArrayIndex = 0;
volatile int flow_pulse_count = 0;

float V, P, udistance;
float min_set_level = 0;
float max_set_level = 0;
int lowLevelCounter = 0;
int highLevelCounter = 0;
bool lastActionWasFilling;
bool hasFilledAfterLow = false;
bool timeSynced = false;
bool ledState = false;
String IP_status;
unsigned long lastWiFiAttempt = 0;
unsigned long lastSendTime = 0;
unsigned long lastDrainCycleTime = 0;
unsigned long belowMinTime = 0;
unsigned long prev_time = 0, current_time = 0;
unsigned long prev_time_mesh = 0;
unsigned char udata[4] = {};
float Kp = 1200.0, Ki = 0.2, Kd = 0;
float inte = 0.0, lastError = 0.0;
unsigned long lastTime = 0;
bool firstRun = true;

unsigned long ivAutoOffTime = 0;

mesh_cfg_t mesh;
mesh_cfg_t mesh_config;

esp_task_wdt_config_t twdt_config = {
  .timeout_ms = WDT_TIMEOUT,
  .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,
  .trigger_panic = true,
};


int currentAutomationState = AUTOMATION_IDLE;
unsigned long automationStartTime = 0;
unsigned long stepStartTime = 0;
bool automationRunning = false;
int currentCycleHour = -1;

// Power Management Variables
unsigned long lastPowerCheck = 0;
bool generatorActive = false;
float gridVoltage = 0;
float generatorVoltage = 0;

// Global variables to store mesh data
float o_temp = 0.0;
float o_hum = 0.0;
float level =  0.0;

// Sensor timing variables
struct SensorTiming {
  unsigned long dht_time = 0;
  unsigned long ultrasonic_time = 0;
  unsigned long ph_time = 0;
  unsigned long pressure_time = 0;
  unsigned long tds_time = 0;
  unsigned long flow_time = 0;
  unsigned long voltage_time = 0;
  unsigned long total_sensor_time = 0;
};

SensorTiming sensorTiming;

struct SensorData {
  float i_temp;
  float i_hum;
  float o_temp;
  float o_hum;
  float level;
  float ph;
  float tds;
  float flow_rate;
  float total_liters;
  float pressure;
  float grid_voltage;
  int   dac_value;
  String IP_status;
  unsigned long timestamp;
  SensorTiming timing; // Add timing information
};

// ============================
// 🕒 RTC Time Management
// ============================
struct RTCTime {
  int year = 2024;
  int month = 1;
  int day = 1;
  int hour = 0;
  int minute = 0;
  int second = 0;
  unsigned long lastMillis = 0;
  bool ntpSynced = false;
};

RTCTime rtcTime;

// ============================
// 📡 Mesh Child Node Tracking
// ============================
struct ChildNode {
  String nodeName;
  uint8_t mac[6];
  bool connected;
  unsigned long lastSeen;
};


std::vector<SensorData> offlineBuffer;
std::vector<ChildNode> childNodes;
const unsigned long CHILD_TIMEOUT = 60000; // 1 minute timeout

// ============================
// 📝 Forward Declarations
// ============================
void setupWebServer();
void handleRoot();
void handleSaveConfig();
void handleReset();
void handleStatus();
void loadConfig();
void saveConfig();
void resetConfig();
void startConfigMode();
void setupMainSystem();
void TaskWDT_Core0(void *pvParameters);
float readACVoltage(int pin);
void switchToGridPower();
void switchToGeneratorPower();
void managePowerSupply();
bool setupWiFi();
void checkHeap();
void setupTime();
void send_sensor_data();
float read_ultrasonic_distance();
float read_ph_sensor();
FlowData read_flow_sensor();
float read_pressure_sensor();
float read_tds_sensor(float temperature);
int getMedianNum(int bArray[], int iFilterLen);
double averageArray(int *arr, int number);
void print_mesh_status();
void on_message(char *topic, uint8_t *payload, unsigned int length);
void sendMessage(const char *message, const uint8_t *target_mac);
void reconnectWiFi();
void receive_sensor_data();
bool isMeshRunning();
void printSensorTiming(const SensorTiming& timing);
void check_scheduled_automation();
void execute_automation_sequence();
void saveAutomationConfig();
void loadAutomationConfig();
void saveSensorThresholds();
void loadSensorThresholds();
void check_sensor_thresholds(float temp, float hum, float level, float pressure, float flow, float ph, float tds);
void send_automation_alarm(const char* message);
void start_automation_cycle(int hour);
void stop_automation();
void checkConfigModeTimeout();
void wifiManagement();
void sendMeshCommand(const String& valveName, bool state);
void setupMeshNetwork();
void printMeshStatus();
bool isValidMAC(const uint8_t* mac);
void setupChildNodes();
void updateChildNode(const uint8_t* mac, const String& message);
void monitorChildNodes();
void printChildNodeStatus();

// ============================
// 🚀 Setup (Fixed)
// ============================

void setup() {
  Serial.flush();
  Serial.end();
  delay(100);
  Serial.begin(115200);
  delay(100);

#ifdef SERIAL_DEBUG
  Serial.println("🔧 Starting Water System...");
#endif

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load configuration
  loadConfig();
  
  // Load automation configuration
  loadAutomationConfig();

  // Load sensor thresholds
  loadSensorThresholds();

  // Check if device is properly configured
  bool hasValidConfig = deviceConfig.configured && 
                       strlen(deviceConfig.wifiSSID) > 0 && 
                       strlen(deviceConfig.mqttUsername) > 0;
  
  if (hasValidConfig) {
#ifdef SERIAL_DEBUG
    Serial.println("✅ Device is configured, trying to connect to WiFi...");
    Serial.print("📶 WiFi: ");
    Serial.println(deviceConfig.wifiSSID);
#endif
    configMode = false;
    
    // Try to connect to WiFi with timeout (20 seconds)
    unsigned long wifiStartTime = millis();
    bool wifiConnected = false;
    
    while (millis() - wifiStartTime < WIFI_CONNECT_TIMEOUT) {
      if (setupWiFi()) {
        wifiConnected = true;
        break;
      }
      delay(1000);
#ifdef SERIAL_DEBUG
      Serial.print(".");
#endif
    }
    
    if (wifiConnected) {
      setupMainSystem();
    } else {
#ifdef SERIAL_DEBUG
      Serial.println("\n❌ WiFi connection failed within 20 seconds - Starting Configuration Mode");
#endif
      // 🔄 ENTER CONFIG MODE AFTER 20 SECONDS FAILURE
      startConfigMode();
    }
  } else {
    // No valid configuration found, start in AP mode immediately
#ifdef SERIAL_DEBUG
    Serial.println("📡 No configuration found - Starting Configuration Mode");
#endif
    configMode = true;
    configModeStartTime = millis();
    startConfigMode();
  }
}

void setupMainSystem() {
#ifdef SERIAL_DEBUG
  Serial.println("🚀 Setting up main system...");
#endif

  // 🔧 CRITICAL FIX: Set WiFi mode to AP+STA for Mesh + WiFi coexistence
  WiFi.mode(WIFI_AP_STA);
#ifdef SERIAL_DEBUG
  Serial.println("📡 WiFi Mode: AP+STA (for Mesh + WiFi coexistence)");
#endif

  // Ensure WiFi is in a clean state before starting
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);
    delay(1000);
  }

  esp_task_wdt_deinit();
  delay(1000);
  esp_task_wdt_init(&twdt_config);
  esp_task_wdt_add(NULL);

  xTaskCreatePinnedToCore(TaskWDT_Core0, "Core0_WDT_Task", 2048, NULL, 1, NULL, 0);

  setupTime();

  espClient.setCACert(root_ca);
  zClient.init(deviceConfig.mqttUsername, deviceConfig.mqttPassword);
  zClient.connect();
  zClient.subscribe(on_message);

  EEPROM.get(EEPROM_MIN_ADDR, min_set_level);
  EEPROM.get(EEPROM_MAX_ADDR, max_set_level);
  EEPROM.get(EEPROM_STATE_ADDR, lastActionWasFilling);

  dht.begin();

  // 🔧 FIXED GPIO CONFIGURATION
  // Configure OUTPUT pins
  pinMode(DP_1, OUTPUT); digitalWrite(DP_1, LOW);
  esp_task_wdt_reset();
  pinMode(IV, OUTPUT); digitalWrite(IV, LOW);
  pinMode(sv_1, OUTPUT); digitalWrite(sv_1, LOW);
  pinMode(SV_4, OUTPUT); digitalWrite(SV_4, LOW);
  pinMode(DP_2, OUTPUT); digitalWrite(DP_2, LOW);
  pinMode(IP, OUTPUT); digitalWrite(IP, LOW);
  // Note: SV_2, SV_3 are now controlled via mesh network
  pinMode(sv_8, OUTPUT); digitalWrite(sv_8, LOW);
  pinMode(GRID_RELAY_PIN, OUTPUT);
  pinMode(GENERATOR_RELAY_PIN, OUTPUT);
  
  // Configure INPUT pins WITHOUT pull-up (for input-only pins)
  pinMode(DHTPIN, INPUT);  // DHT22 - input only, no pull-up
  pinMode(FLOW_SENSOR_PIN, INPUT);  // Flow sensor - input only
  
  // Configure TRIG/ECHO pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);  // Input only, no pull-up
  
  // Configure ADC pins WITHOUT pull-up
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
  pinMode(TDS_SENSOR_PIN, INPUT);
  pinMode(PH_SENSOR_PIN, INPUT);
  
  // Power monitoring inputs
  pinMode(GRID_POWER_PIN, INPUT);
  pinMode(GENERATOR_POWER_PIN, INPUT);

  // Initialize Serial2 for ultrasonic
  Serial2.begin(9600, SERIAL_8N1, ECHO_PIN, TRIG_PIN);
  
  // Configure ADC attenuation
  analogSetPinAttenuation(PRESSURE_SENSOR_PIN, ADC_11db);
  
  // Attach interrupt for flow sensor
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), countPulse, FALLING);

  // Initialize relay states
  digitalWrite(GRID_RELAY_PIN, LOW);
  digitalWrite(GENERATOR_RELAY_PIN, LOW);

  // Setup mesh network
  setupMeshNetwork();

  // ✅ Initialize child node tracking
  setupChildNodes();

  // Initial power check
  managePowerSupply();

#ifdef SERIAL_DEBUG
  Serial.println("✅ System Ready!");
#endif

  WiFi.reconnect();
}

void setupMeshNetwork() {
#ifdef SERIAL_DEBUG
  Serial.println("🔄 Initializing Mesh Network with WiFi coexistence...");
#endif

  // First, ensure WiFi is properly configured in AP+STA mode
  if (WiFi.getMode() != WIFI_AP_STA) {
    WiFi.mode(WIFI_AP_STA);
#ifdef SERIAL_DEBUG
    Serial.println("📡 Reconfigured WiFi to AP+STA mode for Mesh");
#endif
  }

  esp_err_t err = esp_mesh_init();
  if (err != ESP_OK) {
      Serial.printf("\n❌ Mesh initialization failed: %s\n", esp_err_to_name(err));
      return;
  }

  mesh_config = MESH_INIT_CONFIG_DEFAULT();

  esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER);
  esp_mesh_fix_root(true);

  strlcpy((char*)mesh_config.router.ssid, mesh_ssid.c_str(), sizeof(mesh_config.router.ssid));
  mesh_config.router.ssid_len = mesh_ssid.length();
  strlcpy((char*)mesh_config.mesh_ap.password, mesh_password.c_str(), sizeof(mesh_config.mesh_ap.password));
  memcpy(mesh_config.mesh_id.addr, mesh_id, 6);

  mesh_config.router.allow_router_switch = false;
  mesh_config.channel = 1;
  mesh_config.allow_channel_switch = false;
  mesh_config.mesh_ap.max_connection = 3;
  mesh_config.mesh_ap.nonmesh_max_connection = 0;

  err = esp_mesh_set_config(&mesh_config);
  if (err != ESP_OK) {
      Serial.printf("\n❌ Failed to set mesh config: %s\n", esp_err_to_name(err));
      return;
  }

  err = esp_mesh_start();
  if (err != ESP_OK) {
      Serial.printf("\n❌ Mesh start failed: %s\n", esp_err_to_name(err));
  } else {
      Serial.println("✅ Mesh started successfully");
  }

  err = esp_mesh_set_type(MESH_ROOT);
  delay(3000);
  if (err != ESP_OK) {
      Serial.println("❌ Failed to set mesh type!");
      return;
  } else {
#ifdef SERIAL_DEBUG
      Serial.println("✅ Mesh Set as Root Node");
#endif
  }
}

bool isMeshRunning() {
  // Check if mesh is initialized and running
  int layer = esp_mesh_get_layer();
  return (layer >= 0);
}

// ============================
// 🔄 OTA Functions (Autonomous Only)
// ============================

void performRemoteOTA(String firmwareURL) {
#ifdef SERIAL_DEBUG
  Serial.println("🚀 Starting GitHub OTA Update...");
  Serial.println("📥 Downloading from: " + firmwareURL);
#endif

  // Stop all critical operations
  digitalWrite(IP, LOW);
  digitalWrite(IV, LOW);
  digitalWrite(DP_1, LOW);
  digitalWrite(DP_2, LOW);
  digitalWrite(sv_1, LOW);
  digitalWrite(SV_4, LOW);
  digitalWrite(sv_8, LOW);
  
  // Turn off mesh valves
  sendMeshCommand("SV_2", false);
  sendMeshCommand("SV_3", false);
  sendMeshCommand("SV_4", false);
  
  // Disconnect from Zoho
  zClient.disconnect();
  
  delay(1000);

  WiFiClient client;
  
  // Perform update
  t_httpUpdate_return ret = httpUpdate.update(client, firmwareURL);
  
  switch(ret) {
    case HTTP_UPDATE_OK:
#ifdef SERIAL_DEBUG
      Serial.println("✅ Update successful - Rebooting");
#endif
      ESP.restart();
      break;
      
    case HTTP_UPDATE_FAILED:
#ifdef SERIAL_DEBUG
      Serial.printf("❌ Update failed: %s\n", httpUpdate.getLastErrorString().c_str());
#endif
      // Reconnect to Zoho after failed update
      zClient.connect();
      break;
      
    case HTTP_UPDATE_NO_UPDATES:
#ifdef SERIAL_DEBUG
      Serial.println("ℹ️ No updates available");
#endif
      break;
  }
}

bool verifyFirmwareURL(String url) {
  // Basic security check - verify URL is from GitHub
  if (url.indexOf("https://github.com/") != 0) {
#ifdef SERIAL_DEBUG
    Serial.println("❌ OTA Security: URL not from GitHub");
#endif
    return false;
  }
  
  if (url.indexOf("/releases/download/") == -1) {
#ifdef SERIAL_DEBUG
    Serial.println("❌ OTA Security: Not a valid GitHub releases URL");
#endif
    return false;
  }
  
  // Verify it's from your repo
  if (url.indexOf(GITHUB_REPO) == -1) {
#ifdef SERIAL_DEBUG
    Serial.println("❌ OTA Security: URL not from authorized repository");
#endif
    return false;
  }
  
  return true;
}

void checkGitHubForUpdates() {
#ifdef SERIAL_DEBUG
  Serial.println("🔍 Checking GitHub for updates...");
  Serial.println("📋 Current Version: " + String(FIRMWARE_VERSION));
#endif

  if (WiFi.status() != WL_CONNECTED) {
#ifdef SERIAL_DEBUG
    Serial.println("📶 No WiFi connection - skipping update check");
#endif
    return;
  }

  HTTPClient http;
  http.begin(VERSION_FILE_URL);
  
  int httpCode = http.GET();
  
  if (httpCode == 200) {
    String latestVersion = http.getString();
    latestVersion.trim();
    
#ifdef SERIAL_DEBUG
    Serial.println("📦 Latest Version: " + latestVersion);
#endif

    if (latestVersion != FIRMWARE_VERSION) {
#ifdef SERIAL_DEBUG
      Serial.println("🎯 New version available! Starting update...");
#endif
      
      String downloadURL = "https://github.com/" + String(GITHUB_REPO) + "/releases/download/" + latestVersion + "/firmware.bin";
      
      if (verifyFirmwareURL(downloadURL)) {
        performRemoteOTA(downloadURL);
      } else {
#ifdef SERIAL_DEBUG
        Serial.println("❌ URL verification failed - aborting update");
#endif
      }
    } else {
#ifdef SERIAL_DEBUG
      Serial.println("✅ Already running latest version");
#endif
    }
  } else {
#ifdef SERIAL_DEBUG
    Serial.println("❌ Failed to check version: HTTP " + String(httpCode));
#endif
  }
  
  http.end();
}

void printMeshStatus() {
#ifdef SERIAL_DEBUG
  int layer = esp_mesh_get_layer();
  mesh_addr_t parent_bssid;
  esp_err_t parent_err = esp_mesh_get_parent_bssid(&parent_bssid);
  
  if (layer >= 0) {
    Serial.printf("📡 Mesh Status: Running\n");
    Serial.printf("📊 Layer: %d\n", layer);
    
    if (parent_err == ESP_OK) {
      Serial.print("👥 Connected to parent: ");
      for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", parent_bssid.addr[i]);
        if (i < 5) Serial.print(":");
      }
      Serial.println();
    } else {
      Serial.println("👥 No parent connection (Root node)");
    }
  } else {
    Serial.println("❌ Mesh not running");
  }
#endif
}

bool isValidMAC(const uint8_t* mac) {
  for (int i = 0; i < 6; i++) {
    if (mac[i] != 0x00 && mac[i] != 0xFF) {
      return true;
    }
  }
  return false;
}

void sendMessage(const char *message, const uint8_t *target_mac) {
  // Check if mesh is running first
  if (!isMeshRunning()) {
#ifdef SERIAL_DEBUG
    Serial.println("❌ Mesh not running - cannot send message");
#endif
    return;
  }

  // Validate MAC address
  if (!isValidMAC(target_mac)) {
#ifdef SERIAL_DEBUG
    Serial.println("❌ Invalid target MAC address");
#endif
    return;
  }

  mesh_data_t data;
  data.data = (uint8_t *)message;
  data.size = strlen(message) + 1;
  data.proto = MESH_PROTO_BIN;
  data.tos = MESH_TOS_P2P;

  mesh_addr_t target_addr;
  memcpy(target_addr.addr, target_mac, 6);

  // Try to send with retry mechanism
  int retries = 3;
  for (int i = 0; i < retries; i++) {
    esp_err_t err = esp_mesh_send(&target_addr, &data, MESH_DATA_P2P, NULL, 0);
    
    if (err == ESP_OK) {
#ifdef SERIAL_DEBUG
      Serial.printf("✅ Message sent successfully to ");
      for (int j = 0; j < 6; j++) {
        Serial.printf("%02X", target_mac[j]);
        if (j < 5) Serial.print(":");
      }
      Serial.println();
#endif
      return;
    } else if (err == ESP_ERR_MESH_NO_ROUTE_FOUND) {
#ifdef SERIAL_DEBUG
      Serial.printf("🔄 No route found (attempt %d/%d), retrying...\n", i + 1, retries);
#endif
      delay(500); // Wait before retry
    } else {
#ifdef SERIAL_DEBUG
      Serial.printf("❌ Send error: %s\n", esp_err_to_name(err));
#endif
      break;
    }
  }

#ifdef SERIAL_DEBUG
  Serial.println("❌ Failed to send message after retries");
#endif
}

void sendMeshCommand(const String& valveName, bool state) {
  String message = valveName + (state ? " ON" : " OFF");
  
#ifdef SERIAL_DEBUG
  Serial.printf("📡 Looking for child node: %s\n", valveName.c_str());
#endif

  // Find the correct child node by name
  uint8_t* target_mac = nullptr;
  String nodeName = "";
  
  for (auto& child : childNodes) {
    if (child.nodeName == valveName && child.connected) {
      target_mac = child.mac;
      nodeName = child.nodeName;
      break;
    }
  }

  if (target_mac == nullptr) {
#ifdef SERIAL_DEBUG
    Serial.printf("❌ No connected child found for: %s\n", valveName.c_str());
    
    // Print available children
    Serial.println("📋 Available child nodes:");
    for (auto& child : childNodes) {
      Serial.printf("  %s: %s (", child.nodeName.c_str(), child.connected ? "Connected" : "Disconnected");
      for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", child.mac[i]);
        if (i < 5) Serial.print(":");
      }
      Serial.println(")");
    }
#endif
    return;
  }

#ifdef SERIAL_DEBUG
  Serial.printf("🎯 Sending to %s (", nodeName.c_str());
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", target_mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.printf("): %s\n", message.c_str());
#endif

  // Send the message
  sendMessage(message.c_str(), target_mac);
  delay(200);
}

void setupChildNodes() {
  // Initialize known child nodes (you'll update these with actual MACs)
  ChildNode sv2 = {"SV_2", {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, false, 0};
  ChildNode sv3 = {"SV_3", {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, false, 0};
  ChildNode sv4 = {"SV_4", {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, false, 0};
  
  childNodes.push_back(sv2);
  childNodes.push_back(sv3);
  childNodes.push_back(sv4);
}

void updateChildNode(const uint8_t* mac, const String& message) {
  for (auto& child : childNodes) {
    if (memcmp(child.mac, mac, 6) == 0) {
      child.connected = true;
      child.lastSeen = millis();
#ifdef SERIAL_DEBUG
      Serial.printf("✅ %s is connected (MAC: ", child.nodeName.c_str());
      for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", mac[i]);
        if (i < 5) Serial.print(":");
      }
      Serial.println(")");
#endif
      return;
    }
  }
  
  // New child detected - try to identify from message
  if (message.indexOf("HELLO from SV_2") != -1) {
    memcpy(childNodes[0].mac, mac, 6);
    childNodes[0].connected = true;
    childNodes[0].lastSeen = millis();
#ifdef SERIAL_DEBUG
    Serial.println("🎯 Identified new child: SV_2");
#endif
  } else if (message.indexOf("HELLO from SV_3") != -1) {
    memcpy(childNodes[1].mac, mac, 6);
    childNodes[1].connected = true;
    childNodes[1].lastSeen = millis();
#ifdef SERIAL_DEBUG
    Serial.println("🎯 Identified new child: SV_3");
#endif
  } else if (message.indexOf("HELLO from SV_4") != -1) {
    memcpy(childNodes[2].mac, mac, 6);
    childNodes[2].connected = true;
    childNodes[2].lastSeen = millis();
#ifdef SERIAL_DEBUG
    Serial.println("🎯 Identified new child: SV_4");
#endif
  } else {
    // Unknown child
#ifdef SERIAL_DEBUG
    Serial.printf("🔍 Unknown child detected: ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", mac[i]);
      if (i < 5) Serial.print(":");
    }
    Serial.printf(" - Message: %s\n", message.c_str());
#endif
  }
}

void monitorChildNodes() {
  static unsigned long lastMonitorTime = 0;
  if (millis() - lastMonitorTime > 30000) { // Every 30 seconds
    lastMonitorTime = millis();

    bool anyConnected = false;

    for (auto& child : childNodes) {
      // Check for timeout
      if (child.connected && (millis() - child.lastSeen > CHILD_TIMEOUT)) {
        child.connected = false;

        // 🔥 Clear stored MAC address when disconnected
        memset(child.mac, 0, 6);

#ifdef SERIAL_DEBUG
        Serial.printf("⚠️ %s disconnected (timeout) — MAC cleared\n", child.nodeName.c_str());
#endif
      }

      if (child.connected) {
        anyConnected = true;
      }
    }

    if (!anyConnected) {
#ifdef SERIAL_DEBUG
      Serial.println("🔍 No child nodes connected - scanning...");
#endif
    }

    // Print child node status
    printChildNodeStatus();
  }
}



// Initialize RTC with compile time (approximate)
void initializeRTC() {
  // Get compile time (this is approximate but better than nothing)
  String compileTime = __TIME__;
  String compileDate = __DATE__;
  
  // Parse compile time (HH:MM:SS)
  int compileHour = compileTime.substring(0, 2).toInt();
  int compileMinute = compileTime.substring(3, 5).toInt();
  int compileSecond = compileTime.substring(6, 8).toInt();
  
  // Parse compile date (MMM DD YYYY)
  String monthStr = compileDate.substring(0, 3);
  int compileDay = compileDate.substring(4, 6).toInt();
  int compileYear = compileDate.substring(9, 13).toInt();
  
  // Convert month string to number
  int monthNumbers[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
  int compileMonth = 1;
  if (monthStr == "Jan") compileMonth = 1;
  else if (monthStr == "Feb") compileMonth = 2;
  else if (monthStr == "Mar") compileMonth = 3;
  else if (monthStr == "Apr") compileMonth = 4;
  else if (monthStr == "May") compileMonth = 5;
  else if (monthStr == "Jun") compileMonth = 6;
  else if (monthStr == "Jul") compileMonth = 7;
  else if (monthStr == "Aug") compileMonth = 8;
  else if (monthStr == "Sep") compileMonth = 9;
  else if (monthStr == "Oct") compileMonth = 10;
  else if (monthStr == "Nov") compileMonth = 11;
  else if (monthStr == "Dec") compileMonth = 12;
  
  rtcTime.year = compileYear;
  rtcTime.month = compileMonth;
  rtcTime.day = compileDay;
  rtcTime.hour = compileHour;
  rtcTime.minute = compileMinute;
  rtcTime.second = compileSecond;
  rtcTime.lastMillis = millis();
  rtcTime.ntpSynced = false;
  
#ifdef SERIAL_DEBUG
  Serial.printf("🕒 RTC initialized with compile time: %04d-%02d-%02d %02d:%02d:%02d\n",
                rtcTime.year, rtcTime.month, rtcTime.day,
                rtcTime.hour, rtcTime.minute, rtcTime.second);
#endif
}

// Update RTC time from NTP
void updateRTCFromNTP() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    rtcTime.year = timeinfo.tm_year + 1900;
    rtcTime.month = timeinfo.tm_mon + 1;
    rtcTime.day = timeinfo.tm_mday;
    rtcTime.hour = timeinfo.tm_hour;
    rtcTime.minute = timeinfo.tm_min;
    rtcTime.second = timeinfo.tm_sec;
    rtcTime.lastMillis = millis();
    rtcTime.ntpSynced = true;
    
#ifdef SERIAL_DEBUG
    Serial.printf("✅ RTC updated from NTP: %04d-%02d-%02d %02d:%02d:%02d\n",
                  rtcTime.year, rtcTime.month, rtcTime.day,
                  rtcTime.hour, rtcTime.minute, rtcTime.second);
#endif
  }
}

// Maintain RTC time using millis() - call this frequently
void maintainRTC() {
  unsigned long currentMillis = millis();
  unsigned long elapsed = currentMillis - rtcTime.lastMillis;
  
  if (elapsed >= 1000) { // 1 second has passed
    int secondsToAdd = elapsed / 1000;
    rtcTime.second += secondsToAdd;
    rtcTime.lastMillis += secondsToAdd * 1000;
    
    // Handle time rollover
    if (rtcTime.second >= 60) {
      rtcTime.minute += rtcTime.second / 60;
      rtcTime.second = rtcTime.second % 60;
    }
    if (rtcTime.minute >= 60) {
      rtcTime.hour += rtcTime.minute / 60;
      rtcTime.minute = rtcTime.minute % 60;
    }
    if (rtcTime.hour >= 24) {
      rtcTime.day += rtcTime.hour / 24;
      rtcTime.hour = rtcTime.hour % 24;
      // Note: For simplicity, we don't handle month/year rollover
      // In a production system, you'd want to add proper date handling
    }
  }
}

// Get current time (uses RTC as primary source)
bool getCurrentTime(int* hour, int* minute, int* second) {
  maintainRTC(); // Always update RTC first
  
  *hour = rtcTime.hour;
  *minute = rtcTime.minute;
  *second = rtcTime.second;
  
  return rtcTime.ntpSynced; // Return whether time is NTP-synced
}


// Periodically try to resync with NTP when WiFi is available
void periodicTimeResync() {
  static unsigned long lastResyncAttempt = 0;
  const unsigned long RESYNC_INTERVAL = 3600000; // Try every hour
  
  if (WiFi.status() == WL_CONNECTED && !rtcTime.ntpSynced) {
    if (millis() - lastResyncAttempt >= RESYNC_INTERVAL) {
#ifdef SERIAL_DEBUG
      Serial.println("🔄 Attempting NTP resync...");
#endif
      
      struct tm timeinfo;
      configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
      delay(5000); // Wait for NTP response
      
      if (getLocalTime(&timeinfo)) {
        updateRTCFromNTP();
#ifdef SERIAL_DEBUG
        Serial.println("✅ NTP resync successful!");
#endif
      } else {
#ifdef SERIAL_DEBUG
        Serial.println("❌ NTP resync failed, continuing with RTC");
#endif
      }
      
      lastResyncAttempt = millis();
    }
  }
}

void printChildNodeStatus() {
#ifdef SERIAL_DEBUG
  Serial.println("\n📋 CHILD NODE STATUS:");
  for (auto& child : childNodes) {
    Serial.printf("  %s: ", child.nodeName.c_str());
    if (child.connected) {
      Serial.printf("✅ Connected (");
      for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", child.mac[i]);
        if (i < 5) Serial.print(":");
      }
      Serial.printf(") - Last seen: %lu seconds ago\n", (millis() - child.lastSeen) / 1000);
    } else {
      Serial.printf("❌ Disconnected");
      if (child.mac[0] != 0x00) {
        Serial.printf(" (MAC: ");
        for (int i = 0; i < 6; i++) {
          Serial.printf("%02X", child.mac[i]);
          if (i < 5) Serial.print(":");
        }
        Serial.print(")");
      }
      Serial.println();
    }
  }
  Serial.println();
#endif
}

void receive_sensor_data() {
  mesh_addr_t from;
  mesh_data_t data;
  uint8_t buffer[256] = { 0 };
  data.data = buffer;
  data.size = sizeof(buffer);

  int flag = 0;
  int err = esp_mesh_recv(&from, &data, 0, &flag, NULL, 0);

  if (err == ESP_OK && data.size > 0) {
    buffer[data.size] = '\0';
    String message = String((char*)buffer);
    
    // Update child node tracking
    updateChildNode(from.addr, message);
    
    Serial.println("\n📩 MESH MESSAGE RECEIVED:");
    Serial.printf("📡 From: ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", from.addr[i]);
      if (i < 5) Serial.print(":");
    }
    Serial.println();
    
    Serial.printf("🏷️  Flag: %s\n", 
                  flag == MESH_DATA_FROMDS ? "From Root" : 
                  flag == MESH_DATA_TODS ? "From Leaf" : "Intermediate");
    
    Serial.printf("💬 Message: %s\n", message.c_str());

    // Parse sensor data from child nodes
    if (message.indexOf("o_temp:") != -1 && message.indexOf("o_hum:") != -1) {
      sscanf(message.c_str(), "o_temp:%fC o_hum:%f%%", &o_temp, &o_hum);
      Serial.printf("✅ Extracted: 🌡️ o_temp=%.2f°C | 💧 o_hum=%.2f%%\n", o_temp, o_hum);
    }
    
    // Handle HELLO messages
    else if (message.indexOf("HELLO from") != -1) {
      Serial.printf("👋 %s\n", message.c_str());
      
      // Send acknowledgment
      String ack = "ACK from Root: " + message;
      sendMessage(ack.c_str(), from.addr);
    }
    
    // Handle status updates
    else if (message.indexOf(" ON") != -1 || message.indexOf(" OFF") != -1) {
      Serial.printf("🔌 Relay Status: %s\n", message.c_str());
    }
  }
}

void startConfigMode() {
  // Stop any previous WiFi connections
  WiFi.disconnect(true);
  delay(1000);
  
  // Start SoftAP for configuration (AP mode only for config)
  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(CONFIG_AP_SSID, CONFIG_AP_PASSWORD);
  
  if (apStarted) {
#ifdef SERIAL_DEBUG
    Serial.print("📡 Configuration AP Started: ");
    Serial.println(CONFIG_AP_SSID);
    Serial.print("🔐 Password: ");
    Serial.println(CONFIG_AP_PASSWORD);
    Serial.print("🌐 AP IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("⏰ Configuration mode will last for 15 minutes");
    Serial.println("💡 Connect to this WiFi and go to http://192.168.4.1 to configure");
#endif
  } else {
#ifdef SERIAL_DEBUG
    Serial.println("❌ Failed to start Configuration AP!");
#endif
    return;
  }
  
  // Setup web server
  setupWebServer();
  
  // Reset config mode timer
  configModeStartTime = millis();
  configMode = true;
}

// ============================
// 🌐 Web Server Setup
// ============================

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSaveConfig);
  server.on("/reset", handleReset);
  server.on("/status", handleStatus);
  server.begin();
#ifdef SERIAL_DEBUG
  Serial.println("🌐 HTTP server started");
#endif
}

void handleRoot() {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <title>Water System Configuration</title>
    <meta name="viewport" content="width=device-width, initial=1">
    <style>
      body { font-family: Arial; margin: 40px; background: #f5f5f5; }
      .container { max-width: 500px; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
      .section { margin-bottom: 20px; padding: 15px; border: 1px solid #ddd; border-radius: 5px; }
      h2 { color: #333; border-bottom: 2px solid #4CAF50; padding-bottom: 10px; }
      label { display: block; margin: 10px 0 5px; font-weight: bold; }
      input { width: 100%; padding: 8px; margin: 5px 0 15px; border: 1px solid #ddd; border-radius: 4px; box-sizing: border-box; }
      button { background: #4CAF50; color: white; padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer; margin: 5px; }
      button:hover { background: #45a049; }
      .danger { background: #dc3545; }
      .danger:hover { background: #c82333; }
      .info { background: #17a2b8; }
      .info:hover { background: #138496; }
      .status { padding: 10px; margin: 10px 0; border-radius: 5px; }
      .success { background: #d4edda; color: #155724; border: 1px solid #c3e6cb; }
      .error { background: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }
      .info { background: #d1ecf1; color: #0c5460; border: 1px solid #bee5eb; }
      .warning { background: #fff3cd; color: #856404; border: 1px solid #ffeaa7; }
      .required { color: red; }
    </style>
  </head>
  <body>
    <div class="container">
      <h1>Water System Configuration</h1>
      
      <div class="section">
        <h2>WiFi Configuration</h2>
        <label for="ssid">WiFi SSID: <span class="required">*</span></label>
        <input type="text" id="ssid" name="ssid" placeholder="Enter your WiFi SSID" required>
        
        <label for="password">WiFi Password:</label>
        <input type="password" id="password" name="password" placeholder="Enter your WiFi password">
      </div>

      <div class="section">
        <h2>Zoho IoT Configuration</h2>
        <label for="mqttUser">MQTT Username: <span class="required">*</span></label>
        <input type="text" id="mqttUser" name="mqttUser" placeholder="Enter Zoho MQTT username" required>
        
        <label for="mqttPass">MQTT Password: <span class="required">*</span></label>
        <input type="password" id="mqttPass" name="mqttPass" placeholder="Enter Zoho MQTT password" required>
      </div>

      <div style="text-align: center;">
        <button onclick="saveConfig()">Save Configuration</button>
        <button onclick="checkStatus()">System Status</button>
        <button class="info" onclick="checkOTAStatus()">OTA Status</button>
        <button type="button" onclick="resetConfig()">Soft Reset</button>
      </div>

      <div id="message"></div>
    </div>

    <script>
      function saveConfig() {
        const config = {
          ssid: document.getElementById('ssid').value,
          password: document.getElementById('password').value,
          mqttUser: document.getElementById('mqttUser').value,
          mqttPass: document.getElementById('mqttPass').value
        };

        if (!config.ssid || !config.mqttUser || !config.mqttPass) {
          showMessage('❌ All fields are required for configuration!', 'error');
          return;
        }

        if (config.ssid.length < 1 || config.mqttUser.length < 10) {
          showMessage('Please enter valid WiFi SSID and MQTT credentials', 'error');
          return;
        }

        fetch('/save', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(config)
        })
        .then(response => response.json())
        .then(data => {
          showMessage(data.message, data.success ? 'success' : 'error');
          if (data.success) {
            setTimeout(() => { 
              showMessage('Configuration saved! Device will restart and connect to your network.', 'success');
            }, 1000);
          }
        })
        .catch(error => {
          showMessage('Error saving configuration: ' + error, 'error');
        });
      }

      function checkStatus() {
        fetch('/status')
          .then(response => response.json())
          .then(data => {
            let statusHtml = '<div class="status ' + (data.connected ? 'success' : 'error') + '">';
            statusHtml += '<strong>Mode:</strong> ' + data.mode + '<br>';
            statusHtml += '<strong>WiFi:</strong> ' + (data.connected ? 'Connected' : 'Disconnected') + '<br>';
            statusHtml += '<strong>Configured:</strong> ' + (data.configured ? 'Yes' : 'No') + '<br>';
            statusHtml += '<strong>IP Address:</strong> ' + (data.ip || 'Unknown') + '<br>';
            statusHtml += '<strong>Firmware Version:</strong> )rawliteral" + String(FIRMWARE_VERSION) + R"rawliteral(<br>';
            statusHtml += '</div>';
            showMessage(statusHtml, 'info');
          });
      }

        function checkOTAStatus() {
          let statusHtml = '<div class="status info">';
          statusHtml += '<strong>🔄 Automatic OTA Updates</strong><br>';
          statusHtml += '<strong>Current Version:</strong> )rawliteral" + String(FIRMWARE_VERSION) + R"rawliteral(<br>';
          statusHtml += '<strong>GitHub Repo:</strong> )rawliteral" + String(GITHUB_REPO) + R"rawliteral(<br>';
          statusHtml += '<strong>Auto-Check:</strong> Every 24 hours<br>';
          statusHtml += '<strong>Status:</strong> ✅ Active<br>';
          statusHtml += '<br><strong>How to update:</strong><br>';
          statusHtml += '1. Upload new firmware.bin to GitHub Releases<br>';
          statusHtml += '2. Update version.txt with new version number<br>';
          statusHtml += '3. Device will auto-update within 24 hours<br>';
          statusHtml += '</div>';
          showMessage(statusHtml, 'info');
        }

      function resetConfig() {
        if (confirm('Are you sure you want to reset all configuration?')) {
          fetch('/reset')
            .then(response => response.json())
            .then(data => {
              showMessage(data.message, 'success');
              setTimeout(() => location.reload(), 2000);
            });
        }
      }

      function showMessage(message, type) {
        const msgDiv = document.getElementById('message');
        msgDiv.innerHTML = '<div class="status ' + type + '">' + message + '</div>';
      }
    </script>
  </body>
  </html>
  )rawliteral";
  
  server.send(200, "text/html", html);
}

void handleSaveConfig() {
  if (server.method() == HTTP_POST) {
    String body = server.arg("plain");
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
      server.send(400, "application/json", "{\"success\": false, \"message\": \"JSON parsing error\"}");
      return;
    }
    
    // Print received configuration
#ifdef SERIAL_DEBUG
    Serial.println("📋 Received Configuration:");
    Serial.print("📶 WiFi SSID: ");
    Serial.println(doc["ssid"].as<String>());
    Serial.print("🔐 WiFi Password: ");
    Serial.println(doc["password"].as<String>());
    Serial.print("☁️ MQTT Username: ");
    Serial.println(doc["mqttUser"].as<String>());
    Serial.print("🔑 MQTT Password: ");
    Serial.println(doc["mqttPass"].as<String>());
#endif
    
    // Save configuration
    strlcpy(deviceConfig.wifiSSID, doc["ssid"], sizeof(deviceConfig.wifiSSID));
    strlcpy(deviceConfig.wifiPassword, doc["password"], sizeof(deviceConfig.wifiPassword));
    strlcpy(deviceConfig.mqttUsername, doc["mqttUser"], sizeof(deviceConfig.mqttUsername));
    strlcpy(deviceConfig.mqttPassword, doc["mqttPass"], sizeof(deviceConfig.mqttPassword));
    
    deviceConfig.configured = true;
    saveConfig();
    
    server.send(200, "application/json", "{\"success\": true, \"message\": \"Configuration saved successfully!\"}");
    
#ifdef SERIAL_DEBUG
    Serial.println("✅ Configuration saved to EEPROM!");
    Serial.println("🔄 Restarting in 3 seconds...");
#endif
    delay(3000);
    ESP.restart();
  }
}

void handleReset() {
  resetConfig();
  server.send(200, "application/json", "{\"success\": true, \"message\": \"Configuration reset successfully!\"}");
}

void handleStatus() {
  DynamicJsonDocument doc(256);
  doc["mode"] = configMode ? "Configuration" : "Active";
  doc["connected"] = (WiFi.status() == WL_CONNECTED);
  doc["configured"] = deviceConfig.configured;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

// ============================
// 💾 Configuration Management
// ============================

void loadConfig() {
  // Initialize with empty defaults first
  resetConfig();
  
  // Try to read from EEPROM
  EEPROM.get(EEPROM_CONFIG_ADDR, deviceConfig);
  
  // Check if configuration is valid (non-empty SSID and MQTT username)
  if (deviceConfig.configured && 
      strlen(deviceConfig.wifiSSID) > 0 && 
      strlen(deviceConfig.mqttUsername) > 0) {
#ifdef SERIAL_DEBUG
    Serial.println("📖 Loaded configuration from EEPROM");
    Serial.print("📶 WiFi: ");
    Serial.println(deviceConfig.wifiSSID);
#endif
  } else {
#ifdef SERIAL_DEBUG
    Serial.println("❌ No valid configuration in EEPROM - using AP mode");
#endif
    resetConfig();
  }
}

void saveConfig() {
  EEPROM.put(EEPROM_CONFIG_ADDR, deviceConfig);
  if (EEPROM.commit()) {
#ifdef SERIAL_DEBUG
    Serial.println("💾 Configuration saved to EEPROM");
#endif
  } else {
#ifdef SERIAL_DEBUG
    Serial.println("❌ Failed to save configuration to EEPROM");
#endif
  }
}

void resetConfig() {
  memset(&deviceConfig, 0, sizeof(deviceConfig));
  
  // Set empty/default values instead of hardcoded credentials
  strlcpy(deviceConfig.wifiSSID, "", sizeof(deviceConfig.wifiSSID));
  strlcpy(deviceConfig.wifiPassword, "", sizeof(deviceConfig.wifiPassword));
  strlcpy(deviceConfig.mqttUsername, "", sizeof(deviceConfig.mqttUsername));
  strlcpy(deviceConfig.mqttPassword, "", sizeof(deviceConfig.mqttPassword));
  
  deviceConfig.configured = false;
#ifdef SERIAL_DEBUG
  Serial.println("🔄 Configuration reset to empty defaults");
#endif
}

void saveAutomationConfig() {
  EEPROM.put(EEPROM_AUTOMATION_CONFIG_ADDR, automationConfig);
  if (EEPROM.commit()) {
#ifdef SERIAL_DEBUG
    Serial.println("💾 Automation configuration saved to EEPROM");
#endif
  } else {
#ifdef SERIAL_DEBUG
    Serial.println("❌ Failed to save automation configuration to EEPROM");
#endif
  }
}

void loadAutomationConfig() {
  AutomationConfig loadedConfig;
  EEPROM.get(EEPROM_AUTOMATION_CONFIG_ADDR, loadedConfig);
  
  // Simple validation
  if (loadedConfig.startHour >= 0 && loadedConfig.startHour <= 23 &&
      loadedConfig.endHour >= 0 && loadedConfig.endHour <= 24 &&
      loadedConfig.targetFillLevel >= 0 && loadedConfig.targetFillLevel <= 125.0) {
    automationConfig = loadedConfig;
#ifdef SERIAL_DEBUG
    Serial.println("📖 Loaded automation configuration from EEPROM");
#endif
  } else {
#ifdef SERIAL_DEBUG
    Serial.println("❌ Invalid automation config in EEPROM - using defaults");
#endif
    // Initialize with defaults
    automationConfig = AutomationConfig();
    saveAutomationConfig(); // Save defaults to EEPROM
  }
}

void saveSensorThresholds() {
  EEPROM.put(EEPROM_SENSOR_THRESHOLDS_ADDR, sensorThresholds);
  if (EEPROM.commit()) {
#ifdef SERIAL_DEBUG
    Serial.println("💾 Sensor thresholds saved to EEPROM");
#endif
  } else {
#ifdef SERIAL_DEBUG
    Serial.println("❌ Failed to save sensor thresholds to EEPROM");
#endif
  }
}

void loadSensorThresholds() {
  SensorThresholds loadedThresholds;
  EEPROM.get(EEPROM_SENSOR_THRESHOLDS_ADDR, loadedThresholds);
  
  // Simple validation
  if (loadedThresholds.min_temp >= 0 && loadedThresholds.max_temp <= 100 &&
      loadedThresholds.min_hum >= 0 && loadedThresholds.max_hum <= 100 &&
      loadedThresholds.min_level >= 0 && loadedThresholds.max_level <= 125) {
    sensorThresholds = loadedThresholds;
#ifdef SERIAL_DEBUG
    Serial.println("📖 Loaded sensor thresholds from EEPROM");
#endif
  } else {
#ifdef SERIAL_DEBUG
    Serial.println("❌ Invalid sensor thresholds in EEPROM - using defaults");
#endif
    // Initialize with defaults
    sensorThresholds = SensorThresholds();
    saveSensorThresholds(); // Save defaults to EEPROM
  }
}

void check_sensor_thresholds(float temp, float hum, float level, float pressure, float flow, float ph, float tds) {
  String alarm_message = "";
  
  if (temp < sensorThresholds.min_temp) {
    alarm_message = "LOW TEMPERATURE: " + String(temp, 1) + "°C (Min: " + String(sensorThresholds.min_temp, 1) + "°C)";
  } else if (temp > sensorThresholds.max_temp) {
    alarm_message = "HIGH TEMPERATURE: " + String(temp, 1) + "°C (Max: " + String(sensorThresholds.max_temp, 1) + "°C)";
  } else if (hum < sensorThresholds.min_hum) {
    alarm_message = "LOW HUMIDITY: " + String(hum, 1) + "% (Min: " + String(sensorThresholds.min_hum, 1) + "%)";
  } else if (hum > sensorThresholds.max_hum) {
    alarm_message = "HIGH HUMIDITY: " + String(hum, 1) + "% (Max: " + String(sensorThresholds.max_hum, 1) + "%)";
  } else if (level < sensorThresholds.min_level) {
    alarm_message = "LOW WATER LEVEL: " + String(level, 1) + "L (Min: " + String(sensorThresholds.min_level, 1) + "L)";
  } else if (level > sensorThresholds.max_level) {
    alarm_message = "HIGH WATER LEVEL: " + String(level, 1) + "L (Max: " + String(sensorThresholds.max_level, 1) + "L)";
  } else if (pressure < sensorThresholds.min_pressure) {
    alarm_message = "LOW PRESSURE: " + String(pressure, 1) + " bar (Min: " + String(sensorThresholds.min_pressure, 1) + " bar)";
  } else if (pressure > sensorThresholds.max_pressure) {
    alarm_message = "HIGH PRESSURE: " + String(pressure, 1) + " bar (Max: " + String(sensorThresholds.max_pressure, 1) + " bar)";
  } else if (flow < sensorThresholds.min_flow && flow > 0.1) { // Only alert if flow is active but low
    alarm_message = "LOW FLOW RATE: " + String(flow, 1) + " LPM (Min: " + String(sensorThresholds.min_flow, 1) + " LPM)";
  } else if (flow > sensorThresholds.max_flow) {
    alarm_message = "HIGH FLOW RATE: " + String(flow, 1) + " LPM (Max: " + String(sensorThresholds.max_flow, 1) + " LPM)";
  } else if (ph < sensorThresholds.min_ph) {
    alarm_message = "LOW pH: " + String(ph, 1) + " (Min: " + String(sensorThresholds.min_ph, 1) + ")";
  } else if (ph > sensorThresholds.max_ph) {
    alarm_message = "HIGH pH: " + String(ph, 1) + " (Max: " + String(sensorThresholds.max_ph, 1) + ")";
  } else if (tds < sensorThresholds.min_tds && tds > 1.0) { // Only alert if TDS is active but low
    alarm_message = "LOW TDS: " + String(tds, 1) + " ppm (Min: " + String(sensorThresholds.min_tds, 1) + " ppm)";
  } else if (tds > sensorThresholds.max_tds) {
    alarm_message = "HIGH TDS: " + String(tds, 1) + " ppm (Max: " + String(sensorThresholds.max_tds, 1) + " ppm)";
  }
  
  if (alarm_message != "") {
    send_automation_alarm(alarm_message.c_str());
  }
}

void loop() {
  zClient.zyield(); 

  // ✅ CORE FUNCTIONS
  esp_task_wdt_reset();
  managePowerSupply();

  // ✅ RTC TIME MAINTENANCE
  maintainRTC();

  // ✅ MESH OPERATIONS
  receive_sensor_data();  // Receive and process mesh messages
  monitorChildNodes();    // Monitor child node connections

  // ✅ MESH STATUS MONITORING
  static unsigned long lastMeshCheck = 0;
  if (millis() - lastMeshCheck > 30000) {
    printMeshStatus();
    printChildNodeStatus();  // Show child node status
    lastMeshCheck = millis();
  }

  // ✅ AUTOMATION SYSTEM
  check_scheduled_automation();

  // ✅ SENSORS
  if ((millis() - prev_time) >= SENSOR_READ_INTERVAL) {
    prev_time = millis();
    send_sensor_data();
  }

  // ✅ AUTO OTA UPDATE CHECK (Every 24 hours)
  static unsigned long lastUpdateCheck = 0;
  if (millis() - lastUpdateCheck > OTA_CHECK_INTERVAL) {
    checkGitHubForUpdates();
    lastUpdateCheck = millis();
  }

  // ✅ MODE-SPECIFIC FUNCTIONS
  if (configMode) {
    server.handleClient();
    checkConfigModeTimeout();
  } else {
    wifiManagement();
  }
  
  delay(100);
}


void check_scheduled_automation() {
  int currentHour, currentMinute, currentSecond;
  bool timeIsSynced = getCurrentTime(&currentHour, &currentMinute, &currentSecond);
  
  // Debug output for time status
  static int lastReportedHour = -1;
  if (currentHour != lastReportedHour) {
    lastReportedHour = currentHour;
#ifdef SERIAL_DEBUG
    Serial.printf("⏰ Automation Check: %02d:%02d:%02d (%s)\n", 
                  currentHour, currentMinute, currentSecond,
                  timeIsSynced ? "NTP Synced" : "RTC Time");
#endif
  }

  // Check if within automation hours (8 AM to 6 PM)
  if (currentHour < AUTOMATION_START_HOUR || currentHour >= AUTOMATION_END_HOUR) {
    if (automationRunning) {
      // Stop automation if running outside hours
      stop_automation();
#ifdef SERIAL_DEBUG
      Serial.println("⏰ Automation: Outside operating hours - stopping automation");
#endif
    }
    return;
  }

  // Check if automation is enabled
  if (!automationConfig.automationEnabled) {
    if (automationRunning) {
      stop_automation();
    }
    return;
  }

  // Start new cycle every hour (at minute 0)
  if (currentMinute == 0 && currentSecond < 10) { // Check in first 10 seconds of the hour
    if (!automationRunning && currentHour != currentCycleHour) {
      start_automation_cycle(currentHour);
    }
  }

  // Execute automation sequence if running
  if (automationRunning) {
    execute_automation_sequence();
  }
  
  // Periodic time resync attempt
  periodicTimeResync();
}

void start_automation_cycle(int hour) {
  currentAutomationState = AUTOMATION_DRAINING;
  automationStartTime = millis();
  stepStartTime = millis();
  automationRunning = true;
  currentCycleHour = hour;

#ifdef SERIAL_DEBUG
  Serial.println("🚀 AUTOMATION: Starting new hourly cycle");
  Serial.printf("📋 Sequence: Draining → IV Fill → DP1+DP2 → IP+SV1 → IP+SV2,3,4\n");
  Serial.printf("🎯 Target Fill Level: %.1fL\n", automationConfig.targetFillLevel);
#endif
}

void stop_automation() {
  // Turn off all outputs
  digitalWrite(sv_8, LOW);
  digitalWrite(IV, LOW);
  digitalWrite(DP_1, LOW);
  digitalWrite(DP_2, LOW);
  digitalWrite(IP, LOW);
  digitalWrite(sv_1, LOW);
  
  // Send mesh commands to turn off SV_2, SV_3, SV_4
  sendMeshCommand("SV_2", false);
  sendMeshCommand("SV_3", false);
  sendMeshCommand("SV_4", false);
  
  currentAutomationState = AUTOMATION_IDLE;
  automationRunning = false;
  
#ifdef SERIAL_DEBUG
  Serial.println("🛑 Automation: Stopped");
#endif
}

void execute_automation_sequence() {
  if (!automationRunning) return;

  unsigned long currentMillis = millis();

  switch (currentAutomationState) {
    
case AUTOMATION_DRAINING:
    // Check if level is already 0
    if (level <= 0.1) {
#ifdef SERIAL_DEBUG
        Serial.println("✅ Automation: Tank already empty - No draining needed");
        Serial.println("➡️ Proceeding to IV filling");
#endif
        currentAutomationState = AUTOMATION_FILLING;
        stepStartTime = currentMillis;
        break;
    }

    // Start SV_8 to drain tank to 0 (only if level > 0)
    if (currentMillis - stepStartTime == 0) {
        digitalWrite(sv_8, HIGH);
#ifdef SERIAL_DEBUG
        Serial.println("💧 Automation: Starting SV_8 - Draining tank to 0L");
        Serial.printf("📊 Current level: %.2fL\n", level);
#endif
    }

    // Check if level reached 0
    if (level <= 0.1) {
        digitalWrite(sv_8, LOW);
#ifdef SERIAL_DEBUG
        Serial.println("✅ Automation: Draining complete - SV_8 OFF");
        Serial.println("➡️ Proceeding to IV filling");
#endif
        currentAutomationState = AUTOMATION_FILLING;
        stepStartTime = currentMillis;
    }
    
    // Optional: Add a safety timeout to prevent infinite draining
    if (currentMillis - stepStartTime > 600000) { // 10 minutes max safety timeout
        digitalWrite(sv_8, LOW);
#ifdef SERIAL_DEBUG
        Serial.println("⏰ Automation: Draining timeout - SV_8 OFF");
        Serial.println("➡️ Proceeding to IV filling despite incomplete draining");
#endif
        currentAutomationState = AUTOMATION_FILLING;
        stepStartTime = currentMillis;
    }
    break;

    case AUTOMATION_FILLING:
      // Start IV to fill to target level
      if (currentMillis - stepStartTime == 0) {
        digitalWrite(IV, HIGH);
#ifdef SERIAL_DEBUG
        Serial.println("💧 Automation: Starting IV to fill tank");
        Serial.printf("📊 Current level: %.2fL, Target: %.2fL\n", level, automationConfig.targetFillLevel);
#endif
      }

      // Check if target level reached
      if (level >= automationConfig.targetFillLevel) {
        digitalWrite(IV, LOW);
#ifdef SERIAL_DEBUG
        Serial.println("✅ Automation: Target level reached - IV OFF");
        Serial.println("➡️ Proceeding to DP operation");
#endif
        currentAutomationState = AUTOMATION_DP_OPERATION;
        stepStartTime = currentMillis;
        break;
      }

      // Check for fill timeout
      if (currentMillis - stepStartTime > automationConfig.ivTimeout) {
        digitalWrite(IV, LOW);
#ifdef SERIAL_DEBUG
        Serial.println("⏰ Automation: Fill timeout - IV OFF");
        Serial.println("🚨 ALARM: Fill timeout - desired level not reached");
#endif
        // Send alarm to Zoho
        send_automation_alarm("Fill timeout - desired level not reached");
        currentAutomationState = AUTOMATION_DP_OPERATION;
        stepStartTime = currentMillis;
      }
      break;

    case AUTOMATION_DP_OPERATION:
      // Start DP1 and DP2
      if (currentMillis - stepStartTime == 0) {
        digitalWrite(DP_1, HIGH);
        digitalWrite(DP_2, HIGH);
#ifdef SERIAL_DEBUG
        Serial.println("🔌 Automation: Starting DP1 and DP2");
#endif
      }

      // Check if DP operation duration completed
      if (currentMillis - stepStartTime >= automationConfig.dpDuration) {
        digitalWrite(DP_1, LOW);
        digitalWrite(DP_2, LOW);
#ifdef SERIAL_DEBUG
        Serial.println("✅ Automation: DP operation complete");
        Serial.println("➡️ Proceeding to IP+SV1 operation");
#endif
        currentAutomationState = AUTOMATION_IP_SV1_OPERATION;
        stepStartTime = currentMillis;
      }
      break;

    case AUTOMATION_IP_SV1_OPERATION:
      // Start IP and SV1 together
      if (currentMillis - stepStartTime == 0) {
        digitalWrite(IP, HIGH);
        digitalWrite(sv_1, HIGH);
#ifdef SERIAL_DEBUG
        Serial.println("🔌 Automation: Starting IP and SV1 together");
#endif
      }

      // Check if IP+SV1 operation duration completed
      if (currentMillis - stepStartTime >= automationConfig.ipSv1Duration) {
        digitalWrite(IP, LOW);
        digitalWrite(sv_1, LOW);
#ifdef SERIAL_DEBUG
        Serial.println("✅ Automation: IP+SV1 operation complete");
        Serial.println("➡️ Proceeding to IP+SV2,3,4 operation");
#endif
        currentAutomationState = AUTOMATION_IP_SV234_OPERATION;
        stepStartTime = currentMillis;
      }
      break;

case AUTOMATION_IP_SV234_OPERATION:
    // Initialize valve tracking when starting this step
    if (currentMillis - stepStartTime == 0) {
        // Reset valve tracking
        valveTracking.valve1Consumed = 0.0;
        valveTracking.valve2Consumed = 0.0;
        valveTracking.valve3Consumed = 0.0;
        valveTracking.initialLevel = level;
        valveTracking.valve1Active = true;
        valveTracking.valve2Active = true;
        valveTracking.valve3Active = true;
        
        // Start IP and send mesh commands for SV_2, SV_3, SV_4
        digitalWrite(IP, HIGH);
        sendMeshCommand("SV_2", true);
        sendMeshCommand("SV_3", true);
        sendMeshCommand("SV_4", true);
        
#ifdef SERIAL_DEBUG
        Serial.println("🔌 Automation: Starting IP and sending mesh commands for SV2, SV3, SV4");
        Serial.printf("🎯 Consumption Targets - V1: %.1fL, V2: %.1fL, V3: %.1fL\n", 
                     automationConfig.valve1Target, automationConfig.valve2Target, automationConfig.valve3Target);
        Serial.printf("📊 Initial Level: %.2fL\n", valveTracking.initialLevel);
#endif
    }

    // Calculate total water consumed in this step
    float totalConsumed = valveTracking.initialLevel - level;
    
    // Calculate consumption per active valve
    int activeValves = 0;
    if (valveTracking.valve1Active) activeValves++;
    if (valveTracking.valve2Active) activeValves++;
    if (valveTracking.valve3Active) activeValves++;
    
    if (activeValves > 0) {
        float consumptionPerValve = totalConsumed / activeValves;
        
        // Update individual valve consumption
        if (valveTracking.valve1Active) {
            valveTracking.valve1Consumed = consumptionPerValve;
        }
        if (valveTracking.valve2Active) {
            valveTracking.valve2Consumed = consumptionPerValve;
        }
        if (valveTracking.valve3Active) {
            valveTracking.valve3Consumed = consumptionPerValve;
        }
    }

    // Check and send mesh commands to turn off valves when they reach their targets
    bool anyValveTurnedOff = false;
    
    // Valve 1 check (SV_2)
    if (valveTracking.valve1Active && valveTracking.valve1Consumed >= automationConfig.valve1Target) {
        sendMeshCommand("SV_2", false);  // Send OFF command via mesh for SV2 (valve 1)
        valveTracking.valve1Active = false;
#ifdef SERIAL_DEBUG
        Serial.println("✅ Valve 1 (SV2) target reached - Sending OFF via mesh");
        Serial.printf("   Consumed: %.2fL, Target: %.1fL\n", valveTracking.valve1Consumed, automationConfig.valve1Target);
#endif
        anyValveTurnedOff = true;
    }
    
    // Valve 2 check (SV_3)
    if (valveTracking.valve2Active && valveTracking.valve2Consumed >= automationConfig.valve2Target) {
        sendMeshCommand("SV_3", false);  // Send OFF command via mesh for SV3 (valve 2)
        valveTracking.valve2Active = false;
#ifdef SERIAL_DEBUG
        Serial.println("✅ Valve 2 (SV3) target reached - Sending OFF via mesh");
        Serial.printf("   Consumed: %.2fL, Target: %.1fL\n", valveTracking.valve2Consumed, automationConfig.valve2Target);
#endif
        anyValveTurnedOff = true;
    }
    
    // Valve 3 check (SV_4)
    if (valveTracking.valve3Active && valveTracking.valve3Consumed >= automationConfig.valve3Target) {
        sendMeshCommand("SV_4", false);  // Send OFF command via mesh for SV4 (valve 3)
        valveTracking.valve3Active = false;
#ifdef SERIAL_DEBUG
        Serial.println("✅ Valve 3 (SV4) target reached - Sending OFF via mesh");
        Serial.printf("   Consumed: %.2fL, Target: %.1fL\n", valveTracking.valve3Consumed, automationConfig.valve3Target);
#endif
        anyValveTurnedOff = true;
    }
    
    // Debug consumption info
#ifdef SERIAL_DEBUG
    static unsigned long lastConsumptionLog = 0;
    if (currentMillis - lastConsumptionLog > 5000) { // Log every 5 seconds
        lastConsumptionLog = currentMillis;
        Serial.printf("💧 Consumption - Total: %.2fL | V1: %.2fL/%.1fL | V2: %.2fL/%.1fL | V3: %.2fL/%.1fL | Active: %d\n",
                     totalConsumed,
                     valveTracking.valve1Consumed, automationConfig.valve1Target,
                     valveTracking.valve2Consumed, automationConfig.valve2Target,
                     valveTracking.valve3Consumed, automationConfig.valve3Target,
                     activeValves);
    }
#endif

    // Check if all valves are done or timeout reached
    bool allValvesDone = !valveTracking.valve1Active && !valveTracking.valve2Active && !valveTracking.valve3Active;
    
    if (allValvesDone || (currentMillis - stepStartTime >= automationConfig.ipSv234Duration)) {
        // Turn off IP and send final mesh commands
        digitalWrite(IP, LOW);
        
        // Ensure all valves are turned off via mesh
        if (valveTracking.valve1Active) sendMeshCommand("SV_2", false);
        if (valveTracking.valve2Active) sendMeshCommand("SV_3", false);
        if (valveTracking.valve3Active) sendMeshCommand("SV_4", false);
        
#ifdef SERIAL_DEBUG
        if (allValvesDone) {
            Serial.println("✅ Automation: All valves reached their targets");
        } else {
            Serial.println("⏰ Automation: Timeout reached - stopping IP+SV234 operation");
        }
        Serial.printf("📊 Final Consumption - V1: %.2fL, V2: %.2fL, V3: %.2fL\n",
                     valveTracking.valve1Consumed, valveTracking.valve2Consumed, valveTracking.valve3Consumed);
        Serial.println("🎉 AUTOMATION CYCLE COMPLETED");
#endif
        currentAutomationState = AUTOMATION_COMPLETE;
        automationRunning = false;
    }
    break;
}
zClient.zyield(); 
}

void send_automation_alarm(const char* message) {
  if (WiFi.status() == WL_CONNECTED) {
    zClient.addDataPointString("alarm_message", message);
    zClient.addDataPointNumber("alarm_timestamp", millis());
    zClient.dispatch();
#ifdef SERIAL_DEBUG
    Serial.printf("🚨 Alarm sent to Zoho: %s\n", message);
#endif
  }
}

void checkConfigModeTimeout() {
  if (millis() - configModeStartTime >= CONFIG_MODE_DURATION) {
    if (deviceConfig.configured) {
#ifdef SERIAL_DEBUG
      Serial.println("⏰ Config mode timeout - Trying stored credentials again");
#endif
      
      // 🔄 TRY TO RECONNECT WITH STORED CREDENTIALS
      configMode = false;
      unsigned long wifiStartTime = millis();
      bool wifiConnected = false;
      
      while (millis() - wifiStartTime < WIFI_CONNECT_TIMEOUT) {
        if (setupWiFi()) {
          wifiConnected = true;
          break;
        }
        delay(1000);
      }
      
      if (wifiConnected) {
#ifdef SERIAL_DEBUG
        Serial.println("✅ Reconnected with stored credentials - Starting Main System");
#endif
        setupMainSystem();
      } else {
#ifdef SERIAL_DEBUG
        Serial.println("❌ Still can't connect - Restarting Config Mode");
#endif
        // Stay in config mode if still can't connect
        configMode = true;
        configModeStartTime = millis();
        startConfigMode();
      }
    } else {
#ifdef SERIAL_DEBUG
      Serial.println("⏰ Config mode timeout - No configuration received");
      Serial.println("🔄 Restarting in configuration mode...");
#endif
      ESP.restart();
    }
  }
  
  // 🔄 BACKGROUND RECONNECTION ATTEMPT EVERY 3 MINUTES IN CONFIG MODE
  static unsigned long lastBackgroundAttempt = 0;
  if (configMode && deviceConfig.configured && 
      (millis() - lastBackgroundAttempt >= WIFI_RETRY_INTERVAL)) {
    
#ifdef SERIAL_DEBUG
    Serial.println("🔄 Background: Trying stored WiFi credentials...");
#endif
    
    bool tempConnected = setupWiFi();
    if (tempConnected) {
#ifdef SERIAL_DEBUG
      Serial.println("✅ Background: Connected! Switching to Active Mode");
#endif
      configMode = false;
      setupMainSystem();
    } else {
#ifdef SERIAL_DEBUG
      Serial.println("❌ Background: Still can't connect - Staying in Config Mode");
#endif
    }
    startConfigMode();
    lastBackgroundAttempt = millis();
  }
}

void wifiManagement() {
  // Only attempt reconnection if WiFi is actually disconnected
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWiFiAttempt >= WIFI_RETRY_INTERVAL) {
#ifdef SERIAL_DEBUG
      Serial.println("❌ WiFi disconnected - Attempting reconnect...");
#endif
      
      reconnectWiFi();
      
#ifdef SERIAL_DEBUG
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("❌ Reconnection failed - Will retry in 3 minutes");
      }
#endif
      
      lastWiFiAttempt = millis();
    }
  }
  // If WiFi is connected, do nothing - no need for unnecessary reconnection attempts
}

// ============================
// 📶 WiFi Management (Fixed)
// ============================
bool setupWiFi() {
  if (strlen(deviceConfig.wifiSSID) == 0) {
#ifdef SERIAL_DEBUG
    Serial.println("❌ No WiFi configuration found - cannot connect");
#endif
    return false;
  }
  
  // Clean start - Set to AP+STA mode for Mesh coexistence
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_AP_STA);
  delay(1000);
  
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true); // Keep credentials stored
  
#ifdef SERIAL_DEBUG
  Serial.print("📶 Connecting to WiFi: ");
  Serial.println(deviceConfig.wifiSSID);
#endif
  
  WiFi.begin(deviceConfig.wifiSSID, deviceConfig.wifiPassword);
  
  unsigned long startTime = millis();
  
  while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < WIFI_CONNECT_TIMEOUT) {
    delay(500);
#ifdef SERIAL_DEBUG
    Serial.print(".");
#endif
    if (WiFi.status() == WL_CONNECT_FAILED || WiFi.status() == WL_NO_SSID_AVAIL) {
      break;
    }
  }
  
  bool connected = (WiFi.status() == WL_CONNECTED);
  
  if (connected) {
#ifdef SERIAL_DEBUG
    Serial.println("\n✅ WiFi Connected");
    Serial.print("📡 IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("📡 WiFi Mode: ");
    Serial.println(WiFi.getMode() == WIFI_AP_STA ? "AP+STA" : "Other");
#endif
  } else {
#ifdef SERIAL_DEBUG
    Serial.println("\n❌ WiFi connection failed");
    Serial.print("Status: ");
    Serial.println(WiFi.status());
#endif
  }
  
  lastWiFiAttempt = millis();
  return connected;
}

void reconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return; // Already connected
  }

#ifdef SERIAL_DEBUG
  Serial.println("🔄 WiFi disconnected - Attempting reconnect...");
#endif

  // Quick reconnect attempt first
  WiFi.reconnect();
  delay(5000); // Wait 5 seconds for reconnection
  
  if (WiFi.status() != WL_CONNECTED) {
    // If quick reconnect failed, do full reset
    setupWiFi();
  }
}

// ============================
// ⏱ Watchdog Task
// ============================

void TaskWDT_Core0(void *pvParameters) {
  esp_task_wdt_add(NULL);
  while (1) {
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ============================
// ⚡ Power Management Functions
// ============================

float readACVoltage(int pin) {
  const int NUM_SAMPLES = 100;       // 100 samples
  const int SAMPLE_INTERVAL_MS = 10; // 10ms between samples

  unsigned int peakADC = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    unsigned int raw = analogRead(pin);

    if (raw > peakADC) {
      peakADC = raw;
    }
    delay(SAMPLE_INTERVAL_MS);
  }

#ifdef SERIAL_DEBUG
  Serial.print("RAWADC: ");
  Serial.println(peakADC);
#endif

  if (peakADC < ADC_MIN_THRESHOLD) {
    return 0.0;
  }

  // Convert peak ADC to input voltage
  float vPeak = (peakADC / 4095.0) * 3.3;  // Adjust for ADC resolution & Vref

  // Scale up for your voltage divider or amplifier
  const float voltageDividerRatio = 160.0; // Adjust for your circuit
  vPeak *= voltageDividerRatio;

  // Peak to RMS for sine wave
  float vRMS = vPeak * 0.707;

  // Map so that ADC_MIN_THRESHOLD = VOLTAGE_MIN_THRESHOLD → 0V
  if (vRMS < VOLTAGE_MIN_THRESHOLD) {
    vRMS = 0.0;
  }

  return vRMS;
}

void switchToGridPower() {
  digitalWrite(GENERATOR_RELAY_PIN, LOW);
  delay(1000);
  digitalWrite(GRID_RELAY_PIN, HIGH);
  generatorActive = false;
#ifdef SERIAL_DEBUG
  Serial.println("Switched to GEB power");
#endif
}

void switchToGeneratorPower() {
  digitalWrite(GRID_RELAY_PIN, LOW);
  delay(1000);
  digitalWrite(GENERATOR_RELAY_PIN, HIGH);
  generatorActive = true;
#ifdef SERIAL_DEBUG
  Serial.println("Switched to generator power");
#endif
}

void managePowerSupply() {
  unsigned long currentTime = millis();

  if (currentTime - lastPowerCheck >= CHECK_INTERVAL || lastPowerCheck == 0) {
    lastPowerCheck = currentTime;

    // Read both voltages
    gridVoltage = readACVoltage(GRID_POWER_PIN);
    generatorVoltage = readACVoltage(GENERATOR_POWER_PIN);

#ifdef SERIAL_DEBUG
    Serial.printf("Power Status - GEB: %.1fV, Generator: %.1fV\n",
                  gridVoltage, generatorVoltage);
#endif

    // Check grid voltage threshold
    if (gridVoltage >= GRID_CUTOFF_VOLTAGE) {
      // Good grid voltage → stay on grid
      if (!digitalRead(GRID_RELAY_PIN)) {
        switchToGridPower();
      } else {
#ifdef SERIAL_DEBUG
        Serial.println("Grid voltage good - staying on grid power");
#endif
      }
    } else {
      // Grid voltage too low → switch to generator
#ifdef SERIAL_DEBUG
      Serial.println("Grid voltage too low! Switching to generator.");
#endif
      switchToGeneratorPower();
    }
  }
}

void checkHeap() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 60000) {
    lastCheck = millis();
#ifdef SERIAL_DEBUG
    Serial.printf("Free Heap: %d, Min Free: %d\n", 
                 ESP.getFreeHeap(), 
                 ESP.getMinFreeHeap());
#endif
 }
}


// Enhanced time setup
void setupTime() {
  // Initialize RTC first (with compile time)
  initializeRTC();
  
  // Try to sync with NTP
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  
  struct tm timeinfo;
  int retries = 0;
  bool ntpSuccess = false;

  while (!getLocalTime(&timeinfo) && (retries < 10)) {
#ifdef SERIAL_DEBUG
    Serial.println("⏰ Waiting for NTP time...");
#endif
    delay(1000);
    retries++;
  }

  if (retries < 10) {
    // NTP sync successful
    updateRTCFromNTP();
    timeSynced = true;
#ifdef SERIAL_DEBUG
    Serial.println("✅ Time synced from NTP");
#endif
  } else {
    // NTP sync failed, continue with RTC
    timeSynced = false;
#ifdef SERIAL_DEBUG
    Serial.println("⚠️ NTP sync failed - using internal RTC");
    Serial.printf("🕒 Current RTC time: %02d:%02d:%02d\n", 
                  rtcTime.hour, rtcTime.minute, rtcTime.second);
#endif
  }
}

// ============================
// ⏱️ Sensor Timing Display
// ============================

void printSensorTiming(const SensorTiming& timing) {
#ifdef SERIAL_DEBUG
  Serial.println("⏱️ SENSOR READING TIMES:");
  Serial.printf("  🌡️  DHT22 (Temp/Humi): %lu ms\n", timing.dht_time);
  Serial.printf("  📏 Ultrasonic Level: %lu ms\n", timing.ultrasonic_time);
  Serial.printf("  🧪 pH Sensor: %lu ms\n", timing.ph_time);
  Serial.printf("  ⚡ Pressure Sensor: %lu ms\n", timing.pressure_time);
  Serial.printf("  💧 TDS Sensor: %lu ms\n", timing.tds_time);
  Serial.printf("  🌊 Flow Sensor: %lu ms\n", timing.flow_time);
  Serial.printf("  🔌 Voltage Reading: %lu ms\n", timing.voltage_time);
  Serial.printf("  📊 TOTAL SENSOR TIME: %lu ms\n", timing.total_sensor_time);
  Serial.println("----------------------------------------");
#endif
}

  void send_sensor_data() {
    unsigned long total_start_time = millis();
    SensorTiming current_timing = {0};

    
    zClient.reconnect();

    IP_status = digitalRead(IP) == HIGH ? "ON" : "OFF";

    // Read DHT sensor with timing
    unsigned long dht_start = millis();
    float i_temp = dht.readTemperature();
    delay(500);
    float i_hum = dht.readHumidity();
    delay(500);
    current_timing.dht_time = millis() - dht_start;

    // Read ultrasonic level with timing
    unsigned long ultrasonic_start = millis();
    level = read_ultrasonic_distance();
    current_timing.ultrasonic_time = millis() - ultrasonic_start;

    // Read pH sensor with timing
    unsigned long ph_start = millis();
    float ph = read_ph_sensor();
    current_timing.ph_time = millis() - ph_start;

    // Read pressure sensor with timing
    unsigned long pressure_start = millis();
    float pressure = read_pressure_sensor();   // updates global dacValue
    current_timing.pressure_time = millis() - pressure_start;

    // Read TDS sensor with timing
    unsigned long tds_start = millis();
    float tds = read_tds_sensor(i_temp);
    current_timing.tds_time = millis() - tds_start;

    // Read flow sensor with timing
    unsigned long flow_start = millis();
    FlowData flow = read_flow_sensor();
    current_timing.flow_time = flow.read_time; // Use the time from flow sensor itself

    // Read voltage with timing
    unsigned long voltage_start = millis();
    float geb = gridVoltage;  // use last measured value
    if (isnan(geb)) geb = 0;
    current_timing.voltage_time = millis() - voltage_start;

    // Set default values for mesh data (since mesh is disabled)
    o_temp = 0.0;
    o_hum = 0.0;

    // Sanity checks...
    if (isnan(i_temp)) i_temp = 0;
    if (isnan(i_hum)) i_hum = 0;
    if (isnan(o_temp)) o_temp = 0;
    if (isnan(o_hum)) o_hum = 0;
    if (isnan(level)) level = 0;
    if (isnan(ph)) ph = 0;
    if (isnan(tds)) tds = 0;
    if (isnan(pressure)) pressure = 0;

    // Calculate total sensor reading time
    current_timing.total_sensor_time = millis() - total_start_time;

    // Print sensor timing information
    printSensorTiming(current_timing);

  #ifdef SERIAL_DEBUG
    Serial.printf(
      "\n📊 SENSOR DATA:\n"
      "  🌡️  i_temp: %.2f°C | i_hum: %.2f%%\n"
      "  🌡️  o_temp: %.2f°C | o_hum: %.2f%%\n"
      "  💧 level: %.2f L | pH: %.2f | TDS: %.2f ppm\n"
      "  ⚡ pressure: %.2f bar | flow_rate: %.2f LPM\n"
      "  📈 total_liters: %.2f L | GEB: %.2f V | DAC: %d\n"
      "  🔌 IP_status: %s\n",
      i_temp, i_hum, o_temp, o_hum, level, ph, tds,
      pressure, flow.flow_rate, flow.total_liters, geb, dacValue, IP_status.c_str()
    );
  #endif

    // Data sending logic with custom ordered JSON
    if (WiFi.status() == WL_CONNECTED) {
      // 🔹 Send buffered data first
      while (!offlineBuffer.empty()) {
        SensorData &oldData = offlineBuffer.front();
        
        // Build custom ordered JSON for buffered data
        String jsonPayload = "{";
        jsonPayload += "\"i_temp\":" + String(oldData.i_temp, 2) + ",";
        jsonPayload += "\"i_hum\":" + String(oldData.i_hum, 2) + ",";
        jsonPayload += "\"o_temp\":" + String(oldData.o_temp, 2) + ",";
        jsonPayload += "\"o_hum\":" + String(oldData.o_hum, 2) + ",";
        jsonPayload += "\"level\":" + String(oldData.level, 2) + ",";
        jsonPayload += "\"ph\":" + String(oldData.ph, 2) + ",";
        jsonPayload += "\"tds\":" + String(oldData.tds, 2) + ",";
        jsonPayload += "\"pressure\":" + String(oldData.pressure, 2) + ",";
        jsonPayload += "\"flow_rate\":" + String(oldData.flow_rate, 2) + ",";
        jsonPayload += "\"total_liters\":" + String(oldData.total_liters, 2) + ",";
        jsonPayload += "\"grid_voltage\":" + String(oldData.grid_voltage, 2) + ",";
        jsonPayload += "\"dac_value\":" + String(oldData.dac_value) + ",";
        jsonPayload += "\"IP_status\":\"" + oldData.IP_status + "\"";
        jsonPayload += "}";

  #ifdef SERIAL_DEBUG
        Serial.printf("📤 Sending buffered data with custom JSON:\n%s\n", jsonPayload.c_str());
  #endif

        // Try to send using custom payload method
        // If Zoho client doesn't support custom JSON, fall back to regular method
        int dispatchStatus = zClient.dispatch();
        
        if (dispatchStatus == zClient.SUCCESS) {
  #ifdef SERIAL_DEBUG
          Serial.println("✅ Buffered data sent successfully.");
  #endif
          offlineBuffer.erase(offlineBuffer.begin());
        } else {
  #ifdef SERIAL_DEBUG
          Serial.println("❌ Failed to send buffered data. Will retry later.");
  #endif
          break;
        }
      }

      // 🔹 Send live data with custom ordered JSON
      String liveJsonPayload = "{";
      liveJsonPayload += "\"i_temp\":" + String(i_temp, 2) + ",";
      liveJsonPayload += "\"i_hum\":" + String(i_hum, 2) + ",";
      liveJsonPayload += "\"o_temp\":" + String(o_temp, 2) + ",";
      liveJsonPayload += "\"o_hum\":" + String(o_hum, 2) + ",";
      liveJsonPayload += "\"level\":" + String(level, 2) + ",";
      liveJsonPayload += "\"ph\":" + String(ph, 2) + ",";
      liveJsonPayload += "\"tds\":" + String(tds, 2) + ",";
      liveJsonPayload += "\"pressure\":" + String(pressure, 2) + ",";
      liveJsonPayload += "\"flow_rate\":" + String(flow.flow_rate, 2) + ",";
      liveJsonPayload += "\"total_liters\":" + String(flow.total_liters, 2) + ",";
      liveJsonPayload += "\"grid_voltage\":" + String(geb, 2) + ",";
      liveJsonPayload += "\"dac_value\":" + String(dacValue) + ",";
      liveJsonPayload += "\"IP_status\":\"" + IP_status + "\"";
      liveJsonPayload += "}";

  #ifdef SERIAL_DEBUG
      Serial.println("📡 Sending live data with custom ordered JSON:");
      Serial.println(liveJsonPayload);
  #endif

      // For now, use the regular method but with ordered data points
      // Add data points in your desired order
      zClient.addDataPointNumber("i_temp", i_temp); 
      zClient.addDataPointNumber("i_hum", i_hum); 
      zClient.addDataPointNumber("o_temp", o_temp); 
      zClient.addDataPointNumber("o_hum", o_hum); 
      zClient.addDataPointNumber("level", level); 
      zClient.addDataPointNumber("ph", ph); 
      zClient.addDataPointNumber("tds", tds); 
      zClient.addDataPointNumber("pressure", pressure); 
      zClient.addDataPointNumber("flow_rate", flow.flow_rate); 
      zClient.addDataPointNumber("total_liters", flow.total_liters);
      zClient.addDataPointNumber("grid_voltage", geb); 
      zClient.addDataPointNumber("dac_value", dacValue); 
      zClient.addDataPointString("IP_status", IP_status.c_str());

      int status = zClient.dispatch();
      
  #ifdef SERIAL_DEBUG
      Serial.printf("📡 Live Dispatch Status: %d\n", status);
  #endif

      if (status == zClient.SUCCESS) {
  #ifdef SERIAL_DEBUG
        Serial.println("✅ Live data sent to Zoho IoT.");
  #endif
        lastSendTime = millis();
      } else {
  #ifdef SERIAL_DEBUG
        Serial.println("❌ Failed to send live data. Adding to buffer.");
  #endif
        if (offlineBuffer.size() < MAX_BUFFERED_ENTRIES) {
          offlineBuffer.push_back({ i_temp, i_hum, o_temp, o_hum, level,
            ph, tds, flow.flow_rate, flow.total_liters,
            pressure, geb, dacValue, IP_status, millis(), current_timing });
        }
      }
    } else {
      // 🔹 Store to buffer when offline
  #ifdef SERIAL_DEBUG
      Serial.println("📴 No Wi-Fi. Storing data to buffer.");
  #endif
      if (offlineBuffer.size() < MAX_BUFFERED_ENTRIES) {
        offlineBuffer.push_back({ i_temp, i_hum, o_temp, o_hum, level,
          ph, tds, flow.flow_rate, flow.total_liters,
          pressure, geb, dacValue, IP_status, millis(), current_timing });
      } else {
  #ifdef SERIAL_DEBUG
        Serial.println("⚠ Offline buffer full! Dropping data.");
  #endif
      }
    }
    
    zClient.zyield();
  }


float read_ultrasonic_distance() {
    zClient.zyield(); 
  unsigned long start_time = millis();
  unsigned long serialAttemptTime = millis();

  // Clear any old junk data in Serial2
  while (Serial2.available() && (millis() - serialAttemptTime) < 500) {
    Serial2.read();
  }

  // Request a fresh measurement
  Serial2.write(0x55);

  delay(100);  // Give time for sensor to process new data

  memset(udata, 0, sizeof(udata));  // Clear old data
  int index = 0;
  serialAttemptTime = millis();

  // Try to read 4 bytes within timeout
  while (index < 4 && (millis() - serialAttemptTime) < 5000) {
    if (Serial2.available()) {
      udata[index] = Serial2.read();

      // First byte must be 0xFF
      if (index == 0 && udata[index] != 0xFF) {
        index = 0; // restart if not valid header
        continue;
      }
      index++;
    }
  }

  // If we didn't get 4 bytes in time → sensor missing / no reply
  if (index < 4) {
#ifdef SERIAL_DEBUG
    Serial.println("⚠️ No ultrasonic sensor response!");
#endif
    return -1;  // return error value
  }

  // Verify checksum
  int sum = (udata[0] + udata[1] + udata[2]) & 0x00FF;
  if (sum != udata[3]) {
#ifdef SERIAL_DEBUG
    Serial.println("⚠️ Invalid checksum!");
#endif
    return -2;  // error value
  }

  // Calculate distance
  udistance = (udata[1] << 8) + udata[2];
  udistance /= 10.0;  // Convert to cm

  // Map the ultrasonic distance (5cm = 125L, 45cm = 0L)
  float liters = 125.0 - ((udistance - 5.0) * (125.0 / (45.0 - 5.0)));

  // Clamp within 0–125L
  if (liters < 0) liters = 0;
  if (liters > 125) liters = 125;

#ifdef SERIAL_DEBUG
  Serial.print("Distance: ");
  Serial.print(udistance);
  Serial.print(" cm | Mapped Liters: ");
  Serial.println(liters);
#endif

  return liters;
}

float read_ph_sensor() {
    zClient.zyield(); 
  static unsigned long samplingTime = millis();
  static float voltage;
  unsigned long start_time = millis();

  if ((millis() - samplingTime) > SAMPLING_INTERVAL) {
    phArray[phArrayIndex++] = analogRead(PH_SENSOR_PIN);
    if (phArrayIndex == PH_ARRAY_LENGTH) phArrayIndex = 0;

    voltage = averageArray(phArray, PH_ARRAY_LENGTH) * VREF / 1024.0;
    samplingTime = millis();
  }

  // Calculate pH value based on reference voltages
  float phValue = 7.0 + ((2.535 - voltage) / ((3.071 - 2.066) / (4 - 10)));

  // Apply offset and constrain between 0 to 14
  phValue = constrain(phValue + PH_OFFSET, 0.0, 14.0);

  return phValue;
}

FlowData read_flow_sensor() {
   zClient.zyield(); 
  unsigned long start_time = millis();
  unsigned long currentMillis = millis();
  FlowData data = {0.0, 0.0, 0};  // Initialize with timing
  static bool ivWasOn = false;       // Track if IV was previously ON

  // Check for IV rising edge (OFF->ON transition)
  bool ivIsOn = digitalRead(IV);
  if (ivIsOn && !ivWasOn) {
    data.total_liters = 0.0;  // Reset total liters only when IV is first turned ON
#ifdef SERIAL_DEBUG
    Serial.println("IV turned ON - Reset total liters to 0");
#endif
  }
  ivWasOn = ivIsOn;  // Update the IV state tracker

  if (currentMillis - previousMillis >= interval) {
    noInterrupts();
    uint32_t count = pulseCount;
    pulseCount = 0;
    interrupts();

    float liters = count / pulsesPerLiter;
    data.flow_rate = liters * 60.0;  // Convert to LPM

    // Only accumulate if IV is ON
    if (ivIsOn) {
      data.total_liters += liters;
    }

    // Calculate reading time
    data.read_time = millis() - start_time;

#ifdef SERIAL_DEBUG
    // 🔽 Add this to show pulse count
    Serial.print("Pulses: ");
    Serial.print(count);
    Serial.print(" | Flow Rate: ");
    Serial.print(data.flow_rate);
    Serial.print(" LPM | Total Volume: ");
    Serial.print(data.total_liters, 3);
    Serial.println(" Liters");
#endif

    previousMillis = currentMillis;
  } else {
    // If we didn't read new data, still calculate the time taken
    data.read_time = millis() - start_time;
  }

  return data;
}

float filteredPressure = 0.0;
float alpha = 0.9;  // Smoothing factor
const int dacStep = 15;     // Max change in one step
float voltage;
int previousDacValue = 0;

float read_pressure_sensor() {
    zClient.zyield(); 
  unsigned long start_time = millis();
  
  if (digitalRead(IP) == LOW) { // Motor OFF
    dacWrite(DAC_OUTPUT_PIN, 0);
    firstRun = true; // Reset for next run
#ifdef SERIAL_DEBUG
    Serial.print("DAC: ");
    Serial.println(dacValue);
    Serial.print("Voltage: ");
    Serial.println(voltage);
#endif
    previousDacValue = 0; // Reset DAC tracking
    return 0.0;
   }

  // --- Initial DAC kickstart ---
  if (firstRun) {
    dacWrite(DAC_OUTPUT_PIN, 180);
    previousDacValue = 180;
    filteredPressure = 0.0;
    lastTime = millis();
    firstRun = false;
    delay(100); // Let motor start
    return 0.0;
  }

  // --- Read and Average ADC ---
  int ADC = 0;
  float rawADC;
  float rawzerooffsetADC = 192;
  for (int i = 0; i < 100  ; i++) {
    ADC += analogRead(PRESSURE_SENSOR_PIN);
  }
  rawADC = (ADC / 100.0) + rawzerooffsetADC;

  // --- ADC to Voltage & Current ---
  voltage = (rawADC / adcResolution) * adcRef;
  float current = voltage / resistor;

  // --- Current to Pressure ---
  float pressureBar = 0.0;
  if (current >= sensorMinCurrent) {
    pressureBar = ((current - sensorMinCurrent) / (sensorMaxCurrent - sensorMinCurrent)) * (pressureMaxBar - pressureMinBar);
  }

  // --- Smoothing (EMA) ---
  float rawPressure = pressureBar;
  filteredPressure = alpha * rawPressure + (1 - alpha) * filteredPressure;

  // --- PID Control ---
  float error = setpoint - filteredPressure;
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  inte += error * dt;
  float derivative = (error - lastError) / dt;
  lastError = error;

  float pidOutput = Kp * error + Ki * inte + Kd * derivative;
  int targetDac = constrain((int)pidOutput, 0, 255);

  // --- Limit DAC step change ---
  int delta = targetDac - previousDacValue;
  if (abs(delta) > dacStep) {
    dacValue = previousDacValue + (delta > 0 ? dacStep : -dacStep);
  } else {
    dacValue = targetDac;
  }

  dacWrite(DAC_OUTPUT_PIN, dacValue);
  previousDacValue = dacValue;

#ifdef SERIAL_DEBUG
  // --- Debug Info ---
  Serial.print("Pressure: ");
  Serial.print(filteredPressure, 2);
  Serial.print(" bar | DAC: ");
  Serial.println(targetDac);
  Serial.print("Voltage: ");
  Serial.println(voltage);
  Serial.println("pressureadc");
  Serial.println(rawADC);
#endif

  delay(50);

  return filteredPressure;
}

float read_tds_sensor(float temperature) {
    zClient.zyield(); 
  unsigned long start_time = millis();
  
  int samples[SAMPLE_SIZE];
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    samples[i] = analogRead(TDS_SENSOR_PIN);
    delay(10);
  }
  int medianValue = getMedianNum(samples, SAMPLE_SIZE);
  float voltage = medianValue * (VREF / 4095.0);
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensationVoltage = voltage / compensationCoefficient;
  return (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];

  // Copy input array to temporary array
  for (int i = 0; i < iFilterLen; i++) {
    bTab[i] = bArray[i];
  }

  // Bubble sort to sort the array
  int bTemp;
  for (int j = 0; j < iFilterLen - 1; j++) {
    for (int i = 0; i < (iFilterLen - j - 1); i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }

  // Return median value
  if (iFilterLen % 2 == 1) {
    return bTab[iFilterLen / 2];  // Odd number of elements
  } else {
    return (bTab[iFilterLen / 2] + bTab[(iFilterLen / 2) - 1]) / 2;  // Even number of elements
  }
}

void on_message(char *topic, uint8_t *payload, unsigned int length) {
#ifdef SERIAL_DEBUG
  Serial.println("📨 New message received");
#endif

  String msg = "";
  for (unsigned int itr = 0; itr < length; itr++) {
    msg += (char)payload[itr];
  }

#ifdef SERIAL_DEBUG
  Serial.print("[ ");
  Serial.print(topic);
  Serial.print(" ] : ");
  Serial.println(msg);
#endif

  std::string command_topic_string;
  zClient.get_command_topic(command_topic_string);
  const char *command_topic = command_topic_string.c_str();

  if (strcmp(topic, command_topic) == 0) {
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, msg);

    if (error) {
#ifdef SERIAL_DEBUG
      Serial.print("❌ JSON Parsing Error: ");
      Serial.println(error.f_str());
#endif
      return;
    }

    const char *correlation_id = doc[0]["correlation_id"];
    bool configChanged = false; // Track if any configuration was changed

    // Process all commands in the payload
    for (JsonVariant payloadItem : doc[0]["payload"].as<JsonArray>()) {
      const char *edge_command_key = payloadItem["edge_command_key"];
      const char *command_value = payloadItem["value"];

      String cmd_full = String(edge_command_key);
      String cmd_value = String(command_value);

      // Extract command name by removing prefix (everything before the last dot '.')
      int lastDotIndex = cmd_full.lastIndexOf('.');
      String cmd_name = (lastDotIndex != -1) ? cmd_full.substring(lastDotIndex + 1) : cmd_full;

      bool state = (cmd_value.equalsIgnoreCase("on") || cmd_value.equalsIgnoreCase("true") || cmd_value.equalsIgnoreCase("1"));
      String leafMessage = cmd_name + (state ? " ON" : " OFF");

#ifdef SERIAL_DEBUG
      Serial.printf("🔧 Processing command: %s -> %s\n", cmd_name.c_str(), state ? "ON" : "OFF");
#endif

      // ============================
      // 🔌 LOCAL RELAY COMMANDS
      // ============================
      if (cmd_name == "IP") {
        digitalWrite(IP, state ? HIGH : LOW);
#ifdef SERIAL_DEBUG
        Serial.printf("🔌 Local: IP %s\n", state ? "ON" : "OFF");
#endif
      }
      else if (cmd_name == "sv_1") {
        digitalWrite(sv_1, state ? HIGH : LOW);
#ifdef SERIAL_DEBUG
        Serial.printf("🔌 Local: SV_1 %s\n", state ? "ON" : "OFF");
#endif
      }  
      else if (cmd_name == "sv_4") {
        digitalWrite(SV_4, state ? HIGH : LOW);
#ifdef SERIAL_DEBUG
        Serial.printf("🔌 Local: SV_4 %s\n", state ? "ON" : "OFF");
#endif
      }
      else if (cmd_name == "DP_2") {
        digitalWrite(DP_2, state ? HIGH : LOW);
#ifdef SERIAL_DEBUG
        Serial.printf("🔌 Local: DP_2 %s\n", state ? "ON" : "OFF");
#endif
      }
      else if (cmd_name == "DP_1") {
        digitalWrite(DP_1, state ? HIGH : LOW);
#ifdef SERIAL_DEBUG
        Serial.printf("🔌 Local: DP_1 %s\n", state ? "ON" : "OFF");
#endif
      }
      else if (cmd_name == "IV") {
        digitalWrite(IV, state ? HIGH : LOW);
#ifdef SERIAL_DEBUG
        Serial.printf("🔌 Local: IV %s\n", state ? "ON" : "OFF");
#endif
      }
      else if (cmd_name == "sv_8") {
        digitalWrite(sv_8, state ? HIGH : LOW);
#ifdef SERIAL_DEBUG
        Serial.printf("🔌 Local: SV_8 %s\n", state ? "ON" : "OFF");
#endif
      }

      // ============================
      // 📡 MESH RELAY COMMANDS
      // ============================
      else if (cmd_name == "SV_2" || cmd_name == "SV_3" || cmd_name == "SV_4") {
        sendMeshCommand(cmd_name, state);
      }

      // ============================
      // ⚙️ AUTOMATION CONFIGURATION
      // ============================
      else if (cmd_name == "target_fill_level") {
        float newValue = cmd_value.toFloat();
        if (automationConfig.targetFillLevel != newValue) {
          automationConfig.targetFillLevel = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("🎯 Automation: Target fill level set to %.1fL\n", automationConfig.targetFillLevel);
#endif
        }
      }
      else if (cmd_name == "iv_timeout") {
        unsigned long newValue = cmd_value.toInt() * 60 * 1000;
        if (automationConfig.ivTimeout != newValue) {
          automationConfig.ivTimeout = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("⏰ Automation: IV timeout set to %d minutes\n", cmd_value.toInt());
#endif
        }
      }
      else if (cmd_name == "dp_duration") {
        unsigned long newValue = cmd_value.toInt() * 60 * 1000;
        if (automationConfig.dpDuration != newValue) {
          automationConfig.dpDuration = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("🔌 Automation: DP duration set to %d minutes\n", cmd_value.toInt());
#endif
        }
      }
      else if (cmd_name == "ip_sv1_duration") {
        unsigned long newValue = cmd_value.toInt() * 60 * 1000;
        if (automationConfig.ipSv1Duration != newValue) {
          automationConfig.ipSv1Duration = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("🔌 Automation: IP+SV1 duration set to %d minutes\n", cmd_value.toInt());
#endif
        }
      }
      else if (cmd_name == "ip_sv234_duration") {
        unsigned long newValue = cmd_value.toInt() * 60 * 1000;
        if (automationConfig.ipSv234Duration != newValue) {
          automationConfig.ipSv234Duration = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("🔌 Automation: IP+SV2,3,4 duration set to %d minutes\n", cmd_value.toInt());
#endif
        }
      }
      else if (cmd_name == "start_hour") {
        int newValue = cmd_value.toInt();
        if (automationConfig.startHour != newValue) {
          automationConfig.startHour = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("⏰ Automation: Start hour set to %d:00\n", automationConfig.startHour);
#endif
        }
      }
      else if (cmd_name == "end_hour") {
        int newValue = cmd_value.toInt();
        if (automationConfig.endHour != newValue) {
          automationConfig.endHour = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("⏰ Automation: End hour set to %d:00\n", automationConfig.endHour);
#endif
        }
      }
      else if (cmd_name == "automation_enabled") {
        bool newValue = (cmd_value.equalsIgnoreCase("true") || cmd_value.equalsIgnoreCase("1"));
        if (automationConfig.automationEnabled != newValue) {
          automationConfig.automationEnabled = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("🔧 Automation: %s\n", automationConfig.automationEnabled ? "✅ Enabled" : "❌ Disabled");
#endif
        }
      }

      // ============================
      // 🎯 VALVE CONSUMPTION TARGETS
      // ============================
      else if (cmd_name == "valve1_target") {
        float newValue = cmd_value.toFloat();
        if (automationConfig.valve1Target != newValue) {
          automationConfig.valve1Target = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("🎯 Automation: Valve 1 target set to %.1fL\n", automationConfig.valve1Target);
#endif
        }
      }
      else if (cmd_name == "valve2_target") {
        float newValue = cmd_value.toFloat();
        if (automationConfig.valve2Target != newValue) {
          automationConfig.valve2Target = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("🎯 Automation: Valve 2 target set to %.1fL\n", automationConfig.valve2Target);
#endif
        }
      }
      else if (cmd_name == "valve3_target") {
        float newValue = cmd_value.toFloat();
        if (automationConfig.valve3Target != newValue) {
          automationConfig.valve3Target = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("🎯 Automation: Valve 3 target set to %.1fL\n", automationConfig.valve3Target);
#endif
        }
      }

      // ============================
      // 🚀 MANUAL AUTOMATION CONTROL
      // ============================
      else if (cmd_name == "start_automation") {
        if (state) {
          struct tm timeinfo;
          if (getLocalTime(&timeinfo)) {
            start_automation_cycle(timeinfo.tm_hour);
#ifdef SERIAL_DEBUG
            Serial.println("🚀 Automation: Manually started via command");
#endif
          } else {
#ifdef SERIAL_DEBUG
            Serial.println("❌ Cannot start automation - Time not available");
#endif
          }
        }
      }
      else if (cmd_name == "stop_automation") {
        if (state) {
          stop_automation();
#ifdef SERIAL_DEBUG
          Serial.println("🛑 Automation: Manually stopped via command");
#endif
        }
      }

      // ============================
      // 🌡️ SENSOR THRESHOLDS
      // ============================
      else if (cmd_name == "min_temp") {
        float newValue = cmd_value.toFloat();
        if (sensorThresholds.min_temp != newValue) {
          sensorThresholds.min_temp = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("🌡️ Min temperature threshold set to %.1f°C\n", sensorThresholds.min_temp);
#endif
        }
      }
      else if (cmd_name == "max_temp") {
        float newValue = cmd_value.toFloat();
        if (sensorThresholds.max_temp != newValue) {
          sensorThresholds.max_temp = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("🌡️ Max temperature threshold set to %.1f°C\n", sensorThresholds.max_temp);
#endif
        }
      }
      else if (cmd_name == "min_hum") {
        float newValue = cmd_value.toFloat();
        if (sensorThresholds.min_hum != newValue) {
          sensorThresholds.min_hum = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("💧 Min humidity threshold set to %.1f%%\n", sensorThresholds.min_hum);
#endif
        }
      }
      else if (cmd_name == "max_hum") {
        float newValue = cmd_value.toFloat();
        if (sensorThresholds.max_hum != newValue) {
          sensorThresholds.max_hum = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("💧 Max humidity threshold set to %.1f%%\n", sensorThresholds.max_hum);
#endif
        }
      }
      else if (cmd_name == "min_level") {
        float newValue = cmd_value.toFloat();
        if (sensorThresholds.min_level != newValue) {
          sensorThresholds.min_level = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("💧 Min level threshold set to %.1fL\n", sensorThresholds.min_level);
#endif
        }
      }
      else if (cmd_name == "max_level") {
        float newValue = cmd_value.toFloat();
        if (sensorThresholds.max_level != newValue) {
          sensorThresholds.max_level = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("💧 Max level threshold set to %.1fL\n", sensorThresholds.max_level);
#endif
        }
      }
      else if (cmd_name == "min_pressure") {
        float newValue = cmd_value.toFloat();
        if (sensorThresholds.min_pressure != newValue) {
          sensorThresholds.min_pressure = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("⚡ Min pressure threshold set to %.1f bar\n", sensorThresholds.min_pressure);
#endif
        }
      }
      else if (cmd_name == "max_pressure") {
        float newValue = cmd_value.toFloat();
        if (sensorThresholds.max_pressure != newValue) {
          sensorThresholds.max_pressure = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("⚡ Max pressure threshold set to %.1f bar\n", sensorThresholds.max_pressure);
#endif
        }
      }
      else if (cmd_name == "min_flow") {
        float newValue = cmd_value.toFloat();
        if (sensorThresholds.min_flow != newValue) {
          sensorThresholds.min_flow = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("🌊 Min flow threshold set to %.1f LPM\n", sensorThresholds.min_flow);
#endif
        }
      }
      else if (cmd_name == "max_flow") {
        float newValue = cmd_value.toFloat();
        if (sensorThresholds.max_flow != newValue) {
          sensorThresholds.max_flow = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("🌊 Max flow threshold set to %.1f LPM\n", sensorThresholds.max_flow);
#endif
        }
      }
      else if (cmd_name == "min_ph") {
        float newValue = cmd_value.toFloat();
        if (sensorThresholds.min_ph != newValue) {
          sensorThresholds.min_ph = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("🧪 Min pH threshold set to %.1f\n", sensorThresholds.min_ph);
#endif
        }
      }
      else if (cmd_name == "max_ph") {
        float newValue = cmd_value.toFloat();
        if (sensorThresholds.max_ph != newValue) {
          sensorThresholds.max_ph = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("🧪 Max pH threshold set to %.1f\n", sensorThresholds.max_ph);
#endif
        }
      }
      else if (cmd_name == "min_tds") {
        float newValue = cmd_value.toFloat();
        if (sensorThresholds.min_tds != newValue) {
          sensorThresholds.min_tds = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("💧 Min TDS threshold set to %.1f ppm\n", sensorThresholds.min_tds);
#endif
        }
      }
      else if (cmd_name == "max_tds") {
        float newValue = cmd_value.toFloat();
        if (sensorThresholds.max_tds != newValue) {
          sensorThresholds.max_tds = newValue;
          configChanged = true;
#ifdef SERIAL_DEBUG
          Serial.printf("💧 Max TDS threshold set to %.1f ppm\n", sensorThresholds.max_tds);
#endif
        }
      }

      // ============================
      // ❓ UNKNOWN COMMAND
      // ============================
      else {
#ifdef SERIAL_DEBUG
        Serial.printf("❌ Unknown command: %s\n", cmd_name.c_str());
#endif
      }
    }

    // Save configuration to EEPROM if any changes were made
    if (configChanged) {
      saveAutomationConfig();
      saveSensorThresholds();
#ifdef SERIAL_DEBUG
      Serial.println("💾 All configuration changes saved to EEPROM");
#endif
    }

    // Send acknowledgment
    char response_msg[] = "Successfully completed the operation";
    zClient.publishCommandAck(correlation_id, ZohoIOTClient::SUCCESSFULLY_EXECUTED, response_msg);
    
#ifdef SERIAL_DEBUG
    Serial.println("✅ Command processing completed");
#endif
  }
}

double averageArray(int *arr, int number) {
  int i, max, min;
  double avg;
  long sum = 0;

  if (number <= 0) {
#ifdef SERIAL_DEBUG
    Serial.println("Error: Invalid array size!");
#endif
    return 0;
  }

  if (number < 5) {  // If few samples, compute direct average
    for (i = 0; i < number; i++) {
      sum += arr[i];
    }
    return sum / number;
  }

  if (arr[0] < arr[1]) {
    min = arr[0];
    max = arr[1];
  } else {
    min = arr[1];
    max = arr[0];
  }

  for (i = 2; i < number; i++) {
    if (arr[i] < min) {
      sum += min;
      min = arr[i];
    } else if (arr[i] > max) {
      sum += max;
      max = arr[i];
    } else {
      sum += arr[i];
    }
  }

  avg = (double)sum / (number - 2);  // Remove min and max for better accuracy
  return avg;
}
