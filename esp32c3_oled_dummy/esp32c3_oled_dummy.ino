// v2.1 - Battery Optimized with Optional Serial Logging
// Kode untuk ESP32 yang terhubung ke nRF52840 dan Smartphone dengan OLED display dan SpO2 dummy

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Wire.h>
#include "MAX30105.h" 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Logo dimensions
const int LOGO_WIDTH = 17;
const int LOGO_HEIGHT = 17;

// Logo bitmaps
const unsigned char phone1[] PROGMEM = {
  0x00, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08,
  0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08,
  0x00, 0x08, 0x08, 0x00, 0x0f, 0xf8, 0x00, 0x0e, 0x38, 0x00, 0x0f, 0xf8, 0x00, 0x0f, 0xf8, 0x00,
  0x00, 0x00, 0x00
};

const unsigned char no_logo[] PROGMEM = {
  0x00, 0x00, 0x80, 0x0f, 0xf9, 0x00, 0x08, 0x0a, 0x00, 0x08, 0x0c, 0x00, 0x08, 0x08, 0x00, 0x08,
  0x18, 0x00, 0x08, 0x28, 0x00, 0x08, 0x48, 0x00, 0x08, 0x88, 0x00, 0x09, 0x08, 0x00, 0x0a, 0x08,
  0x00, 0x0c, 0x08, 0x00, 0x0f, 0xf8, 0x00, 0x1e, 0x38, 0x00, 0x2f, 0xf8, 0x00, 0x4f, 0xf8, 0x00,
  0x80, 0x00, 0x00
};

// UUIDs
static BLEUUID serviceUUID("12345678-1234-5678-1234-56789abcdef0");  
static BLEUUID charUUID("12345678-1234-5678-1234-56789abcdef1");
static BLEUUID writeCharUUID("87654321-4321-8765-4321-123456789abc");

// BLE Client Variables
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice;

// BLE Server Variables
BLEServer *pServer = nullptr;
BLECharacteristic *pWriteCharacteristic;
BLEAdvertising *pAdvertising;
bool deviceConnected = false;

// Data Storage Variables
uint8_t lastSnoreStatus = 0;
uint32_t lastTimestamp = 0;
volatile bool newDataFromNrf = false;

// MAX30102 Variables (dummy values)
MAX30105 particleSensor;
double ESpO2 = 95.0;
double FSpO2 = 0.7;

// Flag untuk Serial Logging
bool enableSerialLogging = false;

// Function declarations
void updateDisplay();
void calculateSpO2();
void setupBLEServer();
void checkSerialConnection();

// Check for USB connection
void checkSerialConnection() {
  if(Serial) {
    if(!enableSerialLogging) {
      enableSerialLogging = true;
      Serial.println("\n\nSerial logging enabled (USB connected)");
      Serial.println("ESP32 OLED Display Controller - v2.1");
    }
  } else {
    enableSerialLogging = false;
  }
}

// Update OLED Display
void updateDisplay() {
  display.clearDisplay();
  
  // Draw connection status icon in top right
  int logoX = SCREEN_WIDTH - LOGO_WIDTH;
  if (deviceConnected) {
    display.drawBitmap(logoX, 0, phone1, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
  } else {
    display.drawBitmap(logoX, 0, no_logo, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
  }

  // Display SpO2 value
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 25);
  display.print("SpO2: ");
  display.print((int)ESpO2);
  display.print("%");

  display.display();
}

// Callback for BLE Client notifications
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, 
                          uint8_t *pData, size_t length, bool isNotify) {
  if (length == 5) {
    lastSnoreStatus = pData[0];
    memcpy(&lastTimestamp, &pData[1], 4);
    
    if(enableSerialLogging) {
      Serial.print("Received - Snore: ");
      Serial.print(lastSnoreStatus);
      Serial.print(", Timestamp: ");
      Serial.println(lastTimestamp);
    }
    
    newDataFromNrf = true;
    updateDisplay();
  }
}

// BLE Client Callbacks
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) {
    connected = true;
    if(enableSerialLogging) Serial.println("Connected to nRF52840");
    updateDisplay();
  }

  void onDisconnect(BLEClient *pclient) {
    connected = false;
    if(enableSerialLogging) Serial.println("Disconnected from nRF52840");
    updateDisplay();
  }
};

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      if(enableSerialLogging) Serial.println("Smartphone connected");
      updateDisplay();
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      if(enableSerialLogging) Serial.println("Smartphone disconnected");
      delay(500);
      pAdvertising->start();
      if(enableSerialLogging) Serial.println("Advertising restarted");
      updateDisplay();
    }
};

void calculateSpO2() {
  ESpO2 = 96; // Static dummy value for SpO2
  updateDisplay();
}

bool connectToServer() {
  if(enableSerialLogging) {
    Serial.print("Connecting to ");
    Serial.println(myDevice->getAddress().toString().c_str());
  }
    
  BLEClient* pClient = BLEDevice::createClient();
  if(enableSerialLogging) Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  if (!pClient->connect(myDevice)) {
    if(enableSerialLogging) Serial.println(" - Failed to connect");
    return false;
  }
  if(enableSerialLogging) Serial.println(" - Connected to server");

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    if(enableSerialLogging) {
      Serial.print("Failed to find service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
    }
    pClient->disconnect();
    return false;
  }
  if(enableSerialLogging) Serial.println(" - Found service");

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    if(enableSerialLogging) {
      Serial.print("Failed to find characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
    }
    pClient->disconnect();
    return false;
  }
  if(enableSerialLogging) Serial.println(" - Found characteristic");

  if(pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    if(enableSerialLogging) Serial.println(" - Registered for notifications");
  }

  connected = true;
  return true;
}

// Scan callback
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if(enableSerialLogging) {
      Serial.print("BLE Device found: ");
      Serial.println(advertisedDevice.toString().c_str());
    }

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      if(enableSerialLogging) Serial.println("Found our device! Now connecting...");
    }
  }
};

void setupBLEServer() {
  if(enableSerialLogging) Serial.println("Setting up BLE Server...");
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(serviceUUID);

  pWriteCharacteristic = pService->createCharacteristic(
                                   writeCharUUID,
                                   BLECharacteristic::PROPERTY_READ |
                                   BLECharacteristic::PROPERTY_NOTIFY |
                                   BLECharacteristic::PROPERTY_WRITE_NR
                                 );

  pService->start();

  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(serviceUUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  if(enableSerialLogging) Serial.println("BLE Server setup completed. Advertising...");
}

void setup() {
  // Initialize Serial but don't wait for it
  Serial.begin(115200);
  
  // Check USB connection for logging
  checkSerialConnection();
  
  if(enableSerialLogging) {
    Serial.println("ESP32 OLED Display Controller - v2.1");
    Serial.println("Starting with optional serial logging");
  }

  // Initialize I2C
  Wire.begin();

  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    if(enableSerialLogging) Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.display();
  if(enableSerialLogging) Serial.println("OLED Display initialized");

  // Initialize BLE
  BLEDevice::init("ESP32");
  
  // Setup BLE scanning (as client)
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
  if(enableSerialLogging) Serial.println("BLE Scan started");

  // Setup BLE server
  setupBLEServer();

  // Initial display update
  updateDisplay();
}

void loop() {
  // Check USB connection status periodically
  static unsigned long lastCheck = 0;
  if(millis() - lastCheck > 1000) {
    checkSerialConnection();
    lastCheck = millis();
  }

  // Handle connection to nRF52840
  if (doConnect) {
    if (connectToServer()) {
      if(enableSerialLogging) Serial.println("Successfully connected to nRF52840");
    } else if(enableSerialLogging) {
      Serial.println("Failed to connect to nRF52840");
    }
    doConnect = false;
  }

  // Calculate and update SpO2 (dummy value)
  calculateSpO2();

  // Send data to smartphone if new data from nRF
  if (newDataFromNrf) {
    if (deviceConnected) {
      String jsonData = "{\"status\":" + String(lastSnoreStatus) + 
                       ",\"timestamp\":" + String(lastTimestamp) + 
                       ",\"spo2\":" + String(ESpO2, 2) + "}";
      
      pWriteCharacteristic->setValue(jsonData.c_str());
      pWriteCharacteristic->notify();
      
      if(enableSerialLogging) {
        Serial.print("Sent to smartphone: ");
        Serial.println(jsonData);
      }
    } else if(enableSerialLogging) {
      Serial.println("Smartphone not connected. Cannot send data.");
    }
    newDataFromNrf = false;
  }

  delay(50); // Small delay for stability
}