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

// MAX30102 Variables
MAX30105 particleSensor;
double ESpO2 = 95.0;
double FSpO2 = 0.7;

// SpO2 Calculation Variables
double avered = 0; 
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int sampleCount = 0;
const int NumSamples = 100;
const int FINGER_ON = 30000;

// Function declarations
void updateDisplay();
void calculateSpO2();
void setupMAX30102();
void setupBLEServer();

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
    Serial.print("Received - Snore: ");
    Serial.print(lastSnoreStatus);
    Serial.print(", Timestamp: ");
    Serial.println(lastTimestamp);
    
    newDataFromNrf = true;
    updateDisplay(); // Update display when new data arrives
  }
}

// BLE Client Callbacks
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) {
    Serial.println("Connected to nRF52840");
    updateDisplay();
  }

  void onDisconnect(BLEClient *pclient) {
    connected = false;
    Serial.println("Disconnected from nRF52840");
    updateDisplay();
  }
};

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Smartphone connected");
      updateDisplay();
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Smartphone disconnected");
      delay(500);
      pAdvertising->start();
      updateDisplay();
    }
};

void calculateSpO2() {
  ESpO2 = 96; // Static test value for SpO2
  updateDisplay(); // Update display whenever SpO2 value changes
}

bool connectToServer() {
  Serial.print("Connecting to ");
  Serial.println(myDevice->getAddress().toString().c_str());
    
  BLEClient* pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remote BLE Server
  if (!pClient->connect(myDevice)) {
    Serial.println(" - Failed to connect");
    return false;
  }
  Serial.println(" - Connected to server");

  // Get the service
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("Failed to find our service UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  // Get the characteristic
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("Failed to find our characteristic UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  // Read the value of the characteristic
  if(pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  if(pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    Serial.println(" - Registered for notifications");
  }

  connected = true;
  return true;
}

// Scan callback
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      Serial.println("Found our device! Now connecting...");
    }
  }
};

void setupBLEServer() {
  Serial.println("Setting up BLE Server...");
  
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(serviceUUID);

  // Create BLE Characteristic
  pWriteCharacteristic = pService->createCharacteristic(
                                   writeCharUUID,
                                   BLECharacteristic::PROPERTY_READ |
                                   BLECharacteristic::PROPERTY_NOTIFY |
                                   BLECharacteristic::PROPERTY_WRITE_NR
                                 );

  // Start the service
  pService->start();

  // Start advertising
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(serviceUUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE Server setup completed. Advertising...");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32 OLED Display with BLE...");

  // Initialize I2C
  Wire.begin();

  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.display();
  Serial.println("OLED Display initialized");

  // Initialize BLE
  BLEDevice::init("ESP32");
  
  // Setup BLE scanning (as client)
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
  Serial.println("BLE Scan started");

  // Setup BLE server
  setupBLEServer();

  // Initial display update
  updateDisplay();
}

void loop() {
  // Handle connection to nRF52840
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Successfully connected to nRF52840");
    } else {
      Serial.println("Failed to connect to nRF52840");
    }
    doConnect = false;
  }

  // Calculate and update SpO2
  calculateSpO2();

  // Send data to smartphone if new data from nRF
  if (newDataFromNrf) {
    if (deviceConnected) {
      String jsonData = "{\"status\":" + String(lastSnoreStatus) + 
                       ",\"timestamp\":" + String(lastTimestamp) + 
                       ",\"spo2\":" + String(ESpO2, 2) + "}";
      
      pWriteCharacteristic->setValue(jsonData.c_str());
      pWriteCharacteristic->notify();
      
      Serial.print("Sent to smartphone: ");
      Serial.println(jsonData);
    }
    newDataFromNrf = false;
  }

  delay(50); // Small delay for stability
}