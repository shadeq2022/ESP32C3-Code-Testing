#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// UUIDs for BLE services and characteristics
static BLEUUID serviceUUID("12345678-1234-5678-1234-56789abcdef0");  
static BLEUUID charUUID("12345678-1234-5678-1234-56789abcdef1");  
static BLEUUID writeCharUUID("87654321-4321-8765-4321-123456789abc");  

// BLE client variables (to receive data from nRF52840)
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice;

// BLE server variables (to send data to smartphone)
BLEServer *pServer = nullptr;
BLECharacteristic *pWriteCharacteristic;
BLEAdvertising *pAdvertising;
bool deviceConnected = false;

// Variables to store received data from nRF52840
uint8_t lastStatus = 0;
uint32_t lastTimestamp = 0;

// Simulated SpO2 data
int spo2 = 95; // Initial SpO2 value

// Callback function to handle received notifications from nRF52840
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
  if (length == 5) { // Ensure data length is correct (1 byte status + 4 bytes timestamp)
    lastStatus = pData[0]; // Byte 0: Status
    memcpy(&lastTimestamp, &pData[1], 4); // Byte 1-4: Timestamp

    Serial.print("Received from nRF52840 - Status: ");
    Serial.print(lastStatus);
    Serial.print(", Timestamp: ");
    Serial.println(lastTimestamp);

    // Simulate SpO2 change (for testing purposes)
    spo2 = random(94, 100); // Random SpO2 between 94-100%
  }
}

// BLE client callback
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) {}
  void onDisconnect(BLEClient *pclient) {
    connected = false;
    Serial.println("Disconnected from nRF52840");
  }
};

// Function to connect to the nRF52840
bool connectToServer() {
  Serial.print("Connecting to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient *pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  if (!pClient->connect(myDevice)) {
    Serial.println("Failed to connect");
    return false;
  }
  Serial.println("Connected to nRF52840");

  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("Failed to find service UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println("Found service");

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("Failed to find characteristic UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println("Found characteristic");

  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
  }

  connected = true;
  return true;
}

// BLE scan callback
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
    }
  }
};

// BLE Server callbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Smartphone connected.");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Smartphone disconnected. Restarting advertising...");
    delay(1000);  // Delay to ensure the connection is fully closed
    pAdvertising->start();  // Restart advertising after disconnection
  }
};

// BLE Server setup
void setupBLEServer() {
  BLEDevice::init("ESP32_BLE_Server");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(serviceUUID);

  // Characteristic to write data (to be read by smartphone)
  pWriteCharacteristic = pService->createCharacteristic(
    writeCharUUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pWriteCharacteristic->setValue("0"); // Initial value
  pService->start();

  // Start advertising
  pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  Serial.println("BLE Server started, waiting for smartphone to connect...");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Client and Server...");

  // Initialize BLE client to connect to nRF52840
  BLEDevice::init("");
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  // Initialize BLE server to be accessed by smartphone
  setupBLEServer();

  // Seed random number generator for SpO2 simulation
  randomSeed(analogRead(0));
}

void loop() {
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Connected to nRF52840");
    } else {
      Serial.println("Failed to connect to nRF52840");
    }
    doConnect = false;
  }

  if (connected) {
    delay(1000);  // Wait for notifications
  } else {
    BLEDevice::getScan()->start(0);
  }

  // Send data to smartphone if connected
  if (deviceConnected) {
    // Prepare JSON data
    String jsonData = "{\"status\":" + String(lastStatus) + 
                      ",\"timestamp\":" + String(lastTimestamp) + 
                      ",\"spo2\":" + String(spo2) + "}";

    // Send data to smartphone
    pWriteCharacteristic->setValue(jsonData.c_str());
    pWriteCharacteristic->notify();

    Serial.print("Sent data to smartphone: ");
    Serial.println(jsonData);

    delay(1000);  // Send data every 1 second
  }
}