// v2.1 - Battery Optimized with Optional Serial Logging
// Kode untuk ESP32 yang terhubung ke nRF52840 dan Smartphone cuman spo2 dummy karna sensor max belum pasang

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Wire.h>
#include "MAX30105.h" // Pastikan library MAX30105 sudah terinstall

// UUIDs
static BLEUUID serviceUUID("12345678-1234-5678-1234-56789abcdef0");  
static BLEUUID charUUID("12345678-1234-5678-1234-56789abcdef1"); // Karakteristik untuk Menerima dari nRF
static BLEUUID writeCharUUID("87654321-4321-8765-4321-123456789abc"); // Karakteristik untuk Mengirim ke Smartphone

// --- Variabel BLE Client (koneksi ke nRF) ---
static boolean doConnect = false;
static boolean connected = false; // Status koneksi ke nRF
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice;

// --- Variabel BLE Server (koneksi ke Smartphone) ---
BLEServer *pServer = nullptr;
BLECharacteristic *pWriteCharacteristic;
BLEAdvertising *pAdvertising;
bool deviceConnected = false; // Status koneksi dari Smartphone

// --- Variabel Penyimpanan Data ---
uint8_t lastSnoreStatus = 0;
uint32_t lastTimestamp = 0;
volatile bool newDataFromNrf = false; // FLAG untuk menandakan data baru dari nRF

// --- Variabel Sensor MAX30102/MAX30105 ---
MAX30105 particleSensor; // Gunakan MAX30105 jika library Anda namanya itu
double ESpO2 = 95.0;     // Initial SpO2 value (bisa disesuaikan)
double FSpO2 = 0.7;      // Filter factor for SpO2 (0.0 - 1.0)

// --- Variabel Kalkulasi SpO2 ---
double avered = 0; 
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int sampleCount = 0;
const int NumSamples = 100;   // Jumlah sampel sebelum menghitung SpO2
const int FINGER_ON = 30000;  // Threshold IR untuk deteksi jari (sesuaikan jika perlu)

// --- Flag untuk Serial Logging ---
bool enableSerialLogging = false;

// --- Fungsi untuk deteksi USB connection ---
void checkSerialConnection() {
  // Cek apakah serial terhubung (USB plugged in)
  if(Serial) { // Jika serial tersedia
    if(!enableSerialLogging) {
      enableSerialLogging = true;
      Serial.println("\n\nSerial logging enabled (USB connected)");
      Serial.println("ESP32 SmartWatch Controller - v2.1");
    }
  } else {
    enableSerialLogging = false;
  }
}

// --- Fungsi Setup Sensor MAX30102 ---
void setupMAX30102() {
  if(enableSerialLogging) Serial.println("Initializing MAX30102/5...");
  Wire.begin(); // Mulai I2C
  
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) { // 400kHz speed
    if(enableSerialLogging) Serial.println("MAX30102/5 not found. Check wiring/power.");
    while (1); // Hentikan jika sensor tidak terdeteksi
  }
  if(enableSerialLogging) Serial.println("MAX30102/5 Initialized.");

  // Konfigurasi Sensor
  byte ledBrightness = 0x7F;
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 200;
  int pulseWidth = 411;
  int adcRange = 16384;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

// --- Fungsi Kalkulasi SpO2 ---
void calculateSpO2() {
  ESpO2 = 96; // data spo2 testig static
}

// --- Callback ketika menerima notifikasi dari nRF52840 ---
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, 
                          uint8_t *pData, size_t length, bool isNotify) {
  if (length == 5) { // Pastikan panjang data sesuai (1 byte status + 4 byte timestamp)
    lastSnoreStatus = pData[0];
    memcpy(&lastTimestamp, &pData[1], 4); // Salin timestamp
    
    if(enableSerialLogging) {
      Serial.print("Received - Snore: ");
      Serial.print(lastSnoreStatus);
      Serial.print(", Timestamp: ");
      Serial.println(lastTimestamp);
    }
    
    newDataFromNrf = true;
  } else if(enableSerialLogging) {
    Serial.print("Received data with unexpected length: ");
    Serial.println(length);
  }
}

// --- Callback untuk status koneksi BLE Client (ke nRF) ---
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) {
    if(enableSerialLogging) Serial.println("Connected to nRF52840");
  }

  void onDisconnect(BLEClient *pclient) {
    connected = false;
    if(enableSerialLogging) Serial.println("Disconnected from nRF52840");
  }
};

// --- Fungsi untuk konek ke nRF52840 ---
bool connectToServer() {
  if(enableSerialLogging) {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
  }
    
  BLEClient* pClient = BLEDevice::createClient();
  if(enableSerialLogging) Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remote BLE Server.
  if (!pClient->connect(myDevice)) {
     if(enableSerialLogging) Serial.println(" - Failed to connect");
     return false;
  }
  if(enableSerialLogging) Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    if(enableSerialLogging) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
    }
    pClient->disconnect();
    return false;
  }
  if(enableSerialLogging) Serial.println(" - Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    if(enableSerialLogging) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
    }
    pClient->disconnect();
    return false;
  }
  if(enableSerialLogging) Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  if(pRemoteCharacteristic->canRead()) {
    String value = pRemoteCharacteristic->readValue();
    if(enableSerialLogging) {
      Serial.print("The characteristic value was: ");
      Serial.println(value);
    }
  }

  // Mendaftar untuk notifikasi
  if(pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    if(enableSerialLogging) Serial.println(" - Registered for notifications");
  } else if(enableSerialLogging) {
     Serial.println(" - Characteristic cannot notify");
  }

  connected = true;
  return true;
}

// --- Callback ketika perangkat BLE diiklankan ditemukan (Scan) ---
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if(enableSerialLogging) {
      Serial.print("BLE Advertised Device found: ");
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

// --- Callback untuk status koneksi BLE Server (dari Smartphone) ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      if(enableSerialLogging) Serial.println("Smartphone connected");
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      if(enableSerialLogging) Serial.println("Smartphone disconnected");
      delay(500);
      pAdvertising->start(); 
      if(enableSerialLogging) Serial.println("Advertising restarted");
    }
};

// --- Fungsi Setup BLE Server (untuk Smartphone) ---
void setupBLEServer() {
  if(enableSerialLogging) Serial.println("Setting up BLE Server...");
  
  BLEDevice::init("ESP32");
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
  pServer->getAdvertising()->start();
  
  if(enableSerialLogging) Serial.println("BLE Server Advertising started");
}

// --- Fungsi Setup Utama ---
void setup() {
  // Initialize Serial but don't wait for it
  Serial.begin(115200);
  
  // Check if USB is connected for logging
  checkSerialConnection();
  
  if(enableSerialLogging) {
    Serial.println("ESP32 SmartWatch Controller - v2.1");
    Serial.println("Starting in USB mode with serial logging");
  }

  // Inisialisasi Sensor MAX30102/5
  //setupMAX30102();

  // Inisialisasi BLE sebagai Client (untuk konek ke nRF)
  if(enableSerialLogging) Serial.println("Initializing BLE Client...");
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(10, false);

  // Inisialisasi BLE sebagai Server (untuk koneksi dari Smartphone)
  setupBLEServer();

  if(enableSerialLogging) Serial.println("Setup complete. Entering loop...");
}

// --- Fungsi Loop Utama ---
void loop() {
  // Check USB connection status periodically
  static unsigned long lastCheck = 0;
  if(millis() - lastCheck > 1000) { // Check every 1 second
    checkSerialConnection();
    lastCheck = millis();
  }

  // --- Handle koneksi ke nRF52840 (Client) ---
  if (doConnect == true) {
    if (connectToServer()) {
      if(enableSerialLogging) Serial.println("Successfully connected to the nRF52840");
    } else if(enableSerialLogging) {
      Serial.println("Failed to connect to the nRF52840. Retrying scan...");
    }
    doConnect = false;
  }

  // --- Hitung SpO2 secara kontinu ---
  calculateSpO2(); 

  // --- Kirim data ke Smartphone HANYA jika ada data baru dari nRF ---
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

  delay(50);
}