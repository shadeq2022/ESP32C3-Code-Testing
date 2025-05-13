// v2.1 - Battery Optimized
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

// --- Fungsi Setup Sensor MAX30102 ---
void setupMAX30102() {
  Wire.begin(); // Mulai I2C
  
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) { // 400kHz speed
    while (1); // Hentikan jika sensor tidak terdeteksi
  }

  // Konfigurasi Sensor (sesuaikan parameter ini jika perlu)
  byte ledBrightness = 0x7F; // Kecerahan LED (0x00 - 0xFF) -> ~50mA
  byte sampleAverage = 4;    // Rata-rata sampel (1, 2, 4, 8, 16, 32)
  byte ledMode = 2;          // Mode LED (1=Red only, 2=Red+IR, 3=Green only) -> Gunakan 2 untuk SpO2
  int sampleRate = 200;      // Sample rate (50, 100, 200, 400, 800, 1000, 1600, 3200)
  int pulseWidth = 411;      // Lebar pulsa (69, 118, 215, 411) -> Pengaruh ke resolusi ADC
  int adcRange = 16384;       // Range ADC (2048, 4096, 8192, 16384) -> Pengaruh ke sensitivitas

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
    
    newDataFromNrf = true; // *** SET FLAG di sini ***
  }
}

// --- Callback untuk status koneksi BLE Client (ke nRF) ---
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) {
  }

  void onDisconnect(BLEClient *pclient) {
    connected = false;
  }
};

// --- Fungsi untuk konek ke nRF52840 ---
bool connectToServer() {  
  BLEClient* pClient = BLEDevice::createClient();

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remote BLE Server.
  if (!pClient->connect(myDevice)) { // Gunakan myDevice yang ditemukan saat scan
     return false;
  }

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    pClient->disconnect();
    return false;
  }

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    pClient->disconnect();
    return false;
  }

  // Read the value of the characteristic.
  if(pRemoteCharacteristic->canRead()) {
    pRemoteCharacteristic->readValue();
  }

  // Mendaftar untuk notifikasi
  if(pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
  }

  connected = true; // Set status koneksi ke nRF berhasil
  return true;
}

// --- Callback ketika perangkat BLE diiklankan ditemukan (Scan) ---
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop(); // Hentikan scan setelah perangkat target ditemukan
      myDevice = new BLEAdvertisedDevice(advertisedDevice); // Simpan informasi perangkat
      doConnect = true; // Set flag untuk mencoba konek di loop()
    }
  }
};

// --- Callback untuk status koneksi BLE Server (dari Smartphone) ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true; // Smartphone terhubung
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false; // Smartphone terputus
      // Restart advertising agar smartphone lain bisa konek lagi
      delay(500); // Beri jeda sebelum restart advertising
      pAdvertising->start(); 
    }
};

// --- Fungsi Setup BLE Server (untuk Smartphone) ---
void setupBLEServer() {
  // Buat BLE Device dengan nama "SmartWatch"
  BLEDevice::init("ESP32"); // Beri nama unik untuk ESP32 server

  // Buat BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Buat BLE Service
  BLEService *pService = pServer->createService(serviceUUID);

  // Buat BLE Characteristic untuk mengirim data ke smartphone
  pWriteCharacteristic = pService->createCharacteristic(
                                         writeCharUUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_WRITE_NR
                                       );
  
  // Mulai service
  pService->start();

  // Mulai Advertising
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(serviceUUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  pServer->getAdvertising()->start();
}

// --- Fungsi Setup Utama ---
void setup() {
  // Inisialisasi Sensor MAX30102/5
  //setupMAX30102();

  // Inisialisasi BLE sebagai Client (untuk konek ke nRF)
  BLEDevice::init(""); // Nama kosong untuk mode client
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(10, false);

  // Inisialisasi BLE sebagai Server (untuk koneksi dari Smartphone)
  setupBLEServer();
}

// --- Fungsi Loop Utama ---
void loop() {
  // --- Handle koneksi ke nRF52840 (Client) ---
  if (doConnect == true) {
    connectToServer();
    doConnect = false; // Reset flag setelah mencoba konek
  }

  // --- Hitung SpO2 secara kontinu ---
  calculateSpO2(); 

  // --- Kirim data ke Smartphone HANYA jika ada data baru dari nRF ---
  if (newDataFromNrf) { 
    if (deviceConnected) { // Pastikan Smartphone masih terhubung
      // Buat JSON dengan data terbaru
      String jsonData = "{\"status\":" + String(lastSnoreStatus) + 
                       ",\"timestamp\":" + String(lastTimestamp) + 
                       ",\"spo2\":" + String(ESpO2, 2) + "}";
      
      // Kirim data via BLE Notify ke Smartphone
      pWriteCharacteristic->setValue(jsonData.c_str());
      pWriteCharacteristic->notify();
    }
    
    newDataFromNrf = false; // *** RESET FLAG di sini setelah data diproses/dikirim ***
  }

  // Beri jeda sedikit di loop utama untuk stabilitas
  delay(50); // Delay 50ms (bisa disesuaikan)
}