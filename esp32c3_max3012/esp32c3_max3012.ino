//version 2.0
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
  Serial.println("Ini kode v2");
  Serial.println("Initializing MAX30102/5...");
  Wire.begin(); // Mulai I2C
  
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) { // 400kHz speed
    Serial.println("MAX30102/5 not found. Check wiring/power.");
    while (1); // Hentikan jika sensor tidak terdeteksi
  }
  Serial.println("MAX30102/5 Initialized.");

  // Konfigurasi Sensor (sesuaikan parameter ini jika perlu)
  byte ledBrightness = 0x7F; // Kecerahan LED (0x00 - 0xFF) -> ~50mA
  byte sampleAverage = 4;    // Rata-rata sampel (1, 2, 4, 8, 16, 32)
  byte ledMode = 2;          // Mode LED (1=Red only, 2=Red+IR, 3=Green only) -> Gunakan 2 untuk SpO2
  int sampleRate = 200;      // Sample rate (50, 100, 200, 400, 800, 1000, 1600, 3200)
  int pulseWidth = 411;      // Lebar pulsa (69, 118, 215, 411) -> Pengaruh ke resolusi ADC
  int adcRange = 16384;       // Range ADC (2048, 4096, 8192, 16384) -> Pengaruh ke sensitivitas

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  // Aktifkan pembacaan suhu internal jika diinginkan (opsional)
  // particleSensor.enableDIETEMPRDY(); 
}

// --- Fungsi Kalkulasi SpO2 ---
void calculateSpO2() {
  particleSensor.check(); // Cek data baru dari sensor
  
  while (particleSensor.available()) { // Proses semua data yang tersedia di FIFO
    // PERHATIKAN: Pada beberapa board MH-ET LIVE MAX30102, pin RED dan IR tertukar. 
    // Jika nilai SpO2 aneh, coba tukar getFIFOIR() dan getFIFORed() di bawah ini.
    uint32_t irValue = particleSensor.getFIFOIR();  
    uint32_t redValue = particleSensor.getFIFORed(); 

    // Filter low-pass sederhana untuk mendapatkan komponen DC
    avered = avered * 0.95 + (double)redValue * 0.05;
    aveir = aveir * 0.95 + (double)irValue * 0.05;
    
    // Hitung jumlah kuadrat perbedaan dari rata-rata (untuk komponen AC / RMS)
    sumredrms += (redValue - avered) * (redValue - avered);
    sumirrms += (irValue - aveir) * (irValue - aveir);
    
    sampleCount++;
    
    // Hitung SpO2 setiap NumSamples sampel
    if (sampleCount % NumSamples == 0) {
      double R = (sqrt(sumredrms / NumSamples) / avered) / (sqrt(sumirrms / NumSamples) / aveir);
      // Formula SpO2 (mungkin perlu kalibrasi/penyesuaian berdasarkan sensor dan subjek)
      // Formula ini adalah estimasi umum, hasil terbaik mungkin memerlukan kalibrasi
      //double SpO2 = 110.0 - 25.0 * R; // Coba formula ini atau: 
      double SpO2 = -45.060*R*R + 30.354*R + 94.845;
      
      // Terapkan filter pada hasil SpO2 untuk menghaluskan
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;
      
      // Validasi: Jika tidak ada jari (IR rendah) atau nilai di luar batas wajar
      if (irValue < FINGER_ON) {
        ESpO2 = 0; // Tandai sebagai tidak valid jika jari tidak terdeteksi
      } else {
        // Batasi nilai SpO2 dalam rentang fisiologis yang wajar
        if (ESpO2 > 100.0) ESpO2 = 100.0;
        if (ESpO2 < 80.0) ESpO2 = 80.0; // Batas bawah bisa disesuaikan
      }
      
      // Reset RMS sum dan sample count untuk perhitungan berikutnya
      sumredrms = 0.0; 
      sumirrms = 0.0; 
      // sampleCount = 0; // Reset sampleCount di sini agar perhitungan selalu per NumSamples
    }
    // Reset sampleCount di luar if, jika ingin perhitungan per NumSamples sejak awal program
    if (sampleCount >= NumSamples) sampleCount = 0;

    particleSensor.nextSample(); // Maju ke sampel berikutnya di FIFO
  }
}

// --- Callback ketika menerima notifikasi dari nRF52840 ---
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, 
                          uint8_t *pData, size_t length, bool isNotify) {
  if (length == 5) { // Pastikan panjang data sesuai (1 byte status + 4 byte timestamp)
    lastSnoreStatus = pData[0];
    memcpy(&lastTimestamp, &pData[1], 4); // Salin timestamp
    
    Serial.print("Received - Snore: ");
    Serial.print(lastSnoreStatus);
    Serial.print(", Timestamp: ");
    Serial.println(lastTimestamp);
    
    newDataFromNrf = true; // *** SET FLAG di sini ***
  } else {
    Serial.print("Received data with unexpected length: ");
    Serial.println(length);
  }
}

// --- Callback untuk status koneksi BLE Client (ke nRF) ---
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) {
    Serial.println("onConnect - Client"); // Debug message
  }

  void onDisconnect(BLEClient *pclient) {
    connected = false;
    Serial.println("Disconnected from nRF52840");
    // Mungkin perlu logic untuk mencoba reconnect di sini atau di loop()
  }
};

// --- Fungsi untuk konek ke nRF52840 ---
bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());
    
  BLEClient* pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remote BLE Server.
  if (!pClient->connect(myDevice)) { // Gunakan myDevice yang ditemukan saat scan
     Serial.println(" - Failed to connect");
     return false;
  }
  Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  if(pRemoteCharacteristic->canRead()) {
    String value = pRemoteCharacteristic->readValue(); // <-- UBAH MENJADI String (Arduino)
    Serial.print("The characteristic value was: ");
    Serial.println(value); // <-- TIDAK PERLU .c_str() lagi
  }

  // Mendaftar untuk notifikasi
  if(pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    Serial.println(" - Registered for notifications");
  } else {
     Serial.println(" - Characteristic cannot notify");
  }

  connected = true; // Set status koneksi ke nRF berhasil
  return true;
}

// --- Callback ketika perangkat BLE diiklankan ditemukan (Scan) ---
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop(); // Hentikan scan setelah perangkat target ditemukan
      myDevice = new BLEAdvertisedDevice(advertisedDevice); // Simpan informasi perangkat
      doConnect = true; // Set flag untuk mencoba konek di loop()
      Serial.println("Found our device! Now connecting...");
    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

// --- Callback untuk status koneksi BLE Server (dari Smartphone) ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true; // Smartphone terhubung
      Serial.println("Smartphone connected");
      // Anda mungkin ingin menghentikan advertising di sini jika tidak ingin koneksi ganda
      // BLEDevice::startAdvertising(); 
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false; // Smartphone terputus
      Serial.println("Smartphone disconnected");
      // Restart advertising agar smartphone lain bisa konek lagi
      delay(500); // Beri jeda sebelum restart advertising
      pAdvertising->start(); 
      Serial.println("Advertising restarted");
    }
};

// --- Fungsi Setup BLE Server (untuk Smartphone) ---
void setupBLEServer() {
  Serial.println("Setting up BLE Server...");
  // Buat BLE Device dengan nama "SmartWatch"
  BLEDevice::init("ESP32"); // Beri nama unik untuk ESP32 server

  // Buat BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks()); // Set callback untuk koneksi smartphone

  // Buat BLE Service (gunakan UUID yang sama dengan service nRF untuk konsistensi, atau beda jika perlu)
  BLEService *pService = pServer->createService(serviceUUID); // Atau UUID baru khusus server

  // Buat BLE Characteristic untuk mengirim data ke smartphone
  pWriteCharacteristic = pService->createCharacteristic(
                                         writeCharUUID, // UUID khusus untuk kirim data
                                         BLECharacteristic::PROPERTY_READ   |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_WRITE_NR // Optional: jika ingin smartphone bisa menulis sesuatu
                                       );
  // Anda bisa menambahkan descriptor di sini jika perlu (misal, CCCD untuk notifikasi)
  // pWriteCharacteristic->addDescriptor(new BLE2902());                             

  // Mulai service
  pService->start();

  // Mulai Advertising
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(serviceUUID); // Iklankan service UUID
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  // BLEDevice::startAdvertising(); // Start advertising diganti ke pAdvertising->start()
  pServer->getAdvertising()->start();
  Serial.println("BLE Server Advertising started");
}

// --- Fungsi Setup Utama ---
void setup() {
  Serial.begin(115200);
  while (!Serial); // Tunggu Serial siap (khusus untuk beberapa board seperti Leonardo)
  Serial.println("Starting ESP32 SmartWatch Controller...");

  // Inisialisasi Sensor MAX30102/5
  setupMAX30102();

  // Inisialisasi BLE sebagai Client (untuk konek ke nRF)
  Serial.println("Initializing BLE Client...");
  BLEDevice::init(""); // Nama kosong untuk mode client
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349); // Atur interval scan
  pBLEScan->setWindow(449);    // Atur window scan
  pBLEScan->setActiveScan(true); // Scan aktif untuk mendapatkan nama perangkat
  pBLEScan->start(10, false); // Mulai scan selama 10 detik, jangan auto stop setelah selesai
  Serial.println("BLE Scan started...");

  // Inisialisasi BLE sebagai Server (untuk koneksi dari Smartphone)
  setupBLEServer(); // Panggil fungsi setup server

  Serial.println("Setup complete. Entering loop...");
}

// --- Fungsi Loop Utama ---
void loop() {
  // --- Handle koneksi ke nRF52840 (Client) ---
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("Successfully connected to the nRF52840");
    } else {
      Serial.println("Failed to connect to the nRF52840. Retrying scan...");
      // Jika gagal konek, mulai scan lagi
      BLEDevice::getScan()->start(10, false); 
    }
    doConnect = false; // Reset flag setelah mencoba konek
  }

  // Jika terputus dari nRF, coba scan lagi (opsional, bisa disesuaikan)
  // if (!connected && !doConnect) {
  //   Serial.println("Disconnected from nRF. Restarting scan...");
  //   BLEDevice::getScan()->start(5, false); // Scan sebentar
  // }


  // --- Hitung SpO2 secara kontinu ---
  calculateSpO2(); 

  // --- Kirim data ke Smartphone HANYA jika ada data baru dari nRF ---
  if (newDataFromNrf) { 
    if (deviceConnected) { // Pastikan Smartphone masih terhubung
      // Buat JSON dengan data terbaru
      String jsonData = "{\"status\":" + String(lastSnoreStatus) + 
                       ",\"timestamp\":" + String(lastTimestamp) + 
                       ",\"spo2\":" + String(ESpO2, 2) + "}"; // Tampilkan SpO2 dengan 2 desimal
      
      // Kirim data via BLE Notify ke Smartphone
      pWriteCharacteristic->setValue(jsonData.c_str()); // Set nilai karakteristik
      pWriteCharacteristic->notify(); // Kirim notifikasi
      
      // Cetak ke Serial Monitor HANYA setelah mengirim
      Serial.print("Sent to smartphone: ");
      Serial.println(jsonData);
    } else {
        Serial.println("Smartphone not connected. Cannot send data.");
    }
    
    newDataFromNrf = false; // *** RESET FLAG di sini setelah data diproses/dikirim ***
  }

  // Beri jeda sedikit di loop utama untuk stabilitas
  delay(50); // Delay 50ms (bisa disesuaikan)
}