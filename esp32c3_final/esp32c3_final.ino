//----------------------------------------------------------------//
//                          LIBRARIES                             //
//----------------------------------------------------------------//
#include <Wire.h>               // Untuk komunikasi I2C
#include <Adafruit_GFX.h>       // Library grafis Adafruit
#include <Adafruit_SSD1306.h>   // Library driver OLED SSD1306

#include <BLEDevice.h>          // Library BLE untuk ESP32
#include <BLEUtils.h>
#include <BLEServer.h>

#include "MAX30105.h"           // Library untuk sensor MAX30102/MAX30105

//----------------------------------------------------------------//
//                       OLED DEFINITIONS                         //
//----------------------------------------------------------------//
#define SCREEN_WIDTH 128        // Lebar OLED dalam piksel
#define SCREEN_HEIGHT 64       // Tinggi OLED dalam piksel
#define OLED_RESET -1          // Pin RESET OLED (-1 jika tidak digunakan)
#define SCREEN_ADDRESS 0x3C    // Alamat I2C OLED (umumnya 0x3C atau 0x3D)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Dimensi logo
const int LOGO_WIDTH = 17;
const int LOGO_HEIGHT = 17;

// Status layar OLED
#define SCREEN_PHONE_INFO 1
#define SCREEN_NRF_PROMPT 2
int currentOledScreen = SCREEN_NRF_PROMPT; // Layar awal adalah prompt NRF
bool oled_nrfConnected = false;        // Status koneksi NRF untuk OLED
bool oled_phoneBLEConnected = false; // Status koneksi Smartphone untuk OLED (logo)
unsigned long lastOledUpdateTime = 0;
const unsigned long oledUpdateInterval = 2000; // Interval update layar info (jika NRF terhubung)

// Bitmap logo (phone1 & no_logo)
const unsigned char phone1_logo[] PROGMEM = {
    0x00, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08,
    0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08,
    0x00, 0x08, 0x08, 0x00, 0x0f, 0xf8, 0x00, 0x0e, 0x38, 0x00, 0x0f, 0xf8, 0x00, 0x0f, 0xf8, 0x00,
    0x00, 0x00, 0x00};
const unsigned char no_phone_logo[] PROGMEM = {
    0x00, 0x00, 0x80, 0x0f, 0xf9, 0x00, 0x08, 0x0a, 0x00, 0x08, 0x0c, 0x00, 0x08, 0x08, 0x00, 0x08,
    0x18, 0x00, 0x08, 0x28, 0x00, 0x08, 0x48, 0x00, 0x08, 0x88, 0x00, 0x09, 0x08, 0x00, 0x0a, 0x08,
    0x00, 0x0c, 0x08, 0x00, 0x0f, 0xf8, 0x00, 0x1e, 0x38, 0x00, 0x2f, 0xf8, 0x00, 0x4f, 0xf8, 0x00,
    0x80, 0x00, 0x00};

//----------------------------------------------------------------//
//                        BLE DEFINITIONS                         //
//----------------------------------------------------------------//
// UUIDs (dari esp_max30102.txt)
static BLEUUID serviceUUID("12345678-1234-5678-1234-56789abcdef0");
static BLEUUID charUUID_NRF_READ("12345678-1234-5678-1234-56789abcdef1");    // Karakteristik untuk Menerima dari nRF
static BLEUUID charUUID_PHONE_WRITE("87654321-4321-8765-4321-123456789abc"); // Karakteristik untuk Mengirim ke Smartphone

// Variabel BLE Client (koneksi ke nRF)
static boolean bleClient_doConnectToNrf = false;
static boolean bleClient_nrfConnected = false; // Status koneksi aktual ke nRF
static BLERemoteCharacteristic *pRemoteCharacteristicNrf;
static BLEAdvertisedDevice *myNrfDevice;

// Variabel BLE Server (koneksi dari Smartphone)
BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristicPhone;
bool bleServer_smartphoneConnected = false; // Status koneksi aktual dari Smartphone

// Variabel Penyimpanan Data dari nRF
uint8_t lastSnoreStatusFromNrf = 0;
uint32_t lastTimestampFromNrf = 0;
volatile bool newDataFromNrf = false; // Flag data baru dari nRF

//----------------------------------------------------------------//
//                   MAX30102 SENSOR DEFINITIONS                  //
//----------------------------------------------------------------//
MAX30105 particleSensor;
double ESpO2 = 0.0;      // Nilai SpO2 yang sudah difilter
double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int sampleCount = 0;
const int SAMPLES_FOR_SPO2_CALC = 100; // Jumlah sampel sebelum menghitung SpO2
const int FINGER_DETECT_THRESHOLD_IR = 30000; // Threshold IR untuk deteksi jari
const double SPO2_FILTER_FACTOR = 0.7;    // Faktor filter untuk SpO2

//----------------------------------------------------------------//
//                      PROTOTYPE FUNGSI                          //
//----------------------------------------------------------------//
void updateOledDisplay();
void setupMAX30102();
void calculateSpO2();
bool connectToNrfServer();
void setupBleServerForPhone();
void sendJsonToSmartphone();

//----------------------------------------------------------------//
//                       CALLBACK BLE CLIENT (NRF)                //
//----------------------------------------------------------------//
static void nrfNotifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
    if (length == 5) { // 1 byte status + 4 byte timestamp
        lastSnoreStatusFromNrf = pData[0];
        memcpy(&lastTimestampFromNrf, &pData[1], 4);

        Serial.print("Diterima dari nRF - Snore: ");
        Serial.print(lastSnoreStatusFromNrf);
        Serial.print(", Timestamp: ");
        Serial.println(lastTimestampFromNrf);
        newDataFromNrf = true;
    } else {
        Serial.print("Menerima data dari nRF dengan panjang tidak sesuai: ");
        Serial.println(length);
    }
}

class MyNrfClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient *pclient_nrf) {
        Serial.println("Terhubung ke server nRF");
        // Di sini tidak langsung set bleClient_nrfConnected = true, tunggu konfirmasi characteristic
    }

    void onDisconnect(BLEClient *pclient_nrf) {
        bleClient_nrfConnected = false;
        oled_nrfConnected = false; // Update status OLED
        Serial.println("Terputus dari server nRF");
        // Pertimbangkan untuk scan ulang otomatis atau beri tahu pengguna
        BLEDevice::getScan()->start(5, false); // Coba scan lagi
    }
};

class MyAdvertisedNrfDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.print("Perangkat BLE ditemukan: ");
        Serial.println(advertisedDevice.toString().c_str());
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
            BLEDevice::getScan()->stop();
            myNrfDevice = new BLEAdvertisedDevice(advertisedDevice);
            bleClient_doConnectToNrf = true;
            Serial.println("Perangkat nRF target ditemukan! Mencoba menghubungkan...");
        }
    }
};

//----------------------------------------------------------------//
//                    CALLBACK BLE SERVER (PHONE)                 //
//----------------------------------------------------------------//
class MyPhoneServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer_phone) {
        bleServer_smartphoneConnected = true;
        oled_phoneBLEConnected = true; // Update status OLED
        Serial.println("Smartphone terhubung");
        // Hentikan advertising jika hanya ingin satu koneksi smartphone
        // BLEDevice::getAdvertising()->stop();
    }

    void onDisconnect(BLEServer *pServer_phone) {
        bleServer_smartphoneConnected = false;
        oled_phoneBLEConnected = false; // Update status OLED
        Serial.println("Smartphone terputus");
        delay(500); // Beri jeda
        pServer_phone->getAdvertising()->start(); // Mulai advertising lagi
        Serial.println("Advertising untuk smartphone dimulai ulang");
    }
};

//----------------------------------------------------------------//
//                          SETUP                                 //
//----------------------------------------------------------------//
void setup() {
    Serial.begin(115200);
    Serial.println("Memulai ESP32 Sensor Hub...");

    // Inisialisasi I2C - sesuaikan pin jika ESP32-C3 Anda tidak menggunakan default
    // Untuk ESP32-C3, pin default mungkin GPIO8 (SDA) dan GPIO9 (SCL)
    Wire.begin(); // atau Wire.begin(SDA_PIN_ESP32C3, SCL_PIN_ESP32C3);
    // Jika menggunakan pin 8 & 9 untuk I2C, pastikan tidak ada konflik dengan kode motor/buzzer
    // pinMode(8, OUTPUT); // CONFLICT if 8 is I2C SDA
    // pinMode(9, OUTPUT); // CONFLICT if 9 is I2C SCL
    // digitalWrite(8, HIGH);
    // digitalWrite(9, HIGH);

    // Inisialisasi OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("Alokasi SSD1306 gagal"));
        for (;;); // Loop selamanya
    }
    Serial.println(F("SSD1306 Diinisialisasi."));
    display.clearDisplay();
    display.display();

    // Inisialisasi Sensor MAX30102
    setupMAX30102();

    // Inisialisasi BLE Device (CUKUP SEKALI)
    BLEDevice::init("ESP32_SensorHub"); // Beri nama unik untuk ESP32 Anda

    // Setup BLE Client (untuk nRF)
    Serial.println("Inisialisasi BLE Client untuk nRF...");
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedNrfDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(10, false); // Scan selama 10 detik, jangan auto-stop setelah selesai
    Serial.println("BLE Scan untuk nRF dimulai...");

    // Setup BLE Server (untuk Smartphone)
    setupBleServerForPhone();

    // Tampilan awal OLED
    oled_nrfConnected = bleClient_nrfConnected; // Update status awal
    oled_phoneBLEConnected = bleServer_smartphoneConnected;
    updateOledDisplay();
    lastOledUpdateTime = millis();

    Serial.println("Setup selesai. Memasuki loop utama...");
}

//----------------------------------------------------------------//
//                        LOOP UTAMA                              //
//----------------------------------------------------------------//
void loop() {
    unsigned long currentTime = millis();

    // 1. Handle Koneksi ke nRF (BLE Client)
    if (bleClient_doConnectToNrf) {
        if (connectToNrfServer()) {
            Serial.println("Berhasil terhubung dan mendaftar notifikasi dari nRF.");
            // bleClient_nrfConnected akan diupdate di connectToNrfServer atau callback
        } else {
            Serial.println("Gagal terhubung ke nRF. Mencoba scan ulang...");
            BLEDevice::getScan()->start(10, false); // Mulai scan lagi
        }
        bleClient_doConnectToNrf = false; // Reset flag
    }

    // Update status koneksi nRF untuk OLED (jika berubah)
    if (oled_nrfConnected != bleClient_nrfConnected) {
        oled_nrfConnected = bleClient_nrfConnected;
    }
    // Update status koneksi Phone untuk OLED (jika berubah)
    if (oled_phoneBLEConnected != bleServer_smartphoneConnected) {
        oled_phoneBLEConnected = bleServer_smartphoneConnected;
    }

    // 2. Hitung SpO2 secara kontinu
    calculateSpO2();

    // 3. Logika Layar OLED
    if (!oled_nrfConnected) { // Jika NRF tidak terhubung
        if (currentOledScreen != SCREEN_NRF_PROMPT) {
            currentOledScreen = SCREEN_NRF_PROMPT;
            updateOledDisplay();
        } else { // Tetap di prompt, update jika ada perubahan logo phone
             if(currentTime - lastOledUpdateTime > 500){ // Update berkala untuk logo
                updateOledDisplay();
                lastOledUpdateTime = currentTime;
            }
        }
    } else { // NRF terhubung
        if (currentOledScreen == SCREEN_NRF_PROMPT) { // Transisi dari prompt ke info
            currentOledScreen = SCREEN_PHONE_INFO;
            updateOledDisplay();
            lastOledUpdateTime = currentTime;
        } else if (currentOledScreen == SCREEN_PHONE_INFO) {
            // Update layar info secara berkala
            if (currentTime - lastOledUpdateTime > oledUpdateInterval) {
                updateOledDisplay();
                lastOledUpdateTime = currentTime;
            }
        }
    }


    // 4. Kirim Data JSON ke Smartphone via BLE Server
    if (newDataFromNrf) {
        if (bleServer_smartphoneConnected) {
            sendJsonToSmartphone();
        } else {
            Serial.println("Smartphone tidak terhubung. Data tidak dikirim.");
        }
        newDataFromNrf = false; // Reset flag setelah diproses
    }

    delay(50); // Jeda singkat untuk stabilitas
}

//----------------------------------------------------------------//
//                    FUNGSI IMPLEMENTASI                         //
//----------------------------------------------------------------//

void updateOledDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // 1. Status NRF (kiri atas)
    display.setCursor(0, 0);
    if (oled_nrfConnected) {
        display.print(F("nrf=conect"));
    } else {
        display.print(F("nrf=disconect"));
    }

    // 2. Logo Koneksi Smartphone (kanan atas)
    int logoX = SCREEN_WIDTH - LOGO_WIDTH;
    int logoY = 0;
    if (oled_phoneBLEConnected) {
        display.drawBitmap(logoX, logoY, phone1_logo, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
    } else {
        display.drawBitmap(logoX, logoY, no_phone_logo, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
    }

    // 3. Garis Navbar
    int textHeightApprox = 8; // Perkiraan tinggi teks ukuran 1
    int navbarY = max(LOGO_HEIGHT, textHeightApprox) + 2; // 2 piksel padding
    display.drawLine(0, navbarY, SCREEN_WIDTH, navbarY, SSD1306_WHITE);

    // 4. Konten Spesifik Layar
    int contentY = navbarY + 5; // Konten dimulai 5 piksel di bawah navbar

    if (currentOledScreen == SCREEN_NRF_PROMPT) {
        display.setTextSize(1);
        int textWidthNrf1 = 13 * 6; // "Hidupkan nrf"
        int textWidthNrf2 = 18 * 6; // "terlebih dahulu!!"
        display.setCursor((SCREEN_WIDTH - textWidthNrf1) / 2, contentY + 10);
        display.println(F("Hidupkan nrf"));
        display.setCursor((SCREEN_WIDTH - textWidthNrf2) / 2, contentY + 20);
        display.println(F("terlebih dahulu!!"));
    } else if (currentOledScreen == SCREEN_PHONE_INFO) {
        // Tampilkan SpO2
        display.setCursor(0, contentY);
        display.print(F("SpO2: "));
        if (ESpO2 == 0 && particleSensor.getFIFOIR() < FINGER_DETECT_THRESHOLD_IR) {
            display.print(F("--%"));
        } else {
            // Tampilkan 1 desimal kecuali untuk 0, 80 atau 100
            display.print(ESpO2, (ESpO2 == 100.0 || ESpO2 == 0.0 || ESpO2 == 80.0) ? 0 : 1);
            display.print(F("%"));
        }
        // Tambahkan info lain jika perlu, misal status dari nRF
        display.setCursor(0, contentY + 12);
        display.print(F("Status: "));
        display.print(lastSnoreStatusFromNrf);
        display.setCursor(0, contentY + 24);
        display.print(F("TS: "));
        display.print(lastTimestampFromNrf);
    }
    display.display();
}

void setupMAX30102() {
    Serial.println("Inisialisasi MAX30102/5...");
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) { // 400kHz speed
        Serial.println("MAX30102/5 tidak ditemukan. Periksa kabel/daya.");
        // Pertimbangkan loop tak terbatas di sini jika sensor kritis
        // while (1);
    } else {
        Serial.println("MAX30102/5 Diinisialisasi.");
        // Konfigurasi Sensor (sesuaikan jika perlu)
        byte ledBrightness = 0x7F; // Kecerahan LED (~50mA)
        byte sampleAverage = 4;    // Rata-rata sampel (1, 2, 4, 8, 16, 32)
        byte ledMode = 2;          // Mode LED (2 = Red+IR untuk SpO2)
        int sampleRate = 200;      // Sample rate (mis. 50, 100, 200, ...)
        int pulseWidth = 411;      // Lebar pulsa (pengaruh ke resolusi ADC)
        int adcRange = 16384;      // Range ADC (pengaruh ke sensitivitas)
        particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    }
}

void calculateSpO2() {
    particleSensor.check(); // Cek data baru dari sensor

    while (particleSensor.available()) { // Proses semua data di FIFO
        uint32_t irValue = particleSensor.getFIFOIR();
        uint32_t redValue = particleSensor.getFIFORed();

        avered = avered * 0.95 + (double)redValue * 0.05;
        aveir = aveir * 0.95 + (double)irValue * 0.05;

        sumredrms += (redValue - avered) * (redValue - avered);
        sumirrms += (irValue - aveir) * (irValue - aveir);

        sampleCount++;
        if (sampleCount % SAMPLES_FOR_SPO2_CALC == 0) {
            double R = (sqrt(sumredrms / SAMPLES_FOR_SPO2_CALC) / avered) / (sqrt(sumirrms / SAMPLES_FOR_SPO2_CALC) / aveir);
            // Formula SpO2 (mungkin perlu kalibrasi)
            double spo2_calculated = -45.060 * R * R + 30.354 * R + 94.845;
            
            // Simpan ke ESpO2 dengan filter
            ESpO2 = SPO2_FILTER_FACTOR * ESpO2 + (1.0 - SPO2_FILTER_FACTOR) * spo2_calculated;

            if (irValue < FINGER_DETECT_THRESHOLD_IR) {
                ESpO2 = 0; // Tidak ada jari atau pembacaan tidak valid
            } else {
                // Batasi nilai SpO2 dalam rentang fisiologis
                if (ESpO2 > 100.0) ESpO2 = 100.0;
                if (ESpO2 < 80.0 && ESpO2 != 0) ESpO2 = 80.0; // Batas bawah, jangan ganggu jika sudah 0
            }
            sumredrms = 0.0;
            sumirrms = 0.0;
            // sampleCount tidak direset di sini agar perhitungan selalu per SAMPLES_FOR_SPO2_CALC
        }
        // Reset sampleCount di luar if jika ingin perhitungan per SAMPLES_FOR_SPO2_CALC dari awal program
        if (sampleCount >= SAMPLES_FOR_SPO2_CALC) sampleCount = 0;
        particleSensor.nextSample(); // Maju ke sampel berikutnya
    }
    // Jika jari dilepas, ESpO2 bisa diset ke 0 lebih cepat
    if (particleSensor.getFIFOIR() < FINGER_DETECT_THRESHOLD_IR && ESpO2 != 0 && sampleCount < 10) {
      ESpO2 = 0;
    }
}

bool connectToNrfServer() {
    Serial.print("Mencoba koneksi ke nRF: ");
    Serial.println(myNrfDevice->getAddress().toString().c_str());

    BLEClient *pClientNrf = BLEDevice::createClient();
    Serial.println(" - Klien BLE dibuat");
    pClientNrf->setClientCallbacks(new MyNrfClientCallbacks());

    if (!pClientNrf->connect(myNrfDevice)) {
        Serial.println(" - Gagal terhubung ke server nRF");
        return false;
    }
    Serial.println(" - Terhubung ke server nRF");

    BLERemoteService *pRemoteServiceNrf = pClientNrf->getService(serviceUUID);
    if (pRemoteServiceNrf == nullptr) {
        Serial.print("Gagal menemukan service UUID nRF: ");
        Serial.println(serviceUUID.toString().c_str());
        pClientNrf->disconnect();
        return false;
    }
    Serial.println(" - Service nRF ditemukan");

    pRemoteCharacteristicNrf = pRemoteServiceNrf->getCharacteristic(charUUID_NRF_READ);
    if (pRemoteCharacteristicNrf == nullptr) {
        Serial.print("Gagal menemukan characteristic UUID nRF: ");
        Serial.println(charUUID_NRF_READ.toString().c_str());
        pClientNrf->disconnect();
        return false;
    }
    Serial.println(" - Characteristic nRF ditemukan");

    if (pRemoteCharacteristicNrf->canNotify()) {
        pRemoteCharacteristicNrf->registerForNotify(nrfNotifyCallback);
        Serial.println(" - Berhasil mendaftar untuk notifikasi dari nRF");
    } else {
        Serial.println(" - Characteristic nRF tidak bisa memberi notifikasi");
        pClientNrf->disconnect(); // Tidak berguna jika tidak bisa notifikasi
        return false;
    }

    bleClient_nrfConnected = true; // Koneksi berhasil dan characteristic ditemukan
    oled_nrfConnected = true;      // Update status OLED
    return true;
}

void setupBleServerForPhone() {
    Serial.println("Setup BLE Server untuk Smartphone...");
    // Buat BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyPhoneServerCallbacks());

    // Buat BLE Service
    BLEService *pServicePhone = pServer->createService(serviceUUID); // Bisa serviceUUID yang sama atau beda

    // Buat BLE Characteristic untuk mengirim data ke smartphone
    pCharacteristicPhone = pServicePhone->createCharacteristic(
        charUUID_PHONE_WRITE,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_WRITE_NR // Optional: jika smartphone ingin menulis sesuatu
    );

    // Mulai service
    pServicePhone->start();

    // Mulai Advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(serviceUUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // Membantu masalah koneksi iPhone
    pAdvertising->setMinPreferred(0x12);
    // BLEDevice::startAdvertising(); // Digantikan oleh pServer->getAdvertising()->start();
    pServer->getAdvertising()->start();
    Serial.println("BLE Server untuk Smartphone mulai advertising");
}

void sendJsonToSmartphone() {
    // Buat JSON dengan data terbaru
    String jsonData = "{\"status\":" + String(lastSnoreStatusFromNrf) +
                      ",\"timestamp\":" + String(lastTimestampFromNrf) +
                      ",\"spo2\":" + String(ESpO2, (ESpO2 == 100.0 || ESpO2 == 0.0 || ESpO2 == 80.0) ? 0 : 2) + // 2 desimal untuk SpO2
                      "}";

    // Kirim data via BLE Notify ke Smartphone
    pCharacteristicPhone->setValue(jsonData.c_str());
    pCharacteristicPhone->notify();

    Serial.print("Terkirim ke smartphone: ");
    Serial.println(jsonData);
}