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
#define SCREEN_HEIGHT 64        // Tinggi OLED dalam piksel
#define OLED_RESET -1           // Pin RESET OLED (-1 jika tidak digunakan)
#define SCREEN_ADDRESS 0x3C     // Alamat I2C OLED (umumnya 0x3C atau 0x3D)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Inisialisasi objek display OLED

// Dimensi logo
const int LOGO_WIDTH = 17;      // Lebar logo
const int LOGO_HEIGHT = 17;     // Tinggi logo

// Status layar OLED
#define SCREEN_PHONE_INFO 1     // Konstanta untuk layar info HP & Sensor
#define SCREEN_NRF_PROMPT 2     // Konstanta untuk layar prompt NRF
int currentOledScreen = SCREEN_NRF_PROMPT; // Layar awal adalah prompt NRF
bool oled_nrfConnected = false; // Status koneksi NRF untuk tampilan OLED
bool oled_phoneBLEConnected = false; // Status koneksi Smartphone untuk tampilan logo di OLED
unsigned long lastOledUpdateTime = 0; // Waktu terakhir OLED diupdate
const unsigned long oledUpdateInterval = 2000; // Interval update layar info (jika NRF terhubung)

// Bitmap logo (phone1 & no_logo)
const unsigned char phone1_logo[] PROGMEM = { // Data bitmap untuk logo telepon terhubung
    0x00, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08,
    0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08,
    0x00, 0x08, 0x08, 0x00, 0x0f, 0xf8, 0x00, 0x0e, 0x38, 0x00, 0x0f, 0xf8, 0x00, 0x0f, 0xf8, 0x00,
    0x00, 0x00, 0x00};
const unsigned char no_phone_logo[] PROGMEM = { // Data bitmap untuk logo telepon tidak terhubung
    0x00, 0x00, 0x80, 0x0f, 0xf9, 0x00, 0x08, 0x0a, 0x00, 0x08, 0x0c, 0x00, 0x08, 0x08, 0x00, 0x08,
    0x18, 0x00, 0x08, 0x28, 0x00, 0x08, 0x48, 0x00, 0x08, 0x88, 0x00, 0x09, 0x08, 0x00, 0x0a, 0x08,
    0x00, 0x0c, 0x08, 0x00, 0x0f, 0xf8, 0x00, 0x1e, 0x38, 0x00, 0x2f, 0xf8, 0x00, 0x4f, 0xf8, 0x00,
    0x80, 0x00, 0x00};

//----------------------------------------------------------------//
//                     ACTUATOR DEFINITIONS                       //
//----------------------------------------------------------------//
#define BUZZER_PIN 3  // Pin GPIO untuk buzzer
#define MOTOR_PIN 10  // Pin GPIO untuk motor getar

//----------------------------------------------------------------//
//                        BLE DEFINITIONS                         //
//----------------------------------------------------------------//
static BLEUUID serviceUUID("12345678-1234-5678-1234-56789abcdef0");       // UUID untuk service BLE utama
static BLEUUID charUUID_NRF_READ("12345678-1234-5678-1234-56789abcdef1");    // Karakteristik untuk Menerima data dari nRF (notifikasi)
static BLEUUID charUUID_PHONE_WRITE("87654321-4321-8765-4321-123456789abc"); // Karakteristik untuk Mengirim data ke Smartphone

// Variabel BLE Client (untuk koneksi ke nRF)
static boolean bleClient_doConnectToNrf = false;       // Flag untuk memicu upaya koneksi ke nRF
static boolean bleClient_nrfConnected = false;         // Status koneksi aktual ke nRF
static BLERemoteCharacteristic *pRemoteCharacteristicNrf; // Pointer ke karakteristik remote nRF
static BLEAdvertisedDevice *myNrfDevice;               // Pointer ke perangkat nRF yang ditemukan

// Variabel BLE Server (untuk koneksi dari Smartphone)
BLEServer *pServer = nullptr;                          // Pointer ke objek server BLE
BLECharacteristic *pCharacteristicPhone;               // Pointer ke karakteristik untuk komunikasi dengan HP
bool bleServer_smartphoneConnected = false;            // Status koneksi aktual dari Smartphone

// Variabel Penyimpanan Data dari nRF
uint8_t lastSnoreStatusFromNrf = 0;      // Status mendengkur terakhir yang diterima dari nRF
uint32_t lastTimestampFromNrf = 0;     // Timestamp terakhir yang diterima dari nRF
volatile bool newDataFromNrf = false;    // Flag menandakan adanya data baru dari nRF

//----------------------------------------------------------------//
//                   MAX30102 SENSOR DEFINITIONS                  //
//----------------------------------------------------------------//
MAX30105 particleSensor; // Objek untuk sensor MAX30102/MAX30105
double ESpO2 = 0.0;      // Nilai SpO2 yang sudah difilter (hasil akhir)
double avered = 0;       // Rata-rata nilai merah untuk kalkulasi SpO2
double aveir = 0;        // Rata-rata nilai infra merah untuk kalkulasi SpO2
double sumirrms = 0;     // Akumulasi untuk kalkulasi RMS IR
double sumredrms = 0;    // Akumulasi untuk kalkulasi RMS RED
int sampleCount = 0;     // Jumlah sampel yang telah dikumpulkan
const int SAMPLES_FOR_SPO2_CALC = 100; // Jumlah sampel yang dibutuhkan sebelum menghitung SpO2
const int FINGER_DETECT_THRESHOLD_IR = 30000; // Threshold IR untuk deteksi jari pada sensor
const double SPO2_FILTER_FACTOR = 0.7; // Faktor filter untuk memperhalus nilai SpO2

//----------------------------------------------------------------//
//              SpO2 STABILIZATION DEFINITIONS                    //
//----------------------------------------------------------------//
bool isSpo2Stabilizing = false; // Flag menandakan apakah SpO2 sedang dalam periode stabilisasi
unsigned long spo2StabilizationStartTime = 0; // Waktu dimulainya periode stabilisasi SpO2
const unsigned long SPO2_STABILIZATION_PERIOD = 10000; // Durasi periode stabilisasi (10 detik)
static double prevESpO2ForStabilization = 0.0; // Menyimpan nilai ESpO2 sebelumnya untuk deteksi awal bacaan valid

//----------------------------------------------------------------//
//                      PROTOTYPE FUNGSI                          //
//----------------------------------------------------------------//
void updateOledDisplay();      // Fungsi untuk memperbarui tampilan OLED
void setupMAX30102();          // Fungsi untuk setup sensor MAX30102
void calculateSpO2();          // Fungsi untuk menghitung nilai SpO2
bool connectToNrfServer();     // Fungsi untuk menghubungkan ke server nRF (BLE Client)
void setupBleServerForPhone(); // Fungsi untuk setup server BLE (untuk Smartphone)
void sendJsonToSmartphone();   // Fungsi untuk mengirim data JSON ke Smartphone
void manageActuators();        // Fungsi untuk mengelola buzzer dan motor getar

//----------------------------------------------------------------//
//                       CALLBACK BLE CLIENT (NRF)                //
//----------------------------------------------------------------//
// Callback ini dipanggil ketika ada notifikasi data dari nRF52
static void nrfNotifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
    if (length == 5) { // Data yang diharapkan: 1 byte status + 4 byte timestamp
        lastSnoreStatusFromNrf = pData[0]; // Ambil status mendengkur
        memcpy(&lastTimestampFromNrf, &pData[1], 4); // Ambil timestamp

        // Debug print ke Serial Monitor (hanya terlihat jika USB terhubung)
        Serial.print("Diterima dari nRF - Snore: ");
        Serial.print(lastSnoreStatusFromNrf);
        Serial.print(", Timestamp: ");
        Serial.println(lastTimestampFromNrf);
        newDataFromNrf = true; // Set flag bahwa ada data baru
    } else {
        Serial.print("Menerima data dari nRF dengan panjang tidak sesuai: ");
        Serial.println(length);
    }
}

// Callback untuk event koneksi BLE Client (ke nRF)
class MyNrfClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient *pclient_nrf) {
        Serial.println("Terhubung ke server nRF");
    }

    void onDisconnect(BLEClient *pclient_nrf) {
        bleClient_nrfConnected = false;
        oled_nrfConnected = false; // Update status OLED
        Serial.println("Terputus dari server nRF");
        BLEDevice::getScan()->start(5, false); // Coba scan lagi selama 5 detik
    }
};

// Callback untuk hasil scan perangkat BLE (mencari nRF)
class MyAdvertisedNrfDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.print("Perangkat BLE ditemukan: ");
        Serial.println(advertisedDevice.toString().c_str());
        // Cek apakah perangkat yang ditemukan memiliki service UUID yang dicari
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
            BLEDevice::getScan()->stop(); // Hentikan scan jika perangkat target ditemukan
            myNrfDevice = new BLEAdvertisedDevice(advertisedDevice); // Simpan informasi perangkat target
            bleClient_doConnectToNrf = true; // Set flag untuk mencoba koneksi
            Serial.println("Perangkat nRF target ditemukan! Mencoba menghubungkan...");
        }
    }
};

//----------------------------------------------------------------//
//                    CALLBACK BLE SERVER (PHONE)                 //
//----------------------------------------------------------------//
// Callback untuk event koneksi BLE Server (dari Smartphone)
class MyPhoneServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer_phone) {
        bleServer_smartphoneConnected = true;
        oled_phoneBLEConnected = true; // Update status OLED
        Serial.println("Smartphone terhubung");
    }

    void onDisconnect(BLEServer *pServer_phone) {
        bleServer_smartphoneConnected = false;
        oled_phoneBLEConnected = false; // Update status OLED
        Serial.println("Smartphone terputus");
        delay(500); // Beri jeda sebelum memulai advertising lagi
        pServer_phone->getAdvertising()->start(); // Mulai advertising lagi agar HP bisa reconnect
        Serial.println("Advertising untuk smartphone dimulai ulang");
    }
};

//----------------------------------------------------------------//
//                          SETUP                                 //
//----------------------------------------------------------------//
void setup() {
    Serial.begin(115200); // Inisialisasi Serial Monitor
    Serial.println("Memulai ESP32 Sensor Hub...");

    Wire.begin(); // Inisialisasi I2C

    // Inisialisasi OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("Alokasi SSD1306 gagal"));
        for (;;); // Hentikan program jika OLED gagal diinisialisasi
    }
    Serial.println(F("SSD1306 Diinisialisasi."));
    display.clearDisplay(); // Bersihkan buffer display
    
    // Tampilkan layar "Initializing..."
    display.setTextSize(2); 
    display.setTextColor(SSD1306_WHITE);
    const char* initMsg = "Initializing...";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(initMsg, 0, 0, &x1, &y1, &w, &h); // Hitung dimensi teks untuk centering
    display.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT - h) / 2); // Posisikan kursor di tengah
    display.print(initMsg); // Cetak pesan
    display.display();      // Tampilkan di OLED
    delay(2000);            // Tahan layar "Initializing..." selama 2 detik

    // Inisialisasi Sensor MAX30102
    setupMAX30102();

    // Inisialisasi BLE Device (ESP32)
    BLEDevice::init("ESP32_SensorHub"); // Nama perangkat ESP32 saat advertising

    // Setup BLE Client (untuk koneksi ke nRF)
    Serial.println("Inisialisasi BLE Client untuk nRF...");
    BLEScan *pBLEScan = BLEDevice::getScan(); // Dapatkan objek scanner BLE
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedNrfDeviceCallbacks()); // Set callback untuk hasil scan
    pBLEScan->setInterval(1349); // Interval scan
    pBLEScan->setWindow(449);    // Jendela scan
    pBLEScan->setActiveScan(true); // Mode scan aktif
    pBLEScan->start(10, false);    // Mulai scan selama 10 detik (non-blocking untuk operasi scan utama)
    Serial.println("BLE Scan untuk nRF dimulai...");

    // Setup BLE Server (untuk koneksi dari Smartphone)
    setupBleServerForPhone();

    // Inisialisasi Pin Aktuator
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); // Pastikan buzzer mati di awal
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW);  // Pastikan motor getar mati di awal

    // Pengaturan awal status OLED
    oled_nrfConnected = bleClient_nrfConnected;
    oled_phoneBLEConnected = bleServer_smartphoneConnected;
    updateOledDisplay(); // Tampilkan layar OLED awal (kemungkinan besar prompt NRF)
    lastOledUpdateTime = millis();

    Serial.println("Setup selesai. Memasuki loop utama...");
}

//----------------------------------------------------------------//
//                        LOOP UTAMA                              //
//----------------------------------------------------------------//
void loop() {
    unsigned long currentTime = millis(); // Dapatkan waktu saat ini

    // 1. Handle Koneksi ke nRF (BLE Client)
    if (bleClient_doConnectToNrf) { // Jika ada permintaan koneksi ke nRF
        if (connectToNrfServer()) {
            Serial.println("Berhasil terhubung dan mendaftar notifikasi dari nRF.");
        } else {
            Serial.println("Gagal terhubung ke nRF. Mencoba scan ulang...");
            BLEDevice::getScan()->start(10, false); // Coba scan lagi
        }
        bleClient_doConnectToNrf = false; // Reset flag permintaan koneksi
    }

    // Sinkronisasi status koneksi aktual dengan status untuk OLED
    if (oled_nrfConnected != bleClient_nrfConnected) {
        oled_nrfConnected = bleClient_nrfConnected;
    }
    if (oled_phoneBLEConnected != bleServer_smartphoneConnected) {
        oled_phoneBLEConnected = bleServer_smartphoneConnected;
    }

    // 2. Hitung SpO2 secara kontinu
    calculateSpO2();

    // 3. Kelola Aktuator (Buzzer dan Motor Getar)
    manageActuators();

    // 4. Logika Layar OLED
    if (!oled_nrfConnected) { // Jika nRF tidak terhubung
        if (currentOledScreen != SCREEN_NRF_PROMPT) { // Jika layar saat ini bukan prompt NRF
            currentOledScreen = SCREEN_NRF_PROMPT; // Ganti ke layar prompt NRF
            updateOledDisplay(); // Update tampilan
        } else { // Jika sudah di layar prompt NRF, update secara berkala (misal, untuk animasi loading jika ada)
             if(currentTime - lastOledUpdateTime > 500){ // Update setiap 500ms
                updateOledDisplay();
                lastOledUpdateTime = currentTime;
            }
        }
    } else { // Jika nRF terhubung
        if (currentOledScreen == SCREEN_NRF_PROMPT) { // Jika sebelumnya di layar prompt NRF
            currentOledScreen = SCREEN_PHONE_INFO; // Ganti ke layar info sensor
            updateOledDisplay(); // Update tampilan
            lastOledUpdateTime = currentTime; // Reset waktu update
        } else if (currentOledScreen == SCREEN_PHONE_INFO) { // Jika sudah di layar info sensor
            if (currentTime - lastOledUpdateTime > oledUpdateInterval) { // Update sesuai interval
                updateOledDisplay();
                lastOledUpdateTime = currentTime;
            }
        }
    }

    // 5. Kirim Data JSON ke Smartphone via BLE Server (jika ada data baru dari nRF)
    if (newDataFromNrf) {
        if (bleServer_smartphoneConnected) { // Hanya kirim jika HP terhubung
            sendJsonToSmartphone();
        } else {
            Serial.println("Smartphone tidak terhubung. Data dari nRF tidak dikirim.");
        }
        newDataFromNrf = false; // Reset flag data baru setelah diproses
    }

    delay(50); // Jeda singkat untuk stabilitas loop
}

//----------------------------------------------------------------//
//                    FUNGSI IMPLEMENTASI                         //
//----------------------------------------------------------------//

void updateOledDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Tampilkan status koneksi nRF
    display.setCursor(0, 0);
    if (oled_nrfConnected) {
        display.print(F("nrf=conect"));
    } else {
        display.print(F("nrf=disconect"));
    }

    // Tampilkan logo status koneksi HP
    int logoX = SCREEN_WIDTH - LOGO_WIDTH;
    int logoY = 0;
    if (oled_phoneBLEConnected) {
        display.drawBitmap(logoX, logoY, phone1_logo, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
    } else {
        display.drawBitmap(logoX, logoY, no_phone_logo, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
    }

    // Gambar garis pemisah (navbar)
    int textHeightApprox = 8; // Perkiraan tinggi teks default
    int navbarY = max(LOGO_HEIGHT, textHeightApprox) + 2; // Posisi Y garis pemisah
    display.drawLine(0, navbarY, SCREEN_WIDTH, navbarY, SSD1306_WHITE);
    int contentY = navbarY + 5; // Posisi Y untuk konten di bawah garis

    // Tampilkan konten sesuai layar saat ini
    if (currentOledScreen == SCREEN_NRF_PROMPT) {
        display.setTextSize(1);
        // Center teks pesan prompt NRF
        int textWidthNrf1 = 13 * 6; // Perkiraan lebar "Hidupkan nrf"
        int textWidthNrf2 = 18 * 6; // Perkiraan lebar "terlebih dahulu!!"
        display.setCursor((SCREEN_WIDTH - textWidthNrf1) / 2, contentY + 10);
        display.println(F("Hidupkan nrf"));
        display.setCursor((SCREEN_WIDTH - textWidthNrf2) / 2, contentY + 20);
        display.println(F("terlebih dahulu!!"));
    } else if (currentOledScreen == SCREEN_PHONE_INFO) {
        display.setCursor(0, contentY);
        display.print(F("SpO2: "));
        // Tampilkan "--%" jika jari tidak terdeteksi atau ESpO2 = 0
        if (ESpO2 == 0 && particleSensor.getFIFOIR() < FINGER_DETECT_THRESHOLD_IR) {
            display.print(F("--%"));
        } else {
            // Tampilkan nilai SpO2 dengan 0 atau 1 desimal
            display.print(ESpO2, (ESpO2 == 100.0 || ESpO2 == 0.0 || ESpO2 == 80.0) ? 0 : 1);
            display.print(F("%"));
        }
        display.setCursor(0, contentY + 12);
        display.print(F("Status: "));
        display.print(lastSnoreStatusFromNrf); // Status mendengkur dari nRF
        display.setCursor(0, contentY + 24);
        display.print(F("TS: "));
        display.print(lastTimestampFromNrf); // Timestamp dari nRF
    }
    display.display(); // Tampilkan buffer ke OLED
}

void setupMAX30102() {
    Serial.println("Inisialisasi MAX30102/5...");
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) { // Coba mulai sensor dengan kecepatan I2C tinggi
        Serial.println("MAX30102/5 tidak ditemukan. Periksa kabel/daya.");
        // Bisa tambahkan loop tak terbatas di sini jika sensor adalah kritikal
    } else {
        Serial.println("MAX30102/5 Diinisialisasi.");
        // Konfigurasi sensor MAX30102 (nilai ini bisa disesuaikan)
        byte ledBrightness = 0x7F; // Kecerahan LED (0x00-0xFF)
        byte sampleAverage = 4;    // Jumlah sampel rata-rata (1, 2, 4, 8, 16, 32)
        byte ledMode = 2;          // Mode LED (1: Red only, 2: Red & IR, 3: Red, IR & Green)
        int sampleRate = 200;      // Sample rate (sampel per detik: 50, 100, 200, 400, 800, 1000, 1600, 3200)
        int pulseWidth = 411;      // Lebar pulsa LED (69, 118, 215, 411 us)
        int adcRange = 16384;      // Rentang ADC (2048, 4096, 8192, 16384 nA)
        particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    }
}

void calculateSpO2() {
    particleSensor.check(); // Cek status sensor dan baca data FIFO
    uint32_t irValueCurrentLoop = 0; // Untuk menyimpan nilai IR terakhir yang valid dalam loop ini

    while (particleSensor.available()) { // Jika ada data sampel baru di FIFO
        irValueCurrentLoop = particleSensor.getFIFOIR(); // Dapatkan nilai IR
        uint32_t redValue = particleSensor.getFIFORed(); // Dapatkan nilai Red

        // Kalkulasi rata-rata bergerak sederhana untuk baseline
        avered = avered * 0.95 + (double)redValue * 0.05;
        aveir = aveir * 0.95 + (double)irValueCurrentLoop * 0.05;
        // Kalkulasi sum of squares untuk RMS AC component
        sumredrms += (redValue - avered) * (redValue - avered);
        sumirrms += (irValueCurrentLoop - aveir) * (irValueCurrentLoop - aveir);

        sampleCount++;
        if (sampleCount % SAMPLES_FOR_SPO2_CALC == 0) { // Hitung SpO2 setiap SAMPLES_FOR_SPO2_CALC sampel
            double R = (sqrt(sumredrms / SAMPLES_FOR_SPO2_CALC) / avered) / (sqrt(sumirrms / SAMPLES_FOR_SPO2_CALC) / aveir);
            // Rumus SpO2 berdasarkan nilai R (Ratio of Ratios) , mungkin perlu kalibrasi
            // double spo2_calculated = -45.060 * R * R + 30.354 * R + 94.845;
            double spo2_calculated = (3.1626 - R) / 0.0281;
            
            // Terapkan filter ke nilai SpO2
            ESpO2 = SPO2_FILTER_FACTOR * ESpO2 + (1.0 - SPO2_FILTER_FACTOR) * spo2_calculated;
            
            // Jika tidak ada jari terdeteksi (IR rendah), set SpO2 ke 0
            if (irValueCurrentLoop < FINGER_DETECT_THRESHOLD_IR) {
                ESpO2 = 0;
            } else { // Batasi nilai SpO2 dalam rentang wajar
                if (ESpO2 > 100.0) ESpO2 = 100.0;
                // Baris ini mungkin menyebabkan SpO2 tampak mulai dari 80% jika bacaan awal sebenarnya lebih rendah
                if (ESpO2 < 80.0 && ESpO2 != 0.0) ESpO2 = 80.0; 
            }
            // Reset akumulator RMS
            sumredrms = 0.0;
            sumirrms = 0.0;
        }
        if (sampleCount >= SAMPLES_FOR_SPO2_CALC) sampleCount = 0; // Reset counter sampel
        particleSensor.nextSample(); // Maju ke sampel berikutnya di FIFO
    }
    
    // Jika tidak ada sampel yang diproses (misalnya, buffer kosong di awal atau setelah jari dicabut)
    // Coba dapatkan nilai IR saat ini untuk pengecekan jari terakhir
    if (irValueCurrentLoop == 0) { 
        irValueCurrentLoop = particleSensor.getFIFOIR(); 
    }

    // Kondisi tambahan untuk memastikan SpO2 = 0 jika jari tidak ada (berdasarkan IR rendah)
    // Ini penting agar logika stabilisasi tahu kapan jari dicabut
    if (irValueCurrentLoop < FINGER_DETECT_THRESHOLD_IR && ESpO2 != 0.0) {
      ESpO2 = 0;
    }

    // Logika untuk memulai/menghentikan periode stabilisasi SpO2
    bool fingerJustPlaced = (ESpO2 != 0.0 && prevESpO2ForStabilization == 0.0);
    bool fingerJustRemoved = (ESpO2 == 0.0 && prevESpO2ForStabilization != 0.0);

    if (fingerJustPlaced) {
        // Hanya mulai timer dan print pesan jika tidak sedang dalam proses stabilisasi
        // Ini mencegah timer di-reset jika ada fluktuasi singkat ESpO2 dari 0 ke non-0 saat sudah stabilisasi.
        // Namun, untuk kasus jari dicabut lalu dipasang lagi, ini akan restart timer.
        if (!isSpo2Stabilizing) { 
            Serial.println("SpO2: Jari terdeteksi, periode stabilisasi dimulai (10 detik).");
            spo2StabilizationStartTime = millis(); // Catat waktu mulai stabilisasi
        }
        isSpo2Stabilizing = true; // Set flag bahwa sedang stabilisasi
    } else if (fingerJustRemoved) {
        if (isSpo2Stabilizing) { // Hanya print jika memang sedang stabilisasi lalu jari dicabut
            Serial.println("SpO2: Jari dicabut/bacaan hilang, periode stabilisasi direset.");
        }
        isSpo2Stabilizing = false; // Reset flag stabilisasi
    }
    prevESpO2ForStabilization = ESpO2; // Simpan nilai ESpO2 saat ini untuk perbandingan di iterasi berikutnya
}

void manageActuators() {
    // Variabel static untuk menyimpan state antar pemanggilan fungsi
    static unsigned long lastBuzzerChangeTime = 0;     // Waktu terakhir buzzer intermittent diubah state-nya
    static bool buzzerIntermittentPhysicalStateOn = false; // Status fisik buzzer intermittent (ON/OFF)
    static byte currentBuzzerMode = 0;                 // Mode buzzer saat ini (0:OFF, 1:Intermittent, 2:Continuous)

    static bool motorIsRunning = false;                // Status apakah motor sedang dalam siklus 1 detik ON
    static unsigned long motorRunStartTime = 0;        // Waktu motor mulai dinyalakan
    static bool prevSnoreStatusWasOneForMotor = false; // Status mendengkur sebelumnya (untuk deteksi perubahan ke 1)

    unsigned long currentTime = millis(); // Waktu saat ini
    bool activelyStabilizingThisCycle = false; // Flag lokal, apakah siklus ini masih dalam periode stabilisasi aktif

    // Cek dan update status stabilisasi SpO2
    if (isSpo2Stabilizing) { // Jika flag global menandakan kita *sedang* atau *baru saja* stabilisasi
        if (currentTime - spo2StabilizationStartTime < SPO2_STABILIZATION_PERIOD) {
            activelyStabilizingThisCycle = true; // Masih dalam periode 10 detik
        } else {
            // Periode 10 detik telah berakhir
            Serial.println("Actuators: Periode stabilisasi SpO2 selesai."); // Pesan ini akan tercetak sekali
            isSpo2Stabilizing = false; // Update flag global: tidak lagi stabilisasi
            activelyStabilizingThisCycle = false; // Tidak aktif stabilisasi di siklus ini juga
        }
    }

    // --- Logika Buzzer ---
    byte newTargetBuzzerMode = 0; // Mode buzzer target default: OFF

    if (activelyStabilizingThisCycle) {
        // Jika sedang aktif stabilisasi, target mode buzzer tetap 0 (OFF)
        // Pesan Serial.println opsional bisa ditambahkan di sini untuk debugging
        // Serial.println("Actuators: SpO2 sedang stabilisasi, buzzer ditahan.");
    } else {
        // Tentukan mode buzzer target berdasarkan ESpO2 (hanya jika tidak sedang stabilisasi aktif)
        // ESpO2 diperbarui oleh fungsi calculateSpO2()
        if (ESpO2 > 0 && ESpO2 < 88.0) { // SpO2 di bawah 88% (dan valid > 0)
            newTargetBuzzerMode = 2; // Buzzer nyala terus menerus
        } else if (ESpO2 >= 88.0 && ESpO2 < 95.0) { // SpO2 antara 88% dan 94.x%
            newTargetBuzzerMode = 1; // Buzzer nyala putus-putus
        }
        // Jika ESpO2 >= 95% atau ESpO2 = 0, newTargetBuzzerMode tetap 0 (OFF)
    }

    // Handle transisi atau aksi mode buzzer yang sedang berjalan
    if (newTargetBuzzerMode != currentBuzzerMode) { // Jika ada perubahan mode buzzer
        currentBuzzerMode = newTargetBuzzerMode; // Update mode saat ini
        if (currentBuzzerMode == 2) { // Transisi ke mode Continuous
            digitalWrite(BUZZER_PIN, HIGH);
        } else if (currentBuzzerMode == 1) { // Transisi ke mode Intermittent
            buzzerIntermittentPhysicalStateOn = true; // Mulai dengan buzzer ON
            digitalWrite(BUZZER_PIN, HIGH);
            lastBuzzerChangeTime = currentTime; // Catat waktu untuk interval berikutnya
        } else { // Transisi ke mode OFF (currentBuzzerMode == 0)
            digitalWrite(BUZZER_PIN, LOW);
        }
    } else { // Jika mode buzzer tidak berubah
        if (currentBuzzerMode == 1) { // Jika sedang dalam mode Intermittent
            if (currentTime - lastBuzzerChangeTime >= 500) { // Cek interval 500ms
                lastBuzzerChangeTime = currentTime; // Reset timer interval
                buzzerIntermittentPhysicalStateOn = !buzzerIntermittentPhysicalStateOn; // Toggle state buzzer
                digitalWrite(BUZZER_PIN, buzzerIntermittentPhysicalStateOn ? HIGH : LOW);
            }
        }
        // Untuk mode Continuous (2) atau OFF (0), tidak ada aksi berulang berbasis waktu di sini
    }

    // --- Logika Motor Getar ---
    // Pemicu motor adalah jika newDataFromNrf true DAN lastSnoreStatusFromNrf BARU SAJA berubah menjadi 1
    if (newDataFromNrf) { // Hanya cek jika ada data baru dari nRF
        if (lastSnoreStatusFromNrf == 1 && !prevSnoreStatusWasOneForMotor) {
            // Status mendengkur baru saja berubah dari bukan 1 menjadi 1
            if (!motorIsRunning) { // Hanya nyalakan jika motor tidak sedang dalam siklus 1 detiknya
                motorIsRunning = true;
                motorRunStartTime = currentTime;
                digitalWrite(MOTOR_PIN, HIGH);
                // Serial.println("Motor: ON"); // Debug
            }
        }
    }
    // Update status mendengkur sebelumnya SETELAH pengecekan di atas, dan HANYA jika ada data baru
    // Ini penting agar prevSnoreStatusWasOneForMotor merefleksikan status *sebelum* data baru saat ini
    if (newDataFromNrf) {
      prevSnoreStatusWasOneForMotor = (lastSnoreStatusFromNrf == 1);
    }

    // Kelola durasi motor menyala (matikan setelah 1 detik)
    if (motorIsRunning) {
        if (currentTime - motorRunStartTime >= 1000) { // 1000 ms = 1 detik
            digitalWrite(MOTOR_PIN, LOW);
            motorIsRunning = false;
            // Serial.println("Motor: OFF"); // Debug
        }
    }
}

bool connectToNrfServer() {
    Serial.print("Mencoba koneksi ke nRF: ");
    Serial.println(myNrfDevice->getAddress().toString().c_str());

    BLEClient *pClientNrf = BLEDevice::createClient();
    Serial.println(" - Klien BLE dibuat");
    pClientNrf->setClientCallbacks(new MyNrfClientCallbacks());

    // Coba konek ke server nRF
    if (!pClientNrf->connect(myNrfDevice)) {
        Serial.println(" - Gagal terhubung ke server nRF");
        return false;
    }
    Serial.println(" - Terhubung ke server nRF");

    // Dapatkan service dari server nRF
    BLERemoteService *pRemoteServiceNrf = pClientNrf->getService(serviceUUID);
    if (pRemoteServiceNrf == nullptr) {
        Serial.print("Gagal menemukan service UUID nRF: ");
        Serial.println(serviceUUID.toString().c_str());
        pClientNrf->disconnect();
        return false;
    }
    Serial.println(" - Service nRF ditemukan");

    // Dapatkan karakteristik dari service
    pRemoteCharacteristicNrf = pRemoteServiceNrf->getCharacteristic(charUUID_NRF_READ);
    if (pRemoteCharacteristicNrf == nullptr) {
        Serial.print("Gagal menemukan characteristic UUID nRF: ");
        Serial.println(charUUID_NRF_READ.toString().c_str());
        pClientNrf->disconnect();
        return false;
    }
    Serial.println(" - Characteristic nRF ditemukan");

    // Daftar untuk notifikasi jika karakteristik mendukungnya
    if (pRemoteCharacteristicNrf->canNotify()) {
        pRemoteCharacteristicNrf->registerForNotify(nrfNotifyCallback);
        Serial.println(" - Berhasil mendaftar untuk notifikasi dari nRF");
    } else {
        Serial.println(" - Characteristic nRF tidak bisa memberi notifikasi");
        pClientNrf->disconnect();
        return false;
    }

    bleClient_nrfConnected = true; // Set status terkoneksi
    oled_nrfConnected = true;      // Update status untuk OLED
    return true;
}

void setupBleServerForPhone() {
    Serial.println("Setup BLE Server untuk Smartphone...");
    pServer = BLEDevice::createServer(); // Buat server BLE
    pServer->setCallbacks(new MyPhoneServerCallbacks()); // Set callback untuk event server

    // Buat service BLE
    BLEService *pServicePhone = pServer->createService(serviceUUID);

    // Buat karakteristik BLE
    pCharacteristicPhone = pServicePhone->createCharacteristic(
        charUUID_PHONE_WRITE,
        BLECharacteristic::PROPERTY_READ   | // Bisa dibaca
        BLECharacteristic::PROPERTY_NOTIFY | // Bisa mengirim notifikasi
        BLECharacteristic::PROPERTY_WRITE_NR // Bisa ditulis tanpa response (opsional, tergantung kebutuhan)
    );
    pServicePhone->start(); // Mulai service

    // Mulai advertising agar ESP32 bisa ditemukan oleh Smartphone
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(serviceUUID); // Tambahkan service UUID ke advertising packet
    pAdvertising->setScanResponse(true); // Izinkan scan response
    pAdvertising->setMinPreferred(0x06); // Pengaturan interval advertising (opsional)
    pAdvertising->setMinPreferred(0x12); // Pengaturan interval advertising (opsional)
    pServer->getAdvertising()->start();  // Mulai advertising
    Serial.println("BLE Server untuk Smartphone mulai advertising");
}

void sendJsonToSmartphone() {
    // Buat string JSON dengan data sensor dan status
    String jsonData = "{\"status\":" + String(lastSnoreStatusFromNrf) +
                      ",\"timestamp\":" + String(lastTimestampFromNrf) +
                      ",\"spo2\":" + String(ESpO2, (ESpO2 == 100.0 || ESpO2 == 0.0 || ESpO2 == 80.0) ? 0 : 2) + // Format SpO2 dengan 0 atau 2 desimal
                      "}";
    pCharacteristicPhone->setValue(jsonData.c_str()); // Set nilai karakteristik
    pCharacteristicPhone->notify(); // Kirim notifikasi ke Smartphone yang terhubung & subscribe

    Serial.print("Terkirim ke smartphone: "); // Debug print
    Serial.println(jsonData);
}