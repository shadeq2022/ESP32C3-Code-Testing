//----------------------------------------------------------------//
//                                LIBRARIES                                 //
//----------------------------------------------------------------//
#include <Wire.h>           // Untuk komunikasi I2C
#include <Adafruit_GFX.h>   // Library grafis Adafruit
#include <Adafruit_SSD1306.h> // Library driver OLED SSD1306

#include <BLEDevice.h>      // Library BLE untuk ESP32
#include <BLEUtils.h>
#include <BLEServer.h>

#include "MAX30105.h"       // Library untuk sensor MAX30102/MAX30105

//----------------------------------------------------------------//
//                             OLED DEFINITIONS                             //
//----------------------------------------------------------------//
#define SCREEN_WIDTH 128        // Lebar OLED dalam piksel
#define SCREEN_HEIGHT 64        // Tinggi OLED dalam piksel
#define OLED_RESET -1           // Pin RESET OLED (-1 jika tidak digunakan)
#define SCREEN_ADDRESS 0x3C     // Alamat I2C OLED (umumnya 0x3C atau 0x3D)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Inisialisasi objek display OLED

// Dimensi logo
const int LOGO_WIDTH = 17;      // Lebar logo telepon
const int LOGO_HEIGHT = 17;     // Tinggi logo telepon

// Dimensi logo ZZZ (BARU)
const int LOGO_ZZZ_WIDTH = 40;
const int LOGO_ZZZ_HEIGHT = 17;

// Status layar OLED
#define SCREEN_PHONE_INFO 1     // Konstanta untuk layar info HP & Sensor
#define SCREEN_NRF_PROMPT 2     // Konstanta untuk layar prompt NRF
#define SCREEN_SNORING_ALERT 3  // Konstanta untuk layar peringatan mendengkur BARU
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

// Bitmap logo ZZZ
const unsigned char logo_zzz [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf0, 0x00, 
	0x00, 0x00, 0x1f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x0f, 
	0xfc, 0x01, 0x80, 0x00, 0x00, 0x3c, 0x03, 0x80, 0x00, 0x00, 0x70, 0x07, 0x00, 0x1f, 0xc1, 0xe0, 
	0x0e, 0x00, 0x00, 0x81, 0x80, 0x1f, 0xf8, 0x01, 0x03, 0x80, 0x1f, 0xfc, 0x02, 0x07, 0x00, 0x00, 
	0x00, 0x04, 0x07, 0xff, 0x80, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xc0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00
};

//----------------------------------------------------------------//
//                           ACTUATOR DEFINITIONS                           //
//----------------------------------------------------------------//
#define BUZZER_PIN 3  // Pin GPIO untuk buzzer
#define MOTOR_PIN 10  // Pin GPIO untuk motor getar

//----------------------------------------------------------------//
//                            BLE DEFINITIONS                             //
//----------------------------------------------------------------//
static BLEUUID serviceUUID("12345678-1234-5678-1234-56789abcdef0"); // UUID untuk service BLE utama
static BLEUUID charUUID_NRF_READ("12345678-1234-5678-1234-56789abcdef1");    // Karakteristik untuk Menerima data dari nRF (notifikasi)
static BLEUUID charUUID_PHONE_WRITE("87654321-4321-8765-4321-123456789abc"); // Karakteristik untuk Mengirim data ke Smartphone

// Variabel BLE Client (untuk koneksi ke nRF)
static boolean bleClient_doConnectToNrf = false; // Flag untuk memicu upaya koneksi ke nRF
static boolean bleClient_nrfConnected = false;       // Status koneksi aktual ke nRF
static BLERemoteCharacteristic *pRemoteCharacteristicNrf; // Pointer ke karakteristik remote nRF
static BLEAdvertisedDevice *myNrfDevice;           // Pointer ke perangkat nRF yang ditemukan

// Variabel BLE Server (untuk koneksi dari Smartphone)
BLEServer *pServer = nullptr; // Pointer ke objek server BLE
BLECharacteristic *pCharacteristicPhone;           // Pointer ke karakteristik untuk komunikasi dengan HP
bool bleServer_smartphoneConnected = false; // Status koneksi aktual dari Smartphone
bool phoneServerStarted = false; // Flag to ensure phone BLE server is started only once

// Variabel Penyimpanan Data dari nRF
uint8_t lastSnoreStatusFromNrf = 0; // Status mendengkur terakhir yang diterima dari nRF
uint32_t lastTimestampFromNrf = 0; // Timestamp terakhir yang diterima dari nRF
volatile bool newDataFromNrf = false; // Flag menandakan adanya data baru dari nRF

//----------------------------------------------------------------//
//                       MAX30102 SENSOR DEFINITIONS                         //
//----------------------------------------------------------------//
MAX30105 particleSensor; // Objek untuk sensor MAX30102/MAX30105
double ESpO2 = 0.0;       // Nilai SpO2 yang sudah difilter (hasil akhir)
double avered = 0; // Rata-rata nilai merah untuk kalkulasi SpO2
double aveir = 0; // Rata-rata nilai infra merah untuk kalkulasi SpO2
double sumirrms = 0;      // Akumulasi untuk kalkulasi RMS IR
double sumredrms = 0; // Akumulasi untuk kalkulasi RMS RED
int sampleCount = 0;      // Jumlah sampel yang telah dikumpulkan
const int SAMPLES_FOR_SPO2_CALC = 100; // Jumlah sampel yang dibutuhkan sebelum menghitung SpO2
const int FINGER_DETECT_THRESHOLD_IR = 30000; // Threshold IR untuk deteksi jari pada sensor
const double SPO2_FILTER_FACTOR = 0.7; // Faktor filter untuk memperhalus nilai SpO2

//----------------------------------------------------------------//
//                     SpO2 STABILIZATION DEFINITIONS                        //
//----------------------------------------------------------------//
bool isSpo2Stabilizing = false; // Flag menandakan apakah SpO2 sedang dalam periode stabilisasi
unsigned long spo2StabilizationStartTime = 0; // Waktu dimulainya periode stabilisasi SpO2
const unsigned long SPO2_STABILIZATION_PERIOD = 20000; // Durasi periode stabilisasi (20 detik)
static double prevESpO2ForStabilization = 0.0; // Menyimpan nilai ESpO2 sebelumnya untuk deteksi awal bacaan valid

//----------------------------------------------------------------//
//                             PROTOTYPE FUNGSI                             //
//----------------------------------------------------------------//
void updateOledDisplay(); // Fungsi untuk memperbarui tampilan OLED
void setupMAX30102();           // Fungsi untuk setup sensor MAX30102
void calculateSpO2(); // Fungsi untuk menghitung nilai SpO2
bool connectToNrfServer();      // Fungsi untuk menghubungkan ke server nRF (BLE Client)
void setupBleServerForPhone(); // Fungsi untuk setup server BLE (untuk Smartphone)
void sendJsonToSmartphone();   // Fungsi untuk mengirim data JSON ke Smartphone
void manageActuators(); // Fungsi untuk mengelola buzzer dan motor getar

//----------------------------------------------------------------//
//                      CALLBACK BLE CLIENT (NRF)                        //
//----------------------------------------------------------------//
static void nrfNotifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
    if (length == 5) { // Data yang diharapkan: 1 byte status + 4 byte timestamp
        lastSnoreStatusFromNrf = pData[0]; // Ambil status mendengkur dari byte pertama data yang diterima
        memcpy(&lastTimestampFromNrf, &pData[1], 4); // Salin 4 byte berikutnya ke variabel timestamp

        Serial.print("Diterima dari nRF - Snore: "); // Cetak label status mendengkur
        Serial.print(lastSnoreStatusFromNrf);
        Serial.print(", Timestamp: ");
        Serial.println(lastTimestampFromNrf);
        newDataFromNrf = true; // Set flag bahwa ada data baru dari nRF
    } else {
        Serial.print("Menerima data dari nRF dengan panjang tidak sesuai: "); // Cetak pesan error jika panjang data tidak sesuai
        Serial.println(length);
    }
}

class MyNrfClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient *pclient_nrf) {
        Serial.println("Terhubung ke server nRF"); // Cetak pesan saat berhasil terhubung ke nRF
    }

    void onDisconnect(BLEClient *pclient_nrf) {
        bleClient_nrfConnected = false; // Set status koneksi nRF ke false
        oled_nrfConnected = false; // Update status OLED untuk koneksi nRF
        Serial.println("Terputus dari server nRF");
        BLEDevice::getScan()->start(5, false); // Coba scan lagi perangkat BLE selama 5 detik secara non-blocking
    }
};

class MyAdvertisedNrfDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.print("Perangkat BLE ditemukan: "); // Cetak pesan bahwa perangkat BLE ditemukan
        Serial.println(advertisedDevice.toString().c_str());
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) { // Cek apakah perangkat yang ditemukan memiliki service UUID yang dicari
            BLEDevice::getScan()->stop(); // Hentikan scan jika perangkat target (nRF) ditemukan
            myNrfDevice = new BLEAdvertisedDevice(advertisedDevice); // Simpan informasi perangkat nRF yang ditemukan
            bleClient_doConnectToNrf = true; // Set flag untuk memicu upaya koneksi ke nRF
            Serial.println("Perangkat nRF target ditemukan! Mencoba menghubungkan..."); // Cetak pesan bahwa target nRF ditemukan
        }
    }
};
//----------------------------------------------------------------//
//                     CALLBACK BLE SERVER (PHONE)                       //
//----------------------------------------------------------------//
class MyPhoneServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer_phone) {
        bleServer_smartphoneConnected = true; // Set status koneksi smartphone ke true
        Serial.println("Smartphone terhubung (Callback)"); // Cetak pesan saat smartphone terhubung
    }

    void onDisconnect(BLEServer *pServer_phone) {
        bleServer_smartphoneConnected = false; // Set status koneksi smartphone ke false
        Serial.println("Smartphone terputus (Callback)");
        delay(500); // Beri jeda 500ms sebelum memulai advertising lagi
        pServer_phone->getAdvertising()->start(); // Mulai advertising lagi agar smartphone bisa melakukan reconnect
        Serial.println("Advertising untuk smartphone dimulai ulang"); // Cetak pesan bahwa advertising dimulai ulang
    }
};
//----------------------------------------------------------------//
//                                 SETUP                                  //
//----------------------------------------------------------------//
void setup() {
    Serial.begin(115200); // Inisialisasi Serial Monitor dengan baud rate 115200
    Serial.println("Memulai ESP32 Sensor Hub...");

    Wire.begin(); // Inisialisasi komunikasi I2C

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { // Inisialisasi OLED
        Serial.println(F("Alokasi SSD1306 gagal")); // Cetak pesan error jika OLED gagal diinisialisasi
        for (;;); // Hentikan program jika OLED gagal diinisialisasi
    }
    Serial.println(F("SSD1306 Diinisialisasi."));
    display.clearDisplay(); // Bersihkan buffer display OLED
    
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE); // Set warna teks menjadi putih
    const char* initMsg = "Initializing...";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(initMsg, 0, 0, &x1, &y1, &w, &h); // Hitung dimensi (lebar & tinggi) teks untuk centering
    display.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT - h) / 2); // Posisikan kursor di tengah layar OLED
    display.print(initMsg); // Cetak pesan "Initializing..."
    display.display(); // Tampilkan konten buffer ke layar OLED
    delay(2000); // Tahan layar "Initializing..." selama 2 detik

    setupMAX30102(); // Panggil fungsi untuk setup sensor MAX30102
    BLEDevice::init("ESP32_SensorHub"); // Inisialisasi BLE dan set nama perangkat ESP32 saat advertising

    Serial.println("Inisialisasi BLE Client untuk nRF..."); // Cetak pesan inisialisasi BLE client
    BLEScan *pBLEScan = BLEDevice::getScan(); // Dapatkan objek scanner BLE dari ESP32
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedNrfDeviceCallbacks()); // Set callback yang akan dipanggil ketika perangkat BLE ditemukan
    pBLEScan->setInterval(1349); // Set interval scan BLE (dalam 0.625ms unit, jadi 1349 * 0.625ms = ~843ms)
    pBLEScan->setWindow(449); // Set jendela scan BLE (dalam 0.625ms unit, jadi 449 * 0.625ms = ~280ms)
    pBLEScan->setActiveScan(true); // Set mode scan aktif (meminta advertising data tambahan)
    pBLEScan->start(10, false); // Mulai scan BLE selama 10 detik, non-blocking (scan berlanjut di background)
    Serial.println("BLE Scan untuk nRF dimulai..."); // Cetak pesan bahwa scan BLE dimulai

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); // Pastikan buzzer mati di awal
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW); // Pastikan motor getar mati di awal

    oled_nrfConnected = bleClient_nrfConnected;
    oled_phoneBLEConnected = false; 
    
    currentOledScreen = SCREEN_NRF_PROMPT;
    updateOledDisplay(); // Tampilkan layar OLED awal (NRF prompt with phone disconnected)
    lastOledUpdateTime = millis(); // Catat waktu update OLED terakhir
    Serial.println("Setup selesai. Memasuki loop utama...");
}

//----------------------------------------------------------------//
//                               LOOP UTAMA                               //
//----------------------------------------------------------------//
void loop() {
    unsigned long currentTime = millis(); // Dapatkan waktu saat ini dalam milidetik
    bool oledNeedsImmediateUpdate = false; 

    if (bleClient_doConnectToNrf) { // Jika ada permintaan untuk menghubungkan ke nRF
        if (connectToNrfServer()) {
            Serial.println("Berhasil terhubung dan mendaftar notifikasi dari nRF."); // Cetak pesan sukses koneksi
        } else {
            Serial.println("Gagal terhubung ke nRF. Mencoba scan ulang..."); // Cetak pesan gagal koneksi
            BLEDevice::getScan()->start(10, false); // Coba scan lagi perangkat BLE selama 10 detik, non-blocking
        }
        bleClient_doConnectToNrf = false; // Reset flag permintaan koneksi ke nRF
    }

    if (bleClient_nrfConnected && !phoneServerStarted) {
        Serial.println("nRF terhubung. Memulai BLE Server untuk Smartphone sekarang...");
        setupBleServerForPhone();
        phoneServerStarted = true;
    }

    if (oled_nrfConnected != bleClient_nrfConnected) {
        oled_nrfConnected = bleClient_nrfConnected; // Update status koneksi nRF untuk OLED jika ada perubahan
        oledNeedsImmediateUpdate = true; 
    }
    if (phoneServerStarted && (oled_phoneBLEConnected != bleServer_smartphoneConnected)) {
        oled_phoneBLEConnected = bleServer_smartphoneConnected; // Update status koneksi smartphone untuk OLED jika ada perubahan
        oledNeedsImmediateUpdate = true; 
    }

    calculateSpO2(); // Panggil fungsi untuk menghitung SpO2
    manageActuators(); // Panggil fungsi untuk mengelola aktuator

    bool reasonToUpdateOLEDFromLogic = false; 
    int targetScreen = currentOledScreen; // Layar target, defaultnya sama dengan layar saat ini

    if (!oled_nrfConnected) { // Jika nRF tidak terhubung
        targetScreen = SCREEN_NRF_PROMPT; // Selalu tampilkan prompt NRF
        if (currentOledScreen != targetScreen) { // Jika layar berubah
            reasonToUpdateOLEDFromLogic = true; 
        } else if (currentTime - lastOledUpdateTime > 500) { // Update periodik untuk layar NRF_PROMPT (misal animasi loading)
            reasonToUpdateOLEDFromLogic = true; 
        }
    } else { // Jika nRF terhubung (oled_nrfConnected is true)
        if (lastSnoreStatusFromNrf == 1) {
            targetScreen = SCREEN_SNORING_ALERT; // Ganti ke layar peringatan mendengkur
            if (currentOledScreen != targetScreen) { // Jika layar berubah ke peringatan mendengkur
                reasonToUpdateOLEDFromLogic = true; 
            }
            // Untuk SCREEN_SNORING_ALERT, pembaruan periodik tidak diperlukan kecuali ada animasi
            // Jika ingin pembaruan periodik, tambahkan kondisi 'else if' di sini
        } else { 
            targetScreen = SCREEN_PHONE_INFO; // Target kembali ke layar info SpO2
            if (currentOledScreen != targetScreen) { // Jika ada transisi dari NRF_PROMPT atau SNORING_ALERT
                reasonToUpdateOLEDFromLogic = true; 
            } else if (currentTime - lastOledUpdateTime > oledUpdateInterval) { // Update periodik untuk SCREEN_PHONE_INFO
                reasonToUpdateOLEDFromLogic = true; 
            }
        }
    }

    if (oledNeedsImmediateUpdate || reasonToUpdateOLEDFromLogic) { 
        currentOledScreen = targetScreen; // Set layar saat ini ke layar target
        updateOledDisplay(); // Panggil fungsi untuk menggambar ulang OLED
        lastOledUpdateTime = currentTime; // Catat waktu update terakhir
    }

    if (newDataFromNrf) { // Jika ada data baru dari nRF
        if (phoneServerStarted && bleServer_smartphoneConnected) { 
            sendJsonToSmartphone(); // Panggil fungsi untuk mengirim data JSON ke smartphone
        } else {
            Serial.println("Smartphone tidak terhubung atau server belum aktif. Data dari nRF tidak dikirim."); // Pesan jika smartphone tidak terhubung
        }
        newDataFromNrf = false; // Reset flag data baru setelah diproses
    }

    delay(50); // Jeda singkat 50ms untuk stabilitas loop dan memberi waktu proses lain
}

//----------------------------------------------------------------//
//                         FUNGSI IMPLEMENTASI                         //
//----------------------------------------------------------------//

void updateOledDisplay() {
    display.clearDisplay(); // Bersihkan buffer display OLED sebelum menggambar
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    if (currentOledScreen == SCREEN_SNORING_ALERT) {
        // --- BAGIAN BARU UNTUK LAYAR PERINGATAN MENDENGKUR ---
        // Gambar logo ZZZ di tengah atas
        int logoZzzX = (SCREEN_WIDTH - LOGO_ZZZ_WIDTH) / 2;
        int logoZzzY = 0; // Atau sedikit margin dari atas jika diinginkan, misal 2 atau 3 piksel
        display.drawBitmap(logoZzzX, logoZzzY, logo_zzz, LOGO_ZZZ_WIDTH, LOGO_ZZZ_HEIGHT, SSD1306_WHITE);

        // Posisi Y untuk konten teks di bawah logo ZZZ
        int contentYSnoring = LOGO_ZZZ_HEIGHT + 5; // Beri jarak 5 piksel setelah logo

        // Tampilkan pesan peringatan mendengkur
        display.setTextSize(1); // Ukuran teks normal untuk pesan peringatan
        display.setTextColor(SSD1306_WHITE);
        const char* alertMsgL1 = "Snoring detected!";
        const char* alertMsgL2 = "Perbaiki posisi tidur";

        int16_t tempX, tempY;
        uint16_t wL1, hL1, wL2, hL2;
        display.getTextBounds(alertMsgL1, 0, 0, &tempX, &tempY, &wL1, &hL1); // Dapatkan dimensi teks peringatan baris 1
        display.getTextBounds(alertMsgL2, 0, 0, &tempX, &tempY, &wL2, &hL2); // Dapatkan dimensi teks peringatan baris 2

        // Hitung Posisi Y untuk baris pertama agar blok teks berada di tengah area di bawah logo ZZZ
        uint16_t totalTextHeight = hL1 + hL2 + 3; // Tinggi total dua baris + spasi kecil (3 piksel)
        // Area yang tersedia untuk teks: SCREEN_HEIGHT - contentYSnoring
        int availableHeightForText = SCREEN_HEIGHT - contentYSnoring;
        int textBlockY = contentYSnoring + (availableHeightForText - totalTextHeight) / 2;
        
        display.setCursor((SCREEN_WIDTH - wL1) / 2, textBlockY); // Atur kursor untuk centering teks peringatan baris 1
        display.println(alertMsgL1);
        display.setCursor((SCREEN_WIDTH - wL2) / 2, textBlockY + hL1 + 3); // Posisikan baris kedua di bawah baris pertama + spasi
        display.println(alertMsgL2);

    } else {
        // --- BAGIAN LAMA UNTUK LAYAR NRF_PROMPT DAN PHONE_INFO ---
        display.setCursor(0, 0); // Atur kursor ke posisi (0,0) (pojok kiri atas)
        if (oled_nrfConnected) {
            display.print(F("nrf=conect")); // Tampilkan status nRF terhubung
        } else {
            display.print(F("nrf=disconect")); // Tampilkan status nRF tidak terhubung
        }

        int logoX = SCREEN_WIDTH - LOGO_WIDTH; // Hitung posisi X untuk logo (kanan atas)
        int logoY = 0;
        if (oled_phoneBLEConnected) {
            display.drawBitmap(logoX, logoY, phone1_logo, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE); // Gambar logo telepon terhubung
        } else {
            display.drawBitmap(logoX, logoY, no_phone_logo, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE); // Gambar logo telepon tidak terhubung
        }

        int textHeightApprox = 8; // Perkiraan tinggi teks default (ukuran 1)
        int navbarY = max(LOGO_HEIGHT, textHeightApprox) + 2; // Hitung posisi Y garis pemisah di bawah logo/teks status
        display.drawLine(0, navbarY, SCREEN_WIDTH, navbarY, SSD1306_WHITE);
        int contentYDefault = navbarY + 5; // Posisi Y untuk konten di bawah garis pemisah

        if (currentOledScreen == SCREEN_NRF_PROMPT) {
            display.setTextSize(1); // Set ukuran teks untuk pesan prompt NRF
            const char* nrfMsg1 = "Hidupkan nrf";
            const char* nrfMsg2 = "terlebih dahulu!!";
            int16_t x1_nrf, y1_nrf;
            uint16_t w1_nrf, h1_nrf, w2_nrf, h2_nrf;
            display.getTextBounds(nrfMsg1, 0, 0, &x1_nrf, &y1_nrf, &w1_nrf, &h1_nrf); // Dapatkan dimensi teks baris 1
            display.getTextBounds(nrfMsg2, 0, 0, &x1_nrf, &y1_nrf, &w2_nrf, &h2_nrf); // Dapatkan dimensi teks baris 2

            display.setCursor((SCREEN_WIDTH - w1_nrf) / 2, contentYDefault + 10); // Atur kursor untuk centering teks baris 1
            display.println(F("Hidupkan nrf")); // Tampilkan pesan baris pertama
            display.setCursor((SCREEN_WIDTH - w2_nrf) / 2, contentYDefault + 20); // Atur kursor untuk centering teks baris 2
            display.println(F("terlebih dahulu!!")); // Tampilkan pesan baris kedua
        } else if (currentOledScreen == SCREEN_PHONE_INFO) {
            display.setTextSize(3); // Ukuran teks lebih besar untuk SpO2
            display.setTextColor(SSD1306_WHITE);
            String spo2Text;
            if (ESpO2 == 0.0 && particleSensor.getFIFOIR() < FINGER_DETECT_THRESHOLD_IR) {
                spo2Text = "--%"; // Teks jika SpO2 tidak valid atau jari tidak ada
            } else {
                spo2Text = String(ESpO2, (ESpO2 == 100.0 || ESpO2 == 0.0 || ESpO2 == 80.0) ? 0 : 1) + "%"; // Tampilkan nilai SpO2 dengan 0 desimal jika nilainya bulat (100, 0, 80) atau 1 desimal untuk lainnya
            }

            int16_t x1_spo2, y1_spo2;
            uint16_t w_spo2, h_spo2;
            display.getTextBounds(spo2Text, 0, 0, &x1_spo2, &y1_spo2, &w_spo2, &h_spo2); // Dapatkan dimensi teks SpO2
            int spo2YPos = contentYDefault + ((SCREEN_HEIGHT - contentYDefault - h_spo2) / 2) ; // Posisi Y disesuaikan agar di tengah area konten di bawah navbar
            display.setCursor((SCREEN_WIDTH - w_spo2) / 2, spo2YPos); // Atur kursor untuk centering teks SpO2
            display.print(spo2Text);
        }
    }
    display.display(); 
}

void setupMAX30102() {
    Serial.println("Inisialisasi MAX30102/5..."); // Cetak pesan inisialisasi sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) { // Coba mulai sensor dengan kecepatan I2C tinggi (400kHz)
        Serial.println("MAX30102/5 tidak ditemukan. Periksa kabel/daya."); // Pesan jika sensor tidak ditemukan
    } else {
        Serial.println("MAX30102/5 Diinisialisasi."); // Pesan jika sensor berhasil diinisialisasi
        byte ledBrightness = 0x7F; // Atur kecerahan LED (0x00-0xFF), 0x7F = 127 (setengah dari maksimum)
        byte sampleAverage = 4; // Jumlah sampel rata-rata (1, 2, 4, 8, 16, 32)
        byte ledMode = 2; // Mode LED (1: Red only, 2: Red & IR, 3: Red, IR & Green).
        int sampleRate = 200; // Sample rate (sampel per detik: 50, 100, 200, 400, 800, 1000, 1600, 3200)
        int pulseWidth = 411; // Lebar pulsa LED (69, 118, 215, 411 us). Pengaruh ke resolusi ADC.
        int adcRange = 16384; // Rentang ADC (2048, 4096, 8192, 16384 nA). Pengaruh ke sensitivitas.
        particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Terapkan konfigurasi ke sensor
    }
}

void calculateSpO2() {
    particleSensor.check(); // Cek status sensor dan baca data dari FIFO sensor
    uint32_t irValueCurrentLoop = 0; // Variabel untuk menyimpan nilai IR terakhir yang valid dalam loop ini

    while (particleSensor.available()) { // Jika ada data sampel baru di FIFO sensor
        irValueCurrentLoop = particleSensor.getFIFOIR(); // Dapatkan nilai mentah dari LED Infra Merah (IR)
        uint32_t redValue = particleSensor.getFIFORed(); // Dapatkan nilai mentah dari LED Merah (Red)

        avered = avered * 0.95 + (double)redValue * 0.05; // Filter rata-rata bergerak untuk nilai Red
        aveir = aveir * 0.95 + (double)irValueCurrentLoop * 0.05; // Filter rata-rata bergerak untuk nilai IR
        sumredrms += (redValue - avered) * (redValue - avered); // Akumulasi kuadrat selisih Red dari rata-ratanya
        sumirrms += (irValueCurrentLoop - aveir) * (irValueCurrentLoop - aveir); // Akumulasi kuadrat selisih IR dari rata-ratanya

        sampleCount++; // Tambah jumlah sampel yang telah diproses
        if (sampleCount % SAMPLES_FOR_SPO2_CALC == 0) { // Hitung SpO2 setiap SAMPLES_FOR_SPO2_CALC sampel
            double R = (sqrt(sumredrms / SAMPLES_FOR_SPO2_CALC) / avered) / (sqrt(sumirrms / SAMPLES_FOR_SPO2_CALC) / aveir); // Rumus SpO2 berdasarkan nilai R.
            double spo2_calculated = (3.1626 - R) / 0.0281; // kalibrasi
            
            ESpO2 = SPO2_FILTER_FACTOR * ESpO2 + (1.0 - SPO2_FILTER_FACTOR) * spo2_calculated; // Terapkan filter low-pass sederhana ke nilai SpO2 untuk memperhalus hasil
            if (irValueCurrentLoop < FINGER_DETECT_THRESHOLD_IR) {
                ESpO2 = 0; // Set SpO2 ke 0 jika jari tidak terdeteksi
            } else { 
                if (ESpO2 > 100.0) ESpO2 = 100.0; // Batasi atas SpO2 ke 100%
                if (ESpO2 < 80.0 && ESpO2 != 0.0) ESpO2 = 80.0; // Batasi bawah SpO2 ke 80% (kecuali jika sudah 0)
            }
            sumredrms = 0.0; // Reset sum square RMS Red
            sumirrms = 0.0; // Reset sum square RMS IR
        }
        if (sampleCount >= SAMPLES_FOR_SPO2_CALC) sampleCount = 0; // Reset counter sampel jika sudah mencapai batas
        particleSensor.nextSample(); // Maju ke sampel berikutnya di FIFO sensor
    }
    
    if (irValueCurrentLoop == 0) { 
        irValueCurrentLoop = particleSensor.getFIFOIR(); // Ambil nilai IR terakhir jika belum ada yang diproses di loop ini
    }

    if (irValueCurrentLoop < FINGER_DETECT_THRESHOLD_IR && ESpO2 != 0.0) {
      ESpO2 = 0; // Pastikan SpO2 adalah 0 jika IR di bawah threshold dan ESpO2 bukan 0
    }

    bool fingerJustPlaced = (ESpO2 != 0.0 && prevESpO2ForStabilization == 0.0); // Deteksi jika jari baru diletakkan (SpO2 berubah dari 0 ke non-0)
    bool fingerJustRemoved = (ESpO2 == 0.0 && prevESpO2ForStabilization != 0.0); // Deteksi jika jari baru dicabut (SpO2 berubah dari non-0 ke 0)
    if (fingerJustPlaced) {
        if (!isSpo2Stabilizing) { 
            Serial.println("SpO2: Jari terdeteksi, periode stabilisasi dimulai (15 detik)."); // Pesan saat stabilisasi dimulai
            spo2StabilizationStartTime = millis(); // Catat waktu mulai stabilisasi
        }
        isSpo2Stabilizing = true; // Set flag bahwa sedang stabilisasi
    } else if (fingerJustRemoved) {
        if (isSpo2Stabilizing) { 
            Serial.println("SpO2: Jari dicabut/bacaan hilang, periode stabilisasi direset."); // Pesan saat stabilisasi direset karena jari dicabut
        }
        isSpo2Stabilizing = false; // Reset flag stabilisasi
    }
    prevESpO2ForStabilization = ESpO2; // Simpan nilai ESpO2 saat ini untuk perbandingan di iterasi berikutnya
}

void manageActuators() {
    static unsigned long lastBuzzerChangeTime = 0; // Waktu terakhir buzzer intermittent diubah state-nya (ON/OFF)
    static bool buzzerIntermittentPhysicalStateOn = false; // Status fisik buzzer intermittent (apakah sedang ON atau OFF)
    static byte currentBuzzerMode = 0; // Mode buzzer saat ini (0:OFF, 1:Intermittent, 2:Continuous)

    static bool motorIsRunning = false; // Status apakah motor sedang dalam siklus 1 detik ON
    static unsigned long motorRunStartTime = 0; // Waktu motor mulai dinyalakan
    static bool prevSnoreStatusWasOneForMotor = false; // Status mendengkur sebelumnya (untuk deteksi perubahan ke 1)

    unsigned long currentTime = millis(); // Waktu saat ini
    bool activelyStabilizingThisCycle = false; // Flag lokal, apakah siklus ini masih dalam periode stabilisasi aktif

    if (isSpo2Stabilizing) {
        if (currentTime - spo2StabilizationStartTime < SPO2_STABILIZATION_PERIOD) {
            activelyStabilizingThisCycle = true; // Masih dalam periode 15 detik stabilisasi
        } else {
            Serial.println("Actuators: Periode stabilisasi SpO2 selesai."); // Pesan ini akan tercetak sekali setelah periode stabilisasi selesai
            isSpo2Stabilizing = false; // Nonaktifkan flag stabilisasi global
            activelyStabilizingThisCycle = false; // Tidak aktif stabilisasi di siklus ini juga
        }
    }

    byte newTargetBuzzerMode = 0; // Mode buzzer target default: OFF

    if (activelyStabilizingThisCycle) {
        // Serial.println("Actuators: SpO2 sedang stabilisasi, buzzer ditahan."); // Komentar debug: buzzer ditahan selama stabilisasi
    } else {
        if (ESpO2 > 0 && ESpO2 < 88.0) { // SpO2 di bawah 88% (dan valid > 0)
            newTargetBuzzerMode = 2; // Buzzer nyala terus menerus (Mode Continuous)
        } else if (ESpO2 >= 88.0 && ESpO2 < 95.0) { // SpO2 antara 88% dan 94.x%
            newTargetBuzzerMode = 1; // Buzzer nyala putus-putus (Mode Intermittent)
        }
    }

    if (newTargetBuzzerMode != currentBuzzerMode) {
        currentBuzzerMode = newTargetBuzzerMode; // Update mode saat ini dengan mode target baru
        if (currentBuzzerMode == 2) { // Transisi ke mode Continuous
            digitalWrite(BUZZER_PIN, HIGH); // Nyalakan buzzer secara kontinu
        } else if (currentBuzzerMode == 1) { // Transisi ke mode Intermittent
            buzzerIntermittentPhysicalStateOn = true; // Mulai dengan buzzer ON untuk mode intermittent
            digitalWrite(BUZZER_PIN, HIGH); // Nyalakan buzzer (awal dari siklus intermittent)
            lastBuzzerChangeTime = currentTime; // Catat waktu untuk interval berikutnya
        } else { // Transisi ke mode OFF (currentBuzzerMode == 0)
            digitalWrite(BUZZER_PIN, LOW); // Matikan buzzer
        }
    } else { 
        if (currentBuzzerMode == 1) { // Jika sedang dalam mode Intermittent
            if (currentTime - lastBuzzerChangeTime >= 500) { // Cek apakah interval 500ms telah berlalu
                lastBuzzerChangeTime = currentTime; // Reset timer interval
                buzzerIntermittentPhysicalStateOn = !buzzerIntermittentPhysicalStateOn; // Toggle state buzzer (ON ke OFF, atau OFF ke ON)
                digitalWrite(BUZZER_PIN, buzzerIntermittentPhysicalStateOn ? HIGH : LOW); // Set pin buzzer sesuai state baru
            }
        }
    }

    if (newDataFromNrf) { // Hanya cek jika ada data baru dari nRF
        if (lastSnoreStatusFromNrf == 1 && !prevSnoreStatusWasOneForMotor) {
            if (!motorIsRunning) { // Hanya nyalakan jika motor tidak sedang dalam siklus 0,5 detiknya
                motorIsRunning = true; // Set flag bahwa motor sedang berjalan
                motorRunStartTime = currentTime; // Catat waktu motor mulai dinyalakan
                digitalWrite(MOTOR_PIN, HIGH); // Nyalakan motor getar
            }
        }
    }
    if (newDataFromNrf) {
        prevSnoreStatusWasOneForMotor = (lastSnoreStatusFromNrf == 1); // Simpan status mendengkur saat ini untuk perbandingan di iterasi berikutnya
    }

    if (motorIsRunning) {
        if (currentTime - motorRunStartTime >= 500) { // Cek apakah motor sudah menyala selama 500 ms
            digitalWrite(MOTOR_PIN, LOW); // Matikan motor getar
            motorIsRunning = false; // Set flag bahwa motor tidak lagi berjalan
        }
    }
}

bool connectToNrfServer() {
    Serial.print("Mencoba koneksi ke nRF: "); // Pesan bahwa sedang mencoba koneksi
    Serial.println(myNrfDevice->getAddress().toString().c_str()); // Cetak alamat MAC perangkat nRF yang akan dikoneksi

    BLEClient *pClientNrf = BLEDevice::createClient(); // Buat objek BLE client baru
    Serial.println(" - Klien BLE dibuat");
    pClientNrf->setClientCallbacks(new MyNrfClientCallbacks()); // Set callback untuk event koneksi/diskoneksi client
    if (!pClientNrf->connect(myNrfDevice)) { // Lakukan koneksi ke perangkat nRF yang telah ditemukan
        Serial.println(" - Gagal terhubung ke server nRF"); // Pesan jika koneksi gagal
        return false; // Kembalikan false jika gagal terhubung
    }
    Serial.println(" - Terhubung ke server nRF"); // Pesan jika berhasil terhubung
    BLERemoteService *pRemoteServiceNrf = pClientNrf->getService(serviceUUID); // Dapatkan service BLE berdasarkan UUID yang ditentukan
    if (pRemoteServiceNrf == nullptr) {
        Serial.print("Gagal menemukan service UUID nRF: "); 
        Serial.println(serviceUUID.toString().c_str());
        pClientNrf->disconnect(); 
        return false; // Kembalikan false jika service tidak ditemukan
    }
    Serial.println(" - Service nRF ditemukan"); // Pesan jika service berhasil ditemukan
    pRemoteCharacteristicNrf = pRemoteServiceNrf->getCharacteristic(charUUID_NRF_READ); // Dapatkan karakteristik untuk membaca data dari nRF
    if (pRemoteCharacteristicNrf == nullptr) {
        Serial.print("Gagal menemukan characteristic UUID nRF: "); 
        Serial.println(charUUID_NRF_READ.toString().c_str());
        pClientNrf->disconnect(); 
        return false; // Kembalikan false jika karakteristik tidak ditemukan
    }
    Serial.println(" - Characteristic nRF ditemukan"); // Pesan jika karakteristik berhasil ditemukan
    if (pRemoteCharacteristicNrf->canNotify()) { // Cek apakah karakteristik mendukung notifikasi
        pRemoteCharacteristicNrf->registerForNotify(nrfNotifyCallback); // Daftarkan callback untuk menerima notifikasi dari karakteristik ini
        Serial.println(" - Berhasil mendaftar untuk notifikasi dari nRF"); 
    } else {
        Serial.println(" - Characteristic nRF tidak bisa memberi notifikasi"); // Pesan jika karakteristik tidak mendukung notifikasi
        pClientNrf->disconnect();
        return false; // Kembalikan false jika tidak bisa mendaftar notifikasi
    }

    bleClient_nrfConnected = true; // Set status terkoneksi ke nRF menjadi true
    oled_nrfConnected = true; // Update status untuk OLED bahwa nRF terhubung
    return true; // Kembalikan true jika semua proses koneksi dan pendaftaran notifikasi berhasil
}

void setupBleServerForPhone() {
    Serial.println("Setup BLE Server untuk Smartphone...");
    pServer = BLEDevice::createServer(); // Buat objek server BLE
    pServer->setCallbacks(new MyPhoneServerCallbacks()); // Set callback untuk event koneksi/diskoneksi server dari smartphone

    BLEService *pServicePhone = pServer->createService(serviceUUID); // Buat service BLE baru dengan UUID yang sama
    pCharacteristicPhone = pServicePhone->createCharacteristic(
        charUUID_PHONE_WRITE, // UUID untuk karakteristik yang akan digunakan smartphone
        BLECharacteristic::PROPERTY_READ   | 
        BLECharacteristic::PROPERTY_NOTIFY | 
        BLECharacteristic::PROPERTY_WRITE_NR  // Karakteristik ini bisa ditulis oleh client tanpa response (opsional)
    );
    pServicePhone->start(); // Mulai service BLE yang telah dibuat

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising(); // Dapatkan objek advertising
    pAdvertising->addServiceUUID(serviceUUID); // Tambahkan service UUID ke paket advertising
    pAdvertising->setScanResponse(true); // Izinkan scan response (untuk mengirim data tambahan jika diminta)
    pAdvertising->setMinPreferred(0x06); // Pengaturan interval advertising minimum (n * 0.625 ms)
    pAdvertising->setMaxPreferred(0x12); // Pengaturan interval advertising maksimum (n * 0.625 ms)
    pServer->getAdvertising()->start(); // Mulai advertising agar smartphone dapat menemukan ESP32
    Serial.println("BLE Server untuk Smartphone mulai advertising"); // Pesan bahwa server BLE mulai advertising
}

void sendJsonToSmartphone() {
    int stabilizationStatus = (isSpo2Stabilizing ? 1 : 0); // Konversi status stabilisasi boolean ke integer (1 atau 0)
    String jsonData = "{\"status\":" + String(lastSnoreStatusFromNrf) +
                      ",\"timestamp\":" + String(lastTimestampFromNrf) +
                      ",\"spo2\":" + String(ESpO2, (ESpO2 == 100.0 || ESpO2 == 0.0 || ESpO2 == 80.0) ? 0 : 2) + 
                      ",\"stabilizing\":" + String(stabilizationStatus) + // Tambahkan status stabilisasi ke JSON
                      "}";
    pCharacteristicPhone->setValue(jsonData.c_str()); // Set nilai karakteristik dengan string JSON
    pCharacteristicPhone->notify(); // Kirim notifikasi ke Smartphone yang terhubung & subscribe ke karakteristik ini

    Serial.print("Terkirim ke smartphone: "); // Cetak pesan bahwa data dikirim
    Serial.println(jsonData); // Cetak data JSON yang dikirim
}