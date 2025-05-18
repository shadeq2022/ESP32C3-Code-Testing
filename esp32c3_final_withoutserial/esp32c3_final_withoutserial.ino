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

        // Serial.print() hanya akan terlihat jika USB terhubung ke PC
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
    }

    void onDisconnect(BLEClient *pclient_nrf) {
        bleClient_nrfConnected = false;
        oled_nrfConnected = false; // Update status OLED
        Serial.println("Terputus dari server nRF");
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
    }

    void onDisconnect(BLEServer *pServer_phone) {
        bleServer_smartphoneConnected = false;
        oled_phoneBLEConnected = false; // Update status OLED
        Serial.println("Smartphone terputus");
        delay(500);
        pServer_phone->getAdvertising()->start();
        Serial.println("Advertising untuk smartphone dimulai ulang");
    }
};

//----------------------------------------------------------------//
//                          SETUP                                 //
//----------------------------------------------------------------//

void setup() {
    Serial.begin(115200); // [cite: 35]
    Serial.println("Memulai ESP32 Sensor Hub..."); 

    // Inisialisasi I2C
    Wire.begin(); 

    // Inisialisasi OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { 
        Serial.println(F("Alokasi SSD1306 gagal")); 
        for (;;); // Don't proceed if OLED allocation fails
    }
    Serial.println(F("SSD1306 Diinisialisasi."));
    display.clearDisplay();
    
    // Display "Initializing..." screen
    display.setTextSize(1); // Use a slightly larger text size for this message
    display.setTextColor(SSD1306_WHITE);
    const char* initMsg = "Initializing...";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(initMsg, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT - h) / 2);
    display.print(initMsg);
    display.display();
    delay(2000); // Display the "Initializing..." message for 2 seconds

    // Inisialisasi Sensor MAX30102
    setupMAX30102(); 

    // Inisialisasi BLE Device
    BLEDevice::init("ESP32_SensorHub"); 

    // Setup BLE Client (untuk nRF)
    Serial.println("Inisialisasi BLE Client untuk nRF..."); 
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedNrfDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(10, false); // Start scanning for 10 seconds (non-blocking for the main scan operation itself)
    Serial.println("BLE Scan untuk nRF dimulai...");

    // Setup BLE Server (untuk Smartphone)
    setupBleServerForPhone(); 

    // Tampilan awal OLED
    // currentOledScreen is already SCREEN_NRF_PROMPT by default 
    // The "Initializing..." screen was a one-shot display.
    // The loop() will now take over and show SCREEN_NRF_PROMPT if nRF is not connected.
    oled_nrfConnected = bleClient_nrfConnected; 
    oled_phoneBLEConnected = bleServer_smartphoneConnected; 
    
    // The first call to updateOledDisplay() will display the screen
    // based on currentOledScreen (which is SCREEN_NRF_PROMPT 
    // and the actual connection statuses.
    updateOledDisplay(); 
    lastOledUpdateTime = millis();

    Serial.println("Setup selesai. Memasuki loop utama..."); // 
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
        } else {
            Serial.println("Gagal terhubung ke nRF. Mencoba scan ulang...");
            BLEDevice::getScan()->start(10, false);
        }
        bleClient_doConnectToNrf = false;
    }

    if (oled_nrfConnected != bleClient_nrfConnected) {
        oled_nrfConnected = bleClient_nrfConnected;
    }
    if (oled_phoneBLEConnected != bleServer_smartphoneConnected) {
        oled_phoneBLEConnected = bleServer_smartphoneConnected;
    }

    // 2. Hitung SpO2 secara kontinu
    calculateSpO2();

    // 3. Logika Layar OLED
    if (!oled_nrfConnected) {
        if (currentOledScreen != SCREEN_NRF_PROMPT) {
            currentOledScreen = SCREEN_NRF_PROMPT;
            updateOledDisplay();
        } else {
             if(currentTime - lastOledUpdateTime > 500){
                updateOledDisplay();
                lastOledUpdateTime = currentTime;
            }
        }
    } else {
        if (currentOledScreen == SCREEN_NRF_PROMPT) {
            currentOledScreen = SCREEN_PHONE_INFO;
            updateOledDisplay();
            lastOledUpdateTime = currentTime;
        } else if (currentOledScreen == SCREEN_PHONE_INFO) {
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
        newDataFromNrf = false;
    }

    delay(50);
}

//----------------------------------------------------------------//
//                    FUNGSI IMPLEMENTASI                         //
//----------------------------------------------------------------//

void updateOledDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    if (oled_nrfConnected) {
        display.print(F("nrf=conect"));
    } else {
        display.print(F("nrf=disconect"));
    }

    int logoX = SCREEN_WIDTH - LOGO_WIDTH;
    int logoY = 0;
    if (oled_phoneBLEConnected) {
        display.drawBitmap(logoX, logoY, phone1_logo, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
    } else {
        display.drawBitmap(logoX, logoY, no_phone_logo, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
    }

    int textHeightApprox = 8;
    int navbarY = max(LOGO_HEIGHT, textHeightApprox) + 2;
    display.drawLine(0, navbarY, SCREEN_WIDTH, navbarY, SSD1306_WHITE);

    int contentY = navbarY + 5;

    if (currentOledScreen == SCREEN_NRF_PROMPT) {
        display.setTextSize(1);
        int textWidthNrf1 = 13 * 6;
        int textWidthNrf2 = 18 * 6;
        display.setCursor((SCREEN_WIDTH - textWidthNrf1) / 2, contentY + 10);
        display.println(F("Hidupkan nrf"));
        display.setCursor((SCREEN_WIDTH - textWidthNrf2) / 2, contentY + 20);
        display.println(F("terlebih dahulu!!"));
    } else if (currentOledScreen == SCREEN_PHONE_INFO) {
        display.setCursor(0, contentY);
        display.print(F("SpO2: "));
        if (ESpO2 == 0 && particleSensor.getFIFOIR() < FINGER_DETECT_THRESHOLD_IR) {
            display.print(F("--%"));
        } else {
            display.print(ESpO2, (ESpO2 == 100.0 || ESpO2 == 0.0 || ESpO2 == 80.0) ? 0 : 1);
            display.print(F("%"));
        }
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
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30102/5 tidak ditemukan. Periksa kabel/daya.");
    } else {
        Serial.println("MAX30102/5 Diinisialisasi.");
        byte ledBrightness = 0x7F;
        byte sampleAverage = 4;
        byte ledMode = 2;
        int sampleRate = 200;
        int pulseWidth = 411;
        int adcRange = 16384;
        particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    }
}

void calculateSpO2() {
    particleSensor.check();

    while (particleSensor.available()) {
        uint32_t irValue = particleSensor.getFIFOIR();
        uint32_t redValue = particleSensor.getFIFORed();

        avered = avered * 0.95 + (double)redValue * 0.05;
        aveir = aveir * 0.95 + (double)irValue * 0.05;

        sumredrms += (redValue - avered) * (redValue - avered);
        sumirrms += (irValue - aveir) * (irValue - aveir);

        sampleCount++;
        if (sampleCount % SAMPLES_FOR_SPO2_CALC == 0) {
            double R = (sqrt(sumredrms / SAMPLES_FOR_SPO2_CALC) / avered) / (sqrt(sumirrms / SAMPLES_FOR_SPO2_CALC) / aveir);
            double spo2_calculated = -45.060 * R * R + 30.354 * R + 94.845;
            
            ESpO2 = SPO2_FILTER_FACTOR * ESpO2 + (1.0 - SPO2_FILTER_FACTOR) * spo2_calculated;

            if (irValue < FINGER_DETECT_THRESHOLD_IR) {
                ESpO2 = 0;
            } else {
                if (ESpO2 > 100.0) ESpO2 = 100.0;
                if (ESpO2 < 80.0 && ESpO2 != 0) ESpO2 = 80.0;
            }
            sumredrms = 0.0;
            sumirrms = 0.0;
        }
        if (sampleCount >= SAMPLES_FOR_SPO2_CALC) sampleCount = 0;
        particleSensor.nextSample();
    }
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
        pClientNrf->disconnect();
        return false;
    }

    bleClient_nrfConnected = true;
    oled_nrfConnected = true;
    return true;
}

void setupBleServerForPhone() {
    Serial.println("Setup BLE Server untuk Smartphone...");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyPhoneServerCallbacks());

    BLEService *pServicePhone = pServer->createService(serviceUUID);

    pCharacteristicPhone = pServicePhone->createCharacteristic(
        charUUID_PHONE_WRITE,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_WRITE_NR
    );

    pServicePhone->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(serviceUUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    pServer->getAdvertising()->start();
    Serial.println("BLE Server untuk Smartphone mulai advertising");
}

void sendJsonToSmartphone() {
    String jsonData = "{\"status\":" + String(lastSnoreStatusFromNrf) +
                      ",\"timestamp\":" + String(lastTimestampFromNrf) +
                      ",\"spo2\":" + String(ESpO2, (ESpO2 == 100.0 || ESpO2 == 0.0 || ESpO2 == 80.0) ? 0 : 2) +
                      "}";

    pCharacteristicPhone->setValue(jsonData.c_str());
    pCharacteristicPhone->notify();

    Serial.print("Terkirim ke smartphone: "); // Hanya terlihat jika USB terhubung
    Serial.println(jsonData);
}