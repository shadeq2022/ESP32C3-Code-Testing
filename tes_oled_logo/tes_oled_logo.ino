#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Tentukan ukuran layar OLED Anda
#define SCREEN_WIDTH 128 // Lebar layar OLED dalam piksel
#define SCREEN_HEIGHT 64  // Tinggi layar OLED dalam piksel (umumnya 64 atau 32)

// Deklarasi untuk layar SSD1306 yang terhubung via I2C (SDA, SCL)
// Jika layar Anda tidak memiliki pin RESET, gunakan -1
#define OLED_RESET -1 // Pin RESET OLED (atau -1 jika tidak digunakan)
// Alamat I2C default untuk SSD1306 adalah 0x3C. Beberapa mungkin 0x3D.
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Dimensi logo (kedua logo berukuran sama 17x17)
const int LOGO_WIDTH = 17;
const int LOGO_HEIGHT = 17;

// Variabel untuk memilih logo mana yang akan ditampilkan:
// Setel ke 1 untuk logo PHONE
// Setel ke 2 untuk logo NO
int currentScreen = 1; // Nilai default saat pertama kali menyala

// Data bitmap logo pertama ('phone1', 17x17px)
const unsigned char phone1[] PROGMEM = {
0x00, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08,
0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x08, 0x08,
0x00, 0x08, 0x08, 0x00, 0x0f, 0xf8, 0x00, 0x0e, 0x38, 0x00, 0x0f, 0xf8, 0x00, 0x0f, 0xf8, 0x00,
0x00, 0x00, 0x00
};

// Data bitmap logo kedua ('no', 17x17px) - Sesuai yang Anda berikan
const unsigned char no_logo[] PROGMEM = {
0x00, 0x00, 0x80, 0x0f, 0xf9, 0x00, 0x08, 0x0a, 0x00, 0x08, 0x0c, 0x00, 0x08, 0x08, 0x00, 0x08,
0x18, 0x00, 0x08, 0x28, 0x00, 0x08, 0x48, 0x00, 0x08, 0x88, 0x00, 0x09, 0x08, 0x00, 0x0a, 0x08,
0x00, 0x0c, 0x08, 0x00, 0x0f, 0xf8, 0x00, 0x1e, 0x38, 0x00, 0x2f, 0xf8, 0x00, 0x4f, 0xf8, 0x00,
0x80, 0x00, 0x00
};

// --- Deklarasi Fungsi ---
void updateDisplay();


void setup() {
  Serial.begin(115200);

  // Untuk ESP32-C3, pin I2C default biasanya GPIO8 (SDA) dan GPIO9 (SCL).
  // Jika Anda menggunakan pin kustom, inisialisasi Wire di sini:
  // Wire.begin(SDA_PIN, SCL_PIN);
  // Jika tidak, Wire.begin() tanpa argumen akan menggunakan default.

  // Inisialisasi SSD1306 dengan alamat I2C
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Alokasi SSD1306 gagal"));
    for(;;); // Jangan lanjutkan, loop selamanya
  }

  Serial.println(F("SSD1306 diinisialisasi."));
  Serial.println(F("Ketik '1' di Serial Monitor untuk logo PHONE."));
  Serial.println(F("Ketik '2' di Serial Monitor untuk logo NO."));


  // Tampilkan logo awal (sesuai nilai default currentScreen)
  updateDisplay();
}

void loop() {
  // Cek apakah ada data yang masuk dari Serial Monitor
  if (Serial.available() > 0) {
    // Baca karakter yang diterima
    char incomingByte = Serial.read();

    // Proses input
    if (incomingByte == '1') {
      if (currentScreen != 1) { // Hanya update jika berbeda
        currentScreen = 1;
        Serial.println(F("Menerima '1'. Mengganti ke logo PHONE (layar 1)."));
        updateDisplay(); // Panggil fungsi untuk menggambar ulang
      }
    } else if (incomingByte == '2') {
      if (currentScreen != 2) { // Hanya update jika berbeda
        currentScreen = 2;
        Serial.println(F("Menerima '2'. Mengganti ke logo NO (layar 2)."));
        updateDisplay(); // Panggil fungsi untuk menggambar ulang
      }
    } else {
      // Abaikan karakter newline atau carriage return yang sering dikirim
      // saat menekan Enter di Serial Monitor
      if (incomingByte != '\n' && incomingByte != '\r') {
        Serial.print(F("Input tidak valid: "));
        Serial.println(incomingByte);
        Serial.println(F("Ketik '1' untuk logo PHONE atau '2' untuk logo NO."));
      }
    }
  }

  // Loop terus berjalan, menunggu input serial
}

// Fungsi untuk membersihkan layar dan menggambar logo berdasarkan currentScreen
void updateDisplay() {
  display.clearDisplay(); // Bersihkan buffer tampilan

  // Hitung posisi X untuk logo agar berada di sudut kanan atas
  int logoX = SCREEN_WIDTH - LOGO_WIDTH;
  int logoY = 0; // Posisi Y di bagian paling atas

  // Set pengaturan teks untuk pesan di layar (opsional)
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Gambar bitmap logo berdasarkan pilihan currentScreen
  // Parameter drawBitmap: x, y, bitmap_array, width, height, color
  if (currentScreen == 1) {
    display.drawBitmap(logoX, logoY, phone1, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
    // Tampilkan pesan di layar juga (opsional)
    display.setCursor(0, 0);
    display.println(F("Layar 1 (PHONE)"));
  } else if (currentScreen == 2) {
    display.drawBitmap(logoX, logoY, no_logo, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
    // Tampilkan pesan di layar juga (opsional)
    display.setCursor(0, 0);
    display.println(F("Layar 2 (NO)"));
  } else {
     // Ini seharusnya tidak terjadi jika input divalidasi dengan benar,
     // tapi baik untuk penanganan kesalahan
     display.setCursor(0, 0);
     display.println(F("LAYAR TDK VALID"));
     Serial.println(F("Kesalahan internal: Nomor layar tidak valid dalam updateDisplay."));
  }

  display.display(); // Tampilkan buffer ke layar OLED
}