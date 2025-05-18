void setup() {
  pinMode(10, OUTPUT);  // Buzzer di GPIO9
  pinMode(3, OUTPUT);  // Buzzer di GPIO9

}

void loop() {
  digitalWrite(3, HIGH); // Buzzer ON
  delay(100);            // Bip selama 100 ms
  digitalWrite(3, LOW);  // Buzzer OFF
  delay(1000);           // Tunggu 1 detik sebelum bip berikutnya
  digitalWrite(10, HIGH); // Buzzer ON
  delay(100);            // Bip se`lama 100 ms
  digitalWrite(10, LOW);  // Buzzer OFF
  delay(1000); 
}