/*************************************************************************
 *  WEARABLE MONITORING - ESP32-C6 Zero + Blynk
 *  Gabungan modul:
 *    - DHT22
 *    - MAX30102
 *    - MLX90614
 *    - MPU6050
 *
 *  PERBAIKAN VERSI FIX:
 *  1. V2 = KELEMBAPAN DHT22 TERKALIBRASI & RATA-RATA
 *     (disamakan dengan dht22_blynk.ino)
 *  2. V1 = SUHU MLX90614 TERKALIBRASI
 *     (disamakan dengan mlx90614.ino)
 *  3. V10 dan V11 TIDAK DIGUNAKAN.
 *  4. DHT22 tetap menggunakan offset -32.0 dan moving average 10 sampel.
 *  5. MLX90614 tetap menggunakan SLOPE 1.0 dan OFFSET 2.1.
 *  6. MLX90614 diinisialisasi segera setelah I2C agar kondisi pembacaan
 *     sedekat mungkin dengan program MLX standalone.
 *  7. I2C dipaksa 100 kHz (standard) agar konsisten.
 *  8. Notifikasi Blynk + buzzer aktif bersamaan saat alarm baru terdeteksi.
 *  9. MODUL MAX30102/BPM DIGANTI dengan versi max30102.ino:
 *     - BPM dirata-rata 10 sampel (float) dengan buang outlier (median +-20%)
 *     - BPM ditampilkan dibatasi rentang 65 - 95
 *     - Deteksi jari (IR >= 50000), data di-reset saat jari dilepas
 *     - Setting sensor: LED 0x3C, avg 4, 100 Hz, pulse 411, ADC 4096
 *
 *  MAPPING BLYNK:
 *  V0 = BPM MAX30102
 *  V1 = Suhu MLX90614 terkalibrasi
 *  V2 = Kelembapan DHT22 terkalibrasi (rata-rata)
 *  V3 = MIN BPM
 *  V4 = MAX BPM
 *  V5 = MIN Suhu MLX90614
 *  V6 = MAX Suhu MLX90614
 *  V7 = MIN Kelembapan DHT22
 *  V8 = MAX Kelembapan DHT22
 *  V9 = Status MPU6050
 *************************************************************************/

// ============================================================
// BLYNK
// ============================================================
//#define BLYNK_TEMPLATE_ID "TMPL6B8SS3MCe"
//#define BLYNK_TEMPLATE_NAME "smart bracelet"
//#define BLYNK_AUTH_TOKEN "Yt9DktEGBah1V4LWpzqx5y4lmjTnyZPx"

#define BLYNK_TEMPLATE_ID    "TMPL6Q18IknDh"
#define BLYNK_TEMPLATE_NAME  "Wearable Lansia"
#define BLYNK_AUTH_TOKEN     "JnT_PvEGZK4rOD1bYcf3_QQ-VWVuENt7"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <math.h>
#include <DHT.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MLX90614.h>

// ============================================================
// WIFI
// ============================================================
char ssid[] = "Virtual nii";
char pass[] = "12345678";

// ============================================================
// PIN
// ============================================================
#define DHTPIN      14
#define DHTTYPE     DHT22

#define SDA_PIN     4
#define SCL_PIN     5

#define BUZZER_PIN  3
#define MPU_ADDR    0x68

// Jika buzzer active-LOW, ubah false menjadi true.
#define BUZZER_ACTIVE_LOW false

const unsigned long DURASI_BUZZER = 5000;

// ============================================================
// KALIBRASI DHT22
// SAMA DENGAN dht22_blynk.ino
// ============================================================
float OFFSET_KALIBRASI_RH = -32.0;

#define JUMLAH_SAMPEL_RH 10

float bufferRH[JUMLAH_SAMPEL_RH];
int indeksBufferRH = 0;
int jumlahDataRHMasuk = 0;

// ============================================================
// KALIBRASI MLX90614
// SAMA DENGAN mlx90614.ino
// ============================================================
#define CALIBRATION_SLOPE   1.0
#define CALIBRATION_OFFSET  2.1

// Rentang suhu normal
#define SUHU_NORMAL_MIN 36.1
#define SUHU_NORMAL_MAX 37.2

// ============================================================
// BLYNK EVENT CODES
// Buat event dengan kode yang SAMA di Blynk Console.
// ============================================================
#define EVENT_BPM_ALERT          "bpm_alert"
#define EVENT_TEMPERATURE_ALERT  "temperature_alert"
#define EVENT_HUMIDITY_ALERT     "humidity_alert"
#define EVENT_FALL_DETECTED      "fall_detected"

// ============================================================
// OBJEK SENSOR
// ============================================================
DHT dht(DHTPIN, DHTTYPE);
MAX30105 particleSensor;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
BlynkTimer timer;

// ============================================================
// SETTING MIN/MAX DARI BLYNK
// ============================================================
float bpmMin   = 60.0;
float bpmMax   = 100.0;

float mlxMin   = 36.00;
float mlxMax   = 37.20;

float humidMin = 40.0;
float humidMax = 60.0;

// ============================================================
// VARIABEL DHT22
// ============================================================
float suhuDHT       = NAN;
float kelembapanDHT = NAN;

// ============================================================
// VARIABEL MAX30102 (dari modul max30102.ino)
// ============================================================

// ---------- PENGATURAN KALIBRASI BPM ----------
const int  BPM_MIN     = 65;     // batas bawah tampilan
const int  BPM_MAX     = 95;     // batas atas tampilan
const bool PAKAI_BATAS = true;   // true = hasil dipaksa masuk 65-95
const int  RAW_MIN     = 40;     // beat mentah di bawah ini dianggap noise
const int  RAW_MAX     = 180;    // beat mentah di atas ini dianggap noise
const long IR_JARI     = 50000;  // ambang jari terdeteksi
const int  MIN_SAMPEL  = 3;      // minimal data sebelum BPM ditampilkan
// ----------------------------------------------

const byte RATE_SIZE = 10;       // jumlah sampel rata-rata
float rates[RATE_SIZE];
byte  rateSpot  = 0;
byte  rateCount = 0;
long  lastBeat  = 0;

float beatsPerMinute = 0;
int   beatAvg        = 0;
long  irValue        = 0;
bool  jariAda        = false;

// ============================================================
// VARIABEL MLX90614
// ============================================================
float objectTemp = NAN;
float ambientTemp = NAN;

// ============================================================
// VARIABEL MPU6050
// ============================================================
int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

float ax, ay, az;
float totalAcc;

const float TH_NORMAL_MIN    = 0.90;
const float TH_NORMAL_MAX    = 1.10;
const float TH_JATUH         = 1.80;

String lastStatusMPU = "";
String statusMPUsaatIni = "Normal";

bool sedangKonfirmasiJatuh = false;
unsigned long waktuTerdeteksiJatuh = 0;

const unsigned long DURASI_KONFIRMASI = 100;

bool kondisiJatuhTerkonfirmasi = false;

// ============================================================
// ALARM
// ============================================================
bool alarmHumid = false;
bool alarmBPM   = false;
bool alarmMLX   = false;
bool alarmJatuh = false;

// Status sebelumnya agar notifikasi hanya terkirim saat
// kondisi berubah dari normal menjadi abnormal.
bool alarmHumidSebelumnya = false;
bool alarmBPMSebelumnya   = false;
bool alarmMLXSebelumnya   = false;
bool alarmJatuhSebelumnya = false;

// ============================================================
// BUZZER
// ============================================================
bool buzzerSedangAktif = false;
unsigned long waktuBuzzerMulai = 0;

// Status koneksi Blynk dan antrian notifikasi.
// Event alarm disimpan sementara jika koneksi sedang putus,
// lalu dikirim setelah Blynk tersambung kembali.
bool blynkPernahTerhubung = false;
bool notifikasiTertunda = false;
String eventTertunda = "";
String pesanTertunda = "";

// ============================================================
// FUNGSI BUZZER
// ============================================================
void setBuzzer(bool aktif) {
#if BUZZER_ACTIVE_LOW
  digitalWrite(BUZZER_PIN, aktif ? LOW : HIGH);
#else
  digitalWrite(BUZZER_PIN, aktif ? HIGH : LOW);
#endif
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("SMART BRACELET MULAI");

  Serial.println();
  Serial.println("==============================================");
  Serial.println(" WEARABLE MONITORING - ESP32-C6");
  Serial.println(" DHT22 + MAX30102 + MLX90614 + MPU6050");
  Serial.println("==============================================");

  pinMode(BUZZER_PIN, OUTPUT);
  setBuzzer(false);

  // ----------------------------------------------------------
  // DHT22
  // ----------------------------------------------------------
  Serial.println();
  Serial.println("[1] Inisialisasi DHT22...");
  dht.begin();

  for (int i = 0; i < JUMLAH_SAMPEL_RH; i++) {
    bufferRH[i] = 0;
  }

  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------
  Serial.println("[2] Inisialisasi I2C...");
  Wire.begin(SDA_PIN, SCL_PIN);

  // Sama seperti program MLX standalone:
  // I2C standard speed = 100 kHz.
  Wire.setClock(100000);

  // ----------------------------------------------------------
  // MLX90614
  // DIINISIALISASI SEBELUM MAX30102
  // agar urutan inisialisasi mendekati kode standalone.
  // ----------------------------------------------------------
  Serial.println("[3] Inisialisasi MLX90614...");

  if (!mlx.begin()) {
    Serial.println("ERROR: MLX90614 tidak ditemukan!");
    Serial.println("Periksa SDA=GPIO4, SCL=GPIO5, VCC dan GND.");
  } else {
    Serial.println("OK: MLX90614 terdeteksi.");
  }

  // Pastikan kembali I2C standard 100 kHz.
  Wire.setClock(100000);

  // ----------------------------------------------------------
  // MAX30102
  // ----------------------------------------------------------
  Serial.println("[4] Inisialisasi MAX30102...");

  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("WARNING: MAX30102 tidak ditemukan!");
  } else {
    // Pengaturan dari modul max30102.ino (dikalibrasi agar tidak saturasi)
    byte ledBrightness = 0x3C; // 60
    byte sampleAverage = 4;
    byte ledMode       = 2;    // Red + IR
    int  sampleRate    = 100;  // 100 Hz cocok untuk deteksi detak
    int  pulseWidth    = 411;
    int  adcRange      = 4096;

    particleSensor.setup(
      ledBrightness,
      sampleAverage,
      ledMode,
      sampleRate,
      pulseWidth,
      adcRange
    );

    particleSensor.setPulseAmplitudeGreen(0);

    Serial.println("OK: MAX30102 siap.");
  }

  // Setelah konfigurasi MAX30102, pastikan bus kembali 100 kHz.
  Wire.setClock(100000);

  // ----------------------------------------------------------
  // MPU6050
  // ----------------------------------------------------------
  Serial.println("[5] Inisialisasi MPU6050...");

  Wire.beginTransmission(MPU_ADDR);

  if (Wire.endTransmission() != 0) {
    Serial.println("WARNING: MPU6050 tidak ditemukan!");
  } else {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    Serial.println("OK: MPU6050 terdeteksi & aktif.");
  }

  // ----------------------------------------------------------
  // WIFI + BLYNK
  // ----------------------------------------------------------
  Serial.println("[6] Menghubungkan WiFi/Blynk...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  if (Blynk.connected()) {
    blynkPernahTerhubung = true;
    Serial.println("OK: Blynk terhubung.");
  } else {
    Serial.println("WARNING: Blynk belum terhubung.");
  }

  // ----------------------------------------------------------
  // TIMER
  // ----------------------------------------------------------
  timer.setInterval(2000L, bacaKirimDHT);
  timer.setInterval(1000L, bacaKirimMLX);
  timer.setInterval(200L, bacaKirimMPU);
  timer.setInterval(1000L, kirimBPM);
  timer.setInterval(100L, evaluasiBuzzer);
  timer.setInterval(1000L, cekKoneksiBlynk);

  Serial.println();
  Serial.println("Sistem siap.");
  Serial.println("----------------------------------------------");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  Blynk.run();
  timer.run();

  // MAX30102 perlu dibaca sesering mungkin.
  bacaMAX30102();
}

// ============================================================
// BLYNK CONNECTED
// V10 & V11 TIDAK DISINKRONKAN KARENA TIDAK DIGUNAKAN.
// ============================================================
BLYNK_CONNECTED() {
  Blynk.syncVirtual(V3, V4, V5, V6, V7, V8);
}

// ============================================================
// DHT22
//
// PENTING:
// V2 SEKARANG = KELEMBAPAN DHT22
// sama seperti dht22_blynk.ino.
//
// V10 dan V11 tidak digunakan.
// ============================================================
void bacaKirimDHT() {
  float suhu = dht.readTemperature();
  float kMentah = dht.readHumidity();

  if (isnan(suhu) || isnan(kMentah)) {
    Serial.println("DHT22 ERROR: gagal membaca sensor.");
    return;
  }

  suhuDHT = suhu;

  // ----------------------------------------------------------
  // Kalibrasi kelembapan
  // Rumus sama:
  // kelembapan terkalibrasi = RH mentah + (-32.0)
  // ----------------------------------------------------------
  float kKalibrasi = kMentah + OFFSET_KALIBRASI_RH;

  if (kKalibrasi < 0) {
    kKalibrasi = 0;
  }

  if (kKalibrasi > 100) {
    kKalibrasi = 100;
  }

  // ----------------------------------------------------------
  // Moving average 10 sampel
  // ----------------------------------------------------------
  bufferRH[indeksBufferRH] = kKalibrasi;

  indeksBufferRH =
    (indeksBufferRH + 1) % JUMLAH_SAMPEL_RH;

  if (jumlahDataRHMasuk < JUMLAH_SAMPEL_RH) {
    jumlahDataRHMasuk++;
  }

  float totalRH = 0;

  for (int i = 0; i < jumlahDataRHMasuk; i++) {
    totalRH += bufferRH[i];
  }

  float rataRataRH =
    totalRH / jumlahDataRHMasuk;

  kelembapanDHT = rataRataRH;

  // ----------------------------------------------------------
  // STATUS
  // ----------------------------------------------------------
  String statusRH;

  if (rataRataRH < humidMin) {
    statusRH = "Rendah";
  }
  else if (rataRataRH > humidMax) {
    statusRH = "Tinggi";
  }
  else {
    statusRH = "Normal";
  }

  // ----------------------------------------------------------
  // SERIAL DEBUG
  // ----------------------------------------------------------
  Serial.println();
  Serial.println("========== DHT22 ==========");
  Serial.print("Suhu DHT22          : ");
  Serial.print(suhuDHT, 1);
  Serial.println(" C");

  Serial.print("RH Mentah            : ");
  Serial.print(kMentah, 1);
  Serial.println(" %");

  Serial.print("RH Terkalibrasi      : ");
  Serial.print(kKalibrasi, 1);
  Serial.println(" %");

  Serial.print("RH Rata-rata 10 data : ");
  Serial.print(kelembapanDHT, 1);
  Serial.println(" %");

  Serial.print("Status RH            : ");
  Serial.println(statusRH);
  Serial.println("============================");

  // ----------------------------------------------------------
  // BLYNK
  //
  // V2 = KELEMBAPAN DHT22
  // ----------------------------------------------------------
  Blynk.virtualWrite(V2, kelembapanDHT);
}

// ============================================================
// MAX30102 - DARI MODUL max30102.ino
// ============================================================

// ---------- Rata-rata dengan membuang outlier ----------
int hitungRataRata() {
  if (rateCount < MIN_SAMPEL) return 0;

  // 1) cari median
  float tmp[RATE_SIZE];
  for (byte i = 0; i < rateCount; i++) tmp[i] = rates[i];
  for (byte i = 0; i < rateCount - 1; i++)
    for (byte j = i + 1; j < rateCount; j++)
      if (tmp[j] < tmp[i]) { float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
  float median = tmp[rateCount / 2];

  // 2) rata-rata hanya dari nilai yang dekat median (+-20%)
  float sum = 0;
  byte n = 0;
  for (byte i = 0; i < rateCount; i++) {
    if (fabs(rates[i] - median) <= median * 0.20) {
      sum += rates[i];
      n++;
    }
  }
  float avg = (n > 0) ? (sum / n) : median;

  // 3) batasi ke rentang 65-95
  if (PAKAI_BATAS) {
    avg = constrain(avg, BPM_MIN, BPM_MAX);
  }

  return (int)(avg + 0.5);
}

void resetDataBPM() {
  rateSpot = 0;
  rateCount = 0;
  beatAvg = 0;
  beatsPerMinute = 0;
  lastBeat = 0;
}

void bacaMAX30102() {
  irValue = particleSensor.getIR();
  jariAda = (irValue >= IR_JARI);

  if (!jariAda) {
    resetDataBPM();   // jari dilepas -> mulai ulang, V0 menjadi 0
  }
  else if (checkForBeat(irValue)) {
    long now   = millis();
    long delta = now - lastBeat;
    lastBeat   = now;

    if (delta > 0) {
      beatsPerMinute = 60000.0 / delta;

      if (beatsPerMinute > RAW_MIN && beatsPerMinute < RAW_MAX) {
        rates[rateSpot++] = beatsPerMinute;
        rateSpot %= RATE_SIZE;
        if (rateCount < RATE_SIZE) rateCount++;

        beatAvg = hitungRataRata();

        Serial.print("BPM mentah=");
        Serial.print(beatsPerMinute);
        Serial.print(" | BPM rata-rata=");
        Serial.println(beatAvg);
      }
    }
  }
}

// ============================================================
// KIRIM BPM
// ============================================================
void kirimBPM() {
  Serial.print("MAX30102 -> BPM Avg: ");
  Serial.println(beatAvg);

  Blynk.virtualWrite(V0, beatAvg);
}

// ============================================================
// MLX90614
//
// RUMUS HARUS SAMA DENGAN mlx90614.ino:
//
// suhuKalibrasi = (suhuMentah * 1.0) + 2.1
//
// V1 = suhu MLX terkalibrasi
//
// Tidak menggunakan averaging tambahan supaya hasil sedekat
// mungkin dengan kode mlx90614.ino standalone.
// ============================================================
void bacaKirimMLX() {
  // Pastikan I2C tetap standard 100 kHz.
  Wire.setClock(100000);

  float objectTempRaw = mlx.readObjectTempC();
  float ambient = mlx.readAmbientTempC();

  if (isnan(objectTempRaw) || isnan(ambient)) {
    Serial.println("MLX90614 ERROR: gagal membaca sensor.");
    return;
  }

  // ----------------------------------------------------------
  // Kalibrasi IDENTIK dengan mlx90614.ino
  // ----------------------------------------------------------
  float objectTempCal =
    (objectTempRaw * CALIBRATION_SLOPE)
    + CALIBRATION_OFFSET;

  objectTemp = objectTempCal;
  ambientTemp = ambient;

  // ----------------------------------------------------------
  // STATUS SUHU
  // ----------------------------------------------------------
  String statusSuhu;

  if (objectTempCal < SUHU_NORMAL_MIN) {
    statusSuhu = "Rendah";
  }
  else if (objectTempCal <= SUHU_NORMAL_MAX) {
    statusSuhu = "Normal";
  }
  else {
    statusSuhu = "Tinggi";
  }

  // ----------------------------------------------------------
  // SERIAL DEBUG
  // ----------------------------------------------------------
  Serial.println();
  Serial.println("========= MLX90614 =========");

  Serial.print("Suhu Object RAW     : ");
  Serial.print(objectTempRaw, 2);
  Serial.println(" C");

  Serial.print("Suhu Object CAL     : ");
  Serial.print(objectTempCal, 2);
  Serial.println(" C");

  Serial.print("Suhu Ambient        : ");
  Serial.print(ambientTemp, 2);
  Serial.println(" C");

  Serial.print("Status Suhu         : ");
  Serial.println(statusSuhu);

  Serial.println("=============================");

  // ----------------------------------------------------------
  // BLYNK
  //
  // V1 = suhu MLX90614 terkalibrasi
  // ----------------------------------------------------------
  Blynk.virtualWrite(V1, objectTemp);
}

// ============================================================
// MPU6050 - BACA DATA
// ============================================================
void bacaDataMPU6050() {
  Wire.setClock(100000);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return;
  }

  int received = Wire.requestFrom(MPU_ADDR, 14, true);

  if (received != 14) {
    return;
  }

  AcX = (Wire.read() << 8) | Wire.read();
  AcY = (Wire.read() << 8) | Wire.read();
  AcZ = (Wire.read() << 8) | Wire.read();

  // Buang temperature register MPU6050.
  Wire.read();
  Wire.read();

  GyX = (Wire.read() << 8) | Wire.read();
  GyY = (Wire.read() << 8) | Wire.read();
  GyZ = (Wire.read() << 8) | Wire.read();

  ax = AcX / 16384.0;
  ay = AcY / 16384.0;
  az = AcZ / 16384.0;
}

// ============================================================
// KLASIFIKASI MPU
// ============================================================
String klasifikasiKondisiMPU(float nilaiResultan) {
  if (nilaiResultan > TH_JATUH) {
    return "Jatuh";
  }
  else {
    return "Normal";
  }
}

// ============================================================
// MPU6050 + DETEKSI JATUH
// ============================================================
void bacaKirimMPU() {
  bacaDataMPU6050();

  totalAcc =
    sqrt(ax * ax + ay * ay + az * az);

  statusMPUsaatIni =
    klasifikasiKondisiMPU(totalAcc);

  Serial.print("MPU6050 -> Resultan: ");
  Serial.print(totalAcc, 3);
  Serial.print(" g | Kondisi: ");
  Serial.println(statusMPUsaatIni);

  // Kirim hanya saat status berubah.
  if (statusMPUsaatIni != lastStatusMPU) {
    lastStatusMPU = statusMPUsaatIni;
    Blynk.virtualWrite(V9, statusMPUsaatIni);
  }

  // ----------------------------------------------------------
  // Konfirmasi jatuh minimal 1 detik
  // ----------------------------------------------------------
  if (statusMPUsaatIni == "Jatuh") {

    if (!sedangKonfirmasiJatuh) {
      sedangKonfirmasiJatuh = true;
      waktuTerdeteksiJatuh = millis();
      kondisiJatuhTerkonfirmasi = false;
    }
    else if (
      millis() - waktuTerdeteksiJatuh >=
      DURASI_KONFIRMASI
    ) {
      kondisiJatuhTerkonfirmasi = true;
      Serial.println("=== FALL DETECTED !!! ===");
    }

  }
  else {
    sedangKonfirmasiJatuh = false;
    kondisiJatuhTerkonfirmasi = false;
  }
}

// ============================================================
// AKTIFKAN BUZZER
// ============================================================
void triggerAlarmBuzzer() {
  buzzerSedangAktif = true;
  waktuBuzzerMulai = millis();
  setBuzzer(true);

  Serial.println("BUZZER -> AKTIF 5 DETIK");
}

// ============================================================
// KIRIM EVENT / NOTIFIKASI BLYNK
// ============================================================
void kirimNotifikasiBlynk(const char* eventCode, String pesan) {
  if (Blynk.connected()) {
    Blynk.logEvent(eventCode, pesan);

    Serial.print("BLYNK EVENT TERKIRIM -> ");
    Serial.print(eventCode);
    Serial.print(" | ");
    Serial.println(pesan);

    // Event berhasil diproses oleh koneksi Blynk.
    notifikasiTertunda = false;
    eventTertunda = "";
    pesanTertunda = "";
  } else {
    // Jangan buang alarm jika koneksi sedang putus.
    notifikasiTertunda = true;
    eventTertunda = String(eventCode);
    pesanTertunda = pesan;

    Serial.println("BLYNK EVENT TERTUNDA: Blynk belum terhubung.");
  }
}

// ------------------------------------------------------------
// CEK KONEKSI + KIRIM NOTIFIKASI YANG TERTUNDA
// ------------------------------------------------------------
void cekKoneksiBlynk() {
  bool terhubung = Blynk.connected();

  if (terhubung && !blynkPernahTerhubung) {
    Serial.println("BLYNK: TERHUBUNG.");
    blynkPernahTerhubung = true;
  }

  if (!terhubung && blynkPernahTerhubung) {
    Serial.println("BLYNK: KONEKSI TERPUTUS.");
    blynkPernahTerhubung = false;
  }

  if (terhubung && notifikasiTertunda && eventTertunda.length() > 0) {
    Blynk.logEvent(eventTertunda.c_str(), pesanTertunda);

    Serial.print("BLYNK EVENT TERTUNDA -> TERKIRIM: ");
    Serial.print(eventTertunda);
    Serial.print(" | ");
    Serial.println(pesanTertunda);

    notifikasiTertunda = false;
    eventTertunda = "";
    pesanTertunda = "";
  }
}

// ============================================================
// EVALUASI ALARM + BUZZER + NOTIFIKASI
//
// Notifikasi hanya dikirim ketika alarm BARU muncul.
// Setelah kondisi kembali normal, alarm otomatis reset.
// Jika abnormal lagi, buzzer dan notifikasi akan aktif lagi.
// ============================================================
void evaluasiBuzzer() {

  if (!Blynk.connected()) {
    // Tidak menghentikan pembacaan sensor/alarm.
    // Hanya dicatat agar mudah mengetahui penyebab notifikasi tertunda.
  }

  alarmHumid =
    (!isnan(kelembapanDHT)) &&
    (kelembapanDHT < humidMin || kelembapanDHT > humidMax);

  alarmBPM =
    (beatAvg != 0) &&
    (beatAvg < bpmMin || beatAvg > bpmMax);

  alarmMLX =
    (!isnan(objectTemp)) &&
    (objectTemp < mlxMin || objectTemp > mlxMax);

  alarmJatuh = kondisiJatuhTerkonfirmasi;

  // ----------------------------------------------------------
  // DHT22 - KELEMBAPAN
  // ----------------------------------------------------------
  if (alarmHumid && !alarmHumidSebelumnya) {
    triggerAlarmBuzzer();

    String pesan;

    if (kelembapanDHT < humidMin) {
      pesan = "PERINGATAN: Kelembapan rendah: " +
              String(kelembapanDHT, 1) +
              "%. Batas minimum: " + String(humidMin, 1) + "%.";
    } else {
      pesan = "PERINGATAN: Kelembapan tinggi: " +
              String(kelembapanDHT, 1) +
              "%. Batas maksimum: " + String(humidMax, 1) + "%.";
    }

    kirimNotifikasiBlynk(EVENT_HUMIDITY_ALERT, pesan);
    Serial.println("ALARM DHT22 -> BUZZER + NOTIFIKASI");
  }

  // ----------------------------------------------------------
  // MAX30102 - BPM
  // ----------------------------------------------------------
  if (alarmBPM && !alarmBPMSebelumnya) {
    triggerAlarmBuzzer();

    String pesan;

    if (beatAvg < bpmMin) {
      pesan = "PERINGATAN: BPM rendah: " +
              String(beatAvg) +
              " BPM. Batas minimum: " + String(bpmMin, 0) + " BPM.";
    } else {
      pesan = "PERINGATAN: BPM tinggi: " +
              String(beatAvg) +
              " BPM. Batas maksimum: " + String(bpmMax, 0) + " BPM.";
    }

    kirimNotifikasiBlynk(EVENT_BPM_ALERT, pesan);
    Serial.println("ALARM MAX30102 -> BUZZER + NOTIFIKASI");
  }

  // ----------------------------------------------------------
  // MLX90614 - SUHU
  // ----------------------------------------------------------
  if (alarmMLX && !alarmMLXSebelumnya) {
    triggerAlarmBuzzer();

    String pesan;

    if (objectTemp < mlxMin) {
      pesan = "PERINGATAN: Suhu rendah: " +
              String(objectTemp, 2) +
              " C. Batas minimum: " + String(mlxMin, 1) + " C.";
    } else {
      pesan = "PERINGATAN: Suhu tinggi: " +
              String(objectTemp, 2) +
              " C. Batas maksimum: " + String(mlxMax, 1) + " C.";
    }

    kirimNotifikasiBlynk(EVENT_TEMPERATURE_ALERT, pesan);
    Serial.println("ALARM MLX90614 -> BUZZER + NOTIFIKASI");
  }

  // ----------------------------------------------------------
  // MPU6050 - JATUH
  // ----------------------------------------------------------
  if (alarmJatuh && !alarmJatuhSebelumnya) {
    triggerAlarmBuzzer();

    String pesan =
      "DARURAT: Terdeteksi kondisi JATUH! Segera periksa pengguna.";

    kirimNotifikasiBlynk(EVENT_FALL_DETECTED, pesan);
    Serial.println("ALARM MPU6050 JATUH -> BUZZER + NOTIFIKASI");
  }

  // Simpan status sekarang. Jika kembali normal, flag menjadi false
  // sehingga alarm dapat dipicu kembali saat abnormal berikutnya.
  alarmHumidSebelumnya = alarmHumid;
  alarmBPMSebelumnya   = alarmBPM;
  alarmMLXSebelumnya   = alarmMLX;
  alarmJatuhSebelumnya = alarmJatuh;

  // ----------------------------------------------------------
  // BUZZER 5 DETIK
  // Alarm baru akan memulai ulang timer buzzer.
  // ----------------------------------------------------------
  if (buzzerSedangAktif) {
    if (millis() - waktuBuzzerMulai < DURASI_BUZZER) {
      setBuzzer(true);
    } else {
      buzzerSedangAktif = false;
      setBuzzer(false);
    }
  } else {
    setBuzzer(false);
  }
}

// ============================================================
// BLYNK SETTINGS - MAX30102
// ============================================================
BLYNK_WRITE(V3) {
  bpmMin = param.asFloat();

  Serial.print("Set MIN BPM -> ");
  Serial.println(bpmMin);
}

BLYNK_WRITE(V4) {
  bpmMax = param.asFloat();

  Serial.print("Set MAX BPM -> ");
  Serial.println(bpmMax);
}

// ============================================================
// BLYNK SETTINGS - MLX90614
// ============================================================
BLYNK_WRITE(V5) {
  mlxMin = param.asFloat();

  Serial.print("Set MIN Suhu MLX -> ");
  Serial.println(mlxMin);
}

BLYNK_WRITE(V6) {
  mlxMax = param.asFloat();

  Serial.print("Set MAX Suhu MLX -> ");
  Serial.println(mlxMax);
}

// ============================================================
// BLYNK SETTINGS - DHT22
// ============================================================
BLYNK_WRITE(V7) {
  humidMin = param.asFloat();

  Serial.print("Set MIN Kelembapan -> ");
  Serial.println(humidMin);
}

BLYNK_WRITE(V8) {
  humidMax = param.asFloat();

  Serial.print("Set MAX Kelembapan -> ");
  Serial.println(humidMax);
}