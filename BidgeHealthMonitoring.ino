#define BLYNK_TEMPLATE_ID "YourTemplateID"
#define BLYNK_TEMPLATE_NAME "YourDeviceName"
#define BLYNK_AUTH_TOKEN "YourBlynkAuthToken"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BlynkSimpleEsp32.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_ADS1X15.h> // For strain gauge via ADC
#include <arduinoFFT.h>

// Temperature sensors
#define ONE_WIRE_BUS_1 4  // GPIO for first temperature sensor
#define ONE_WIRE_BUS_2 5  // GPIO for second temperature sensor

// Strain gauge via ADS1115 ADC
Adafruit_ADS1115 ads;

// MPU6050
Adafruit_MPU6050 mpu;

// FFT
#define SAMPLES 64 // Must be power of 2
#define SAMPLING_FREQUENCY 100.0 // Hz

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT FFT = ArduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);
// Natural frequency of the bridge (Hz)
#define BRIDGE_NATURAL_FREQ 5.0
#define RESONANCE_TOLERANCE 0.3 // Hz

// Tilt Thresholds
#define TILT_THRESHOLD 10.0 // degrees

// Strain Gauge Calibration
#define LOAD_LIMIT 1000.0 // Maximum safe load (arbitrary units)

// Thermal expansion calculation
#define BRIDGE_LENGTH 50.0 // meters
#define EXPANSION_COEFF 0.000012 // per degree Celsius (steel)

// ------- Globals -------
char auth[] = BLYNK_AUTH_TOKEN;

// Temperature sensors
OneWire oneWire1(ONE_WIRE_BUS_1);
OneWire oneWire2(ONE_WIRE_BUS_2);
DallasTemperature sensors1(&oneWire1);
DallasTemperature sensors2(&oneWire2);

// Alert flags
bool vibration_alert = false;
bool tilt_alert = false;
bool temp_alert = false;
bool strain_alert = false;

// ---- Helper Functions ----
void sendAlert(String msg) {
  Blynk.logEvent("bridge_alert", msg); // Blynk Event
  Serial.println("ALERT: " + msg);
}

void checkVibrationAndResonance() {
  // Collect samples
  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    vReal[i] = a.acceleration.x; // Use one axis or magnitude for complex analysis
    vImag[i] = 0;
    delay(1000 / SAMPLING_FREQUENCY);
  }
  // FFT
FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
FFT.compute(FFT_FORWARD);
FFT.complexToMagnitude();
  // Find major frequency
  double maxMag = 0;
  int maxIndex = 0;
  for (int i = 1; i < SAMPLES / 2; i++) {
    if (vReal[i] > maxMag) {
      maxMag = vReal[i];
      maxIndex = i;
    }
  }
  double freq = (maxIndex * SAMPLING_FREQUENCY) / SAMPLES;
  Serial.print("Dominant vibration frequency: "); Serial.println(freq);

  // Resonance check
  if (abs(freq - BRIDGE_NATURAL_FREQ) < RESONANCE_TOLERANCE) {
    if (!vibration_alert) {
      sendAlert("DANGER: Vibration near bridge resonance frequency!");
      vibration_alert = true;
    }
  } else {
    vibration_alert = false; // Reset alert
  }
}

void checkTilt() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate tilt angle from accelerometer (simple pitch approximation)
  double pitch = atan2(a.acceleration.x, sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180.0 / PI;

  Serial.print("Pitch: "); Serial.println(pitch);

  if (abs(pitch) > TILT_THRESHOLD) {
    if (!tilt_alert) {
      sendAlert("ALERT: Bridge tilt exceeds normal value!");
      tilt_alert = true;
    }
  } else {
    tilt_alert = false;
  }
}

void checkTemperatureGradientAndExpansion() {
  sensors1.requestTemperatures();
  sensors2.requestTemperatures();
  float temp1 = sensors1.getTempCByIndex(0);
  float temp2 = sensors2.getTempCByIndex(0);

  Serial.print("Temp1: "); Serial.println(temp1);
  Serial.print("Temp2: "); Serial.println(temp2);

  float temp_gradient = temp2 - temp1;
  float avg_temp = (temp1 + temp2) / 2.0;
  float expansion = BRIDGE_LENGTH * EXPANSION_COEFF * temp_gradient; // delta L

  Serial.print("Expansion: "); Serial.println(expansion);

  // Alert on abnormal expansion (example threshold: > 1cm = 0.01m)
  if (abs(expansion) > 0.01) {
    if (!temp_alert) {
      sendAlert("ALERT: Abnormal bridge expansion detected!");
      temp_alert = true;
    }
  } else {
    temp_alert = false;
  }
}

void checkStrainGauge() {
  int16_t adc0 = ads.readADC_SingleEnded(0);
  // Calibrate to convert ADC to load units
  double load = (adc0 - 15000) * (LOAD_LIMIT / 12000.0); // Example mapping, calibrate for your sensor

  Serial.print("Load: "); Serial.println(load);

  if (load > LOAD_LIMIT * 0.9) {
    if (!strain_alert) {
      sendAlert("ALERT: Bridge load approaching/exceeding safe limit!");
      strain_alert = true;
    }
  } else {
    strain_alert = false;
  }
}

void setup() {
  Serial.begin(115200);
  Blynk.begin(auth, "YourWiFiSSID", "YourWiFiPassword");

  // MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  // ADC for strain gauge
  if (!ads.begin()) {
    Serial.println("Failed to find ADS1115 ADC");
    while (1) delay(10);
  }

  // Temp sensors
  sensors1.begin();
  sensors2.begin();

  Serial.println("Bridge Health Monitor Started");
}

void loop() {
  Blynk.run();

  checkVibrationAndResonance();
  checkTilt();
  checkTemperatureGradientAndExpansion();
  checkStrainGauge();

  delay(5000); // Main loop delay (5 seconds)
}
