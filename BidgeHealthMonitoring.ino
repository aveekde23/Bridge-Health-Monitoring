#define BLYNK_TEMPLATE_ID "YourTemplateID"
#define BLYNK_TEMPLATE_NAME "YourDeviceName"
#define BLYNK_AUTH_TOKEN "YourBlynkAuthToken"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BlynkSimpleEsp32.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HX711.h>
#include <arduinoFFT.h>

// Temperature sensors
#define ONE_WIRE_BUS_1 4  // GPIO for first temperature sensor
#define ONE_WIRE_BUS_2 5  // GPIO for second temperature sensor

// HX711 pins for strain gauge/load cell
#define HX711_DOUT 18
#define HX711_SCK  19

// MPU6050 sensors
Adafruit_MPU6050 mpu_vibration; // Address 0x68 (AD0 to GND)
Adafruit_MPU6050 mpu_tilt;      // Address 0x69 (AD0 to 3.3V)

// HX711 for strain gauge / load cell
HX711 scale;

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
#define EXPANSION_RATE_THRESHOLD_PER_HOUR 0.002 // meters/hour (example: 2mm/hr)

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

// Expansion rate monitoring
float prev_expansion = 0.0;
unsigned long prev_expansion_time = 0; // in ms

// ---- Helper Functions ----
void sendAlert(String msg) {
  Blynk.logEvent("bridge_alert", msg); // Blynk Event
  Serial.println("ALERT: " + msg);
}

void checkVibrationAndResonance() {
  // Collect samples from vibration MPU6050 (address 0x68)
  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t a, g, temp;
    mpu_vibration.getEvent(&a, &g, &temp);
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
  // Use the tilt MPU6050 (address 0x69)
  mpu_tilt.getEvent(&a, &g, &temp);

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

  // --- Rate of change calculation (per hour) ---
  unsigned long now = millis();
  if (prev_expansion_time > 0) {
    float hours = (now - prev_expansion_time) / (1000.0 * 60.0 * 60.0); // ms to hours
    if (hours > 0) {
      float rate_per_hour = (expansion - prev_expansion) / hours;
      Serial.print("Expansion rate per hour: "); Serial.println(rate_per_hour, 6);
      if (abs(rate_per_hour) > EXPANSION_RATE_THRESHOLD_PER_HOUR) {
        sendAlert("ALERT: High rate of bridge expansion/contraction detected!");
      }
    }
  }
  prev_expansion = expansion;
  prev_expansion_time = now;
  // --- End rate of change ---

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
  // Read load cell value from HX711, average 10 samples
  float load = scale.get_units(10);

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

  // MPU6050 - Vibration (AD0 to GND, address 0x68)
  if (!mpu_vibration.begin(0x68)) {
    Serial.println("Failed to find MPU6050 (vibration)");
    while (1) delay(10);
  }
  // MPU6050 - Tilt (AD0 to 3.3V, address 0x69)
  if (!mpu_tilt.begin(0x69)) {
    Serial.println("Failed to find MPU6050 (tilt)");
    while (1) delay(10);
  }

  // HX711 for strain gauge/load cell
  scale.begin(HX711_DOUT, HX711_SCK);

  // If you want to tare (zero) the scale at start:
  Serial.println("Taring the load cell, please ensure it's unloaded...");
  scale.tare();
  Serial.println("Tare complete.");

  // If you want to calibrate, set the scale factor here
  // scale.set_scale(calibration_factor);
  // You will need to determine this value for your hardware

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
