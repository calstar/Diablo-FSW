/*
 * ESP32 Arduino Template for PT Sensors
 * Compatible with Diablo FSW PT Integration System
 * 
 * This template shows how to send PT sensor data to the Diablo FSW system.
 * Only sends time and pressure data (no temperature or other measurements).
 */

// Include libraries for ADS126X ADC (adjust based on your hardware)
#include <ADS126X.h>
#include <SPI.h>

// Hardware configuration - adjust pin numbers for your setup
#define DRDY_PIN 14
#define DOUT 41    // MISO
#define DIN 5      // MOSI
#define SCLK 13 
#define CS 37
#define START 43

// PT sensor configuration - 9 specific engine locations
#define PT_NUM_START 0
#define NUM_PTS 9           // 9 PT sensors for specific engine locations
#define TEXT_OUTPUT 0       // 1 = ASCII for debugging, 0 = binary for production

// PT Location mapping (adjust channel assignments based on your hardware wiring)
// Channel 0: Pressurant Tank PT
// Channel 1: Kero Inlet PT  
// Channel 2: Kero Outlet PT
// Channel 3: Lox Inlet PT
// Channel 4: Lox Outlet PT
// Channel 5: Injector PT
// Channel 6: Chamber Wall PT #1
// Channel 7: Chamber Wall PT #2
// Channel 8: Nozzle Exit PT

// Buffer settings
static const size_t PKT_MAX = 1000;   // packet size in bytes
static const uint32_t FLUSH_MS = 10;  // flush interval in ms

static uint8_t pktA[PKT_MAX];
static uint8_t pktB[PKT_MAX];
static uint8_t* active = pktA;
static uint8_t* standby = pktB;
static size_t used = 0;
static uint32_t lastFlushMs = 0;

ADS126X adc;
int pos_pin = 0;     // AIN0
int neg_pin = 0xA;   // AINCOM

// Sample record structure - MUST match Diablo FSW expectations
#pragma pack(push, 1)
struct SampleRecord {
  uint32_t t_us;              // timestamp in microseconds
  uint8_t  channel;           // sensor channel (0-9)
  int32_t  volt_reader;       // raw ADC reading
  float    voltage;           // converted voltage
  uint32_t read_time_us;      // read time in microseconds
  float    samples_per_second; // calculated sample rate
  uint32_t sent_us;           // sent timestamp
};
#pragma pack(pop)

// Helper function declarations
inline void flushPacketIfNeeded(bool force = false);
void sendPTSensorData();

void setup() {
  Serial.begin(115200);
  SPI.begin(SCLK, DOUT, DIN, CS);

  adc.begin(CS);
  adc.startADC1();
  adc.setRate(ADS126X_RATE_38400);
  adc.enableInternalReference();
  
  pinMode(DRDY_PIN, INPUT);
  pinMode(START, OUTPUT);
  digitalWrite(START, LOW);
  
  Serial.println("ESP32 PT Sensor System Started");
  Serial.println("Sending data to Diablo FSW system...");
}

void loop() {
  sendPTSensorData();
  delay(10); // 100 Hz update rate
}

void sendPTSensorData() {
  float voltages[NUM_PTS];
  bool sensor_active[NUM_PTS] = {false}; // Track which sensors are actually connected
  
  // Read all PT sensors
  for (uint8_t ch = 0; ch < NUM_PTS; ch++) {
    // Skip if sensor is not connected (you can implement sensor detection logic here)
    // For now, we'll read all channels and let the FSW system handle missing data
    
    long raw = adc.readADC1(ch, neg_pin);
    
    // Wait for DRDY if needed
    for (int j = 0; j < 4; j++) {
      while (digitalRead(DRDY_PIN) == HIGH) {
        // wait
      }
    }
    
    uint32_t t_start = micros();
    long raw_reading = adc.readADC1(ch, neg_pin);
    uint32_t read_time = micros() - t_start;
    
    // Convert to volts (adjust based on your PT specifications)
    float volts = (float)raw_reading * 2.5f / 2147483648.0f;
    voltages[ch] = volts;
    
    // Check if sensor is active (has reasonable voltage reading)
    // Adjust thresholds based on your PT specifications
    if (volts > 0.1f && volts < 4.9f) {
      sensor_active[ch] = true;
      
      if (!TEXT_OUTPUT) {
        // Binary mode - send SampleRecord
        float sps = (read_time > 0) ? (1000000.0f / (float)read_time) : 0.0f;
        
        SampleRecord rec;
        rec.t_us = t_start;
        rec.channel = ch;
        rec.volt_reader = (int32_t)raw_reading;
        rec.voltage = volts;
        rec.read_time_us = read_time;
        rec.samples_per_second = sps;
        rec.sent_us = 0;
        
        // Buffer packet
        if (used + sizeof(rec) > PKT_MAX) {
          flushPacketIfNeeded(true);
        }
        
        memcpy(active + used, &rec, sizeof(rec));
        used += sizeof(rec);
        
        // Flush periodically
        flushPacketIfNeeded(false);
      }
    }
  }
  
  if (TEXT_OUTPUT) {
    // Text mode for debugging
    for (int i = 0; i < NUM_PTS; i++) {
      if (sensor_active[i]) {
        Serial.print(voltages[i], 6);
      } else {
        Serial.print("NaN"); // Mark inactive sensors
      }
      if (i < NUM_PTS - 1) Serial.print(" ");
    }
    Serial.print("\r\n"); // CRLF as expected by the Python script
  }
}

inline void flushPacketIfNeeded(bool force = false) {
  uint32_t now = millis();
  if (force || (used > 0 && (now - lastFlushMs >= FLUSH_MS))) {
    uint32_t send_ts = micros();
    if (used > 0) {
      size_t count = used / sizeof(SampleRecord);
      SampleRecord* recs = reinterpret_cast<SampleRecord*>(active);
      for (size_t i = 0; i < count; ++i) {
        recs[i].sent_us = send_ts;
      }
    }
    Serial.write(active, used);
    Serial.println();  // optional delimiter
    uint8_t* tmp = active;
    active = standby;
    standby = tmp;
    used = 0;
    lastFlushMs = now;
  }
}

/*
 * Configuration Notes:
 * 
 * 1. Adjust pin definitions (DRDY_PIN, DOUT, DIN, SCLK, CS, START) based on your hardware
 * 2. Adjust sensor detection logic in sendPTSensorData() function
 * 3. Set TEXT_OUTPUT = 1 for debugging, 0 for production
 * 4. The system handles missing sensors automatically - only active sensors send data
 * 5. Raw voltage data is sent (calibration applied later in Diablo FSW system)
 * 
 * PT Location Mapping:
 * - Channel 0: Pressurant Tank PT
 * - Channel 1: Kero Inlet PT  
 * - Channel 2: Kero Outlet PT
 * - Channel 3: Lox Inlet PT
 * - Channel 4: Lox Outlet PT
 * - Channel 5: Injector PT
 * - Channel 6: Chamber Wall PT #1
 * - Channel 7: Chamber Wall PT #2
 * - Channel 8: Nozzle Exit PT
 * 
 * Expected Behavior:
 * - Sends raw voltage data for sensors 0-8 that are connected and reading valid voltages
 * - Missing sensors will not send data (e.g., if only sensors 1, 2, 6, 8 are connected)
 * - Diablo FSW system will only process data from active sensors
 * - Raw voltage data is stored for later calibration to pressure
 * - Binary mode is more efficient for production use
 */
