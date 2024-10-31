#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <FarmMonitoring_2024_inferencing.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include <WiFi.h>
#include <ThingSpeak.h>


// Pin Definitions
const int TOUCH_PIN = D1;   // Pin for limit Switch (box opened detection)
const int REPEL_PIN = D3;   // Pin to control the repelling device (speaker)

// GSM Module Serial
HardwareSerial gsmSerial(1);  // Use hardware serial 1 for GSM module
const int GSM_RX_PIN = D10;    // GSM module RX pin
const int GSM_TX_PIN = D9;    // GSM module TX pin
const int GSM_BAUD = 9600;

// New variables for GSM call detection
bool callReceived = false;

// Variables for security
bool boxOpenAlertSent = false;
bool lastBoxState = false;  // To track the previous state of the box

// GPS Module Serial
HardwareSerial gpsSerial(2);  // Use hardware serial 2 for GPS module
const int GPS_RX_PIN = D8;    // GPS module RX pin
const int GPS_TX_PIN = D7;    // GPS module TX pin
const int GPS_BAUD = 9600;

// TinyGPS++ object
TinyGPSPlus gps;

// Variables for bird detection and SMS sending
bool smsSent = false;
bool repelStatus = false;
unsigned long lastRepelTime = 0;
unsigned long lastRepelStartTime = 0;
const float BIRD_DETECTION_THRESHOLD = 0.5;
const unsigned long REPEL_DURATION = 10000; // 10 seconds in milliseconds
float birdProbability = 0.0;

String incomingData = "";
bool callInProgress = false;
unsigned long callStartTime = 0;
const unsigned long CALL_TIMEOUT = 3000; // 30 seconds timeout for call handling

// Variables for bird detection and repel control
bool repelActive = false;
unsigned long repelStartTime = 0;


// Audio inference
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
static const uint32_t sample_buffer_size = 4096; // Increased from 2048
static int16_t sampleBuffer[sample_buffer_size];
static bool debug_nn = false;
static bool record_status = true;

// Function prototypes
bool sendSMS(String recipient, String message);
void sendBirdDetectionSMS();
String getLocationLink();
void activateRepel();
void deactivateRepel();
void printDebugInfo();
static int i2s_init(uint32_t sampling_rate);
static int i2s_deinit(void);
static void audio_inference_callback(uint32_t n_bytes);
static void capture_samples(void* arg);
static bool microphone_inference_start(uint32_t n_samples);
static bool microphone_inference_record(void);
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);
void checkGSMData();
bool isAuthorizedCaller(String number);

// ThingSpeak settings
const char* ssid = "HomeLab-Net-2G";
const char* password = "HomeLab24/7";
unsigned long channelID = 2688179;
const char* writeAPIKey = "H90ZNTIAHGQG8N1P";

WiFiClient client;

// Counters for detections and calls
long detectionCount = 0;
long callCount = 0;

// Timer for ThingSpeak updates
unsigned long lastThingSpeakUpdate = 0;
const unsigned long thingSpeakInterval = 60000; // Update every 1 minute


void setup() {
    Serial.begin(115200);
    //while (!Serial);
    Serial.println("Integrated Weever Bird Repellent System with TinyML");
    Serial.println("-----------------------------------------------");

    // Initialize WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Initialize ThingSpeak
    ThingSpeak.begin(client);

    // Initialize GSM and GPS serial
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    gsmSerial.begin(GSM_BAUD, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
    delay(1000);
    // Set up GSM module for call detection
     Serial.println("GSM Serial port initialized");
    
    // Initialize GSM with single try
    Serial.println("\nStarting GSM initialization...");
    initGSM();  // No more retries, just one attempt
    
    // Initialize lastBoxState
    lastBoxState = digitalRead(TOUCH_PIN) == HIGH;

    // Set pin modes
    pinMode(REPEL_PIN, OUTPUT);
    digitalWrite(REPEL_PIN, LOW);  // Ensure the repel is initially off
    pinMode(TOUCH_PIN, INPUT_PULLUP);  // Use INPUT_PULLUP for the limit switch

    // Initialize audio inference
    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        Serial.println("ERR: Could not allocate audio buffer");
        return;
    }

    Serial.println("Setup completed. Starting main loop...");
}

void loop() {
    // Check for incoming GSM data
    checkGSMData();

  // Check box state immediately at the start of each loop
    bool currentBoxState = digitalRead(TOUCH_PIN) == HIGH;

    // If the box state has changed, take immediate action
    if (currentBoxState != lastBoxState) {
        if (currentBoxState) {
            // Box has just been opened            
            if (!smsSent) {
                // Send SMS alert
                String message = "SECURITY ALERT: Device box opened at " + getLocationLink();
                bool firstSMSSent = sendSMS("+250791207396", message);
                delay(2000);  // Wait 2 seconds between sending messages
                bool secondSMSSent = sendSMS("+250798077466", message);
                
                if (firstSMSSent && secondSMSSent) {
                    smsSent = true;
                    Serial.println("Both SMS alerts sent successfully");
                } else {
                    Serial.println("Failed to send one or both SMS alerts");
                    // You might want to retry or handle the failure case
                }
            }
        } else {
            smsSent = false;  // Reset the SMS sent flag
            Serial.println("Box closed. Buzzer stopped and alert reset.");
        }
        lastBoxState = currentBoxState;  // Update the last known state
    }

    // Check and reset repeller if needed
    if (repelActive && (millis() - repelStartTime >= REPEL_DURATION)) {
        digitalWrite(REPEL_PIN, LOW);
        repelActive = false;
        Serial.println("Repeller deactivated");
    }

    // Update GPS data
    // Update GPS data and print debug info
    updateGPS();

    // Check if box is opened
    if (currentBoxState && !smsSent) {
        sendSMS("+250791207396", "SECURITY ISSUE: Box opened at " + getLocationLink());
        sendSMS("+250798077466", "SECURITY ISSUE: Box opened at " + getLocationLink());
        smsSent = true;
    }

    // Perform audio inference
    bool m = microphone_inference_record();
    if (!m) {
        Serial.println("ERR: Failed to record audio...");
        return;
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        Serial.println("ERR: Failed to run classifier");
        return;
    }

    // Check if bird is detected based on model output
    bool birdDetected = false;
    float maxBirdProbability = 0.0;

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        Serial.print(result.classification[ix].label);
        Serial.print(": ");
        Serial.println(result.classification[ix].value);
        
        // Check for specific bird classes and update maxBirdProbability
        if (strcmp(result.classification[ix].label, "Bulbul") == 0 ||
            strcmp(result.classification[ix].label, "ComWaxbill") == 0 ||
            strcmp(result.classification[ix].label, "RedBilledQuelea") == 0 ||
            strcmp(result.classification[ix].label, "VillageWeaver") == 0 ||
            strcmp(result.classification[ix].label, "WBRchat") == 0 ||
            strcmp(result.classification[ix].label, "Yellow-fronted Canary") == 0) {
            
            if (result.classification[ix].value > maxBirdProbability) {
                maxBirdProbability = result.classification[ix].value;
            }
        }
    }

    if (maxBirdProbability > BIRD_DETECTION_THRESHOLD) {
        birdDetected = true;
    }

    // Check for bird detection
    if (birdDetected) {
        activateRepel();
        detectionCount++;
        if (!smsSent) {
            sendBirdDetectionSMS();
            smsSent = true;
        }
    }

     // Check for call-based activation
    if (callReceived) {
        activateRepel();
        callCount++;
        callReceived = false;
    }


    // Check if it's time to deactivate repel
    if (repelActive && (millis() - repelStartTime >= REPEL_DURATION)) {
        deactivateRepel();
    }

    // Reset SMS sent flag after some time
    if (smsSent && (millis() - repelStartTime >= 60000)) {  // Reset after 1 minute
        smsSent = false;
    }

    // Update ThingSpeak
    if (millis() - lastThingSpeakUpdate >= thingSpeakInterval) {
        updateThingSpeak();
        lastThingSpeakUpdate = millis();
    }
  

    // Print debug info
    printDebugInfo(maxBirdProbability);

    delay(10);
}

bool initGSM() {
    Serial.println("\n--- Starting GSM Module Initialization ---");
    Serial.println("Please wait for 10 seconds while the module powers up...");
    delay(10000); // Initial delay to allow module to power up properly
    
    // Reset the module
    Serial.println("Resetting GSM module...");
    gsmSerial.println("AT+CFUN=1,1");
    delay(20000); // Wait for the module to restart

    // Clear any pending serial data
    while(gsmSerial.available()) {
        Serial.print(char(gsmSerial.read()));
    }

    // Check if the module is responsive with multiple retries
    Serial.println("Checking module response...");
    bool atResponse = false;
    for(int i = 0; i < 5; i++) {  // Try 5 times
        gsmSerial.println("AT");
        delay(1000);
        
        String response = "";
        while(gsmSerial.available()) {
            char c = gsmSerial.read();
            response += c;
            Serial.print(c);  // Print raw response for debugging
        }
        
        if(response.indexOf("OK") != -1) {
            atResponse = true;
            Serial.println("Module is responsive ✓");
            break;
        }
        
        Serial.println("Retry AT command...");
        delay(1000);
    }
    
    if (!atResponse) {
        Serial.println("WARNING: No 'OK' response received, but continuing...");
        // Don't return false here, continue with initialization
    }

    // Check signal quality
    Serial.println("Checking signal quality...");
    gsmSerial.println("AT+CSQ");
    delay(2000);
    
    String response = "";
    while(gsmSerial.available()) {
        char c = gsmSerial.read();
        response += c;
        Serial.print(c);  // Print raw response for debugging
    }
    
    int signalQuality = -1;
    if (response.indexOf("+CSQ:") != -1) {
        // Parse the signal quality from the response string
        int startIdx = response.indexOf("+CSQ:") + 5;
        int endIdx = response.indexOf(",", startIdx);
        if(endIdx != -1) {
            String qualityStr = response.substring(startIdx, endIdx);
            qualityStr.trim();
            signalQuality = qualityStr.toInt();
        }
    }

    if (signalQuality >= 0) {
        int signalPercentage = (signalQuality * 100) / 31;
        Serial.print("Signal quality: ");
        Serial.print(signalQuality);
        Serial.print(" (");
        Serial.print(signalPercentage);
        Serial.println("%)");
        
        if (signalQuality >= 20) {
            Serial.println("Signal strength: Excellent ✓");
        } else if (signalQuality >= 15) {
            Serial.println("Signal strength: Good ✓");
        } else if (signalQuality >= 10) {
            Serial.println("Signal strength: Fair ⚠");
        } else {
            Serial.println("Signal strength: Poor ⚠");
            Serial.println("Warning: Poor signal may affect performance");
        }
    } else {
        Serial.println("WARNING: Couldn't read signal quality, but continuing...");
        // Don't return false here, continue with initialization
    }

    // Check network registration with multiple retries
    Serial.println("Checking network registration...");
    bool networkReg = false;
    for(int i = 0; i < 5; i++) {  // Try 5 times
        gsmSerial.println("AT+CREG?");
        delay(2000);
        
        response = "";
        while(gsmSerial.available()) {
            char c = gsmSerial.read();
            response += c;
            Serial.print(c);  // Print raw response for debugging
        }
        
        if(response.indexOf("+CREG: 0,1") != -1 || response.indexOf("+CREG: 0,5") != -1) {
            networkReg = true;
            Serial.println("Network registration successful ✓");
            break;
        }
        
        Serial.println("Retrying network registration check...");
        delay(2000);
    }
    
    if (!networkReg) {
        Serial.println("WARNING: Network registration status unclear, but continuing...");
        // Don't return false here, continue with initialization
    }

    // Set SMS text mode
    Serial.println("Configuring SMS mode...");
    gsmSerial.println("AT+CMGF=1");
    delay(1000);
    response = "";
    while(gsmSerial.available()) {
        char c = gsmSerial.read();
        response += c;
        Serial.print(c);
    }
    
    if (response.indexOf("OK") == -1) {
        Serial.println("WARNING: SMS mode response unclear, but continuing...");
    } else {
        Serial.println("SMS mode configured ✓");
    }

    // Enable call identification
    Serial.println("Enabling caller identification...");
    gsmSerial.println("AT+CLIP=1");
    delay(1000);
    response = "";
    while(gsmSerial.available()) {
        char c = gsmSerial.read();
        response += c;
        Serial.print(c);
    }
    
    if (response.indexOf("OK") == -1) {
        Serial.println("WARNING: Caller ID setup response unclear, but continuing...");
    } else {
        Serial.println("Caller ID enabled ✓");
    }

    Serial.println("\n=== GSM Module Initialization Complete ===");
    Serial.println("Module is ready for operation!");
    return true;  // Always return true since the module seems to work even with unclear responses
}

void activateRepel() {
    Serial.println("Activating repeller");
    digitalWrite(REPEL_PIN, HIGH);
    repelActive = true;
    repelStartTime = millis();
    Serial.println("Repeller activated");
}

void deactivateRepel() {
    digitalWrite(REPEL_PIN, LOW);
    repelActive = false;
    Serial.println("Repeller deactivated");
}

void sendBirdDetectionSMS() {
    String message = "PEST BIRDS DETECTED at " + getLocationLink();
    sendSMS("+250791207396", message);
    sendSMS("+250798077466", message);
}

// Modified printDebugInfo function with more details
void printDebugInfo(float birdProbability) {
    Serial.print("Max Bird detection probability: ");
    Serial.print(birdProbability);
    Serial.print(" | Threshold: ");
    Serial.print(BIRD_DETECTION_THRESHOLD);
    Serial.print(" | Repel active: ");
    Serial.print(repelActive);
    Serial.print(" | Repel time remaining: ");
    if (repelActive) {
        Serial.print((REPEL_DURATION - (millis() - repelStartTime)) / 1000);
        Serial.print(" seconds");
    } else {
        Serial.print("0 seconds");
    }
    Serial.print(" | Call received: ");
    Serial.print(callReceived);
    Serial.print(" | Memory: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");
}

String getLocationLink() {
    if (gps.location.isValid()) {
        return "http://maps.google.com/maps?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    } else {
        return "GPS location not available";
    }
}

void checkGSMData() {
    while (gsmSerial.available()) {
        char c = gsmSerial.read();
        incomingData += c;
        if (c == '\n') {
            incomingData.trim();
            Serial.println("Received: " + incomingData); // Debug print
            
            if (incomingData.startsWith("+CLIP:")) {
                // Incoming call detected
                String callerNumber = incomingData.substring(incomingData.indexOf('"') + 1);
                callerNumber = callerNumber.substring(0, callerNumber.indexOf('"'));
                Serial.println("Incoming call from: " + callerNumber);
                
                handleIncomingCall(callerNumber);
            } else if (incomingData.indexOf("NO CARRIER") != -1) {
                // Call ended
                callInProgress = false;
                Serial.println("Call ended");
            }
            
            incomingData = ""; // Clear the string for the next line
        }
    }
    
    // Check if we need to end the call due to timeout
    if (callInProgress && (millis() - callStartTime > CALL_TIMEOUT)) {
        gsmSerial.println("ATH"); // Hang up the call
        callInProgress = false;
        Serial.println("Call timed out and ended");
    }
}

// Update the isAuthorizedCaller function
bool isAuthorizedCaller(String number) {
    Serial.println("Checking authorization for number: " + number);
    
    // Remove any leading '+' or '0' from the number
    while (number.startsWith("+") || number.startsWith("0")) {
        number = number.substring(1);
    }
    
    // List of authorized numbers (without country code or leading zeros)
    const String authorizedNumbers[] = {"736410401", "726678112", "735551786", "798077466"};
    const int numAuthorized = sizeof(authorizedNumbers) / sizeof(authorizedNumbers[0]);
    
    for (int i = 0; i < numAuthorized; i++) {
        if (number.endsWith(authorizedNumbers[i])) {
            Serial.println("Authorized number found: " + authorizedNumbers[i]);
            return true;
        }
    }
    Serial.println("Number not authorized");
    return false;
}

// Call handler function
void handleIncomingCall(String callerNumber) {
    Serial.println("Handling call from: " + callerNumber);
    
    // Remove any leading '+' or '0' from the number
    while (callerNumber.startsWith("+") || callerNumber.startsWith("0")) {
        callerNumber = callerNumber.substring(1);
    }
    Serial.println("Processed number: " + callerNumber);
    
    if (isAuthorizedCaller(callerNumber)) {
        Serial.println("Authorized caller: " + callerNumber);
        activateRepel();
        callCount++;
        callInProgress = true;
        callStartTime = millis();
        // Don't answer the call, just let it ring
    } else {
        Serial.println("Unauthorized caller: " + callerNumber);
        gsmSerial.println("ATH"); // Reject the call
    }
}

// sendSMS function
bool sendSMS(String recipient, String message) {
  gsmSerial.print("AT+CMGF=1\r");
  delay(100);
  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(recipient);
  gsmSerial.print("\"\r");
  delay(100);
  gsmSerial.print(message);
  delay(100);
  gsmSerial.println((char)26);
  delay(1000);  // Wait for the message to be sent

  // Check for "OK" response
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {  // 10 second timeout
    if (gsmSerial.find("OK")) {
      Serial.println("SMS sent successfully to " + recipient);
      return true;
    }
  }
  Serial.println("Failed to send SMS to " + recipient);
  return false;
}

void ShowSerialData() {
  while (gsmSerial.available() != 0)
    Serial.write(gsmSerial.read());
  delay(500);
}

// GPS updates function
void updateGPS() {
    unsigned long start = millis();
    do {
        while (gpsSerial.available()) {
            char c = gpsSerial.read();
            Serial.write(c);  // Echo raw GPS data to serial monitor
            gps.encode(c);
        }
    } while (millis() - start < 1000);  // Parse for 1 second

    printGPSInfo();
}

// GPS information
void printGPSInfo() {
    Serial.println("GPS Debug Info:");
    if (gps.location.isValid()) {
        Serial.print("Latitude: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(" Longitude: ");
        Serial.println(gps.location.lng(), 6);
    } else {
        Serial.println("Location: INVALID");
    }

    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());

    Serial.print("HDOP: ");
    Serial.println(gps.hdop.hdop());

    if (gps.date.isValid()) {
        Serial.print("Date: ");
        Serial.print(gps.date.month());
        Serial.print("/");
        Serial.print(gps.date.day());
        Serial.print("/");
        Serial.println(gps.date.year());
    } else {
        Serial.println("Date: INVALID");
    }

    if (gps.time.isValid()) {
        Serial.print("Time: ");
        Serial.print(gps.time.hour());
        Serial.print(":");
        Serial.print(gps.time.minute());
        Serial.print(":");
        Serial.println(gps.time.second());
    } else {
        Serial.println("Time: INVALID");
    }

    Serial.println();
}

//Send Data to Thingspeak
void updateThingSpeak() {
    ThingSpeak.setField(1, static_cast<long>(detectionCount));
    ThingSpeak.setField(2, static_cast<long>(callCount));
    ThingSpeak.setField(3, static_cast<long>(callCount));

    int httpCode = ThingSpeak.writeFields(channelID, writeAPIKey);

    if (httpCode == 200) {
        Serial.println("ThingSpeak update successful");
    } else {
        Serial.println("ThingSpeak update failed. HTTP error code: " + String(httpCode));
    }
}


// Weevil detection functions
static void audio_inference_callback(uint32_t n_bytes) {
    for(int i = 0; i < n_bytes>>1; i++) {
        inference.buffer[inference.buf_count++] = sampleBuffer[i];
        if(inference.buf_count >= inference.n_samples) {
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

static void capture_samples(void* arg) {
    const int32_t i2s_bytes_to_read = (uint32_t)arg;
    size_t bytes_read = i2s_bytes_to_read;

    ei_printf("Starting audio capture...\n");

    while (record_status) {
        esp_err_t ret = i2s_read((i2s_port_t)I2S_NUM_0, (void*)sampleBuffer, i2s_bytes_to_read, &bytes_read, 100);
        if (ret != ESP_OK) {
            ei_printf("Error in I2S read: %d\n", ret);
            continue;
        }

        if (bytes_read <= 0) {
            ei_printf("Error in I2S read : %d\n", bytes_read);
        }
        else {
            if (bytes_read < i2s_bytes_to_read) {
                ei_printf("Partial I2S read: %d\n", bytes_read);
            }

            for (int x = 0; x < bytes_read/2; x++) {
                sampleBuffer[x] = (int16_t)(sampleBuffer[x]) * 4;
            }

            if (record_status) {
                audio_inference_callback(bytes_read);
            }
            else {
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

static bool microphone_inference_start(uint32_t n_samples) {
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));

    if(inference.buffer == NULL) {
        ei_printf("ERR: Failed to allocate audio buffer\n");
        return false;
    }

    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    ei_printf("Initializing I2S...\n");
    if (i2s_init(EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("Failed to start I2S!\n");
        return false;
    }

    ei_sleep(100);
    record_status = true;

    ei_printf("Creating capture task...\n");
    xTaskCreate(capture_samples, "CaptureSamples", 1024 * 8, (void*)sample_buffer_size, 10, NULL);

    return true;
}

static bool microphone_inference_record(void) {
    bool ret = true;
    while (inference.buf_ready == 0) {
        delay(10);
    }
    inference.buf_ready = 0;
    return ret;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
    return 0;
}

static int i2s_init(uint32_t sampling_rate) {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = sampling_rate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 1024,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    i2s_pin_config_t pin_config = {
        .mck_io_num = -1,
        .bck_io_num = -1,
        .ws_io_num = 42,
        .data_out_num = -1,
        .data_in_num = 41
    };
    
    esp_err_t ret = i2s_driver_install((i2s_port_t)I2S_NUM_0, &i2s_config, 0, NULL);
    if (ret != ESP_OK) {
        ei_printf("Error in i2s_driver_install: %d\n", ret);
        return int(ret);
    }

    ret = i2s_set_pin((i2s_port_t)I2S_NUM_0, &pin_config);
    if (ret != ESP_OK) {
        ei_printf("Error in i2s_set_pin: %d\n", ret);
        return int(ret);
    }

    ret = i2s_zero_dma_buffer((i2s_port_t)I2S_NUM_0);
    if (ret != ESP_OK) {
        ei_printf("Error in initializing dma buffer with 0: %d\n", ret);
        return int(ret);
    }

    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif