/* Includes ---------------------------------------------------------------- */
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#define EIDSP_QUANTIZE_FILTERBANK   0  // If your target is limited in memory remove this macro to save 10K RAM
#include <PDM.h>
#include <FarmMonitoring_2024_inferencing.h>

/** Audio buffers, pointers and selectors */
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
static signed short sampleBuffer[2048];
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

// Pin Definitions
const int TOUCH_PIN = 2;     // Pin for limit switch (box opened detection)

// Variable for box state
bool lastBoxState = false; // To track if we've already sent an alert for this open event

// GSM Module Serial
const int GSM_RX_PIN = 9;    // GSM module RX pin
const int GSM_TX_PIN = 10;   // GSM module TX pin
UART gsmSerial(GSM_RX_PIN, GSM_TX_PIN);  // Use hardware serial 1 for GSM module
const int GSM_BAUD = 9600;

// GPS Module Serial
TinyGPSPlus gps;
const int GPS_RX_PIN = D8;   // GPS module RX pin
const int GPS_TX_PIN = D7;   // GPS module TX pin
UART gpsSerial(GPS_RX_PIN, GPS_TX_PIN);  // Use hardware serial 2 for GPS module
const int GPS_BAUD = 9600;

// Bird detection variables
const float BIRD_DETECTION_THRESHOLD = 0.7;
int detectionCount = 0;
const int SMS_THRESHOLD = 10;
const String MAIN_NODE_NUMBER = "726678112"; // Replace with actual main node number
const String USER_NUMBER = "798077466"; // Replace with actual user number


/**
 * @brief      Arduino setup function
 */
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    // comment out the below line to cancel the wait for USB connection (needed for native USB)
    //while (!Serial);
    Serial.println("Pest Bird Repeller System");

    // Initialize GSM
    gsmSerial.begin(GSM_BAUD);
    if (!initGSM()) {
        Serial.println("Failed to initialize GSM module");
        //while (1);  // Halt if GSM initialization fails
    }

    // Initialize GPS
    gpsSerial.begin(GPS_BAUD);

    // Initialize GSM and GPS serial
    gsmSerial.begin(GSM_BAUD);
    gpsSerial.begin(GPS_BAUD);
    
    // Set pin modes
    pinMode(TOUCH_PIN, INPUT);  // Use INPUT_PULLUP for the limit switch
    
    // Set up GSM module for call detection
    gsmSerial.println("AT+CLIP=1"); // Enable Calling Line Identification Presentation
    delay(100);
    gsmSerial.println("AT+CMGF=1"); // Set SMS text mode
    delay(100);

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        return;
    }
}

/**
 * @brief      Arduino main function. Runs the inferencing loop.
 */
void loop()
{

    // Read the limit switch state
    bool boxIsOpen = digitalRead(TOUCH_PIN) == HIGH;

    if (boxIsOpen && !lastBoxState) {
        // Box has just been opened
        sendSecurityAlert();
        lastBoxState = true;
    } else if (!boxIsOpen) {
        // Box is closed, reset the state
        lastBoxState = false;
    }

    ei_printf("Starting inferencing in 2 seconds...\n");

    delay(2000);

    ei_printf("Recording...\n");

    bool m = microphone_inference_record();
    if (!m) {
        ei_printf("ERR: Failed to record audio...\n");
        return;
    }

    ei_printf("Recording done\n");

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", r);
        return;
    }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif


// Checking Specific class classification
 if (microphone_inference_record()) {
        signal_t signal;
        signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
        signal.get_data = &microphone_audio_signal_get_data;
        ei_impulse_result_t result = { 0 };

        EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
        if (r != EI_IMPULSE_OK) {
            Serial.println("ERR: Failed to run classifier");
            return;
        }

        // Check for bird detection
        float maxBirdProbability = 0.0;
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
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
        detectionCount++;
        Serial.println("Bird detected! Probability: " + String(maxBirdProbability));
        
        // Call main node to trigger repeller
        if (makeCall(MAIN_NODE_NUMBER)) {
            Serial.println("Call made successfully");
        } else {
            Serial.println("Failed to make call");
        }
        
        // Send SMS if detection count reaches threshold
        if (detectionCount % SMS_THRESHOLD == 0) {
            String location = getGPSLocation();
            String message = "Alert: " + String(detectionCount) + " bird detections. Location: " + location;
            if (sendSMS(USER_NUMBER, message)) {
                Serial.println("SMS sent successfully");
            } else {
                Serial.println("Failed to send SMS");
            }
        }
    }
 }

        // Update GPS data
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
     
}

bool initGSM(){
  Serial.println("Initializing GSM module...");
    
    // Reset the module
    gsmSerial.println("AT+CFUN=1,1");
    delay(20000); // Wait for the module to restart

    // Check if the module is responsive
    gsmSerial.println("AT");
    delay(1000);
    if (!gsmSerial.find("OK")) {
        Serial.println("Error: Module not responding");
        return false;
    }

    // Check signal quality
    gsmSerial.println("AT+CSQ");
    delay(1000);
    if (gsmSerial.find("+CSQ:")) {
        int signalQuality = gsmSerial.parseInt();
        Serial.print("Signal quality: ");
        Serial.println(signalQuality);
        if (signalQuality < 10 || signalQuality == 99) {
            Serial.println("Warning: Poor signal quality");
        }
    }

    // Check if the module is registered on the network
    gsmSerial.println("AT+CREG?");
    delay(1000);
    if (gsmSerial.find("+CREG: 0,1") || gsmSerial.find("+CREG: 0,5")) {
        Serial.println("Module registered on the network");
    } else {
        Serial.println("Error: Module not registered on the network");
        return false;
    }

    // Set SMS text mode
    gsmSerial.println("AT+CMGF=1");
    delay(1000);
    if (!gsmSerial.find("OK")) {
        Serial.println("Error: Failed to set SMS text mode");
        return false;
    }

    Serial.println("GSM module initialized successfully");
    return true;
}

bool makeCall(String number) {
    Serial.println("Making call to " + number);
    
    // Remove any leading '+' or '0' from the number
    while (number.startsWith("+") || number.startsWith("0")) {
        number = number.substring(1);
    }
    
    // Ensure the number has exactly 9 digits (adjust if needed)
    if (number.length() > 9) {
        number = number.substring(number.length() - 9);
    } else if (number.length() < 9) {
        Serial.println("Invalid number length");
        return false;
    }
    
    // Make the call
    gsmSerial.println("ATD" + number + ";");
    if (!waitForResponse("OK", 5000)) return false;
    
    delay(10000); // Wait for 20 seconds to allow the call to ring
    
    gsmSerial.println("ATH"); // Hang up
    return waitForResponse("OK", 5000);
}

bool sendSMS(String number, String message) {
    Serial.println("Sending SMS to " + number);
    gsmSerial.println("AT+CMGS=\"" + number + "\"");
    if (!waitForResponse(">", 5000)) return false;
    
    gsmSerial.print(message);
    gsmSerial.write(26); // Ctrl+Z to send message
    return waitForResponse("OK", 10000);
}

bool waitForResponse(String expected, unsigned long timeout) {
    String response = "";
    unsigned long start = millis();
    while (millis() - start < timeout) {
        if (gsmSerial.available()) {
            char c = gsmSerial.read();
            response += c;
            if (response.indexOf(expected) != -1) {
                Serial.println("Received: " + response);
                return true;
            }
        }
    }
    Serial.println("Timeout waiting for response. Received: " + response);
    return false;
}

void sendSecurityAlert() {
    String location = getGPSLocation();
    String timestamp = getTimestamp();
    String message = "ALERT: Box opened at " + timestamp + ". Location: " + location;
    
    if (sendSMS(USER_NUMBER, message)) {
        Serial.println("Security alert SMS sent successfully");
    } else {
        Serial.println("Failed to send security alert SMS");
    }
}


String getTimestamp() {
    // If you have a real-time clock module, use it here
    // For now, we'll use the millis() function as a simple timestamp
    unsigned long currentTime = millis();
    int seconds = currentTime / 1000;
    int minutes = seconds / 60;
    int hours = minutes / 60;
    
    String timestamp = String(hours) + ":" + String(minutes % 60) + ":" + String(seconds % 60);
    return timestamp;
}

String getGPSLocation() { //GPS Location function
    if (gps.location.isValid()) {
        return String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    } else {
        return "GPS location not available";
    }
}

/**
 * @brief      PDM buffer full callback
 *             Get data and call audio thread callback
 */
static void pdm_data_ready_inference_callback(void)
{
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

    if (inference.buf_ready == 0) {
        for(int i = 0; i < bytesRead>>1; i++) {
            inference.buffer[inference.buf_count++] = sampleBuffer[i];

            if(inference.buf_count >= inference.n_samples) {
                inference.buf_count = 0;
                inference.buf_ready = 1;
                break;
            }
        }
    }
}

/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));

    if(inference.buffer == NULL) {
        return false;
    }

    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    // configure the data receive callback
    PDM.onReceive(&pdm_data_ready_inference_callback);

    PDM.setBufferSize(4096);

    // initialize PDM with:
    // - one channel (mono mode)
    // - a 16 kHz sample rate
    if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("Failed to start PDM!");
        microphone_inference_end();

        return false;
    }

    // set the gain, defaults to 20
    PDM.setGain(127);

    return true;
}

/**
 * @brief      Wait on new data
 *
 * @return     True when finished
 */
static bool microphone_inference_record(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;

    while(inference.buf_ready == 0) {
        delay(10);
    }

    return true;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);

    return 0;
}

/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void)
{
    PDM.end();
    free(inference.buffer);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif
