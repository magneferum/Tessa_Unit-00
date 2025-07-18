//***********************************************************************
// Electrical Monitoring System using Fuzzy Logic
// Refactored with a high-priority safety override for dangerous voltages.
//***********************************************************************

// --- Library Includes ---
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <time.h>
#include <SoftwareSerial.h>

// --- Firebase Includes ---
#define ENABLE_USER_AUTH
#define ENABLE_DATABASE
#include <FirebaseClient.h>
#include <WiFiClientSecure.h>

// --- Library Lokal ---
#include <PZEM004Tv30.h>
#include <StopWatch.h>
#include <fis_header.h>

// --- Sensitive Configuration ---
#include "secrets.h"

// --- Componenent and Object Inisialization ---
#if !defined(PZEM_RX_PIN) && !defined(PZEM_TX_PIN)
#define PZEM_RX_PIN 14 // D5
#define PZEM_TX_PIN 12 // D6
#endif
SoftwareSerial pzemSWSerial(PZEM_RX_PIN, PZEM_TX_PIN);
PZEM004Tv30 pzem(pzemSWSerial);

StopWatch disturbanceTimer(StopWatch::SECONDS);

// Firebase
UserAuth user_auth(WEB_API_KEY, USER_EMAIL, USER_PASS);
FirebaseApp app;
WiFiClientSecure ssl_client;
using AsyncClient = AsyncClientClass;
AsyncClient aClient(ssl_client);
RealtimeDatabase Database;

// JSON
object_t jsonData, obj1, obj2, obj3, obj4, obj5;
JsonWriter writer;

// --- Global Variable ---
// Fuzzy
const int FIS_INPUT_COUNT = 3;
FIS_TYPE g_fisInput[FIS_INPUT_COUNT];
FIS_TYPE g_fisOutput[1];

// Sensor Data (Raw, Without Clamp)
float raw_voltage = 0.0;
float raw_current_mA = 0.0;
unsigned long raw_disturbance_s = 0;

// Firebase
String uid;
String parentPath;
String voltPath = "/Voltage";
String ampPath = "/Current";
String outPath = "/Fuzzy Output";
String timePath = "/timestamp";
String disturbPath = "/DisturbanceTime";

// Hardware Pin
const int RED_PIN = 0;
const int BLUE_PIN = 4;
const int RELAY_PIN = 5;
const int BUTTON_PIN = 13;

// Timer (Using millis)
unsigned long lastSensorReadTime = 0;
unsigned long lastSerialLogTime = 0;
unsigned long lastFirebaseUploadTime = 0;
const long SENSOR_READ_INTERVAL = 1000;
const long SERIAL_LOG_INTERVAL = 5000;
const long FIREBASE_UPLOAD_INTERVAL = 10000;

// --- Function Declaration ---
void initWiFi();
void handleSensorReadings();
bool handleSafetyOverride(); // <-- FUNGSI BARU
void handleFuzzyLogic();
void handleButtonInput();
void handleDataUpload();
void handleOutput(float output);
int getStateCategory(float output);
unsigned long getTime();
void processData(AsyncResult &aResult);
float zeroIfNan(float v);
void fis_evaluate();
inline float clamp(float value, float minVal, float maxVal);

//***********************************************************************
// FIS Block Code 
//***********************************************************************
FIS_TYPE fis_trapmf(FIS_TYPE x, FIS_TYPE* p) 
{ 
    FIS_TYPE a = p[0], b = p[1], c = p[2], d = p[3]; 
    FIS_TYPE t1 = ((x <= c) ? 1 : ((d < x) ? 0 : ((c != d) ? ((d - x) / (d - c)) : 0))); 
    FIS_TYPE t2 = ((b <= x) ? 1 : ((x < a) ? 0 : ((a != b) ? ((x - a) / (b - a)) : 0))); 
    return (FIS_TYPE) min(t1, t2); 
}
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p) 
{ 
    FIS_TYPE a = p[0], b = p[1], c = p[2]; 
    if (x <= a || x >= c) return 0; 
    return (x <= b) ? (x - a)/(b - a) : (c - x)/(c - b); 
}
_FIS_MF fis_gMF[] = { fis_trapmf, fis_trimf };
int fis_gIMFCount[] = { 3, 2, 2 };
int fis_gOMFCount[] = { 6 };
FIS_TYPE fis_gMFI0Coeff1[] = { 100, 100, 198, 200 }; 
FIS_TYPE fis_gMFI0Coeff2[] = { 198, 220, 242 }; 
FIS_TYPE fis_gMFI0Coeff3[] = { 240, 242, 250, 250 }; 
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2,
fis_gMFI0Coeff3 };
FIS_TYPE fis_gMFI1Coeff1[] = { 0, 0, 70, 100 }; 
FIS_TYPE fis_gMFI1Coeff2[] = { 70, 100, 200, 200 }; 
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2 };
FIS_TYPE fis_gMFI2Coeff1[] = { 0, 0, 40, 60 }; 
FIS_TYPE fis_gMFI2Coeff2[] = { 50, 60, 120, 120 }; 
FIS_TYPE* fis_gMFI2Coeff[] = { fis_gMFI2Coeff1, fis_gMFI2Coeff2 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff, fis_gMFI2Coeff };
FIS_TYPE fis_gMFO0Coeff1[] = { 0, 0, 0, 0.4 }; 
FIS_TYPE fis_gMFO0Coeff2[] = { 0, 0, 0, 0.2 }; 
FIS_TYPE fis_gMFO0Coeff3[] = { 0, 0, 0, 1 }; 
FIS_TYPE fis_gMFO0Coeff4[] = { 0, 0, 0, 0 }; 
FIS_TYPE fis_gMFO0Coeff5[] = { 0, 0, 0, 0.6 }; 
FIS_TYPE fis_gMFO0Coeff6[] = { 0, 0, 0, 0.8 }; 
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3, fis_gMFO0Coeff4, fis_gMFO0Coeff5, fis_gMFO0Coeff6 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff };
int fis_gMFI0[] = { 0, 1, 0 }; 
int fis_gMFI1[] = { 0, 0 }; int fis_gMFI2[] = { 0, 0 }; 
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1, fis_gMFI2};
int* fis_gMFO[] = {};
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
int fis_gRI0[] = { 1, 1, 2 }; 
int fis_gRI1[] = { 1, 2, 2 }; 
int fis_gRI2[] = { 1, 1, 1 }; 
int fis_gRI3[] = { 1, 2, 1 }; 
int fis_gRI4[] = { 3, 1, 2 }; 
int fis_gRI5[] = { 3, 2, 2 }; 
int fis_gRI6[] = { 3, 1, 1 }; 
int fis_gRI7[] = { 3, 2, 1 }; 
int fis_gRI8[] = { 2, 1, 2 }; 
int fis_gRI9[] = { 2, 1, 1 }; 
int fis_gRI10[] = { 2, 2, 1 }; 
int fis_gRI11[] = { 2, 2, 2 }; 
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7, fis_gRI8, fis_gRI9, fis_gRI10, fis_gRI11 };
int fis_gRO0[] = { 1 }; int fis_gRO1[] = { 1 }; 
int fis_gRO2[] = { 2 }; int fis_gRO3[] = { 2 }; 
int fis_gRO4[] = { 3 }; int fis_gRO5[] = { 3 }; 
int fis_gRO6[] = { 6 }; int fis_gRO7[] = { 6 }; 
int fis_gRO8[] = { 4 }; int fis_gRO9[] = { 5 }; 
int fis_gRO10[] = { 5 }; int fis_gRO11[] = { 5 }; 
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7, fis_gRO8, fis_gRO9, fis_gRO10, fis_gRO11 };
FIS_TYPE fis_gIMin[] = { 100, 0, 0 };
FIS_TYPE fis_gIMax[] = { 250, 200, 120 };
FIS_TYPE fis_gOMin[] = { 0 };
FIS_TYPE fis_gOMax[] = { 1 };
inline float clamp(float value, float minVal, float maxVal) 
{
     if (value < minVal) return minVal; if (value > maxVal) return maxVal; return value; 
}
void fis_evaluate() 
{ 
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0 }; 
    FIS_TYPE fuzzyInput1[] = { 0, 0 }; 
    FIS_TYPE fuzzyInput2[] = { 0, 0 }; 
    FIS_TYPE* fuzzyInput[3] = { fuzzyInput0, fuzzyInput1, fuzzyInput2, }; 
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0, 0, 0, 0 }; 
    FIS_TYPE* fuzzyOutput[1] = { fuzzyOutput0, }; 
    FIS_TYPE fuzzyRules[12] = { 0 }; 
    FIS_TYPE fuzzyFires[12] = { 0 }; 
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires }; 
    FIS_TYPE sW = 0; 
    int i, j, r, o; 
        for (i = 0; i < 3; ++i) 
        { 
            for (j = 0; j < fis_gIMFCount[i]; ++j) 
            { 
                fuzzyInput[i][j] = (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]); 
            } 
        } int index = 0; for (r = 0; r < 12; ++r) 
        { 
            if (fis_gRType[r] == 1) { fuzzyFires[r] = 1; 
                for (i = 0; i < 3; ++i) 
                { 
                    index = fis_gRI[r][i]; 
                    if (index > 0) fuzzyFires[r] = min(fuzzyFires[r], fuzzyInput[i][index - 1]); 
                    else if (index < 0) fuzzyFires[r] = min(fuzzyFires[r], (FIS_TYPE)(1.0 - fuzzyInput[i][-index - 1])); 
                    else fuzzyFires[r] = min(fuzzyFires[r], (FIS_TYPE)1.0); 
                    } 
                } 
                else 
                { 
                    fuzzyFires[r] = 0; for (i = 0; i < 3; ++i) 
                    { 
                        index = fis_gRI[r][i]; if (index > 0) fuzzyFires[r] = max(fuzzyFires[r], fuzzyInput[i][index - 1]); 
                        else if (index < 0) fuzzyFires[r] = max(fuzzyFires[r], (FIS_TYPE)(1.0 - fuzzyInput[i][-index - 1])); 
                        else fuzzyFires[r] = max(fuzzyFires[r], (FIS_TYPE)0.0); } } fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r]; sW += fuzzyFires[r]; } if (sW == 0) 
                        { 
                            for (o = 0; o < 1; ++o) { g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2); 
                            } 
                        } else 
                        { for (o = 0; o < 1; ++o) { FIS_TYPE sWI = 0.0; 
                            for (j = 0; j < fis_gOMFCount[o]; ++j) { fuzzyOutput[o][j] = fis_gMFOCoeff[o][j][3]; 
                                for (i = 0; i < 3; ++i) { fuzzyOutput[o][j] += g_fisInput[i] * fis_gMFOCoeff[o][j][i]; 
                                } 
                            }
                             for (r = 0; r < 12; ++r) { index = fis_gRO[r][o] - 1; sWI += fuzzyFires[r] * fuzzyOutput[o][index]; } g_fisOutput[o] = sWI / sW; 
                            } 
                        }
                        g_fisOutput[0] = clamp(g_fisOutput[0], fis_gOMin[0], fis_gOMax[0]); 
                    }

//***********************************************************************
// SETUP: Once per program execution
//***********************************************************************
void setup() {
    Serial.begin(115200);
    pinMode(RED_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    initWiFi();
    configTime(0, 0, "id.pool.ntp.org");
    ssl_client.setInsecure();
    ssl_client.setTimeout(1000);
    ssl_client.setBufferSizes(4096, 1024);
    initializeApp(aClient, app, getAuth(user_auth), processData, "🔐 authTask");
    app.getApp<RealtimeDatabase>(Database);
    Database.url(DATABASE_URL);
}

//***********************************************************************
// LOOP: Continously Running
//***********************************************************************
void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Koneksi WiFi terputus. Mencoba menyambung kembali...");
        initWiFi();
        return;
    }

    app.loop();

    handleSensorReadings();

    // --- For Device Safety ---
    bool safetyTriggered = handleSafetyOverride();
    
    // If safety is triggered, stop the app and turn off the relay
    if (!safetyTriggered) {
        handleFuzzyLogic();
    }

    handleButtonInput();
    handleDataUpload();
}

//***********************************************************************
// Helper Function
//***********************************************************************

/**
 * @brief Read sensor data and update global variable.
 */
void handleSensorReadings() {
    if (millis() - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
        lastSensorReadTime = millis();

        raw_voltage = pzem.voltage();
        float current_A = pzem.current();

        raw_voltage = zeroIfNan(raw_voltage);
        current_A = zeroIfNan(current_A);

        raw_current_mA = current_A * 1000;
    }
}

/**
* @brief Check if safety override is triggered.
 * Check if dangerous out of fuzzy range.
 * @return True if safety is triggered, false otherwise.
 */
bool handleSafetyOverride() {
    // Periksa apakah tegangan mentah berada di luar rentang aman
    if (raw_voltage > fis_gIMax[0] || (raw_voltage < fis_gIMin[0] && raw_voltage > 0)) {
        // Push output to "Dangerous Voltage" (Value > 1.90)
        handleOutput(2.0); 
        
        // Log print if voltage is out of range
        if (millis() - lastSerialLogTime >= SERIAL_LOG_INTERVAL) {
            lastSerialLogTime = millis();
            Serial.print("SAFETY OVERRIDE! Dangerous Voltage Detected: ");
            Serial.println(raw_voltage);
        }
        return true; // safety is triggered
    }
    return false; // Safe condition, continue to fuzzy
}


/**
 * @brief Run fuzzy logic and log print.
 */
void handleFuzzyLogic() {
    // Input clamped value to fuzzy range
    g_fisInput[0] = clamp(raw_voltage, fis_gIMin[0], fis_gIMax[0]);
    g_fisInput[1] = clamp(raw_current_mA, fis_gIMin[1], fis_gIMax[1]);

    raw_disturbance_s = disturbanceTimer.elapsed();
    g_fisInput[2] = clamp(raw_disturbance_s, fis_gIMin[2], fis_gIMax[2]);

    if (g_fisInput[0] <= 198.00 || g_fisInput[0] >= 242.00 || g_fisInput[1] <= 70.00) {
        disturbanceTimer.start();
    } else {
        disturbanceTimer.reset(); // Using reset for disturbances measurement
    }

    fis_evaluate();
    handleOutput(g_fisOutput[0]);

    if (millis() - lastSerialLogTime >= SERIAL_LOG_INTERVAL) {
        lastSerialLogTime = millis();
        Serial.print("Fuzzy Logic -> V: "); Serial.print(g_fisInput[0]);
        Serial.print(" | C: "); Serial.print(g_fisInput[1]);
        Serial.print(" | T: "); Serial.print(g_fisInput[2]);
        Serial.print(" | Out: "); Serial.println(g_fisOutput[0]);
    }
}

/**
 * @brief Check physical input and reset timer and relay.
 */
void handleButtonInput() {
    if (digitalRead(BUTTON_PIN) == HIGH) {
        delay(100);
        digitalWrite(RELAY_PIN, HIGH);
        disturbanceTimer.reset();
        Serial.println("Timer gangguan direset oleh tombol.");
    }
}

/**
 * @brief Continously send data to firebase.
 */
void handleDataUpload() {
    if (app.ready() && (millis() - lastFirebaseUploadTime >= FIREBASE_UPLOAD_INTERVAL)) {
        lastFirebaseUploadTime = millis();
        uid = app.getUid().c_str();
        int current_timestamp = getTime();

        if (current_timestamp == 0) {
            Serial.println("Gagal mendapatkan waktu, pengiriman data dibatalkan.");
            return;
        }

        parentPath = "/UsersData/" + uid + "/readings/" + String(current_timestamp);

        writer.create(obj1, voltPath, raw_voltage); // Send raw voltage data
        writer.create(obj2, ampPath, raw_current_mA); // Send raw current data
        writer.create(obj3, outPath, g_fisOutput[0]);
        writer.create(obj4, timePath, current_timestamp);
        writer.create(obj5, disturbPath, raw_disturbance_s); // Send raw disturbance data
        writer.join(jsonData, 5, obj1, obj2, obj3, obj4, obj5);

        Database.set<object_t>(aClient, parentPath, jsonData, processData, "RTDB_Send_Data");
        Serial.println("Data berhasil dikirim ke Firebase.");
    }
}

/**
 * @brief Output pin control (LED & Relay) base on fuzzy output.
 */
void handleOutput(float output) {
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(RED_PIN, LOW);
    digitalWrite(BLUE_PIN, LOW);

    int category = getStateCategory(output);

    switch (category) {
        case 1: // Overvoltage
        case 4: // Undervoltage
        case 6: // Standby
        case 7: // Dangerous
            digitalWrite(RELAY_PIN, LOW);
            digitalWrite(RED_PIN, HIGH);
            break;
        case 2: // Voltage Swell
        case 5: // Voltage Sag
            digitalWrite(BLUE_PIN, HIGH);
            break;
        case 3: // Normal
            break;
    }
}

/**
 * @brief Translating fuzzy output to state category.
 */
int getStateCategory(float output) {
    if (output >= 0.90 && output < 1.90) return 1;
    else if (output >= 0.70 && output < 0.90) return 2;
    else if (output >= 0.50 && output < 0.70) return 3;
    else if (output >= 0.30 && output < 0.50) return 4;
    else if (output >= 0.10 && output < 0.30) return 5;
    else if (output <= 0.10) return 6;
    else return 7; // Dangerous Voltage
}

/**
 * @brief Wi-fi connection setup.
 */
void initWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Menyambungkan ke WiFi ..");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print('.');
        delay(1000);
    }
    Serial.println("\nWiFi tersambung.");
}

/**
 * @brief Get the current time from NTP server.
 */
unsigned long getTime() {
    time_t now;
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Gagal mendapatkan waktu dari NTP");
        return 0;
    }
    time(&now);
    return now;
}

/**
 * @brief Handle NaN (Not a Number) value.
 */
float zeroIfNan(float v) {
    if (isnan(v)) {
        return 0;
    }
    return v;
}

/**
 * @brief Callback for handling Firebase output value.
 */
void processData(AsyncResult &aResult) {
    if (aResult.isError())
        Serial.printf("Error Firebase: %s\n", aResult.error().message().c_str());
}
