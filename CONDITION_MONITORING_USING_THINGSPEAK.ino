#include <WiFi.h>
#include <ThingSpeak.h>

// Wi-Fi and ThingSpeak Configuration
const char* ssid = "Mchinary-Lab";   // Your Wi-Fi SSID
const char* password = "1234doeee";  // Your Wi-Fi Password
WiFiClient client;

unsigned long Channel_ID = 2507153;  // ThingSpeak Channel ID
const char* API_key = "QUP5EGV9AXSRXQXE"; // ThingSpeak Write API Key

// Timing and Delay
const unsigned long DELAY_INTERVAL = 10000; // 10 seconds
const unsigned long TEMPERATURE_SAMPLE_INTERVAL = 1000; // 1 second

// Temperature Sensor Pins
int tempSensorPins[] = {34, 36, 39};

// Current Sensor Pins
int currentSensorPins[] = {32, 33, 35};

long lastSample[3] = {0, 0, 0};
long sampleSum[3] = {0, 0, 0};
long sampleCount[3] = {0, 0, 0};

const float vpc = (3.3 / 4095) * 1000;

// Sensor Data Storage
float temperatureData[3];
float currentData[3];

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    ThingSpeak.begin(client);

    for (int i = 0; i < 3; i++) {
        pinMode(tempSensorPins[i], INPUT_PULLDOWN);
        pinMode(currentSensorPins[i], INPUT_PULLDOWN);
    }

    xTaskCreatePinnedToCore(readTemperatureSensor1, "ReadTemperatureSensor1", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(readTemperatureSensor2, "ReadTemperatureSensor2", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(readTemperatureSensor3, "ReadTemperatureSensor3", 2048, NULL, 1, NULL, 1);

    xTaskCreatePinnedToCore(readCurrentSensor1, "ReadCurrentSensor1", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(readCurrentSensor2, "ReadCurrentSensor2", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(readCurrentSensor3, "ReadCurrentSensor3", 2048, NULL, 1, NULL, 1);

    xTaskCreatePinnedToCore(uploadData, "UploadData", 4096, NULL, 1, NULL, 1);
}

void loop() {
    // Nothing here, everything is handled by FreeRTOS tasks
}

void readTemperatureSensor1(void * parameter) {
    for (;;) {
        temperatureData[0] = readTemperature(tempSensorPins[0]);
        Serial.print("Temperature sensor 1 - Average: ");
        Serial.println(temperatureData[0]);
        vTaskDelay(TEMPERATURE_SAMPLE_INTERVAL / portTICK_PERIOD_MS);
    }
}

void readTemperatureSensor2(void * parameter) {
    for (;;) {
        temperatureData[1] = readTemperature(tempSensorPins[1]);
         temperatureData[1] = temperatureData[1] - 3;
        Serial.print("Temperature sensor 2 - Average: ");
        Serial.println(temperatureData[1]);
        vTaskDelay(TEMPERATURE_SAMPLE_INTERVAL / portTICK_PERIOD_MS);
    }
}

void readTemperatureSensor3(void * parameter) {
    for (;;) {
        temperatureData[2] = readTemperature(tempSensorPins[2]);
        Serial.print("Temperature sensor 3 - Average: ");
        Serial.println(temperatureData[2]);
        vTaskDelay(TEMPERATURE_SAMPLE_INTERVAL / portTICK_PERIOD_MS);
    }
}

void readCurrentSensor1(void * parameter) {
    int index = 0;
    for (;;) {
        if (millis() - lastSample[index] > 2) {
            float s = analogRead(currentSensorPins[index]) - 2990;
            float y = s * vpc;
            sampleSum[index] += y * y;
            sampleCount[index]++;
            lastSample[index] = millis();
        }
        if (sampleCount[index] == 1000) {
            calculateCurrent(index);
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}

void readCurrentSensor2(void * parameter) {
    int index = 1;
    for (;;) {
        if (millis() - lastSample[index] > 2) {
            float s = analogRead(currentSensorPins[index]) - 2990;
            float y = s * vpc;
            sampleSum[index] += y * y;
            sampleCount[index]++;
            lastSample[index] = millis();
        }
        if (sampleCount[index] == 1000) {
            calculateCurrent(index);
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}

void readCurrentSensor3(void * parameter) {
    int index = 2;
    for (;;) {
        if (millis() - lastSample[index] > 2) {
            float s = analogRead(currentSensorPins[index]) - 2990;
            float y = s * vpc;
            sampleSum[index] += y * y;
            sampleCount[index]++;
            lastSample[index] = millis();
        }
        if (sampleCount[index] == 1000) {
            calculateCurrent(index);
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}

void calculateCurrent(int index) {
    float mean = sampleSum[index] / sampleCount[index];
    float rms = sqrt(mean);
    float mV = rms / 27;
    // println(mv);
    float y1 = (-0.02515 * mV * mV) + (1.546 * mV) - 0.8228;
        if (y1 <= 1) {
            y1 = 0;
        } else {
            y1 -= 1;
        }
    sampleSum[index] = 0;
    sampleCount[index] = 0;
    currentData[index] = y1;
}

void uploadData(void * parameter) {
    for (;;) {
        if (WiFi.status() != WL_CONNECTED) {
            unsigned long start = millis();
            while (WiFi.status() != WL_CONNECTED && (millis() - start) < 20000) {
                WiFi.begin(ssid, password);
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("Wi-Fi connected.");
            } else {
                Serial.println("Failed to connect to Wi-Fi.");
                vTaskDelay(10000 / portTICK_PERIOD_MS);
                continue;
            }
        }

        int maxRetries = 3;
        int retryCount = 0;
        int responseCode = -1;

        while (retryCount < maxRetries && responseCode != 200) {
            for (int i = 0; i < 3; i++) {
                ThingSpeak.setField(i + 1, temperatureData[i]);
                ThingSpeak.setField(i + 4, currentData[i]);
            }

            responseCode = ThingSpeak.writeFields(Channel_ID, API_key);

            if (responseCode == 200) {
                Serial.println("Data successfully sent to ThingSpeak.");
            } else {
                Serial.println("Failed to send data to ThingSpeak. Response code: " + String(responseCode));
                retryCount++;
                vTaskDelay(5000 / portTICK_PERIOD_MS);
            }
        }

        if (retryCount == maxRetries) {
            Serial.println("Failed to send data to ThingSpeak after multiple attempts.");
        }

        vTaskDelay(DELAY_INTERVAL / portTICK_PERIOD_MS);
    }
}

float readTemperature(int pin) {
    float totalTemp = 0;
    int numReadings = 300;

    for (int i = 0; i < numReadings; i++) {
        int rawData = analogRead(pin);
        float temp = 0.266 * rawData - 347;
        totalTemp += temp;
        delay(1);  // Shorter delay to allow task switching
    }

    float avgTemp = totalTemp / numReadings;
    return avgTemp;
}
