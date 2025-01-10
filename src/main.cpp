// Inputs   // TODO : add Loadcell
#define ADC_PT_CH1 39
#define ADC_PT_CH2 34
#define ADC_PT_CH3 35
#define ADC_TC_CH1 25
#define ADC_TC_CH2 26
#define ADC_TC_CH3 27

// Outpputs
#define OUT_SV_CH1 14
#define OUT_SV_CH2 12
#define OUT_IG     4

#include <Arduino.h>
#include <ArduinoJson.h>

String buf_serial = "";
uint32_t prev_ms_log = 0;
uint8_t mode = 0;   // 0 : idle, 1 : sequence       // TODO : design state machine for safety

float tc_raw2phy(int adcValue) {
    float voltage = (adcValue / 4096.0) * 3.3;
    return (voltage - 1.25) / 0.005;    // (5mV/°C)
}

float pt_raw2phy(int adcValue, int range) {
    float voltage = (adcValue / 4096.0) * 3.3;
    return voltage; // TODO : range conversion implementation
}

void processSerialInput(String input) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, input);

    if (!error)
    {
        if (doc["seq"].is<int>()) {
            int cmdCali = doc["cmd_cali"];
            if (cmdCali) {  // Sequence Start
                Serial.println("Sequence Start.");
            }
            else {          // Sequence Stop
                Serial.println("Sequence Stop.");
            }
        }
        else {
            Serial.println("Unknown Command");
        }
    }
    else {
        Serial.print("Invalid JSON.");
        Serial.print(error.f_str());
        Serial.println();
    }

    buf_serial = "";
}

void setup()
{
    Serial.begin(115200);

    pinMode(ADC_PT_CH1, INPUT);
    pinMode(ADC_PT_CH2, INPUT);
    pinMode(ADC_PT_CH3, INPUT);
    pinMode(ADC_TC_CH1, INPUT);
    pinMode(ADC_TC_CH2, INPUT);
    pinMode(ADC_TC_CH3, INPUT);
}

void loop()
{
    while (Serial.available()) {
        char incomingChar = Serial.read();
        if (incomingChar == '\n') {
            processSerialInput(buf_serial);
        }
        else {
            buf_serial += incomingChar;
        }
    }

    uint32_t curr_ms = millis();
    if (curr_ms - prev_ms_log >= 10) {
        prev_ms_log = curr_ms;
        JsonDocument doc;

        // 1. Time
        doc["time"] = millis();

        // 2. State
        doc["state"] = mode;

        // 3. Input
        int adc_pt1 = analogRead(ADC_PT_CH1);
        int adc_pt2 = analogRead(ADC_PT_CH2);
        int adc_pt3 = analogRead(ADC_PT_CH3);
        int adc_tc1 = analogRead(ADC_TC_CH1);
        int adc_tc2 = analogRead(ADC_TC_CH2);
        int adc_tc3 = analogRead(ADC_TC_CH3);

        doc["pt1_raw"] = adc_pt1;
        doc["pt2_raw"] = adc_pt2;
        doc["pt3_raw"] = adc_pt3;
        doc["tc1_raw"] = adc_tc1;
        doc["tc2_raw"] = adc_tc2;
        doc["tc3_raw"] = adc_tc3;

        doc["pt1"] = pt_raw2phy(adc_pt1, 100);
        doc["pt2"] = pt_raw2phy(adc_pt2, 100);
        doc["pt3"] = pt_raw2phy(adc_pt3, 100);
        doc["tc1"] = tc_raw2phy(adc_tc1);
        doc["tc2"] = tc_raw2phy(adc_tc2);
        doc["tc3"] = tc_raw2phy(adc_tc3);

        // 4. Output
        doc["sv1"] = digitalRead(OUT_SV_CH1);
        doc["sv2"] = digitalRead(OUT_SV_CH2);
        doc["ig"]  = digitalRead(OUT_IG);

        // Final : 
        serializeJson(doc, Serial);
        Serial.println();
    }
}
