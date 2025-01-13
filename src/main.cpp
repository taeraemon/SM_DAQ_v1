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
uint32_t prev_ms_seq = 0;
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
            int cmdseq = doc["seq"];
            if ((mode == 0) && (cmdseq == 1)) {         // Sequence Start
                Serial.println("Sequence Start.");
                mode = 1;
                prev_ms_seq = millis();
            }
            else if ((mode == 1) && (cmdseq == 0)) {    // Sequence Stop
                Serial.println("Sequence Stop.");
                mode = 0;
                digitalWrite(OUT_SV_CH1, LOW);
                digitalWrite(OUT_SV_CH2, LOW);
                digitalWrite(OUT_IG, LOW);
            }
            else {
                Serial.println("Not available.");
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
    Serial.begin(921600);

    pinMode(ADC_PT_CH1, INPUT);
    pinMode(ADC_PT_CH2, INPUT);
    pinMode(ADC_PT_CH3, INPUT);
    pinMode(ADC_TC_CH1, INPUT);
    pinMode(ADC_TC_CH2, INPUT);
    pinMode(ADC_TC_CH3, INPUT);

    pinMode(OUT_SV_CH1, OUTPUT);
    pinMode(OUT_SV_CH2, OUTPUT);
    pinMode(OUT_IG,     OUTPUT);
    digitalWrite(OUT_SV_CH1, LOW);
    digitalWrite(OUT_SV_CH2, LOW);
    digitalWrite(OUT_IG,     LOW);
}

void loop()
{
    if (mode) {
        // TODO : Sequence Implement, only for Example
        uint8_t cmd_out_sv_ch1 = LOW;
        uint8_t cmd_out_sv_ch2 = LOW;
        uint8_t cmd_out_ig     = LOW;
        uint8_t cmd_mode       = 1;

        if (millis() - prev_ms_seq >= 1000) {
            cmd_out_ig = HIGH;
        }

        if (millis() - prev_ms_seq >= 2500) {
            cmd_out_ig = LOW;
        }

        if (millis() - prev_ms_seq >= 5000) {
            cmd_mode = 0;
        }

        digitalWrite(OUT_SV_CH1, cmd_out_sv_ch1);
        digitalWrite(OUT_SV_CH2, cmd_out_sv_ch2);
        digitalWrite(OUT_IG,     cmd_out_ig);
        mode = cmd_mode;
    }

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
        doc["time_seq"] = mode * (millis() - prev_ms_seq);

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
