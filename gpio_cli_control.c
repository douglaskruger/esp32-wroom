/* =============================================================================
   Copyright © 2025 Skynet Consulting Ltd.

   This file is the property of Skynet Consulting Ltd. and shall not be reproduced,
   copied, or used as the basis for the manufacture or sale of equipment without
   the expressed written permission of Skynet Consulting Ltd.
   =============================================================================
*/
static const char module_id[] __attribute__((used)) = "$Id: set_gpio.c $";

/*
 * Description:
 * The following program presents a CLI that allows the user to set the GPIO 
 * on/off/pulse and LED on the ESP32C - WROOM
*/
#include <Arduino.h>
#include <Preferences.h>

#define MAX_CMD_LEN 64
#define MAX_PWM_CHANNELS 8
#define LED_PIN 2   // onboard LED

char cmdBuffer[MAX_CMD_LEN];
int cmdIndex = 0;

Preferences prefs;

// Pulse forever state
int pulsePin = -1;
int pulseDuration = 500;
unsigned long lastToggle = 0;
bool pulseState = false;

// PWM management
struct PWMConfig {
    int pin;
    int channel;
    int freq;
    int duty;
    bool active;
};
PWMConfig pwmList[MAX_PWM_CHANNELS];

int allocatePWMChannel() {
    for (int i = 0; i < MAX_PWM_CHANNELS; i++) {
        if (!pwmList[i].active) {
            pwmList[i].active = true;
            pwmList[i].channel = i;
            return i;
        }
    }
    return -1;
}

int findPWMByPin(int pin) {
    for (int i = 0; i < MAX_PWM_CHANNELS; i++) {
        if (pwmList[i].active && pwmList[i].pin == pin) {
            return i;
        }
    }
    return -1;
}

void stopPWM(int pin) {
    int idx = findPWMByPin(pin);
    if (idx >= 0) {
        ledcWrite(pwmList[idx].channel, 0); // stop PWM
        pwmList[idx].active = false;
        Serial.printf("Stopped PWM on GPIO %d (channel %d)\n", pwmList[idx].pin, pwmList[idx].channel);
    }
}

void listStatus() {
    Serial.println("=== Active Outputs ===");
    if (pulsePin != -1) {
        Serial.printf("GPIO %d: PULSEFOREVER, %d ms per state\n", pulsePin, pulseDuration);
    }
    for (int i = 0; i < MAX_PWM_CHANNELS; i++) {
        if (pwmList[i].active) {
            Serial.printf("GPIO %d: PWM, %d Hz, %d%% duty (channel %d)\n",
                          pwmList[i].pin, pwmList[i].freq, pwmList[i].duty, pwmList[i].channel);
        }
    }
    Serial.printf("LED Pin %d: %s\n", LED_PIN, digitalRead(LED_PIN) ? "ON" : "OFF");
    Serial.println("======================");
}

void saveConfig() {
    prefs.begin("gpio-cli", false);
    prefs.putInt("pulsePin", pulsePin);
    prefs.putInt("pulseDuration", pulseDuration);

    prefs.putInt("pwmCount", MAX_PWM_CHANNELS);
    for (int i = 0; i < MAX_PWM_CHANNELS; i++) {
        String key = "pwm" + String(i);
        if (pwmList[i].active) {
            prefs.putInt((key + "_pin").c_str(), pwmList[i].pin);
            prefs.putInt((key + "_freq").c_str(), pwmList[i].freq);
            prefs.putInt((key + "_duty").c_str(), pwmList[i].duty);
            prefs.putBool((key + "_active").c_str(), true);
        } else {
            prefs.putBool((key + "_active").c_str(), false);
        }
    }
    prefs.end();
    Serial.println("Configuration saved to flash.");
}

void loadConfig() {
    prefs.begin("gpio-cli", true);

    pulsePin = prefs.getInt("pulsePin", -1);
    pulseDuration = prefs.getInt("pulseDuration", 500);
    if (pulsePin != -1) {
        pinMode(pulsePin, OUTPUT);
        pulseState = false;
        lastToggle = millis();
        Serial.printf("Restored PULSEFOREVER on GPIO %d, duration %d ms\n", pulsePin, pulseDuration);
    }

    int count = prefs.getInt("pwmCount", 0);
    for (int i = 0; i < count && i < MAX_PWM_CHANNELS; i++) {
        String key = "pwm" + String(i);
        bool active = prefs.getBool((key + "_active").c_str(), false);
        if (active) {
            int pin = prefs.getInt((key + "_pin").c_str(), -1);
            int freq = prefs.getInt((key + "_freq").c_str(), 1000);
            int duty = prefs.getInt((key + "_duty").c_str(), 50);

            int ch = allocatePWMChannel();
            if (ch >= 0 && pin >= 0) {
                pwmList[ch].pin = pin;
                pwmList[ch].freq = freq;
                pwmList[ch].duty = duty;

                //ledcSetup(ch, freq, 10);  // 10-bit resolution
                //ledcAttach(pin, ch);
                int dutyValue = (duty * 1023) / 100;
                ledcWrite(ch, dutyValue);

                Serial.printf("Restored PWM on GPIO %d: %d Hz, %d%% duty (channel %d)\n",
                              pin, freq, duty, ch);
            }
        }
    }
    prefs.end();
}

void processCommand(char *cmd) {
    int pin;
    char action[20];
    int arg1, arg2;

    int args = sscanf(cmd, "%d %s %d %d", &pin, action, &arg1, &arg2);

    if (strcasecmp(cmd, "LIST") == 0) { listStatus(); return; }
    if (strcasecmp(cmd, "SAVE") == 0) { saveConfig(); return; }
    if (strcasecmp(cmd, "LOAD") == 0) { loadConfig(); return; }

    if (args >= 2) {
        if (pin >= 0 && pin <= 39) pinMode(pin, OUTPUT);

        if (strcasecmp(action, "HIGH") == 0) {
            digitalWrite(pin, HIGH);
            Serial.printf("GPIO %d set HIGH\n", pin);
            return;
        }
        else if (strcasecmp(action, "LOW") == 0) {
            digitalWrite(pin, LOW);
            Serial.printf("GPIO %d set LOW\n", pin);
            return;
        }
        else if (strcasecmp(action, "PULSE") == 0) {
            int duration = (args >= 3) ? arg1 : 100;
            digitalWrite(pin, HIGH);
            delay(duration);
            digitalWrite(pin, LOW);
            Serial.printf("GPIO %d pulsed HIGH for %d ms\n", pin, duration);
            return;
        }
        else if (strcasecmp(action, "PULSEFOREVER") == 0) {
            int duration = (args >= 3) ? arg1 : 500;
            pulsePin = pin;
            pulseDuration = duration;
            lastToggle = millis();
            pulseState = false;
            Serial.printf("GPIO %d will now pulse forever, duration %d ms each state\n", pin, duration);
            return;
        }
        else if (strcasecmp(action, "STOP") == 0) {
            if (pulsePin == pin) {
                digitalWrite(pulsePin, LOW);
                pulsePin = -1;
                Serial.printf("Stopped pulsing GPIO %d\n", pin);
            }
            stopPWM(pin);
            return;
        }
        else if (strcasecmp(action, "PWM") == 0) {
            if (args < 4) {
                Serial.println("Usage: <pin> PWM <freq_hz> <duty%>");
                return;
            }
            int freq = arg1;
            int duty = arg2;
            if (duty < 0) duty = 0;
            if (duty > 100) duty = 100;

            int idx = findPWMByPin(pin);
            if (idx >= 0) stopPWM(pin);

            int ch = allocatePWMChannel();
            if (ch < 0) {
                Serial.println("Error: No free PWM channels available.");
                return;
            }

            pwmList[ch].pin = pin;
            pwmList[ch].freq = freq;
            pwmList[ch].duty = duty;

            //ledcSetup(ch, freq, 10);
            //ledcAttachPin(pin, ch);
            int dutyValue = (duty * 1023) / 100;
            ledcWrite(ch, dutyValue);

            Serial.printf("GPIO %d running PWM: %d Hz, %d%% duty (channel %d)\n",
                          pin, freq, duty, ch);
            return;
        }
    }

    // LED control commands
    if (strcasecmp(cmd, "LED ON") == 0) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("LED turned ON");
        return;
    }
    else if (strcasecmp(cmd, "LED OFF") == 0) {
        digitalWrite(LED_PIN, LOW);
        Serial.println("LED turned OFF");
        return;
    }

    Serial.println("Unknown command. Use: HIGH, LOW, PULSE, PULSEFOREVER, PWM, STOP, LIST, SAVE, LOAD, LED ON, LED OFF");
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    Serial.println("ESP32 GPIO CLI Ready (Auto-LOAD enabled).");

    for (int i = 0; i < MAX_PWM_CHANNELS; i++) {
        pwmList[i].active = false;
        pwmList[i].pin = -1;
        pwmList[i].channel = i;
        pwmList[i].freq = 0;
        pwmList[i].duty = 0;
    }

    loadConfig();  // auto-restore config on boot
}

void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdIndex > 0) {
                cmdBuffer[cmdIndex] = '\0';
                processCommand(cmdBuffer);
                cmdIndex = 0;
            }
        } else if (cmdIndex < MAX_CMD_LEN - 1) {
            cmdBuffer[cmdIndex++] = c;
        }
    }

    // Pulse forever
    if (pulsePin != -1) {
        unsigned long now = millis();
        if (now - lastToggle >= (unsigned long)pulseDuration) {
            pulseState = !pulseState;
            digitalWrite(pulsePin, pulseState ? HIGH : LOW);
            lastToggle = now;
        }
    }
}
