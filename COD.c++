#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


//---------------------------------------------------------------Initializare---------------------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pini hardware
const int buzzerPin = 8,
    potPin = A0,
    btn1Pin = A1,
    btn2Pin = A2;
const int latchPin = 10,
    clockPin = 9,
    dataPin = 11;
const int led1Pin = 3, led2Pin = 4;

// Pattern-uri 7-segmente
byte digits[] = {
    0b11111010, // 0
    0b01100000, // 1
    0b11011100, // 2
    0b11110100, // 3
    0b01100110, // 4
    0b10110110, // 5
    0b10111110, // 6
    0b11100000, // 7
    0b11111110, // 8
    0b11100110  // 9
};

// Stări și variabile joc
enum GameState {
    IDLE,
    PLAYING,
};
GameState state = IDLE;
enum Action {
    BTN1,
    BTN2,
    POT,
    NUM_ACTIONS
};

String actions[] = {
    "Apasa Buton 1!",
    "Apasa Buton 2!",
    "Roteste!"
};

int score = 0, currentAction = 0;
unsigned long startTime = 0;
bool isQuickMode = false, potPassed = false;
unsigned long quickModeStartTime = 0; // Timpul pentru modul rapid

const int TIME_LIMIT = 5000,
    QUICK_TIME = 10000;
const int POT_HIGH = 800,
    POT_LOW = 200,
    DEBOUNCE = 50;
int potInitialValue = 0;


//---------------------------------------------------------------Funcții---------------------------------------------------------------
void setDisplay(byte code) {
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, code);
    digitalWrite(latchPin, HIGH);
}

void showNumber(int num) {
    if (num >= 0 && num <= 9) {
        setDisplay(digits[num]);
    } else {
        setDisplay(0);
    }
}

bool buttonPressed(int pin) {
    static unsigned long lastTime[2] = {
        0,
        0
    };
    static bool lastState[2] = {
        HIGH,
        HIGH
    }, pressed[2] = {
        false,
        false
    };

    int idx;
    if (pin == btn1Pin) {
        idx = 0;
    } else {
        idx = 1;
    }
    
    bool reading = digitalRead(pin);

    if (reading != lastState[idx]) lastTime[idx] = millis();

    if (millis() - lastTime[idx] > DEBOUNCE && reading == LOW && !pressed[idx]) {
        pressed[idx] = true;
        lastState[idx] = reading;
        return true;
    }

    if (reading == HIGH) pressed[idx] = false;
    lastState[idx] = reading;
    return false;
}

bool checkAction(int action) {
    switch (action) {
    case BTN1:
        return buttonPressed(btn1Pin);
    case BTN2:
        return buttonPressed(btn2Pin);
    case POT: {
        int val = analogRead(potPin);

        if (!potPassed && (val < 100 || val > 900)) {
            potPassed = true;
            Serial.print("Pot val: ");
            Serial.println(val);
        }

        if (potPassed) {
            if ((potInitialValue < 512 && val > 900) ||
                (potInitialValue > 512 && val < 100)) {
                Serial.print("Rotire completă: ");
                Serial.print(potInitialValue);
                Serial.print("Final: ");
                Serial.println(val);
                return true;
            }
        }
        return false;
    }
    }
    return false;
}

bool wrongAction(int correctAction) {
    for (int i = 0; i < NUM_ACTIONS; i++) {
        if (i != correctAction && checkAction(i))
            return true;
    }
    return false;
}

void showGameOver(String msg) {
    lcd.clear();
    lcd.print(msg);
    lcd.setCursor(0, 1);
    lcd.print("Scor: ");
    lcd.print(score);
    setDisplay(0);
    tone(buzzerPin, 250, 500);
    delay(2000);
    state = IDLE;
}

void newAction() {
    currentAction = random(NUM_ACTIONS);
    potPassed = false;

    // Salvează valoarea inițială a potențiometrului
    potInitialValue = analogRead(potPin);

    lcd.clear();
    lcd.print(actions[currentAction]);
    lcd.setCursor(0, 1);
    lcd.print("Scor: ");
    lcd.print(score);

    // In modul normal -> resetează timpul
    if (!isQuickMode) {
        startTime = millis();
    }

    Serial.print("Noua actiune: ");
    Serial.print(actions[currentAction]);
}

void animation() {
    int melody[] = { 523, 659, 784, 659, 523, 784, 659, 523, 392, 440, 392, 330, 440, 392, 330, 262 };
    int duration[] = { 120, 120, 120, 120, 200, 200, 120, 120, 120, 200, 120, 120, 200, 120, 120, 200 };
    for (int i = 0; i < 16; i++) {
        if (i % 2 == 0) {
            digitalWrite(led1Pin, HIGH);
            digitalWrite(led2Pin, LOW);
        } else {
            digitalWrite(led1Pin, LOW);
            digitalWrite(led2Pin, HIGH);
        }
        tone(buzzerPin, melody[i], duration[i]);
        delay(duration[i] + 30);
    }
    digitalWrite(led1Pin, LOW);
    digitalWrite(led2Pin, LOW);
}


//---------------------------------------------------------------Main---------------------------------------------------------------
void setup() {
    Serial.begin(9600);
    lcd.init();
    lcd.backlight();
    lcd.print("Alege mod:");
    lcd.setCursor(0, 1);
    lcd.print("B1:Norm B2:Rapid");

    pinMode(buzzerPin, OUTPUT);
    pinMode(btn1Pin, INPUT_PULLUP);
    pinMode(btn2Pin, INPUT_PULLUP);
    pinMode(latchPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, OUTPUT);
    pinMode(led1Pin, OUTPUT);
    pinMode(led2Pin, OUTPUT);

    setDisplay(0);
    randomSeed(analogRead(A4));

    // Salvează valoarea inițială a potențiometrului
    potInitialValue = analogRead(potPin);
}

void loop() {
    unsigned long elapsed, remaining;

    if (isQuickMode) {
        elapsed = millis() - quickModeStartTime;
        remaining = (QUICK_TIME - elapsed) / 1000;
    } else {
        elapsed = millis() - startTime;
        remaining = (TIME_LIMIT - elapsed) / 1000;
    }

    static unsigned long lastIdleMelody = 0;

    switch (state) {
    case IDLE:
        if (millis() - lastIdleMelody > 15000) {  // Redă melodia la 15 s
            animation();
            lastIdleMelody = millis();
        }

        if (buttonPressed(btn1Pin)) {
            score = 0;
            isQuickMode = false;
            newAction();
            state = PLAYING;
        } 
        
        if (buttonPressed(btn2Pin)) {
            score = 0;
            isQuickMode = true;
            quickModeStartTime = millis();
            newAction();
            state = PLAYING;
        }
        break;

    case PLAYING: {
        showNumber(max(0, (int) remaining));

        // Verifică dacă timpul a expirat
        bool timeExpired = false;
        if (isQuickMode && elapsed >= QUICK_TIME) {
            timeExpired = true;
        }
        if (!isQuickMode && elapsed >= TIME_LIMIT) {
            timeExpired = true;
        }
        
        if (timeExpired) {
            if (isQuickMode) {
                showGameOver("Timp expirat!");
            } else {
                showGameOver("Prea incet!");
            }
            break;
        }

        if (wrongAction(currentAction)) {
            showGameOver("Actiune gresita!");
            break;
        }

        if (checkAction(currentAction)) {
            score++;
            if (isQuickMode) { // Daca sunt in modul rapid
                tone(buzzerPin, 800, 50);
                newAction(); // Acțiune nouă
            } else {
                lcd.setCursor(8, 1);
                lcd.print(" OK!");
                tone(buzzerPin, 800, 50);
                delay(600);
                newAction(); // resetează timpul
            }
        }
        break;
    }
    }
}
