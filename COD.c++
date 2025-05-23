#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Definire pini pentru senzori și butoane
const int buzzerPin = 8;
const int potentiometruPin = A0;
const int buton1Pin = A1;
const int buton2Pin = A2;

// Definire Pini Registru Deplasare
const int latchPin = 10;
const int clockPin = 9;
const int dataPin = 11;

// Pattern-uri pentru afișajul cu 7 segmente
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

// Stări Joc
enum GameState {
    IDLE,
    WAITING_FOR_ACTION,
    ACTION_IN_PROGRESS,
    ROUND_LOST,
    QUICK_MODE_ACTIVE,
    QUICK_MODE_FINISHED
};
GameState currentGameState = IDLE;

// Variabile Joc
int score = 0;
int currentActionID = 0;
unsigned long actionStartTime = 0;
const int timeLimitSeconds = 5; // mereu 5 secunde

// Variabile pentru modul rapid
const int quickModeTimeLimit = 10; // 10 secunde pentru modul rapid
unsigned long quickModeStartTime = 0;
boolean isQuickMode = false;
int currentQuickActionID = -1; // Adaugă această variabilă globală

// Acțiuni Posibile
enum PlayerAction {
    PRESS_BUTTON_1,
    PRESS_BUTTON_2,
    TURN_POTENTIOMETER,
    NUM_ACTIONS
};
String actionDescriptions[] = {
    "Apasa Buton 1!",
    "Apasa Buton 2!",
    "Roteste!"
};

void clearDisplaySevenSegment();
byte getDigit(int i, bool period); // fără default argument aici
void setDisplay(byte code);
bool isWrongActionPerformed();
void setup();
void loop();
void handleIdleState();
void handleWaitingForActionState();
void handleActionInProgressState();
void handleRoundLostState();
void handleQuickModeState();
void handleQuickModeFinishedState();
bool isButtonPressed(int buttonPin);

// Praguri pentru senzori
const int POTENTIOMETER_THRESHOLD_HIGH = 600;
const int POTENTIOMETER_THRESHOLD_LOW = 30;

// Timp debounce butoane
const unsigned long DEBOUNCE_DELAY = 50;

// Variabilă pentru logica potențiometrului
bool potentiometerPassedHigh = false;

// Funcții pentru Afișajul cu 7 Segmente

//---------------------------------------------------------------Funcții MAIN---------------------------------------------------------------

void clearDisplaySevenSegment() {
    setDisplay(0b00000000);
}

byte getDigit(int i, bool period = false) {
    if (i >= 0 && i < 10) {
        if (period == false) {
            return digits[i];
        } else {
            return digits[i] | 0b00000001;
        }
    }
    return 0b00000000;
}

void setDisplay(byte code) {
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, code);
    digitalWrite(latchPin, HIGH);
}

// Funcție nouă: verifică acțiuni nepermise
bool isWrongActionPerformed() {
    switch (currentActionID) {
    case PRESS_BUTTON_1:
        if (isButtonPressed(buton2Pin)) return true;
        if (analogRead(potentiometruPin) > POTENTIOMETER_THRESHOLD_HIGH) return true;
        break;
    case PRESS_BUTTON_2:
        if (isButtonPressed(buton1Pin)) return true;
        if (analogRead(potentiometruPin) > POTENTIOMETER_THRESHOLD_HIGH) return true;
        break;
    case TURN_POTENTIOMETER:
        if (isButtonPressed(buton1Pin)) return true;
        if (isButtonPressed(buton2Pin)) return true;
        break;
    }
    return false;
}

void setup() {
    Serial.begin(9600);
    lcd.init();                      // Initialize the LCD
    lcd.backlight();                 // Turn on backlight
    lcd.print("Joc Actiune!");
    lcd.setCursor(0, 1);
    lcd.print("Apasa B1 Start");

    pinMode(buzzerPin, OUTPUT);
    pinMode(potentiometruPin, INPUT);
    pinMode(buton1Pin, INPUT_PULLUP);
    pinMode(buton2Pin, INPUT_PULLUP);
    pinMode(latchPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, OUTPUT);

    clearDisplaySevenSegment();
    randomSeed(analogRead(A4));
}

void loop() {
    switch (currentGameState) {
    case IDLE:
        handleIdleState();
        break;
    case WAITING_FOR_ACTION:
        handleWaitingForActionState();
        break;
    case ACTION_IN_PROGRESS:
        handleActionInProgressState();
        break;
    case ROUND_LOST:
        handleRoundLostState();
        break;
    case QUICK_MODE_ACTIVE:
        handleQuickModeState();
        break;
    case QUICK_MODE_FINISHED:
        handleQuickModeFinishedState();
        break;
    }
}

//---------------------------------------------------------------Funcții Comportament---------------------------------------------------------------
void handleIdleState() {
    static bool displayToggle = false;
    static unsigned long lastToggleTime = 0;
    
    // La prima intrare în starea IDLE sau după o perioadă lungă, resetează ecranul
    if (millis() - lastToggleTime > 1000) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Alege mod:");
        lcd.setCursor(0, 1);
        lcd.print("B1:Norm B2:Rapid");
        displayToggle = false;
        lastToggleTime = millis();
    }
    
    if (isButtonPressed(buton1Pin)) {
        score = 0;
        isQuickMode = false;
        lcd.clear();
        lcd.print("Mod Normal");
        lcd.setCursor(0, 1);
        lcd.print("Pregatire...");
        clearDisplaySevenSegment();
        delay(1000);
        currentGameState = WAITING_FOR_ACTION;
    }
    else if (isButtonPressed(buton2Pin)) {
        score = 0;
        isQuickMode = true;
        currentQuickActionID = -1; // Resetăm pentru modul rapid
        lcd.clear();
        lcd.print("Mod Rapid 10s");
        lcd.setCursor(0, 1);
        lcd.print("Pregatire...");
        clearDisplaySevenSegment();
        delay(1000);
        quickModeStartTime = millis();
        currentGameState = QUICK_MODE_ACTIVE;
    }
}

void handleWaitingForActionState() {
    currentActionID = random(0, NUM_ACTIONS);
    potentiometerPassedHigh = false; // reset pentru fiecare rundă nouă
    Serial.print("Comanda: ");
    Serial.println(actionDescriptions[currentActionID]);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(actionDescriptions[currentActionID]);
    lcd.setCursor(0, 1);
    lcd.print("Scor: ");
    lcd.print(score);
    setDisplay(getDigit(timeLimitSeconds));
    actionStartTime = millis();
    currentGameState = ACTION_IN_PROGRESS;
}

void handleActionInProgressState() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - actionStartTime;
    int remainingTimeSeconds = timeLimitSeconds - (elapsedTime / 1000);

    if (remainingTimeSeconds >= 0) {
        setDisplay(getDigit(remainingTimeSeconds));
    } else {
        setDisplay(getDigit(0));
    }

    if (elapsedTime >= (unsigned long) timeLimitSeconds * 1000) {
        currentGameState = ROUND_LOST;
        return;
    }

    // Verificare acțiune greșită
    if (isWrongActionPerformed()) {
        currentGameState = ROUND_LOST;
        return;
    }

    // Verificare acțiune corectă
    bool actionCompleted = false;
    switch (currentActionID) {
    case PRESS_BUTTON_1:
        if (isButtonPressed(buton1Pin)) actionCompleted = true;
        break;
    case PRESS_BUTTON_2:
        if (isButtonPressed(buton2Pin)) actionCompleted = true;
        break;
    case TURN_POTENTIOMETER: {
        int potValue = analogRead(potentiometruPin);
        if (!potentiometerPassedHigh && potValue > POTENTIOMETER_THRESHOLD_HIGH) {
            potentiometerPassedHigh = true;
        }
        if (potentiometerPassedHigh && potValue < POTENTIOMETER_THRESHOLD_LOW) {
            actionCompleted = true;
            potentiometerPassedHigh = false; // reset pentru următoarea rundă
        }
        break;
    }
    }

    if (actionCompleted) {
        score++;
        lcd.setCursor(8, 1);
        lcd.print(" OK!");
        delay(600);
        currentGameState = WAITING_FOR_ACTION;
    }
}

void handleQuickModeState() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - quickModeStartTime;
    int remainingTimeSeconds = quickModeTimeLimit - (elapsedTime / 1000);
    
    // Afișează timpul rămas
    if (remainingTimeSeconds >= 0) {
        setDisplay(getDigit(remainingTimeSeconds));
    } else {
        setDisplay(getDigit(0));
        currentGameState = QUICK_MODE_FINISHED;
        return;
    }
    
    // Generează acțiune dacă nu există una în desfășurare
    if (currentQuickActionID == -1) {
        currentQuickActionID = random(0, NUM_ACTIONS);
        currentActionID = currentQuickActionID; // Important: sincronizare pentru isWrongActionPerformed()
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(actionDescriptions[currentQuickActionID]);
        lcd.setCursor(0, 1);
        lcd.print("Scor:");
        lcd.print(score);
        lcd.print(" T:");
        lcd.print(remainingTimeSeconds);
    }
    
    // Verifică acțiune greșită
    if (isWrongActionPerformed()) {
        // Dacă se face o greșeală, jocul se termină
        currentGameState = QUICK_MODE_FINISHED;
        return;
    }
    
    // Verifică acțiune corectă
    bool actionCompleted = false;
    switch (currentQuickActionID) {
    case PRESS_BUTTON_1:
        if (isButtonPressed(buton1Pin)) actionCompleted = true;
        break;
    case PRESS_BUTTON_2:
        if (isButtonPressed(buton2Pin)) actionCompleted = true;
        break;
    case TURN_POTENTIOMETER: {
        int potValue = analogRead(potentiometruPin);
        if (!potentiometerPassedHigh && potValue > POTENTIOMETER_THRESHOLD_HIGH) {
            potentiometerPassedHigh = true;
        }
        if (potentiometerPassedHigh && potValue < POTENTIOMETER_THRESHOLD_LOW) {
            actionCompleted = true;
            potentiometerPassedHigh = false;
        }
        break;
    }
    }
    
    if (actionCompleted) {
        score++;
        tone(buzzerPin, 800, 50);
        currentQuickActionID = -1; // generează nouă acțiune
    }
}

void handleQuickModeFinishedState() {
    lcd.clear();
    lcd.setCursor(0, 0);
    
    // Verifică dacă jocul s-a terminat din cauza timpului sau a unei greșeli
    if (millis() - quickModeStartTime >= (unsigned long)quickModeTimeLimit * 1000) {
        lcd.print("Timp expirat!");
    } else {
        lcd.print("Gresit! Ai pierdut");
    }
    
    lcd.setCursor(0, 1);
    lcd.print("Scor Final: ");
    lcd.print(score);
    
    clearDisplaySevenSegment();
    tone(buzzerPin, 500, 300);
    delay(500);
    tone(buzzerPin, 200, 500);
    delay(1500);
    
    currentGameState = IDLE;
}

void handleRoundLostState() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Runda Pierduta!");
    lcd.setCursor(0, 1);
    lcd.print("Scor Final: ");
    lcd.print(score);
    clearDisplaySevenSegment();
    tone(buzzerPin, 250, 500);
    delay(500);
    tone(buzzerPin, 200, 500);
    delay(1500);
    currentGameState = IDLE;
    lcd.clear();
    lcd.print("Joc Actiune!");
    lcd.setCursor(0, 1);
    lcd.print("Apasa B1 Start");
}

bool isButtonPressed(int buttonPin) {
    static unsigned long lastDebounceTime[2] = {0, 0};
    static bool lastButtonState[2] = {HIGH, HIGH};
    static bool currentButtonStableState[2] = {HIGH, HIGH};
    static bool buttonPressedEvent[2] = {false, false};

    int buttonIndex = (buttonPin == buton1Pin) ? 0 : 1;
    bool eventDetected = false;
    bool reading = digitalRead(buttonPin);

    if (reading != lastButtonState[buttonIndex]) {
        lastDebounceTime[buttonIndex] = millis();
    }

    if ((millis() - lastDebounceTime[buttonIndex]) > DEBOUNCE_DELAY) {
        if (reading != currentButtonStableState[buttonIndex]) {
            currentButtonStableState[buttonIndex] = reading;
            if (currentButtonStableState[buttonIndex] == LOW) {
                buttonPressedEvent[buttonIndex] = true;
            }
        }
    }
    lastButtonState[buttonIndex] = reading;

    if (buttonPressedEvent[buttonIndex]) {
        buttonPressedEvent[buttonIndex] = false;
        eventDetected = true;
    }
    return eventDetected;
}
