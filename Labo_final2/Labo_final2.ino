#include <Adafruit_seesaw.h>
#include <MeAuriga.h>
#include <ArduinoJson.h>
StaticJsonDocument<256> doc;
#define NB_IR 5
#define CALIBRATION_TIME 5000  
#define PULSE 9
#define RATIO 39.267
#define BUZZER_PIN 45
#define RINGALLLEDS 0
#define LEDNUM 12
#define LEDPIN 44
#define PORT0 0


enum StartState {
  START_CALIBRATION,
  START_ADVANCE,
  START_DETECT_MAIN_LINE
};

StartState startSubState = START_CALIBRATION;


enum ManualAction {
    NONE,
    FORWARD,
    LEFT,
    RIGHT,
    BACKWARD
};
ManualAction manualAct = NONE;

enum AppState { ARRETER,START,SEARCH_LINE,CALIBRATION,MANUAL,DEBUG, TURN, FOLLOW_LINE, CHECK,END,AUTO };
AppState currentState = ARRETER;

short speed = 50;

MeEncoderOnBoard encoderLeft(SLOT2);
MeEncoderOnBoard encoderRight(SLOT1);
MeUltrasonicSensor ultraSensor(PORT_10);
Adafruit_seesaw ss;
MeGyro gyro(PORT0, 0x69);
MeRGBLed led(PORT0,LEDNUM);

static bool automatic = false;

static bool debugFlag = false;

String PreviousCmd = "ARRETER";

String lastCommand = "";

static int dist;

bool hasReachedEnd = false;
bool returning = false;
static bool turnfirst = false;
unsigned long previousTime = 0;
unsigned long ct;
static bool checkfirst = false;
static bool debugMode = true;
  static unsigned long prevTime = 0;
int distance;
double error;
double lastAngle;
double cumulativeAngle;
double turnTargetAngle = 0;
static bool is_turned = false;
bool turnFirstRun = true;
static int rate = 200;
unsigned long previousLoopTime = 0;

// end-of-line detection timing (déclaration unique)
const float END_LINE_SUM_THRESHOLD = 50.0f;
const int END_LINE_CONFIRM = 5;
static int endLineCounter = 0;
static bool stoppedAtEnd = false;
unsigned long endDetectMillis = 0;
const unsigned long END_DETECT_DELAY_MS = 800; // temporisation demandée ~500ms
static bool endPending = false;
static String receivedData = "";
// check state substates
static int checkSubState = 0; // 0 = start left check (turn 90), 1 = after left turn, 2 = turn 180 to check right, 3 = after right check (decide), 4 = finish (U-turn)

void rightEncoderInterrupt(void)
{
  if (digitalRead(encoderRight.getPortB()) == 0)
    encoderRight.pulsePosMinus();
  else
    encoderRight.pulsePosPlus();
}

void leftEncoderInterrupt(void) {
  if (digitalRead(encoderLeft.getPortB()) == 0)
    encoderLeft.pulsePosMinus();
  else
    encoderLeft.pulsePosPlus();
}

void encoderConfig() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);

  encoderRight.setPulse(PULSE);
  encoderLeft.setPulse(PULSE);

  encoderRight.setRatio(RATIO);
  encoderLeft.setRatio(RATIO);

  encoderRight.setPosPid(1.8, 0, 1.2);
  encoderLeft.setPosPid(1.8, 0, 1.2);

  encoderRight.setSpeedPid(0.18, 0, 0);
  encoderLeft.setSpeedPid(0.18, 0, 0);

  // Réglages PWM (ne pas modifier)
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

struct Capteur {
  int valeurMin = 1023;
  int valeurMax = 0;
  int valeurLue = 0;
  float valeurNorm = 0;
};
Capteur capteurs[NB_IR];
int vitesseBase = 60;
float kp = 0.2;
float ki = 0.0;
float kd = 0.05;
float integral = 0;
float lastError = 0;

bool ligneRetrouvee(int seuil) {
    int count = 0;
    for (int i = 0; i < NB_IR; i++) {
        if (capteurs[i].valeurNorm > seuil) {
            count++;
        }
    }
    return (count >= 2);
}
AppState endLineDetection() {
    lireEtNormaliser();

    if (ligneRetrouvee(400)) {
        off();
        integral = 0;
        lastError = 0;
        return FOLLOW_LINE;
    }

    float sommeNorm = 0.0;
    for (int i = 0; i < NB_IR; i++) {
        sommeNorm += capteurs[i].valeurNorm;
    }

    if (sommeNorm < END_LINE_SUM_THRESHOLD) {
        endLineCounter++;
        if (endLineCounter > END_LINE_CONFIRM) {
            off();
            Serial.println("check returned");
            resetTurn(270.0);
           return CHECK;
        }
    } else {
        endLineCounter = 0;
    }

    return FOLLOW_LINE;
}
double getUnwrappedAngle() {
  double angle = gyro.getAngleZ();
  double delta = angle - lastAngle;

  if (delta < -180) delta += 360;
  if (delta > 180) delta -= 360;

  cumulativeAngle += delta;
  lastAngle = angle;

  return cumulativeAngle;
}

int getDistance() {
  static unsigned long previousMillis = 0;
  static int prevdistance = 400; // initialisation sûre

  if (ct - previousMillis >= (unsigned long)rate) {
    previousMillis = ct;
    prevdistance = ultraSensor.distanceCm();
    if (prevdistance > 150) {
      rate = 150;
    } else {
      rate = 80;
    }
  }

  if (prevdistance > 1 && prevdistance < 400) {
    distance = prevdistance;
  }
  return distance;
}

void resetTurn(double targetDegrees) {
  gyro.resetData();
  cumulativeAngle = 0;
  lastAngle = gyro.getAngleZ();
  is_turned = false;
  turnTargetAngle = targetDegrees;
  turnFirstRun = true;
}

bool turn90(short baseSpeed = 40, double targetDegrees = 360.0, bool setStateOnFinish = true) {
  const double tolerance = 8.0;
  gyro.fast_update();

  if (turnFirstRun) {
    turnFirstRun = false;
    cumulativeAngle = 0;
    gyro.resetData();
    lastAngle = gyro.getAngleZ();
    turnTargetAngle = targetDegrees;
    is_turned = false;
  }

  double currentAngle = getUnwrappedAngle();
  error = turnTargetAngle - currentAngle;

  if (fabs(error) < tolerance) {
    encoderLeft.setMotorPwm(0);
    encoderRight.setMotorPwm(0);
    checkfirst = true;
    if (setStateOnFinish ) {
      is_turned = true;
      Serial.println(">>> Turn finished: setting FOLLOW_LINE");
    } else {
      Serial.println(">>> Turn finished (no state change)");
    }
    Serial.print("Angle atteint: ");
    Serial.println(currentAngle);
    return true;
  }

  double Kp = 1.2;
  int pwm = constrain((int)(baseSpeed + Kp * error), 40, 120);
  // ici on applique le même pwm aux deux moteurs (ton code d'origine)
  encoderLeft.setMotorPwm(pwm);
  encoderRight.setMotorPwm(pwm);

  return false;
}

// setStateOnFinish : si true -> change currentState en FOLLOW_LINE quand fini
bool turn360(short baseSpeed = 40, double targetDegrees = 360.0, bool setStateOnFinish = true) {
  const double tolerance = 20.0;
  gyro.fast_update();

  if (turnFirstRun) {
    turnFirstRun = false;
    cumulativeAngle = 0;
    gyro.resetData();
    lastAngle = gyro.getAngleZ();
    turnTargetAngle = targetDegrees;
    is_turned = false;
  }

  double currentAngle = getUnwrappedAngle();
  error = turnTargetAngle - currentAngle;

  if (fabs(error) < tolerance) {
    encoderLeft.setMotorPwm(0);
    encoderRight.setMotorPwm(0);
    checkfirst = true;
    if (setStateOnFinish ) {
      is_turned = true;
      Serial.println(">>> Turn finished: setting FOLLOW_LINE");
    } else {
      Serial.println(">>> Turn finished (no state change)");
    }
    Serial.print("Angle atteint: ");
    Serial.println(currentAngle);
    return true;
  }

  double Kp = 1.2;
  int pwm = constrain((int)(baseSpeed + Kp * error), 40, 120);
  // ici on applique le même pwm aux deux moteurs (ton code d'origine)
  encoderLeft.setMotorPwm(pwm);
  encoderRight.setMotorPwm(pwm);

  return false;
}

void setup() {
  Serial.begin(115200);
  gyro.begin();
  gyro.resetData();
  encoderConfig();
  led.setpin(LEDPIN);
  pinMode(BUZZER_PIN, OUTPUT);
  offLed();
  turnfirst = false;
}

void startCheckProcedure() {
  checkSubState = 0;
  stoppedAtEnd = true;
  endPending = false;
  endLineCounter = 0;
}

bool checkDirection() {
  gyro.fast_update();

  const unsigned long localRate = 500; // renommé pour éviter collision avec global 'rate'
  switch (checkSubState) {
    case 0:
      resetTurn(90.0);
      checkSubState = 1;
      prevTime = ct;
      break;
    case 1: // effectuer le 90 gauche  
      if (turn90(60, 90.0, false)) {
        turnfirst = !turnfirst;
        Serial.println("Turn finish");
        Serial.println(ct);
        Serial.println(prevTime);
        Serial.println(localRate);
        if (ct - prevTime >= localRate) {
          distance = getDistance();
          Serial.println(distance);
          if (distance > 30) {
            stoppedAtEnd = false;
            integral = 0; lastError = 0;
            currentState = FOLLOW_LINE;
            Serial.println(currentState);
            checkSubState = 0;
          } else {
            prevTime = ct;
            resetTurn(180.0);
            checkSubState = 2;
          }
        }
      } else {
        // faire avancer la rotation
        turn90(60, 90.0, false);
      }
      break;
    case 2: // effectuer la rotation 180 (depuis orientation gauche -> se retrouver pointant droite)
      if (turn90(60, 180.0, false)) {
        if (ct - prevTime >= localRate) {
          distance = getDistance();
          if (distance > 30) {
            stoppedAtEnd = false;
            integral = 0; lastError = 0;
            currentState = FOLLOW_LINE;
            checkSubState = 0;
          } else {
            prevTime = ct;
            resetTurn(180.0);
            checkSubState = 3;
          }
        }
      } else {
        turn90(60, 180.0, false);
      }
      break;
    case 3:
      if (turn90(60, 180.0, true)) {
        if (ct - prevTime >= localRate) {
          stoppedAtEnd = false;
          integral = 0; lastError = 0;
          currentState = END;
          checkSubState = 0;
        }
      } else {
         prevTime = ct;
        turn90(60, 180.0, true);
      }
      break;
    default:
      checkSubState = 0;
      currentState = FOLLOW_LINE;
      break;
  }
  return true;
}

bool calibration() {
  if (!ss.begin()) {
    Serial.println("Erreur de connexion seesaw!");
  }
  Serial.println("Connexion OK !");
  Serial.println("=== Calibration automatique en cours... ===");

  unsigned long start = millis();

  resetTurn(360.0);
  while (millis() - start < CALIBRATION_TIME) {
    turn360(50, 360.0, false);
    for (int i = 0; i < NB_IR; i++) {
      int val = ss.analogRead(i);
      if (val < capteurs[i].valeurMin) capteurs[i].valeurMin = val;
      if (val > capteurs[i].valeurMax) capteurs[i].valeurMax = val;
    }
    completeYellow();
  }
  Serial.println("=== Calibration terminée ===");
  for (int i = 0; i < NB_IR; i++) {
    Serial.print("Capteur ");
    Serial.print(i);
    Serial.print(" | Min=");
    Serial.print(capteurs[i].valeurMin);
    Serial.print(" | Max=");
    Serial.println(capteurs[i].valeurMax);
  }
  Serial.println("Début du suivi de ligne...");
  if (is_turned){
      currentState = FOLLOW_LINE;
  }
  return true;
}

void lireEtNormaliser() {
  for (int i = 0; i < NB_IR; i++) {
    capteurs[i].valeurLue = ss.analogRead(i);
    int denom = (capteurs[i].valeurMax - capteurs[i].valeurMin);
    if (denom == 0) denom = 1; // éviter division par 0
    capteurs[i].valeurNorm =
      (float)(capteurs[i].valeurMax - capteurs[i].valeurLue) * 1000.0 /
      (float)denom;
    capteurs[i].valeurNorm = constrain(capteurs[i].valeurNorm, 0, 1000);
  }
}

float calculerPositionLigne() {
  float numerateur = 0;
  float denominateur = 0;

  for (int i = 0; i < NB_IR; i++) {
    numerateur += capteurs[i].valeurNorm * (i - 2);
    denominateur += capteurs[i].valeurNorm;
  }

  if (denominateur == 0) return 0;
  return (numerateur / denominateur) * 1000.0;
}

// PID 
float computePID(float position, float consigne = 0.0f) {
  float error = position - consigne;

  integral += error;
  const float integralLimit = 1000;
  integral = constrain(integral, -integralLimit, integralLimit);

  float derivative = error - lastError;
  lastError = error;

  float output = kp * error + ki * integral + kd * derivative;
  return output;
}

void suivreLigne(float correction) {
  float leftSpeed = vitesseBase + correction;
  float rightSpeed = -(vitesseBase - correction);

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  encoderLeft.setMotorPwm((int)leftSpeed);
  encoderRight.setMotorPwm((int)rightSpeed);
}


void avancer(){
  is_turned = false;
  float position = calculerPositionLigne();

  // 4. Calcul PID
  float correction = computePID(position, 0.0f);

  // 5. Ajustement des moteurs
  suivreLigne(correction);
}


void avancer1() {
     forward(speed);
   
 
}

void reculer() {
  backward(speed);
  bip();
}

void tournerDroite() {
  turnRight(speed);
}

void tournerGauche() {
  turnLeft(speed);
}

void bip() {
  static unsigned long previousTime = 0;
  static bool buzz = false;
  unsigned long currentTime = millis();
  int bipRate = 250;

  if (currentTime - previousTime >= bipRate) {
    previousTime = currentTime;
    if (buzz) {
      analogWrite(BUZZER_PIN, 0);
    } else {
      analogWrite(BUZZER_PIN, 127);
    }
    buzz = !buzz;
  }
}

void off() {
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}

void offLed() {
  led.setColor(0, 0, 0);
  led.show();
}

void completeLed() {
  for (int idx = 0; idx < LEDNUM; idx++) {
    led.setColorAt(idx, 0, 100, 0);
  }
  led.show();
}

void completeYellow() {
  static unsigned long previousTime = 0;
  static bool ledState = false;
  unsigned long currentTime = millis();
  int ledRate = 500;

  if (currentTime - previousTime >= ledRate) {
    previousTime = currentTime;
    if (ledState) {
      for (int idx = 0; idx < LEDNUM; idx++) {
        led.setColorAt(idx, 0, 0, 0);
      }
    } else {
      for (int idx = 0; idx < LEDNUM; idx++) {
        led.setColorAt(idx, 100, 100, 0);
      }
    }
    led.show();
    ledState = !ledState;
  }
}

void completeBlue() {
  static unsigned long previousTime = 0;
  static bool ledState = false;
  unsigned long currentTime = millis();
  int ledRate = 500;

  if (currentTime - previousTime >= ledRate) {
    previousTime = currentTime;
    if (ledState) {
      for (int idx = 0; idx < LEDNUM; idx++) {
        led.setColorAt(idx, 0, 0, 0);
      }
    } else {
      for (int idx = 0; idx < LEDNUM; idx++) {
        led.setColorAt(idx, 0, 0, 100);
      }
    }
    led.show();
    ledState = !ledState;
  }
}

void modedebug() {
  static unsigned long previousTime = 0;
  unsigned long currentTime = millis();
  int debugRate = 2000;
  unsigned long s = ct / 1000;
  if (currentTime - previousTime >= debugRate) {
    previousTime = currentTime;

    doc.clear();
    const char* mode = "MANUAL";

    Serial.println("Deboguage");

    switch(currentState) {
      case MANUAL: mode = "MANUAL"; break;
      case FOLLOW_LINE: mode = "SUIVI_LIGNE"; break;
      case CALIBRATION: mode = "CALIBRATION"; break;
      case CHECK: mode = "CHECK DIRECTION"; break;
    }

    doc["currentState"] = mode;

    doc["Vitesse"] = speed;

    doc["Valeurs PID"]["KP"] = kp;
    doc["Valeurs PID"]["KI"] = ki;
    doc["Valeurs PID"]["KD"] = kd;
    doc["temps"] = s;

    for (int i = 0; i < NB_IR; i++) {
      JsonObject c = doc.createNestedObject("Capteur_" + String(i));
      c["Min"] = capteurs[i].valeurMin;
      c["Max"] = capteurs[i].valeurMax;
    }

    doc["Distance ultrason"] = distance;
    doc["Dernière commande"] = lastCommand;
    doc["CheckSubState"] = checkSubState;

    serializeJson(doc, Serial);
    Serial.println();
  }
}

void calibrerCapteurs() {
  currentState = CALIBRATION;
}
void backward(int speed) {
  encoderLeft.setMotorPwm(-speed);
  encoderRight.setMotorPwm(speed);
}

void forward(int speed) {
  encoderLeft.setMotorPwm(speed);
  encoderRight.setMotorPwm(-speed);
}

void turnRight(int speed) {
  encoderLeft.setMotorPwm(-speed);
  encoderRight.setMotorPwm(-speed);
}

void turnLeft(int speed) {
  encoderLeft.setMotorPwm(speed);
  encoderRight.setMotorPwm(speed);
}

void serialEvent() {
  while (Serial.available()) {
    receivedData = Serial.readStringUntil('\n');
    receivedData.trim();
    if (receivedData.length() > 0) {
      parseData(receivedData);
    }
  }
}

void parseData(String& receivedData) {
  bool isFromBLE = false;

  if (receivedData.length() >= 2) {
    if ((uint8_t)receivedData[0] == 0xFF && (uint8_t)receivedData[1] == 0x55) {
      receivedData.remove(0, 2);
      isFromBLE = true;
    } else if (receivedData.startsWith("!!")) {
      receivedData.remove(0, 2);
    }
  }

  receivedData.toUpperCase();
  if (debugMode) {
    Serial.print("Reçu : ");
    Serial.println(receivedData);
  }

  int index = receivedData.indexOf(',');
  if (index == -1) {
    gererCommandeSimple(receivedData);
  } else {
    String action = receivedData.substring(0, index);
    String parametre = receivedData.substring(index + 1);
    gererCommandeComposee(action, parametre);
  }
}

void gererCommandeSimple(const String& cmd) {
  if (cmd == "MANUAL") {
    currentState = MANUAL;
    off();
    completeLed();
    lastCommand = "MANUAL";
    Serial.println("Mode MANUAL activé");
  } else if (cmd == "W") {
    if (currentState == MANUAL)  manualAct = FORWARD;
    lastCommand = "W";
  } else if (cmd == "S") {
    if (currentState == MANUAL) manualAct= BACKWARD;
    lastCommand = "S";
  }else if (cmd == "STOP") {
    if (currentState == MANUAL) manualAct= NONE;
    lastCommand = "STOP";
  } else if (cmd == "D") {
    if (currentState == MANUAL) manualAct = RIGHT;
    lastCommand = "D";
  } else if (cmd == "A") {
    if (currentState == MANUAL) manualAct = LEFT;
    lastCommand = "A";
    }else if (cmd == "GO") {
    currentState = START;
    lastCommand = "GO";
  } else if (cmd == "K") {
    if (currentState == MANUAL) analogWrite(BUZZER_PIN, 127);
    lastCommand = "KLAXONNER";
  } else if (cmd == "DEBUG") {
    debugFlag = !debugFlag;
    lastCommand = "DEBUG";
    Serial.println(debugFlag ? "Debug activé" : "Debug désactivé");
    if (debugFlag) currentState = DEBUG;
    else currentState = MANUAL;
  } else if (cmd == "ARRETER") {
    lastCommand = "ARRETER";
    off();
    offLed();
    currentState = ARRETER;
  } else if (cmd == "CAL") {
    currentState = CALIBRATION;
    lastCommand = "CAL";
  } else if (cmd == "FL") {
    currentState = FOLLOW_LINE;
    hasReachedEnd = false;
    returning = false;
    integral = 0;
    lastError = 0;
    endLineCounter = 0;
    stoppedAtEnd = false;
    checkSubState = 0;
    lastCommand = "FL";
    Serial.println("Mode suivi ligne activé");
  }
  PreviousCmd = cmd;
}

void gererCommandeComposee(const String& action, const String& parametre) {
  if (action == "VITESSE") {
    vitesseBase = parametre.toInt();
    lastCommand = action + "," + parametre;
    Serial.print("Vitesse définie à: ");
    Serial.println(speed);
  } else if (action == "KP") {
    kp = parametre.toFloat();
    lastCommand = action + "," + parametre;
  } else if (action == "KI") {
    ki = parametre.toFloat();
    lastCommand = action + "," + parametre;
  } else if (action == "KD") {
    kd = parametre.toFloat();
    lastCommand = action + "," + parametre;
  } else if (action == "AUTO") {
    currentState = FOLLOW_LINE;
    dist = parametre.toInt();
  //  avancer();
    lastCommand = action + "," + parametre;
    automatic = true;
  }
}


void stateManager(){
    switch (currentState) {
        case MANUAL:
            // Machine à état MANUAL (sous-états)
            switch (manualAct) {
                case FORWARD:
                    forward(90);
                    break;
                case BACKWARD:
                    backward(80);
                    break;
                case LEFT:
                    turnLeft(75);
                    break;
                case RIGHT:
                    turnRight(75);
                    break;
                case NONE:
                    off();
                    break;
            }
            break;

        case CALIBRATION:
            if (calibration()) {
                currentState = FOLLOW_LINE;
            }
            break;
        case START:
         switch(startSubState) {

           case START_CALIBRATION:
            if (calibration()) {
             startSubState = START_ADVANCE;
            }
           break;
           case START_ADVANCE:
            lireEtNormaliser();
            forward(vitesseBase);
            if (ligneRetrouvee(400)) {
             off();
             resetTurn(270.0);
             startSubState = START_DETECT_MAIN_LINE;
             checkSubState = 0;
             }
          break;
          case START_DETECT_MAIN_LINE:
          if (turn90(60, 270.0, false)) {
             currentState = FOLLOW_LINE;   // ou TURN si tu préfères
           startSubState = START_CALIBRATION; // reset pour prochaine fois
          }else {
            turn90(60, 270.0, false);
          }
          
          break;
         }
      
      break;
       
        case FOLLOW_LINE:
            lireEtNormaliser();
            if (distance > 2 && distance <30){
               checkDirection();
            }
            //else{
            //  avancer();
           // }

           AppState endRes = endLineDetection();

            if (endRes == CHECK) {
             if (turn360(65,270.0, false)) {
               endRes = FOLLOW_LINE;  // ou TURN si tu préfères
             }else {
            turn360(65,270.0, false);
            }
            }else{
             avancer();
            }
            break;

        case CHECK:
            Serial.println("je suis dans check");
            endLineCounter = 0;  
            checkDirection();
            break;
        case DEBUG:
            modedebug(); 
            break;
        case TURN:
            if (turn360(60, 180.0, false)) {
                currentState = FOLLOW_LINE;
            }
            break;

        case ARRETER:
            off();
            break;

        case AUTO:
            // Logique AUTO à implémenter
            break;

        case END:
            off();
            break;

        default:
            currentState = ARRETER;
            break;
    }

}


void loop() {
  ct = millis();
  encoderLeft.loop();
  encoderRight.loop();
  distance = getDistance();
  serialEvent();
    modedebug(); 
  stateManager();
}