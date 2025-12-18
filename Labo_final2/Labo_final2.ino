#include <Adafruit_seesaw.h>
#include <MeAuriga.h>

#define NB_IR 5
#define CALIBRATION_TIME 5000  
#define PULSE 9
#define RATIO 39.267

enum AppState { CALIBRATION, TURN, FOLLOWLINE, CHECK };
AppState currentState = CALIBRATION;
short speed;

MeEncoderOnBoard encoderLeft(SLOT2);
MeEncoderOnBoard encoderRight(SLOT1);
MeUltrasonicSensor ultraSensor(PORT_10);
Adafruit_seesaw ss;
MeGyro gyro(0, 0x69);

static bool turnfirst = false;
unsigned long previousTime = 0;
unsigned long ct;
static bool checkfirst = false;
int distance;
double error;
double lastAngle;
double cumulativeAngle;
double turnTargetAngle = 0;
static bool is_turned = false;
bool turnFirstRun = true;
static int rate = 200;

// end-of-line detection timing (déclaration unique)
const float END_LINE_SUM_THRESHOLD = 50.0f;
const int END_LINE_CONFIRM = 5;
static int endLineCounter = 0;
static bool stoppedAtEnd = false;
unsigned long endDetectMillis = 0;
const unsigned long END_DETECT_DELAY_MS = 200; // temporisation demandée ~500ms
static bool endPending = false;

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

// setStateOnFinish : si true -> change currentState en FOLLOWLINE quand fini
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
      currentState = FOLLOWLINE;
      Serial.println(">>> Turn finished: setting FOLLOWLINE");
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
}

void startCheckProcedure() {
  checkSubState = 0;
  stoppedAtEnd = true;
  endPending = false;
  endLineCounter = 0;
}

void checkDirection() {
  gyro.fast_update();
  static unsigned long prevTime = 0;
  const unsigned long localRate = 500; // renommé pour éviter collision avec global 'rate'
  switch (checkSubState) {
    case 0:
      resetTurn(90.0);
      checkSubState = 1;
      prevTime = ct;
      break;
    case 1: // effectuer le 90 gauche  
      if (turn360(60, 90.0, false)) {
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
            currentState = FOLLOWLINE;
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
        turn360(60, 90.0, false);
      }
      break;
    case 2: // effectuer la rotation 180 (depuis orientation gauche -> se retrouver pointant droite)
      if (turn360(60, 180.0, false)) {
        if (ct - prevTime >= localRate) {
          distance = getDistance();
          if (distance > 30) {
            stoppedAtEnd = false;
            integral = 0; lastError = 0;
            currentState = FOLLOWLINE;
            checkSubState = 0;
          } else {
            prevTime = ct;
            resetTurn(180.0);
            checkSubState = 3;
          }
        }
      } else {
        turn360(60, 180.0, false);
      }
      break;
    case 3:
      if (turn360(60, 180.0, true)) {
        if (ct - prevTime >= localRate) {
          stoppedAtEnd = false;
          integral = 0; lastError = 0;
          checkSubState = 0;
        }
      } else {
         prevTime = ct;
        turn360(60, 180.0, true);
      }
      break;
    default:
      checkSubState = 0;
      currentState = FOLLOWLINE;
      break;
  }
}

void calibration() {
  if (!ss.begin()) {
    Serial.println("Erreur de connexion seesaw!");
    while (1);
  }
  Serial.println("Connexion OK !");
  Serial.println("=== Calibration automatique en cours... ===");

  unsigned long start = millis();

  resetTurn(360.0);
  while (millis() - start < CALIBRATION_TIME) {
    turn360(70, 360.0, false);
    for (int i = 0; i < NB_IR; i++) {
      int val = ss.analogRead(i);
      if (val < capteurs[i].valeurMin) capteurs[i].valeurMin = val;
      if (val > capteurs[i].valeurMax) capteurs[i].valeurMax = val;
    }
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
  currentState = FOLLOWLINE;
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

void stateManager(){
  switch (currentState) {
    case CALIBRATION:
      calibration();
      break;
    case CHECK:
      checkDirection();
      break;
    case FOLLOWLINE:
      avancer();
      break;
    case TURN:
      turn360(60, 180.0, true);
      break;
  }
}

void avancer(){
  is_turned = false;
  float position = calculerPositionLigne();

  // 4. Calcul PID
  float correction = computePID(position, 0.0f);

  // 5. Ajustement des moteurs
  suivreLigne(correction);
}

void loop() {
  ct = millis();
  encoderLeft.loop();
  encoderRight.loop();
  distance = getDistance();

 //if (distance < 33 && currentState == FOLLOWLINE){
 //   encoderLeft.setMotorPwm(0);
  //encoderRight.setMotorPwm(0);
//   if(!turnfirst){
//  currentState = CHECK;
//   }else{
//   turnfirst = !turnfirst;
//     resetTurn(180.0);
//     checkSubState = 2;
//     currentState = CHECK;
    
//   }

// }
  // 1. Lecture et normalisation
  lireEtNormaliser();
  stateManager();

  // calcule somme des valeurs normalisées pour detecter "plus de ligne"
  float sommeNorm = 0.0f;
  for (int i = 0; i < NB_IR; i++) sommeNorm += capteurs[i].valeurNorm;

  if (!endPending) {
    // 2. Détection fin de ligne : si sommeNorm < seuil pendant N cycles -> arrêt
    if (sommeNorm < END_LINE_SUM_THRESHOLD) {
      endLineCounter++;
      if (!stoppedAtEnd && endLineCounter >= END_LINE_CONFIRM) {
        endPending = true;
        endDetectMillis = ct;
        encoderLeft.setMotorPwm(0);
        encoderRight.setMotorPwm(0);
      }
    } else {
      endLineCounter = 0;
    }
  } else {
    if (ct - endDetectMillis >= END_DETECT_DELAY_MS) {
      if (!stoppedAtEnd && endLineCounter >= END_LINE_CONFIRM) {
        // Confirmer fin de ligne -> arrêt et préparer le turn (180°)
        stoppedAtEnd = true;

        //Serial.println("=== FIN DE LIGNE DETECTEE : ARRET ===");
        //resetTurn(180.0);      // préparer un turn de ~180°
       // startCheckProcedure();
        currentState = CHECK ;  
        
      }
    }
    // Si on est arrêté parce qu'on a détecté la fin, on attend le retour de la ligne
    if (stoppedAtEnd && is_turned) {
      // si la ligne réapparait (sommeNorm suffisante) on reprend et réinitialise PID
      if (sommeNorm >= END_LINE_SUM_THRESHOLD) {
         encoderLeft.setMotorPwm(0);
        encoderRight.setMotorPwm(0);
        stoppedAtEnd = false;
        endLineCounter = 0;
        integral = 0;
        lastError = 0;
        currentState = FOLLOWLINE; // <-- important : reprendre le suivi
        Serial.println("=== LIGNE RETROUVEE : REPRISE ===");
        Serial.println("Etat -> FOLLOWLINE");
      }
    }
  }
}
