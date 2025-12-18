#include <Adafruit_seesaw.h>
#include <MeAuriga.h>

#define NB_IR 5
#define CALIBRATION_TIME 5000  
#define PULSE 9
#define RATIO 39.267


enum AppState { CALIBRATION,TURN,FOLLOWLINE};

AppState currentState = CALIBRATION;
short speed;

MeEncoderOnBoard encoderLeft(SLOT2);
MeEncoderOnBoard encoderRight(SLOT1);
Adafruit_seesaw ss;
MeGyro gyro(0, 0x69);

  unsigned long previousTime =0;
  unsigned long ct;

  static double error;
  static double lastAngle; 
  static double cumulativeAngle;
   static double targetAngle = 0;
  static bool is_turned = false;

void rightEncoderInterrupt(void)
{
  if(digitalRead(encoderRight.getPortB()) == 0)
  {
    encoderRight.pulsePosMinus();
  }
  else
  {
    encoderRight.pulsePosPlus();;
  }
}

void leftEncoderInterrupt(void) {
  if(digitalRead(encoderLeft.getPortB()) == 0)
  {
    encoderLeft.pulsePosMinus();
  }
  else
  {
    encoderLeft.pulsePosPlus();
  }
}

// ************* DÉBUT ************

void encoderConfig() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);
  
  encoderRight.setPulse(PULSE);
  encoderLeft.setPulse(PULSE);
  
  encoderRight.setRatio(RATIO);
  encoderLeft.setRatio(RATIO);
  
  encoderRight.setPosPid(1.8,0,1.2);
  encoderLeft.setPosPid(1.8,0,1.2);
  
  encoderRight.setSpeedPid(0.18,0,0);
  encoderLeft.setSpeedPid(0.18,0,0);
  
  // DÉBUT : Ne pas modifier ce code!
  // Configuration de la fréquence du PWM
  // Copier-coller ce code si on désire
  // travailler avec les encodeurs
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  // FIN : Ne pas modifier ce code!
}



struct Capteur {
  int valeurMin = 1023;
  int valeurMax = 0;
  int valeurLue = 0;
  float valeurNorm = 0;
};

Capteur capteurs[NB_IR];

int vitesseBase = 100;  

// PID
float kp = 0.2;
float ki = 0.0;
float kd = 0.05;
float integral = 0;
float lastError = 0;
 static bool firstRun = true;


const float END_LINE_SUM_THRESHOLD = 50.0f; 
const int END_LINE_CONFIRM = 5;             
int endLineCounter = 0;
bool stoppedAtEnd = false;

double getUnwrappedAngle() {
  double angle = gyro.getAngleZ();
  double delta = angle - lastAngle;

  // Si ça saute de +179 → -179 -> on corrige
  if (delta < -180) delta += 360;

  // Si ça saute de -179 → +179 -> on corrige
  if (delta > 180) delta -= 360;

  cumulativeAngle += delta;
  lastAngle = angle;

  return cumulativeAngle;
}
void resetTurn360() {
    gyro.resetData();
    cumulativeAngle = 0;
    lastAngle = gyro.getAngleZ();
    is_turned = false;
    targetAngle = 185;
}



bool turn360(short baseSpeed = 40) {
  static bool firstRun = true;
  const double tolerance = 15.0;   

  gyro.fast_update();

  if (firstRun) {
    firstRun = false;
    cumulativeAngle = 0;
    gyro.resetData();
    lastAngle = gyro.getAngleZ();
    targetAngle = 360.0;
    Serial.println(">>> Starting 360 turn");
  }

  double currentAngle = getUnwrappedAngle();
  error = targetAngle - currentAngle;

  // Stop condition
  if (fabs(error) < tolerance) {
    encoderLeft.setMotorPwm(0);
    encoderRight.setMotorPwm(0);
    currentState = FOLLOWLINE;
    is_turned = true;
     Serial.println(currentState);
    Serial.println(">>> 360 finished!");
    return true;
  }

  double Kp = 1.5;
  int pwm = constrain(baseSpeed + Kp * error, 40, 120);

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


// Normalisation


void calibration(){
if (!ss.begin()) {
    Serial.println("Erreur de connexion seesaw!");
    while (1);
  }
  Serial.println("Connexion OK !");
  Serial.println("=== Calibration automatique en cours... ===");

  unsigned long start = millis();
  while (millis() - start < CALIBRATION_TIME) {
    turn360(70);
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
}

void lireEtNormaliser() {
  for (int i = 0; i < NB_IR; i++) {
    capteurs[i].valeurLue = ss.analogRead(i);

    // Inversé : noir = 1000, blanc = 0
    capteurs[i].valeurNorm =
      (float)(capteurs[i].valeurMax - capteurs[i].valeurLue) * 1000.0 / 
      (float)(capteurs[i].valeurMax - capteurs[i].valeurMin);

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
  float rightSpeed = -(vitesseBase - correction);  // sens opposé

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  encoderLeft.setMotorPwm(leftSpeed);
  encoderRight.setMotorPwm(rightSpeed);
}

void stateManager(){
    switch (currentState) {
    case CALIBRATION:
       calibration();
      break;
    case FOLLOWLINE:
      avancer();
      break;
    case TURN:
      turn360(60);
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
  encoderLeft.loop();
  encoderRight.loop();
  
  // 1. Lecture et normalisation
  lireEtNormaliser();
  stateManager();

 

  // calcule somme des valeurs normalisées pour detecter "plus de ligne"
  float sommeNorm = 0.0f;
  for (int i = 0; i < NB_IR; i++) sommeNorm += capteurs[i].valeurNorm;

  // 2. Détection fin de ligne : si sommeNorm < seuil pendant N cycles -> arrêt
  if (sommeNorm < END_LINE_SUM_THRESHOLD) {
    endLineCounter++;
  } else {
    endLineCounter = 0;
  }

  if (!stoppedAtEnd && endLineCounter >= END_LINE_CONFIRM) {
    // Confirmer fin de ligne -> arrêt
    stoppedAtEnd = true;
    encoderLeft.setMotorPwm(0);
    encoderRight.setMotorPwm(0);
    Serial.println("=== FIN DE LIGNE DETECTEE : ARRET ===");
    resetTurn360();
    currentState = TURN;
    Serial.println(currentState);
  }

  // Si on est arrêté parce qu'on a détecté la fin, on attend le retour de la ligne
  if (stoppedAtEnd && is_turned) {
    // si la ligne réapparait (sommeNorm suffisante) on reprend et réinitialise PID
    if (sommeNorm >= END_LINE_SUM_THRESHOLD) {
      stoppedAtEnd = false;
      endLineCounter = 0;
      integral = 0;
      lastError = 0;
      Serial.println("=== LIGNE RETROUVEE : REPRISE ===");
    }
  }

}