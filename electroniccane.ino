#include "Wire.h"

#define BUTTON_LED_PIN 12
#define LED_PIN 9
#define LED_STRIP_PIN 3
#define BUTTON_SPEAKER_PIN 2
#define SPEAKER_PIN 4
#define LDR_PIN A7

// MPU6050 I2C Address
const int MPU_addr = 0x68;

// MPU sensor variables
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

boolean fall = false;  //stores if a fall has occurred

const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA = ',';
const char SEMICOLON = ';';
const char BLANK = ' ';
const char PERIOD = '.';

void morseSOS() {
  int sm = 150;    // ms (short morse)
  int lm = 400;    // ms (long morse)
  int pbl = 200;   // ms (pause between letters)
  int pbw = 2000;  // ms (pause between words)

  for (int i = 0; i < 3; i++) {
    for (int i = 0; i < 3; i++) {
      tone(SPEAKER_PIN, 800);
      lightsOn();
      delay(sm);
      noTone(SPEAKER_PIN);
      lighsOff();
      delay(pbl);
    }
    for (int i = 0; i < 3; i++) {
      tone(SPEAKER_PIN, 800);
      lightsOn();
      delay(lm);
      noTone(SPEAKER_PIN);
      lighsOff();
      delay(pbl);
    }
    for (int i = 0; i < 3; i++) {
      tone(SPEAKER_PIN, 800);
      lightsOn();
      delay(sm);
      noTone(SPEAKER_PIN);
      lighsOff();
      delay(pbl);
    }
    delay(pbw);
  }
}

void lightsOn() {
  digitalWrite(LED_PIN, HIGH);
  analogWrite(LED_STRIP_PIN, 255);
}

void lighsOff() {
  digitalWrite(LED_PIN, LOW);
  analogWrite(LED_STRIP_PIN, 0);
}

void mpu_read() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void mpu_calibration() {
  ax = (AcX - 234) / 16384.00;
  ay = (AcY - 3652) / 16384.00;
  az = (AcZ + 2375) / 16384.00;

  gx = (GyX + 138) / 131.07;
  gy = (GyY - 32) / 131.07;
  gz = (GyZ) / 131.07;
}

void checkFall() {
  float Raw_AM = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  int AM = Raw_AM * 10;                                   // as values are within 0 to 1, I multiplied it by for using if else conditions
  float angleChange = pow(pow(gx, 2) + pow(gz, 2), 0.5);  // do not consider gy, since is the rotation of the candle

  Serial.print("MPU Read:");
  Serial.print(BLANK);
  Serial.print(LBRACKET);
  Serial.print("ax:");
  Serial.print(ax);
  Serial.print(BLANK);
  Serial.print("ay:");
  Serial.print(ay);
  Serial.print(BLANK);
  Serial.print("az:");
  Serial.print(az);
  Serial.print(BLANK);
  Serial.print("gx:");
  Serial.print(gx);
  Serial.print(BLANK);
  Serial.print("gy:");
  Serial.print(gy);
  Serial.print(BLANK);
  Serial.print("gz:");
  Serial.print(gz);
  Serial.print(RBRACKET);
  Serial.print(SEMICOLON);
  Serial.print(BLANK);
  Serial.print("AM:");
  Serial.print(BLANK);
  Serial.print(AM);
  Serial.print(SEMICOLON);
  Serial.print(BLANK);
  Serial.print("angleChange:");
  Serial.print(BLANK);
  Serial.print(angleChange);
  Serial.println(BLANK);

  if ((AM <= 3 || AM >= 17) && (angleChange >= 120)) {
    fall = true;
  };
}

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);

  // Initialize MPU-6050
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.begin(9600);

  pinMode(BUTTON_LED_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_SPEAKER_PIN, INPUT_PULLUP);
  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(LDR_PIN, INPUT);
}

bool userLedON;
bool userLedOFF;

void loop() {
  mpu_read();
  mpu_calibration();
  checkFall();
  if (fall == true) {
    Serial.println("FALL DETECTED");
    morseSOS();
    fall = false;
  }

  int ldrValue = analogRead(LDR_PIN);
  Serial.print("LDR Read:");
  Serial.print(BLANK);
  Serial.print(LBRACKET);
  Serial.print(ldrValue);
  Serial.print(RBRACKET);
  Serial.println(BLANK);
  if (ldrValue > 900) {
    userLedON = false;
    if (!userLedOFF) {
      lightsOn();
    }
  }
  if (ldrValue < 800) {
    userLedOFF = false;
    if (!userLedON) {
      lighsOff();
    }
  }

  int buttonLEDValue = digitalRead((BUTTON_LED_PIN));
  if (buttonLEDValue == LOW) {
    int ledState = digitalRead(LED_PIN);

    if (ledState == HIGH) {
      lighsOff();
      userLedOFF = true;
    }
    if (ledState == LOW) {
      lightsOn();
      userLedON = true;
    }
    delay(200);
  }

  int buttonSpeakerValue = digitalRead((BUTTON_SPEAKER_PIN));
  if (buttonSpeakerValue == LOW) {
    morseSOS();
  }

  delay(100);  // delay in between loops for stability
}
