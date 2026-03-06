//--------Pin definitions for the L298P Motor Driver----
#define IN1 7    // Motor A direction pin 1
#define IN2 8     // Motor A direction pin 2
#define IN3 9     // Motor B direction pin 1
#define IN4 6     // Motor B direction pin 2
#define ENA 10     // Motor A speed (PWM)
#define ENB 11    // Motor B speed (PWM)
//-----------------------------------------------------

//--------Enter Line Details here---------
bool isBlackLine = true;             // Keep true for black line. Set false for white line
unsigned int lineThickness = 15;     // Line thickness in mm (10 to 35)
unsigned int numSensors = 5;         // Use 5 or 7 sensors
//-----------------------------------------

int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 150;
int currentSpeed = 30;
int sensorWeight[7] = { 4, 2, 1, 0, -1, -2, -4 };
int activeSensors;
float Kp = 0.08;
float Kd = 0.15;
float Ki = 0;

int onLine = 1;
int minValues[7], maxValues[7], threshold[7], sensorValue[7], sensorArray[7];

void setup() {
  Serial.begin(9600);

  // Motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Pushbuttons & LED
  pinMode(11, INPUT_PULLUP);  // Pushbutton
  pinMode(12, INPUT_PULLUP);  // Pushbutton
  pinMode(13, OUTPUT);        // LED

  // Adjust weights for 5 sensors
  lineThickness = constrain(lineThickness, 10, 35);
  if (numSensors == 5) {
    sensorWeight[1] = 4;
    sensorWeight[5] = -4;
  }
}

void loop() {
  while (digitalRead(1)) {}  // Wait for button press
  delay(1000);
  calibrate();
  while (digitalRead(2)) {}  // Wait for another button press
  delay(1000);

  while (true) {
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;

    if (onLine) {
      linefollow();
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
      if (error > 0) {
        motor1run(-50);
        motor2run(lfSpeed);
      } else {
        motor1run(lfSpeed);
        motor2run(-50);
      }
    }
  }
}

void linefollow() {
  error = 0;
  activeSensors = 0;

  int startIdx = (numSensors == 5) ? 1 : 0;
  int endIdx = (numSensors == 5) ? 6 : 7;

  for (int i = startIdx; i < endIdx; i++) {
    error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
    activeSensors += sensorArray[i];
  }

  if (activeSensors > 0) {
    error = error / activeSensors;
  }

  P = error;
  I += error;
  D = error - previousError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  lsp = constrain(lsp, 0, 255);
  rsp = constrain(rsp, 0, 255);

  motor1run(lsp);
  motor2run(rsp);
}

void calibrate() {
  for (int i = 0; i < 7; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  for (int i = 0; i < 10000; i++) {
    motor1run(50);
    motor2run(-50);

    for (int j = 0; j < 7; j++) {
      int val = analogRead(j);
      minValues[j] = min(minValues[j], val);
      maxValues[j] = max(maxValues[j], val);
    }
  }

  for (int i = 0; i < 7; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motor1run(0);
  motor2run(0);
}

void readLine() {
  onLine = 0;
  int startIdx = (numSensors == 5) ? 1 : 0;
  int endIdx = (numSensors == 5) ? 6 : 7;

  for (int i = startIdx; i < endIdx; i++) {
    int raw = analogRead(i);
    sensorValue[i] = isBlackLine ? map(raw, minValues[i], maxValues[i], 0, 1000)
                                 : map(raw, minValues[i], maxValues[i], 1000, 0);
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;

    if ((isBlackLine && sensorArray[i]) || (!isBlackLine && !sensorValue[i])) {
      onLine = 1;
    }
  }
}

void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(motorSpeed));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(motorSpeed));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}