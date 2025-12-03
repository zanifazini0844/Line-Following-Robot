// Motor Pins
#define enA 3     // Right Motor Enable Pin
#define in1 4     // Right Motor in1
#define in2 5     // Right Motor in2
#define enB 9     // Left Motor Enable Pin
#define in3 6    // Left Motor in3
#define in4 7    // Left Motor in4

// Sensor Pins (connected to digital pins)
#define ir1 13     // Leftmost Sensor
#define ir2 12     // Left Sensor 
#define ir3 11    // Middle Sensor
#define ir4 10    // Right Sensor
#define ir5 8    // Rightmost Sensor

int baseSpeed = 140;
int maxSpeed = 200;

float Kp = 10.0;   // Proportional constant
float Ki = 0.9;   // Integral constant
float Kd = 5.0;   // Derivative constant

int previousError = 0;
float integral = 0;
int lastKnownError = 0;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);

  Serial.begin(9600);
}

void loop() {
  int s1 = digitalRead(ir1) == LOW ?  1 : 0;
  int s2 = digitalRead(ir2) == LOW ?  1 : 0;
  int s3 = digitalRead(ir3) == LOW ?  1 : 0;
  int s4 = digitalRead(ir4) == LOW ?  1 : 0;
  int s5 = digitalRead(ir5) == LOW ?  1 : 0;

  // Calculate error
  int weights[5] = { -2, -1, 0, 1, 2 };
  int error = 0, totalActiveSensors = 0;
  int sensors[5] = { s1, s2, s3, s4, s5 };

  for (int i = 0; i < 5; i++) {
    error += sensors[i] * weights[i];
    totalActiveSensors += sensors[i];
  }

  if (totalActiveSensors == 0) {
    if (lastKnownError < 0) setMotors(-baseSpeed / 2, baseSpeed / 2);
    else if (lastKnownError > 0) setMotors(baseSpeed / 2, -baseSpeed / 2);
    else setMotors(-baseSpeed / 2, baseSpeed / 2);
    return;
  }

  error /= totalActiveSensors;
  lastKnownError = error;

  float proportional = error * Kp;
  integral = constrain(integral + error, -50, 50);
  float integralTerm = integral * Ki;
  float derivative = (error - previousError) * Kd;
  int pid = proportional + integralTerm + derivative;

  previousError = error;

  int dynamicBaseSpeed = map(abs(error), 0, 2, baseSpeed, baseSpeed / 2);
  int leftSpeed = constrain(dynamicBaseSpeed + pid, 0, maxSpeed);
  int rightSpeed = constrain(dynamicBaseSpeed - pid, 0, maxSpeed);

  setMotors(leftSpeed, rightSpeed);

  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | PID: ");
  Serial.print(pid);
  Serial.print(" | Left: ");
  Serial.print(leftSpeed);
  Serial.print(" | Right: ");
  Serial.println(rightSpeed);
}

void setMotors(int leftSpeed, int rightSpeed) {
  analogWrite(enA, abs(rightSpeed));
  digitalWrite(in1, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(in2, rightSpeed > 0 ? LOW : HIGH);

  analogWrite(enB, abs(leftSpeed));
  digitalWrite(in3, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(in4, leftSpeed > 0 ? LOW : HIGH);
}