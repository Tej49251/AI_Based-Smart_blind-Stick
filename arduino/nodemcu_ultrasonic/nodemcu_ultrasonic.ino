// === PIN DEFINITIONS (Edit only if wiring changes) ===
#define trigPinL 18
#define echoPinL 19
#define trigPinC 21
#define echoPinC 22
#define trigPinR 25
#define echoPinR 26
#define buzzerPin 27

// === VARIABLES ===
int distanceL, distanceC, distanceR;

void setup() {
  Serial.begin(9600);
  pinMode(trigPinL, OUTPUT); pinMode(echoPinL, INPUT);
  pinMode(trigPinC, OUTPUT); pinMode(echoPinC, INPUT);
  pinMode(trigPinR, OUTPUT); pinMode(echoPinR, INPUT);
  pinMode(buzzerPin, OUTPUT);
}

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000); // Timeout to avoid hang
  return (duration > 0) ? duration * 0.034 / 2 : 999;
}

void loop() {
  distanceL = getDistance(trigPinL, echoPinL);
  distanceC = getDistance(trigPinC, echoPinC);
  distanceR = getDistance(trigPinR, echoPinR);

  Serial.print("L:"); Serial.print(distanceL);
  Serial.print(" C:"); Serial.print(distanceC);
  Serial.print(" R:"); Serial.println(distanceR);

  digitalWrite(buzzerPin, (distanceC < 100 && distanceC > 5) ? HIGH : LOW);

  delay(2500);
}