// Arduino pin assignment
#include <Servo.h>
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9
float raw_ema = 0;
float alpha = 0.5;
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(1350);
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  Serial.print("dist:");
  Serial.print(raw_dist);
  float raw_cm = (raw_dist - 66)/(364 - 66) * 35 + 10;
  Serial.print(" cm:");
  Serial.print(raw_cm);
  raw_ema = (raw_cm)*alpha + (raw_ema)*(1-alpha);
  Serial.print(" ema:");
  Serial.println(raw_ema);

  if(raw_ema < 25.5)
    myservo.writeMicroseconds(1690);
  else
    myservo.writeMicroseconds(1010);
  delay(20);
}
