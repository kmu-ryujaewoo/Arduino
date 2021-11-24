#include <Servo.h>

#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

#define _INTERVAL_DIST 20 // USS interval (unit: ms)
#define _INTERVAL_SERVO 20 // servo interval (unit: ms)
#define _INTERVAL_SERIAL 20 // serial interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_TARGET 255 // target distance (unit: mm)
#define _DIST_MAX 450 // maximum distance to be measured (unit: mm)
#define _SERVO_SPEED 30

#define _DUTY_MIN 1010 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1350 // servo neutral position (90 degree)
#define _DUTY_MAX 1690 // servo full counterclockwise position (180 degree)

// global variables
float dist_min, dist_max, dist_center, dist_raw, dist_prev, dist_ema, duty_target, duty_neu, duty_curr; // unit: mm
float alpha;
float duty_chg_per_interval;
Servo myservo;
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; // unit: ms
bool event_dist, event_servo, event_serial;
float dist_error;
float _kp, pterm;

void setup() {
  pinMode(PIN_LED,OUTPUT);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  dist_center = _DIST_TARGET;
  dist_raw = dist_prev = dist_ema = dist_error = 0.0;
  alpha = 0.5;
  duty_target = duty_neu = duty_curr = _DUTY_NEU;
  _kp = 0.4;
  pterm = 0.0;
  
// initialize serial port
  Serial.begin(57600);  

// initialize event variables
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;

}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
  
void loop() {
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
    
  if(event_dist) {
    event_dist = false;
    dist_raw = (ir_distance() - 68) / (377 - 68) * 350 + 100;
    dist_ema = (dist_raw * alpha) + (dist_ema * (1-alpha));
    dist_error = dist_center - dist_ema + 20;
    duty_curr = duty_target;
    duty_target = (dist_error / 174 * 350 * 2.5);
    if (duty_target + duty_neu < _DUTY_MIN) {
      duty_target = _DUTY_MIN; 
    }
    else if (duty_target + duty_neu > _DUTY_MAX) {
      duty_target = _DUTY_MAX;
    }
    else{
      duty_target += duty_neu;
    }
    pterm = _kp * dist_error;
  }
  
  if(event_servo) {
    event_servo = false;
    duty_curr = duty_curr + ((duty_target - duty_curr) * (_INTERVAL_SERVO / 1000.0) * (_SERVO_SPEED / 180.0));
    myservo.writeMicroseconds(duty_curr);
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("Min:0,Low:200,dist:");
    Serial.print(dist_raw);
    Serial.print(",pterm:"); 
    Serial.print(pterm);
    Serial.print(",duty_target:");
    Serial.print(duty_target);
    Serial.print(",duty_curr:");
    Serial.println(duty_curr);
  }
}
