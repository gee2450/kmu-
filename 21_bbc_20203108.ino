#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10

#define PIN_IR A0
#define PIN_LED 9

#define duty 1550
#define duty_purse 200

#define _DUTY_MAX duty-duty_purse
#define _DUTY_MIN duty+duty_purse

#define _SERVO_SPEED 60
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100 

#define _DIST_MIN 100
#define _DIST_MAX 400
#define _DIST_ALPHA 0.4

#define a 60 // when value 100
#define b 440 // when value 400
#define middle 216 // after handling when value 250

Servo myservo;

float dist_min, dist_max, dist_raw, dist_cali, dist_ema, dist_prev, alpha;
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; 

int duty_chg_per_interval;
int toggle_interval, toggle_interval_cnt;

void setup() {
// initialize servo
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(duty); 
  
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;
  dist_raw = dist_prev = dist_cali = dist_ema = 0.0;

  event_dist = event_servo = event_serial = true;
  
// initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MIN-_DUTY_MAX)*(_SERVO_SPEED / 180)*(_INTERVAL_SERVO / 1000);
}

void loop() {
  
  unsigned long time_curr = millis();
  
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  
  if (event_dist){
    event_dist = false;
    dist_cali = ir_distance();
    dist_ema = ir_distance_filtered();
  }
  
// output the read value to th eserial port
  if (event_serial){
    event_serial = false;
    Serial.print("min:0,max:500,dist_cali:");
    Serial.print(dist_cali);
    Serial.print(",dist_ema:");
    Serial.println(dist_ema);
  }

// Servo
  if(event_servo){
    event_servo = false;
    if (dist_ema > middle){
      myservo.writeMicroseconds(duty+duty_purse);
    }
    else{
      myservo.writeMicroseconds(duty-duty_purse);
    }
  }
  

// LED Set : OFF
  if( middle-30 < dist_ema && dist_ema < middle + 30) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);

}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return 100 + 300.0 / (b-a) * (val-a);
}

float ir_distance_filtered(void){ // return value unit: mm
  dist_raw =  ir_distance();
  if (dist_raw > dist_max) dist_raw = dist_max;
  else if (dist_raw < dist_min) dist_raw = dist_prev;
  else dist_prev = dist_raw;
  
  return alpha*dist_raw + (1 - alpha)*dist_ema;
}
