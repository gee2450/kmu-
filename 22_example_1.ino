#include <Servo.h> //[3104] Servo header File include


/////////////////////////////
// Configurable parameters //
/////////////////////////////


// Arduino pin assignment
#define PIN_LED 9 // [3110] 9번핀 LED 연결        
#define PIN_SERVO 10 // [3110] 10번핀 서보 연결
#define PIN_IR A0 //[3104] 적외선 거리센서 PIN - Analog0 정의 


// Framework setting
#define _DIST_TARGET 255 //[3104] 탁구공을 위치 시킬 목표 
#define _DIST_MIN 100 //[3117] 거리 최소값
#define _DIST_MAX 410 //[3117] 거리 최대값


// Distance sensor
#define _DIST_ALPHA 0.9  //[3099] EMA 필터링을 위한 alpha 값


// Servo range
#define _DUTY_MIN 1890                 //[3100] 최저 서보 위치
#define _DUTY_NEU 1590                 //[3100] 중립 서보 위치
#define _DUTY_MAX 1350                 //[3100] 최대 서보 위치


// Servo speed control
#define _SERVO_ANGLE 30
#define _SERVO_SPEED 30


// Event periods
#define _INTERVAL_DIST 20 //[3099] 각 event 사이에 지정한 시간 간격
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100 


// PID parameters
#define _KP 0.04


// dist 100, 400mm 일때 값, 각자 a,b로 수정
// define으로 해야 할지, int로 해야할지 고민되네요.. // 큰 상관은 없겠지만, 값이 런타임에서 변화하지 않으니 define쪽에 맞다고 생각합니다 // 넵!
#define a 105
#define b 430

// value 보정값을 위한 setting value 개수
#define SEQ_SIZE 10


//////////////////////
// global variables //
//////////////////////


// Servo instance
Servo myservo;


// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema, dist_prev; //[1928] 측정된 값과 ema 필터를 적용한 값


// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
//[3104] 각 event의 진행 시간 저장 변수 
bool event_dist, event_servo, event_serial; 
//[3104] 각 event의 시간체크를 위한 변수 (ex_20초 주기 >> 0초(True,시작), 10초(False), 20초(True))


// Servo speed control
int duty_chg_per_interval; // [3116] 주기 당 서보 duty값 변화량
int duty_target, duty_curr; //[1928] 목표 위치와 현재 위치


// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

//error_curr: 현재 측정값과 목표값의 차이
//error_prev: 직전에 구한 차이로, P제어에서는 사용하지 않을 것임
//control: PID제어의 결과로 얻은 제어값
//pterm: Proportional term, 현재 상태의 error값으로부터 얻은 Proportional gain을 저장하는 변수
// [3099]


// value 보정값을 위한 setting value
// sensor values
float x[SEQ_SIZE] = {70.53, 87.3,  120.5, 157.0, 174.4, 
                     211.0, 260.6, 366.0, 446.6, 464.7};
// real values
float y[SEQ_SIZE] = {100.0, 125.0, 150.0, 200.0, 250.0, 
                     275.0, 300.0, 350.0, 400.0, 440.0};



void setup() {
// initialize GPIO pins for LED and attach servo 
  myservo.attach(PIN_SERVO); // attach servo
  pinMode(PIN_LED,OUTPUT); // initialize GPIO pins


// initialize global variables
  event_dist = event_servo = event_serial = true;


// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);
  duty_curr = _DUTY_NEU;


// initialize serial port
  Serial.begin(57600);


// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MIN - _DUTY_MAX) * (_SERVO_SPEED / _SERVO_ANGLE) * 1.75 * (_INTERVAL_SERVO / 1000.0); 

}



void loop() {
/////////////////////
// Event generator //
///////////////////// 


  unsigned long time_curr = millis();

  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO ){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL ){
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }


////////////////////
// Event handlers //
////////////////////


  if(event_dist) {
    event_dist = false;
  // get a distance reading from the distance sensor
    dist_ema = ir_distance_filtered(); // [3099] dist_ema?

  // PID control logic
    error_curr = dist_ema - _DIST_TARGET;
    pterm = error_curr;// * (pow(error_curr,3)/pow(20.5, 3)); //;
        // [3099]
    iterm = 0;
    dterm = 0;
    control = _KP * pterm + iterm + dterm;


  // duty_target = f(duty_neutral, control)
    //duty_target = control;
  // target보다 멀리 있음
    if (control>0){
      duty_target = _DUTY_NEU + (_DUTY_NEU - _DUTY_MAX) * _SERVO_ANGLE / 180.0 * control;
    }
  // target보다 가까이 있음
    else {
      duty_target = _DUTY_NEU + (_DUTY_MIN - _DUTY_NEU) * _SERVO_ANGLE / 180.0 * control;
    }
    //duty_target = ((control>0)?(_DUTY_NEU - _DUTY_MAX)*_SERVO_ANGLE / 180.0:(_DUTY_MIN - _DUTY_NEU) * _SERVO_ANGLE / 180.0) * control;
  // [3099] 확실하지 않음, 비례이득의 비대칭 해결가능
  // duty_neutral?


  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target < _DUTY_MAX){
      duty_target = _DUTY_MAX;
    }
    else if(duty_target > _DUTY_MIN){
      duty_target = _DUTY_MIN;
    }

  }
  
  if(event_servo) {
    event_servo=false;
    
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target>duty_curr) {
        duty_curr += duty_chg_per_interval;
        if(duty_target < duty_curr){
          duty_curr = duty_target;
        }
     }
    else {
        duty_curr -= duty_chg_per_interval;
        if(duty_target > duty_curr){
          duty_curr = duty_target;
        }
    }
    
    // update servo position
     myservo.writeMicroseconds(duty_curr);
  }


   if(event_serial) {
    event_serial = false; //[3117] // 이거 맞나요? // 저도 이렇게 했어요
    // 아래 출력문은 수정없이 모두 그대로 사용하기 바랍니다.
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    
    //Serial.print(",pterm:");
    //Serial.print(map(pterm,-1000,1000,510,610));
    
    Serial.print(",duty_target:");
    //Serial.print(duty_target);
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    //Serial.print(duty_curr);
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}


float ir_distance(void){ // return value unit: mm
  float value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return value;
}
//[3099]

float ir_distance_sequence(void){
  float real_value;
  int s = 0, e = SEQ_SIZE - 1, m;

  float value = ir_distance();
  
  // binary search, value가 어떤 거리 값인지 확인
  while(s <= e){
    m = (s + e) / 2;
    if(value < x[m]){ e = m - 1; }
    else if(value > x[m+1]){ s = m + 1; }
    else{ break; }
  }
  // out of sequence range: set value to neutral position
  
  // 센서 전원 Off 시, 오차값 발생으로 구간에서 벗어났을 시 등 
  if(s > e){
    if(value > 2000.0 || value < -2000.0) real_value = _DIST_TARGET;
    else if(s == 0) real_value = _DIST_MIN; 
    else real_value = _DIST_MAX;
  }
  
  // calculate real values
  real_value = (y[m+1]-y[m])/(x[m+1]-x[m]) * (value-x[m]) + y[m];
  
  return real_value;
}


float ir_distance_filtered(void){ // return value unit: mm
  dist_raw = ir_distance_sequence();
  if (dist_raw > _DIST_MAX) dist_raw = _DIST_MAX;
  else if (dist_raw < _DIST_MIN) dist_raw = dist_prev;
  else dist_prev = dist_raw;
  
  return _DIST_ALPHA * dist_raw + (1 - _DIST_ALPHA) * dist_ema;
}
