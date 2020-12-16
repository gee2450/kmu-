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
#define _DIST_MAX 400 //[3117] 거리 최대값


// Distance sensor
#define _DIST_ALPHA 0.35  //[3099] EMA 필터링을 위한 alpha 값
#define _SENSOR_ALPHA 0.6

// Servo range
#define _DUTY_MIN 1880                 //[3100] 최저 서보 위치
#define _DUTY_NEU 1520                 //[3100] 중립 서보 위치
#define _DUTY_MAX 1280                 //[3100] 최대 서보 위치


// Servo speed control
#define _SERVO_ANGLE 50
#define _SERVO_SPEED 75


// Event periods
#define _INTERVAL_DIST 20 //[3099] 각 event 사이에 지정한 시간 간격
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100 


// PID parameters
#define _KP 0.8
#define _KI 0.007
#define _KD 100
// 0.75, 0.007, 120
// iterm 최대값
#define _ITERM_MAX 500


// 적외선 센서 노이즈 보정
#define DELAY_MICROS  1500 // under_noise_filter 때 delay time


//////////////////////
// global variables //
//////////////////////


// Servo instance
Servo myservo;


// Distance sensor
float dist_target=_DIST_TARGET; // location to send the ball
float dist_raw, dist_ema, dist_prev; //[1928] 측정된 값과 ema 필터를 적용한 값


// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
//[3104] 각 event의 진행 시간 저장 변수 
bool event_dist, event_servo, event_serial; 
//[3104] 각 event의 시간체크를 위한 변수 (ex_20초 주기 >> 0초(True,시작), 10초(False), 20초(True))


// Servo speed control
int duty_chg_per_interval_up, duty_chg_per_interval_down; // [3116] 주기 당 서보 duty값 변화량
int duty_chg_per_interval_up_max, duty_chg_per_interval_down_max;
int duty_target, duty_curr, duty_ema; //[1928] 목표 위치와 현재 위치
int duty_val;


// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;
//error_curr: 현재 측정값과 목표값의 차이
//error_prev: 직전에 구한 차이로, P제어에서는 사용하지 않을 것임
//control: PID제어의 결과로 얻은 제어값
//pterm: Proportional term, 현재 상태의 error값으로부터 얻은 Proportional gain을 저장하는 변수
// [3099]

const float coE[] = {-0.0000004, 0.0005260, 0.8327958, 35.6473257};
float samples_num = 3;

void setup() {
// initialize GPIO pins for LED and attach servo 
  myservo.attach(PIN_SERVO); // attach servo
  pinMode(PIN_LED,OUTPUT); // initialize GPIO pins


// initialize global variables
  event_dist = event_servo = event_serial = true;
  pterm = iterm = dterm = 0;

// move servo to neutral position
  //myservo.writeMicroseconds(_DUTY_NEU);
  
  duty_curr = _DUTY_NEU;
  error_curr = error_prev = 0;
  
// initialize serial port
  Serial.begin(57600);


// convert angle speed into duty change per interval.
// 최대 최소 각 불균형이 있어, 각각 interval을 구했습니다.
  duty_chg_per_interval_up = duty_chg_per_interval_up_max = 
             (_DUTY_NEU - _DUTY_MAX) * ((float)_SERVO_SPEED / _SERVO_ANGLE * 2 ) 
              * 1.4 * (_INTERVAL_SERVO / 1000.0);
  duty_chg_per_interval_down = duty_chg_per_interval_down_max = 
             (_DUTY_MIN - _DUTY_NEU) * ((float)_SERVO_SPEED / _SERVO_ANGLE * 2 ) 
              * (_INTERVAL_SERVO / 1000.0);  
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
    pterm = _KP * error_curr;  // [3099]
    iterm += _KI * error_curr;
    dterm = _KD * (error_curr - error_prev);

    if (iterm*error_curr<0) iterm /= 2;
    
    if (iterm>=_ITERM_MAX) iterm = _ITERM_MAX;
    else if (iterm<= -_ITERM_MAX) iterm = -_ITERM_MAX;
    
    control = pterm + iterm + dterm;
    //control = map(control,-1100,1100,-350,350);

    duty_val = _DUTY_NEU - control;
    
    duty_target = sensor_filtered(duty_val);

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target < _DUTY_MAX) duty_target = _DUTY_MAX;
    else if(duty_target > _DUTY_MIN) duty_target = _DUTY_MIN;

    error_prev = error_curr;

  }
  
  if(event_servo) {
    event_servo=false;
    
  // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target>duty_curr) {
      duty_curr += duty_chg_per_interval_down;
      if(duty_target < duty_curr){
        duty_curr = duty_target;
      }
    }
    else if (duty_target<duty_curr){
      duty_curr -= duty_chg_per_interval_up;
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
    if (map(dterm,-1000,1000,510,610)<0 | map(dterm,-1000,1000,510,610)>800) dterm = 0;
    
    Serial.print("IR:");
    Serial.print(dist_ema);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");

  }
}


float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;

  val = coE[0] * pow(val, 3) + coE[1] * pow(val, 2) + coE[2] * val + coE[3];

  return val;
}
//[3099]

// ================
float under_noise_filter(void){
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}
float ir_distance_filtered(void){
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }

  // eam 필터 추가
  dist_ema = _DIST_ALPHA*lowestReading + (1-_DIST_ALPHA)*dist_ema;

  if (dist_ema>_DIST_MAX) dist_ema=_DIST_MAX;
  return dist_ema;
}
//===================================================


float sensor_filtered(float sensor_val){
  return _SENSOR_ALPHA * sensor_val + (1 - _SENSOR_ALPHA) * duty_target;
}
