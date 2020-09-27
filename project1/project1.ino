#define PIN_LED 7 // LED connected to digital pin 7

int set_period(int period) { // period: 100 to 10000 (unit: us)
  return period;
}

double set_p = set_period(10000); // set a period
double upper_p = 1000000/set_p/2/100;

void set_duty(int duty){ // duty: 0 to 100 (unit: %)
  for (int i=0 ; i < upper_p ; i += 1){
    digitalWrite(PIN_LED, 0);
    delayMicroseconds(set_p/100*duty);
    digitalWrite(PIN_LED, 1);
    delayMicroseconds(set_p/100*(100-duty));
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  
  Serial.begin(115200);
  while (!Serial){
    ; //wait for serial port to connect.
  }
  Serial.println("start");
}

void loop() {
  for (int duty = 0 ; duty < 100; duty += 1) {
    set_duty(duty);
  }
  for (int duty = 100 ; duty > 0; duty -= 1) {
    set_duty(duty);
  }
  Serial.println("finish");
}
