#define PIN_LED 7
unsigned int toggle=1;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 0);
  delay(1000);
}

void loop() {
  for (int i=0; i<11; i++){
    digitalWrite(PIN_LED,toggle);
    delay(100);
    toggle = (toggle+1)%2;
  }
  while(1){
    digitalWrite(PIN_LED, 1);
  }
}
