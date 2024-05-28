const int estep = 1;
const int edir = 0;
const int en = 14;
const int LEDD_BUILTIN = 27;

unsigned long milli = 0;
unsigned long milliLed = 0;

void setup(){
  pinMode(en, OUTPUT);
  pinMode(estep, OUTPUT);
  pinMode(edir, OUTPUT);
  pinMode(LEDD_BUILTIN, OUTPUT);

  digitalWrite(en, LOW);
  digitalWrite(edir, HIGH);
  milli = millis();
  milliLed = millis();
}

void loop(){
  digitalWrite(estep, HIGH);
  delayMicroseconds(20);
  digitalWrite(estep, LOW);
  delayMicroseconds(20);

  if(millis() > milli + 1000){
    digitalWrite(edir, !digitalRead(edir));
    digitalWrite(LEDD_BUILTIN, HIGH);
    milliLed = millis();
    milli = millis();
  }
  if(millis() > milliLed + 10){
    digitalWrite(LEDD_BUILTIN, LOW);
  }
}