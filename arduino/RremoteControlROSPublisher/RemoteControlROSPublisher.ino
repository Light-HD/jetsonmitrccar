/*volatile int counter = 0;


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  pinMode(A4,INPUT_PULLUP);
  Serial.begin(9600);
  //attachInterrupt(digitalPinToInterrupt(3),count,CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(analogRead(A4));
  delay(500);
}



void count(){
  counter++;
}
*/
#define THROTTLE_INT_PIN 3
#define STEERING_INT_PIN 2


volatile int pwm_value_throttle = 0;
volatile int prev_time_throttle = 0;

volatile int pwm_value_steering = 0;
volatile int prev_time_steering = 0;

String throttle_string = "Throttle Duty Cycle:";
String steering_string = "Steering Duty Cycle:";

void setup() {
  Serial.begin(115200);
  // when pin D2 goes high, call the rising function
  attachInterrupt(digitalPinToInterrupt(THROTTLE_INT_PIN), rising_throttle, RISING);
  attachInterrupt(digitalPinToInterrupt(STEERING_INT_PIN),rising_steering, RISING);
}
 
void loop() { 
  //Serial.println(throttle_string + pwm_value_throttle + '\n' + steering_string + pwm_value_steering);
  Serial.print(pwm_value_throttle);
  Serial.print(' ');
  Serial.println(pwm_value_steering);
  delay(200);
}
 
void rising_throttle() {
  attachInterrupt(digitalPinToInterrupt(THROTTLE_INT_PIN), falling_throttle, FALLING);
  prev_time_throttle = micros();
}
 
void falling_throttle() {
  attachInterrupt(digitalPinToInterrupt(THROTTLE_INT_PIN), rising_throttle, RISING);
  pwm_value_throttle = micros()- prev_time_throttle;
  //Serial.println(pwm_value);
}


void rising_steering(){
  attachInterrupt(digitalPinToInterrupt(STEERING_INT_PIN), falling_steering, FALLING);
  prev_time_steering = micros();
}

void falling_steering(){
  attachInterrupt(digitalPinToInterrupt(STEERING_INT_PIN), rising_steering, RISING);
  pwm_value_steering = micros() - prev_time_steering;
}
