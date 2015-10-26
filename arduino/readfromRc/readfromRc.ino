//Input-pins
#define PWN_SPEED_PIN 14 //geschwindigkeit
#define PWN_ROLL_PIN 15 //seitenruder
#define PWN_YAW_PIN 16 //querruder ?
#define PWN_PITCH_PIN 17 //hoehenruder ?
#define PWN_RC_PIN 18 // rc on/off

//Output-pins
#define PWN_SPEED_PIN_OUTPUT 3 //geschwindigkeit
#define PWN_ROLL_PIN_OUTPUT 5 //seitenruder
#define PWN_YAW_PIN_OUTPUT 7 //querruder ?
#define PWN_PITCH_PIN_OUTPUT 9 //hoehenruder ?
 
volatile int pwm_speed_value = 0;
volatile int pwm_roll_value = 0;
volatile int pwm_yaw_value = 0;
volatile int pwm_pitch_value = 0;
volatile int pwm_rc_value = 0;

volatile bool rc_mode = false;
 
//int pwm_value;
 
void setup() {
  //input-pins
  pinMode(PWN_SPEED_PIN, INPUT);
  pinMode(PWN_ROLL_PIN, INPUT);
  pinMode(PWN_YAW_PIN, INPUT);
  pinMode(PWN_PITCH_PIN, INPUT);
  pinMode(PWN_RC_PIN, INPUT);
  pinMode(13, OUTPUT);
  //output-pins
  pinMode(PWN_SPEED_PIN_OUTPUT, OUTPUT);
  pinMode(PWN_ROLL_PIN_OUTPUT, OUTPUT);
  pinMode(PWN_YAW_PIN_OUTPUT, OUTPUT);
  pinMode(PWN_PITCH_PIN_OUTPUT, OUTPUT);


  //general
  Serial.begin(115200);
}
 
void loop() {
  
  pwm_rc_value = pulseIn(PWN_RC_PIN, HIGH);
  if(pwm_rc_value < 1100 && pwm_rc_value > 1000){
    rc_mode = true;
  }else if(pwm_rc_value > 1800 && pwm_rc_value < 1900){
    rc_mode = false;
  }
  
  if(rc_mode){
    digitalWrite(13,HIGH);
    rc_mode_controll();
  }else{
    digitalWrite(13,LOW);
    autoPilot();
  }
}

void rc_mode_controll(){
  int pwm_speed_value_ = pulseIn(PWN_SPEED_PIN, HIGH);
  int pwm_roll_value_ = pulseIn(PWN_ROLL_PIN, HIGH);
  int pwm_yaw_value_ = pulseIn(PWN_YAW_PIN, HIGH);
  int pwm_pitch_value_ = pulseIn(PWN_PITCH_PIN, HIGH);

  if(pwm_speed_value_ > 1000)
    pwm_speed_value = pwm_speed_value_;
  if(pwm_roll_value_ > 1000)
    pwm_roll_value = pwm_roll_value_;
  if(pwm_yaw_value_ > 1000)
    pwm_yaw_value = pwm_yaw_value_;
  if(pwm_pitch_value_ > 1000)
    pwm_pitch_value = pwm_pitch_value_;

  Serial.print(pwm_speed_value); Serial.print(" ");
  Serial.print(pwm_roll_value); Serial.print(" ");
  Serial.print(pwm_yaw_value); Serial.print(" ");
  Serial.print(pwm_pitch_value); Serial.println();
  
  //TODO
}

void autoPilot(){
  
}

void setPWMs(){
  analogWrite(PWN_SPEED_PIN_OUTPUT, pwm_speed_value);
  analogWrite(PWN_ROLL_PIN_OUTPUT, pwm_roll_value);
  analogWrite(PWN_YAW_PIN_OUTPUT, pwm_yaw_value);
  analogWrite(PWN_PITCH_PIN_OUTPUT, pwm_pitch_value);
  
}

