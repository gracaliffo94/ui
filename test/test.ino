#include "Adafruit_VL53L0X.h"

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 2
#define SHT_LOX2 4

int number_lights_seen = 0;

int number_light_1 = 100;// da modificare quando inizia la gara
int number_light_2 = 100;
int number_light_3 = 100;
int number_light_4 = 100;
int number_light_5 = 100;
int number_light_6 = 100;


#define LASERS_DISTANCE 0.118
 
#define MOTOR_DX_IN1 5
#define MOTOR_DX_IN2 6
#define MOTOR_SX_IN1 9
#define MOTOR_SX_IN2 10
#define RIGHT_LED_PIN 13
#define FRONT_LEFT_LED_PIN A0
#define LEFT_LED_PIN A1
#define ON_RIGHT_WALL_DISTANCE 0.125
#define KP 0.37
#define KD 4
#define RIGHT_BASE_SPEED 165
#define LEFT_BASE_SPEED RIGHT_BASE_SPEED-30
#define ROTATION_SPEED 150
#define MAX_TURN_SPEED 150
#define FRONT_CM_DISTANCE_OBSTACLE_AVOIDANCE_THRESHOLD 15
#define ALARM_COUNTER_THRESHOLD 10
#define SEARCH_ROTATION_SPEED 115
#define TRIGGER_PORT 7
#define ECHO_PORT 8 

#define LIGHT_SENSOR_2 3
#define STATE_LIGHT_FOUND_LED_PIN 11
#define A0 A0
#define LIGHT_SENSOR_1 12
#define IGNORE_TIME_AFTER_LIGHT_FOUND 1700

struct Distance {
    short int front;
    short int rear;
};


//unsigned short previousMillis = 0;  // Salva il tempo dell'iterazione precedente
short int previousError = 0;
short int currentRightBaseSpeed = RIGHT_BASE_SPEED;
short int currentLeftBaseSpeed = LEFT_BASE_SPEED;
short int previousRearDistance = 8191;
short int previousFrontDistance = 8191;
short int frontAlarmCounter = 0;
short int rearAlarmCounter = 0;
bool wallFound = false;
bool first_light_not_found = true;
short int lastState = 0;
unsigned long lastTimeLightFound = millis();
// objects for the vl53l0x
bool notRotated = true;

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

Distance read_dual_sensors() {
  Distance d;
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  if(measure2.RangeStatus != 4) {     // if not out of range
    d.front = measure2.RangeMilliMeter;
  } else {
    d.front = 2000;
  }



  if(measure1.RangeStatus != 4) {
    d.rear = measure1.RangeMilliMeter;
  } else {
    d.rear = 2000;
  }
  
  d.front = constrain(d.front,0,2000);
  d.rear = constrain(d.rear,0,2000);

  Serial.print(F("Front: "));
  Serial.print(d.front);
  Serial.print(F(" "));
  Serial.print(F("Rear: "));
  Serial.print(d.rear);

  return d;
}

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }
  delay(1000); // Aspetta che tutto si stabilizzi

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  
  pinMode( TRIGGER_PORT, OUTPUT );
  pinMode( ECHO_PORT, INPUT );

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  
  
  Serial.println(F("Starting..."));
  setID();

  pinMode(STATE_LIGHT_FOUND_LED_PIN, OUTPUT);
  digitalWrite(STATE_LIGHT_FOUND_LED_PIN,HIGH);
  delay(300);
  digitalWrite(STATE_LIGHT_FOUND_LED_PIN,LOW);
  pinMode(MOTOR_DX_IN1, OUTPUT);
  pinMode(MOTOR_DX_IN2, OUTPUT);
  pinMode(MOTOR_SX_IN1, OUTPUT);
  pinMode(MOTOR_SX_IN2, OUTPUT);
  pinMode(FRONT_LEFT_LED_PIN, INPUT);
  pinMode(RIGHT_LED_PIN, INPUT);
  pinMode(LIGHT_SENSOR_1,INPUT);
  pinMode(LIGHT_SENSOR_2,INPUT);
  pinMode(LEFT_LED_PIN, INPUT);  
 
}

 
void moveForwardWithFeedback(short int pwm, bool emergencyFlagFeedback, short int error_dot){
    short int proportional_feedback = pwm*KP;
    short int derivative_feedback = error_dot*KD;
    short int r_speed = constrain(RIGHT_BASE_SPEED+proportional_feedback+derivative_feedback,0,255);
    short int l_speed = constrain(LEFT_BASE_SPEED-proportional_feedback-derivative_feedback,0,255);
    if (emergencyFlagFeedback){
      l_speed=l_speed/2;
    }
    Serial.print(" FEEDBACK R_PWM:");
    Serial.print(r_speed);
    Serial.print(" L_PWM:");
    Serial.println(l_speed);

    if (r_speed>0){
      analogWrite(MOTOR_DX_IN1, r_speed);
      analogWrite(MOTOR_DX_IN2, 0);
    } else {
      analogWrite(MOTOR_DX_IN1, 0);
      analogWrite(MOTOR_DX_IN2, -r_speed);
    }
    if (l_speed>0){
      analogWrite(MOTOR_SX_IN1, l_speed);
      analogWrite(MOTOR_SX_IN2, 0);
    } else {
      analogWrite(MOTOR_SX_IN1, 0);
      analogWrite(MOTOR_SX_IN2, -l_speed);
    }
}

void moveForward(){
  Serial.print(" FORWARD R_PWM:");
  Serial.print(RIGHT_BASE_SPEED);
  Serial.print(" L_PWM:");
  Serial.println(LEFT_BASE_SPEED);
  analogWrite(MOTOR_DX_IN1, RIGHT_BASE_SPEED + 10);
  analogWrite(MOTOR_DX_IN2, 0);
  analogWrite(MOTOR_SX_IN1, LEFT_BASE_SPEED);
  analogWrite(MOTOR_SX_IN2, 0);
  Serial.println("");
}

bool checkLight(){
  bool light_sensor_1 = digitalRead(LIGHT_SENSOR_1);
  bool light_sensor_2 = digitalRead(LIGHT_SENSOR_2);
  return light_sensor_1 || light_sensor_2;
}

bool checkFrontObstacle(){
  if (frontDistance()<FRONT_CM_DISTANCE_OBSTACLE_AVOIDANCE_THRESHOLD)
    return true;
  bool front_left_obstacle = 1-digitalRead(FRONT_LEFT_LED_PIN);
  bool right_obstacle = 1-digitalRead(RIGHT_LED_PIN);
  bool left_obstacle = 1-digitalRead(LEFT_LED_PIN);
  return left_obstacle || right_obstacle || front_left_obstacle;
}

void rotateLeft(short int speed){
  // Ruota in senso orario
  analogWrite(MOTOR_DX_IN1, speed+18);
  analogWrite(MOTOR_DX_IN2, 0);
  analogWrite(MOTOR_SX_IN1, 0);
  analogWrite(MOTOR_SX_IN2, speed);
}

void rotateRight(short int speed){
  // Ruota in senso orario
  analogWrite(MOTOR_DX_IN1, 0);
  analogWrite(MOTOR_DX_IN2, speed+18);
  analogWrite(MOTOR_SX_IN1, speed);
  analogWrite(MOTOR_SX_IN2, 0);
}
void rotate90CW(){
  // Ruota in senso orario
  analogWrite(MOTOR_DX_IN1, 0);
  analogWrite(MOTOR_DX_IN2, 168);
  analogWrite(MOTOR_SX_IN1, 140);
  analogWrite(MOTOR_SX_IN2, 0);
  delay(600);
}

void rotate90CCW(){
  // Ruota in senso antiorario
  analogWrite(MOTOR_DX_IN1, 168);
  analogWrite(MOTOR_DX_IN2, 0);
  analogWrite(MOTOR_SX_IN1, 0);
  analogWrite(MOTOR_SX_IN2, 140);
  delay(600);
}

void turnRight(int motionTime){
  // Ruota in senso orario
  analogWrite(MOTOR_DX_IN1, 0);
  analogWrite(MOTOR_DX_IN2, 0);
  analogWrite(MOTOR_SX_IN1, MAX_TURN_SPEED);
  analogWrite(MOTOR_SX_IN2, 0);
  delay(motionTime);
}
void stopMotors(){
  // Ruota in senso orario
  analogWrite(MOTOR_DX_IN1, 0);
  analogWrite(MOTOR_DX_IN2, 0);
  analogWrite(MOTOR_SX_IN1, 0);
  analogWrite(MOTOR_SX_IN2, 0);
}

short int frontDistance(){
  //porta bassa l'uscita del trigger
  digitalWrite( TRIGGER_PORT, LOW );
  //invia un impulso di 10microsec su trigger
  digitalWrite( TRIGGER_PORT, HIGH );
  delayMicroseconds( 10 );
  digitalWrite( ECHO_PORT, LOW );
  long duration = pulseIn( ECHO_PORT, HIGH );
  long r = 0.034 * duration / 2;
  return r;
}

void searchForWall(Distance d, short int diff){
  rotateLeft(SEARCH_ROTATION_SPEED);
  if ((d.front<1000 && d.rear<1000) && (diff>-25 && diff<25)){
    rotate90CW();
    wallFound = true;
  }
}


void greenLedThreeSecondsBlocking(){
  digitalWrite(STATE_LIGHT_FOUND_LED_PIN,HIGH);
  delay(3000);
  number_lights_seen = number_lights_seen + 1;
  digitalWrite(STATE_LIGHT_FOUND_LED_PIN,LOW);
}

void loop() {
  /*
  if (number_lights_seen == number_light_1){
    rotate90CCW();
    number_lights_seen = 0;
    number_light_1 = 100;
  }
  if (number_lights_seen == number_light_2){
    rotate90CCW();
    number_lights_seen = 0;
    number_light_2 = 100;
  }
  if (number_lights_seen == number_light_3){
    rotate90CCW();
    number_lights_seen = 0;
    number_light_3 = 100;
  }
  if (number_lights_seen == number_light_4){
    rotate90CCW();
    number_lights_seen = 0;
    number_light_4 = 100;
  }
  if (number_lights_seen == number_light_5){
    rotate90CCW();
    number_lights_seen = 0;
    number_light_5 = 100;
  }
  if (number_lights_seen == number_light_6){
    rotate90CCW();
    number_lights_seen = 0;
    number_light_6 = 100;
  }
  */

  Distance d = read_dual_sensors();
  short int error = d.rear-d.front;

  if (millis()>60000 && notRotated){
    notRotated = false;
    rotate90CCW();
  }
  //if (!wallFound){
  //  searchForWall(d,error);
  //}else{
    Serial.println();
    bool emergencyFlagFeedback = false;
    bool emergencyFlagRotation = false;

    bool frontObstacle = checkFrontObstacle();
    //Serial.print(" FRONT_OBSTACLE ");
    //Serial.print(frontObstacle);
    if (d.front<85 || (d.front<90 && d.rear<100)){
      emergencyFlagFeedback = true;
    }
    if (d.front<80) {
      emergencyFlagRotation = true;
    }
    if(checkLight() && (millis()-lastTimeLightFound>IGNORE_TIME_AFTER_LIGHT_FOUND || first_light_not_found)){
      first_light_not_found = false;
      stopMotors(); // Ferma il robot
      greenLedThreeSecondsBlocking(); // Accendi verde per 3 secondi 
      lastTimeLightFound = millis();
    } else if (frontObstacle || emergencyFlagRotation){
      rotateLeft(ROTATION_SPEED);
      lastState = 0;
    }else if (d.front>300 && d.rear>300){
      if (lastState == 3) rotateRight(ROTATION_SPEED);
      else moveForward();
      lastState = 1;
    }else if (d.front>200 && d.rear<150){
      turnRight(300);
      lastState = 2;
    }else{
      moveForwardWithFeedback(error, emergencyFlagFeedback, error-previousError);
      lastState = 3;
    }
    //delay(10);
    previousError = error;
  //}
  Serial.println("");
}