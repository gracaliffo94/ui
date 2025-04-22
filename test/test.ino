#include "Adafruit_VL53L0X.h"

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 3
#define SHT_LOX2 4


#define LASERS_DISTANCE 0.118
 
#define MOTOR_DX_IN1 5
#define MOTOR_DX_IN2 6
#define MOTOR_SX_IN1 9
#define MOTOR_SX_IN2 10
#define FRONT_RIGHT_LED_PIN 12
#define FRONT_LEFT_LED_PIN 11
#define ON_RIGHT_WALL_DISTANCE 0.17
#define KP 0.37
#define KD 5
#define RIGHT_BASE_SPEED 140
#define LEFT_BASE_SPEED RIGHT_BASE_SPEED-24
#define ROTATION_SPEED 130
#define FRONT_CM_DISTANCE_OBSTACLE_AVOIDANCE_THRESHOLD 10

#define TRIGGER_PORT 7
#define ECHO_PORT 8 
 
struct Distance {
    short int front;
    short int rear;
};


unsigned short previousMillis = 0;  // Salva il tempo dell'iterazione precedente
unsigned short currentMillis = 0;   // Salva il tempo corrente
short int previousError = 0;

// objects for the vl53l0x
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

  // print sensor one reading
  Serial.print(F("Front: "));
  if(measure2.RangeStatus != 4) {     // if not out of range
    d.front = measure2.RangeMilliMeter;
    Serial.print(d.front);
  } else {
    Serial.print(F("Out of range"));
  }


  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("Rear: "));
  if(measure1.RangeStatus != 4) {
    d.rear = measure1.RangeMilliMeter;
    Serial.print(d.rear);
  } else {
    Serial.print(F("Out of range"));
  }
  

  
  Serial.println();
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

  pinMode(MOTOR_DX_IN1, OUTPUT);
  pinMode(MOTOR_DX_IN2, OUTPUT);
  pinMode(MOTOR_SX_IN1, OUTPUT);
  pinMode(MOTOR_SX_IN2, OUTPUT);
  pinMode(FRONT_LEFT_LED_PIN, INPUT);
  pinMode(FRONT_RIGHT_LED_PIN, INPUT);
 
}

Distance getDistance() {
  Distance d;
  VL53L0X_RangingMeasurementData_t measure1;
  //Serial.print("Reading a measurement...1 ");
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  if (measure1.RangeStatus != 4) {  // phase failures have incorrect data
    //Serial.print("Distance (mm): "); Serial.println(measure1.RangeMilliMeter);
    d.rear = measure1.RangeMilliMeter*0.001;
  } else {
    //Serial.println(" out of range ");
    d.rear = 2000;
  }
 
  delay(10);
 
  VL53L0X_RangingMeasurementData_t measure2;
  //Serial.print("Reading a measurement...2 ");
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  if (measure2.RangeStatus != 4) {  // phase failures have incorrect data
    //Serial.print("Distance (mm): "); Serial.println(measure2.RangeMilliMeter);
    d.front = measure2.RangeMilliMeter*0.001;
  } else {
    //Serial.println(" out of range ");
    d.front = 2000;
  }
  delay(10);
 
  return d;
} 

 
void moveForwardWithFeedback(short int pwm, bool emergency_flag, short int error_dot){
    short int proportional_feedback = pwm*KP;
    short int derivative_feedback = error_dot*KD;
    short int r_speed = constrain(RIGHT_BASE_SPEED+proportional_feedback+derivative_feedback,0,255);
    short int l_speed = constrain(LEFT_BASE_SPEED-proportional_feedback-derivative_feedback,0,255);
    if (emergency_flag){
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
  analogWrite(MOTOR_DX_IN1, RIGHT_BASE_SPEED);
  analogWrite(MOTOR_DX_IN2, 0);
  analogWrite(MOTOR_SX_IN1, LEFT_BASE_SPEED);
  analogWrite(MOTOR_SX_IN2, 0);
  Serial.println("");
}


bool checkFrontObstacle(){
  if (frontDistance()<FRONT_CM_DISTANCE_OBSTACLE_AVOIDANCE_THRESHOLD)
    return true;
  bool left_obstacle = 1-digitalRead(FRONT_LEFT_LED_PIN);
  bool right_obstacle = 1-digitalRead(FRONT_RIGHT_LED_PIN);
  return left_obstacle || right_obstacle;
}

void rotateLeft(){
  // Ruota in senso orario
  analogWrite(MOTOR_DX_IN1, ROTATION_SPEED);
  analogWrite(MOTOR_DX_IN2, 0);
  analogWrite(MOTOR_SX_IN1, 0);
  analogWrite(MOTOR_SX_IN2, ROTATION_SPEED);
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
  /*Serial.print( "durata: " );
  Serial.print( duration );
  Serial.print( " , " );
  Serial.print( "distanza: " );
  if( duration > 38000 ) 
    Serial.println( "fuori portata");
  else{ 
    Serial.print( r ); Serial.println( "cm" );
  }*/
  //delay(10);
  return r;
}

void loop() {
  currentMillis = millis();
  double elapsedTime = (currentMillis - previousMillis)*0.001;
  Distance d = read_dual_sensors();
  short int error = d.rear-d.front;
  //Serial.print("Error:");
  //Serial.print(error);
  Serial.println();
  bool emergency_flag = false;

  bool frontObstacle = checkFrontObstacle();
  //Serial.print(" FRONT_OBSTACLE ");
  //Serial.print(frontObstacle);
  if (d.front<100 && d.rear<155){
    emergency_flag = true;
  }
  if (frontObstacle){
    rotateLeft();
  }
  else if (d.front>300 && d.rear>300){
    moveForward();
  }else{
    moveForwardWithFeedback(error, emergency_flag, error-previousError);
  }
  //delay(10);
  previousError = error;
}