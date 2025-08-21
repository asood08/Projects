#define LEFT_MOTOR_FORWARD_PIN  A3
#define LEFT_MOTOR_BACKWARD_PIN  A2
#define RIGHT_MOTOR_FORWARD_PIN  A1
#define RIGHT_MOTOR_BACKWARD_PIN  A0

#define STEPPER_IR_SENSOR_PIN 6

int LEFT_MOTOR_PWM_SPEED = 0;
int LEFT_MOTOR_DIR = 0;
int RIGHT_MOTOR_PWM_SPEED = 0;
int RIGHT_MOTOR_DIR = 0;

unsigned long t0,t1;
unsigned long wheelturns = 0;
unsigned long rpm = 0;

void RPM__ISR() {
  wheelturns++;
}

void setup() {

    Serial.begin(9600);  // Initialize serial communication
    while (!Serial);

    pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_BACKWARD_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);
    //pinMode(STEPPER_IR_SENSOR_PIN, INPUT);
    //attachInterrupt(digitalPinToInterrupt(STEPPER_IR_SENSOR_PIN), RPM__ISR, RISING);
    //t0 = micros();
}

/**************************************************************************************
 INPUT INTO Serial Example

 50 0 75 1

 Above input will give 50% max RPM into left motor
 with forward direction and 75% max rpm into right
 motor with backward direction.


***************************************************************************************/

void loop() {
    if(Serial.available())
    {
      String input = Serial.readString();

      LEFT_MOTOR_PWM_SPEED = input.substring(0, input.indexOf(' ')).toInt();
      input = input.substring(input.indexOf(' ') + 1);
      LEFT_MOTOR_DIR = input.substring(0, input.indexOf(' ')).toInt();
      input = input.substring(input.indexOf(' ') + 1);
      RIGHT_MOTOR_PWM_SPEED = input.substring(0, input.indexOf(' ')).toInt();
      input = input.substring(input.indexOf(' ') + 1);
      RIGHT_MOTOR_DIR = input.substring(0, input.indexOf(' ')).toInt();

      controlWheelMotors(LEFT_MOTOR_PWM_SPEED, LEFT_MOTOR_DIR, RIGHT_MOTOR_PWM_SPEED, RIGHT_MOTOR_DIR);

      Serial.print("ML_SPEED: ");
      Serial.print(LEFT_MOTOR_PWM_SPEED);
      Serial.print(" ML_DIR: ");
      Serial.print(LEFT_MOTOR_DIR);
      Serial.print(" MR_SPEED: ");
      Serial.print(RIGHT_MOTOR_PWM_SPEED);
      Serial.print(" MR_DIR: ");
      Serial.println(RIGHT_MOTOR_DIR); 

    }
    /*t1 = millis();
    if(t1 - t0 > 5000) //wait 5 seconds
    {
      rpm = ((wheelturns)/((float)(t1-t0)))*1000.0*60.0;
      Serial.print("Turns: ");
      Serial.print(wheelturns);
      Serial.print(" dt: ");
      Serial.print(t1 - t0);
      Serial.print(" RPM: ");
      Serial.println(rpm);
      t0 = millis();
      wheelturns = 0;
    }*/
    
}

/************************************************************************************************************
 LEFT_MOTOR_PWM_SPEED (0,100) : % of max RPM into Left Motor
 LEFT_MOTOR_DIR (0 OR 1) : 0 = Forward Direction of Left Motor and 1 = Backward Direction of Left Motor
 RIGHT_MOTOR_PWM_SPEED (0,100) : % of max RPM into Right Motor
 RIGHT_MOTOR_DIR (0 OR 1) : 0 = Forward Direction of Right Motor and 1 = Backward Direction of Right Motor
*************************************************************************************************************/

void controlWheelMotors(int LEFT_MOTOR_PWM_SPEED, int LEFT_MOTOR_DIR, int RIGHT_MOTOR_PWM_SPEED, int RIGHT_MOTOR_DIR)
{
  if(LEFT_MOTOR_DIR == 0)
  {
    analogWrite(LEFT_MOTOR_FORWARD_PIN, (LEFT_MOTOR_PWM_SPEED/100.0)*255.0);
    analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
  }
  else if(LEFT_MOTOR_DIR == 1)
  {
    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
    analogWrite(LEFT_MOTOR_BACKWARD_PIN, (LEFT_MOTOR_PWM_SPEED/100.0)*255.0);
  }

  if(RIGHT_MOTOR_DIR == 0)
  {
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, (RIGHT_MOTOR_PWM_SPEED/100.0)*255.0);
    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
  }
  else if(RIGHT_MOTOR_DIR == 1)
  {
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, (RIGHT_MOTOR_PWM_SPEED/100.0)*255.0);
  }
}
