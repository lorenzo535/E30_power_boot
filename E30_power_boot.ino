
#include <Servo.h>
#include <Streaming.h>
#include "Switch.h"

Servo myservo;  // create servo object to control a servo
#include <NewPing.h>

//Variables and constants for ultrasound switch
unsigned long t1, t2, t3, t4, old_millis, now_millis, time_sig;
unsigned short cycle;
int  dist, old_dist;
bool show_us_distance;
#define DIST_LOW 29
#define DIST_HIGH 30
#define PIN_US_TRIG  A3
#define PIN_US_ECHO  4
#define MAX_DISTANCE 50
NewPing sonar(PIN_US_TRIG, PIN_US_ECHO, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// Pin defintion on Pro Micro board
#define PIN_LOCK_SW1 3
#define PIN_LOCK_SW2 5
#define PIN_PWM_SERVO 6
#define PIN_SERVO_FEEDBACK A2
#define PIN_CURRENT_SENSE A10  //pin 10 on board

#define PIN_INPUT_1  9
#define PIN_INPUT_2  8
#define PIN_INPUT_3  7
#define PIN_INPUT_4  A1
#define POWER_DRIVES A0

#define LOCK_BUTTON   PIN_INPUT_1
#define LOCK_BUTTON_REMOTE  PIN_INPUT_2
#define DIGITAL_IN_LOCK_END_STOP PIN_INPUT_4
#define MOTOR_RUNNING PIN_INPUT_3
// For Switch to work as intended, object must be defined as LOW polarity
// and event released() must be used to detect button pressed. Released goes to 1
// as soon as the button is pressed
Switch InputButton = Switch (LOCK_BUTTON, INPUT, LOW, 200, 300, 250, 10);
Switch InputRemoteButton = Switch (LOCK_BUTTON_REMOTE, INPUT, LOW, 200, 300, 250, 10);

#define PIN_LED_ERR   4
#define PIN_LED_BTNS A3

#define PIN_DIR_MOTOR_CAM  15
#define PIN_DIR_MOTOR_UNLOCKER -1
#define PIN_PWM_MOTOR_CAM 14
#define PIN_PWM_MOTOR_UNLOCKER 16

////////////////STATES AND MODES ///////////////
#define MODE_OPENING  0
#define MODE_CLOSING 1
#define MODE_IDLE 2
#define MODE_SAFETY_CLOSING 3
#define MODE_MANUAL_STOP 4

#define STATE_BOOT_LOCKED 0
#define STATE_ENGAGED 1
#define STATE_SWINGING 2
#define STATE_AT_TOP_END 3
//////////////////////////////////////

#define SERVO_POSITION_ENGAGEMENT 131
#define SERVO_POSITION_ENGAGEMENT_INCREASE_CURRENT 118
#define SERVO_POSITION_TOP_END 70
#define SERVO_POSITION_UNLOCK 117
#define POSITION_TOLERANCE 6

#define CAM_COMMAND_GO_TO_LOCK -1
#define CAM_COMMAND_UNLOCK 1
#define CURRENT_LIMIT 3 //2.8

#define CURRENT_EXTRA_ALLOWANCE_LOCK 1
#define OVERCURRENT_CONSECUTIVE_STEPS 4
#define MV_PER_AMP 100
#define POS_FEEDBACK_LOW_BOUND 1000
#define POS_FEEDBACK_HIGH_BOUND 2000
#define MOTOR_CAM 1
#define MOTOR_UNLOCKER 2



unsigned short mode, old_mode, mode_when_stopped_was;
unsigned short state, old_state;
int pos = 0;    // variable to store the servo position
int current_pos;
unsigned short overcurrent_cnt = 0;
boolean show_current_measure;
boolean show_position;
boolean show_switches;
boolean show_mode;
boolean off_servo;
boolean power;
boolean drivers_on;
boolean old_sw1, old_sw2;
boolean old_lock_cmd_button, old_lock_cmd_remote;
boolean inhibit_first;
boolean servo_stopped;
#define ANALOG_AVERAGING_STEPS  10
float raw_current[ANALOG_AVERAGING_STEPS];
float raw_position[ANALOG_AVERAGING_STEPS];
unsigned long motion_started;
short unsigned int current_av_steps, position_av_steps;



void setup() {

  pinMode(PIN_LOCK_SW1, INPUT);
  pinMode(PIN_LOCK_SW2, INPUT);

  pinMode(PIN_SERVO_FEEDBACK, INPUT);
  pinMode(PIN_CURRENT_SENSE, INPUT);

  pinMode(PIN_INPUT_1, INPUT);
  pinMode(PIN_INPUT_2, INPUT);
  pinMode(PIN_INPUT_3, INPUT);
  pinMode(PIN_INPUT_4, INPUT);

  pinMode(PIN_LED_ERR, OUTPUT);
  pinMode(PIN_LED_BTNS, OUTPUT);

  pinMode(PIN_DIR_MOTOR_CAM, OUTPUT);
  pinMode(PIN_PWM_MOTOR_CAM, OUTPUT);
  pinMode(PIN_PWM_MOTOR_UNLOCKER, OUTPUT);
  pinMode(POWER_DRIVES, OUTPUT);

  pinMode (PIN_US_TRIG, OUTPUT);
  pinMode (PIN_US_ECHO, INPUT);

  StopServo();
  old_sw1 = 0;
  old_sw2 = 0;
  old_state = -1;
  old_mode = -1;
  mode = MODE_IDLE;
  show_switches = 0;
  show_position = 0;

  int j;
  for (j = 0; j < ANALOG_AVERAGING_STEPS; j++)
  {
    raw_current [j] = 0;
    raw_position [j] = 0;
  }

  EvaluateState();
  OutMotor (MOTOR_UNLOCKER, 0);
  OutMotor (MOTOR_CAM, 0);

  InputButton.poll();
  InputRemoteButton.poll();
  inhibit_first = 1;
  servo_stopped = true;
  mode_when_stopped_was = MODE_IDLE;
  drivers_on = true;
  PowerDrivesOff();

  cycle = 0;
  t1 = millis();
  t2 = t1;
  t3 = t2;
  t4 = t3;
  time_sig = t1;
  dist = 20;
  show_us_distance = false;
  old_millis = t1;

}

void StopServo()
{
  if (servo_stopped)
    return;
  int i;
  servo_stopped = true;
  pos = ScalePosition();

  for (i = 0; i < 3; i++)
  {
    myservo.write(pos);
    delay (100);
  }

  mode = MODE_IDLE;


}



void loop() {


//Serial << digitalRead (PIN_INPUT_1) << "  " << digitalRead (PIN_INPUT_2)<< digitalRead (PIN_INPUT_3) << "  " << digitalRead (PIN_INPUT_4) <<"\n";
 // delay(200);
 // return;

  // PowerDrivesOn();
  // UnlockCam();
  // delay(2000);
  // LockCam();
  //delay(2000);
  //return;

  InputButton.poll();
  InputRemoteButton.poll();

  //TestServo();

  if (show_mode)
    DisplayModeAndState();

  ReadKeyboardCmds();

  EvaluateState();

  ReadUserCommands();

  switch (mode)
  {

    case MODE_OPENING :  PowerDrivesOn(); ProcessOpening(); break;

    case MODE_CLOSING :  PowerDrivesOn(); ProcessClosing();  break;

    case MODE_MANUAL_STOP : StopServo(); break;
    case MODE_SAFETY_CLOSING : mode = MODE_IDLE; break;

    case MODE_IDLE :  {
        if ( FootSwitchSwing() && !isMotorRunning())
        {
          if (state == STATE_BOOT_LOCKED)
            mode = MODE_OPENING;
          else
            mode = MODE_CLOSING;
          break;
        }

        if (current_pos < SERVO_POSITION_TOP_END - POSITION_TOLERANCE)
          SetServo(SERVO_POSITION_TOP_END);
        /*          else
                  if (current_pos > SERVO_POSITION_ENGAGEMENT +POSITION_TOLERANCE)
                  SetServo(SERVO_POSITION_ENGAGEMENT);
        */

        else
        {
          StopServo();
          OutMotor(MOTOR_CAM, 0);
          OutMotor(MOTOR_UNLOCKER, 0);
          myservo.detach();
          PowerDrivesOff();
        }
        break;
      }
    default:
      StopServo();
  }

  CurrentProtection();

}

void PowerDrivesOn()
{
  if (drivers_on)
    return;
  digitalWrite(POWER_DRIVES, HIGH);
  delay(200);
  drivers_on = true;
}

void PowerDrivesOff()
{
  if (!drivers_on)
    return;
  digitalWrite(POWER_DRIVES, LOW);
  drivers_on = false;
}
void ProcessOpening()
{
  switch (state)
  {
    case STATE_BOOT_LOCKED : UnlockCam(); /*Serial << "process op boot locked \n";*/ break;

    case STATE_ENGAGED  :

    case STATE_SWINGING :  OutMotor(MOTOR_CAM, 0); SetServo(SERVO_POSITION_TOP_END);/* Serial << "process op swinging\n";*/ break;

    case STATE_AT_TOP_END : StopServo(); Serial << "top end\n"; mode = MODE_IDLE; break;

  }
}


void ProcessClosing ()
{
  switch (state)
  {

    case STATE_AT_TOP_END :
    case STATE_SWINGING :  SetServo(SERVO_POSITION_ENGAGEMENT); /*Serial << "process cl top end , swinging\n";*/  break;

    case STATE_ENGAGED  :  LockCam(); StopServo();  /*erial << "process closing state engaged \n"; */ break;

    case STATE_BOOT_LOCKED : OutMotor(MOTOR_CAM, 0); /*Serial << "process cl boot mocked\n";*/mode = MODE_IDLE; break;
  }

}



void UnlockCam()
{
  myservo.write(SERVO_POSITION_UNLOCK);
  OutMotor(MOTOR_UNLOCKER, 1);
  delay (500);
  OutMotor(MOTOR_UNLOCKER, 0);

  unsigned long start_time = millis();
  OutMotor(MOTOR_CAM, CAM_COMMAND_UNLOCK);

  do
  {
    if ( digitalRead (DIGITAL_IN_LOCK_END_STOP))
    {
      Serial << "got to unlock cam position ... STOP!!!! " << millis() - start_time << "  \n";
      OutMotor(MOTOR_CAM, 0);
      return;
    }
    delay (2);

  } while ( (millis() - start_time) <= 1300);

  Serial << " Cam unlock out on timeout ... STOP!!!! \n";

  OutMotor(MOTOR_CAM, 0);
}

void LockCam()
{
  unsigned long start_time = millis();
  OutMotor(MOTOR_CAM, CAM_COMMAND_GO_TO_LOCK);
  do
  {
    if ( digitalRead (PIN_LOCK_SW2))
    {
      Serial << "got to locked position ... STOP!!!! " << millis() - start_time << "  \n";
      OutMotor(MOTOR_CAM, 0);
      return;

    }
    delay (2);

  } while ( (millis() - start_time) <= 1300);

  Serial << "out on timeout ... STOP!!!! \n";
  OutMotor(MOTOR_CAM, 0);

}


void DisplayModeAndState()
{
  static short u = 0;
  if (u < 40)
  {
    u++;
    return;
  }
  u = 0;

  Serial << "Current mode: " ;

  switch (mode)
  {
    case MODE_IDLE : Serial << " IDLE "; break;
    case MODE_OPENING : Serial << " MODE_OPENING ";  break;
    case MODE_CLOSING : Serial << " MODE_CLOSING "; break;
    case MODE_SAFETY_CLOSING : Serial << " MODE_SAFETY_CLOSING "; break;
    case MODE_MANUAL_STOP: Serial << " MODE_MANUAL_STOP"; break;
  }

  Serial << "Current state: " ;
  switch (state)
  {
    case STATE_BOOT_LOCKED : Serial << "BOOT LOCKED \n"; break;
    case STATE_ENGAGED  : Serial << "BOOT ENGAGED \n"; break;
    case STATE_SWINGING :  Serial << "SWINGING \n"; break;
    case STATE_AT_TOP_END : Serial << "AT TOP END \n"; break;
  }
}



void send_unlock_motor()
{
  OutMotor(MOTOR_UNLOCKER, 1);
  delay(2000);
  OutMotor(MOTOR_UNLOCKER, 0);

}

void CamLock()
{

}

void OutMotor (int _motor_ID, float _command)
{
  // CAM Motor : negative command: locks; positive unlocks (black motor wire on M2A, red on M2B)

  unsigned int motor_dir, motor_PWM;

  //   Serial << "outmotor received ID" << _motor_ID <<"  cmd" << _command <<"\n";
  switch (_motor_ID)
  {
    case MOTOR_CAM : motor_dir = PIN_DIR_MOTOR_CAM; motor_PWM = PIN_PWM_MOTOR_CAM; break;
    case MOTOR_UNLOCKER : motor_dir = PIN_DIR_MOTOR_UNLOCKER; motor_PWM = PIN_PWM_MOTOR_UNLOCKER; break;
  }

  if (_command >= 1.0)
    _command = 1.0;
  if (_command <= -1.0)
    _command = -1.0;

  if (motor_dir != PIN_DIR_MOTOR_UNLOCKER)
    if (_command < 0)
      digitalWrite(motor_dir, 1);
    else
      digitalWrite(motor_dir, 0);


  // unsigned int pwm = (unsigned int) (fabs(_command * 255));
  // analogWrite (motor_PWM,pwm);
  if (_command != 0)
    digitalWrite (motor_PWM, 1);
  else
    digitalWrite (motor_PWM, 0);

  //  Serial << "writing " << motor_PWM  << " command " <<pwm << "\n";
}



void ReadUserCommands()
{
  /*
    //Serial << "pushed " << InputButton.pushed()  ;
    //Serial << "   switched " << InputButton.switched() ;
    //Serial << "   on " << InputButton.on() ;
    if (InputButton.released())
    Serial << "   released " << "\n";

    if (InputButton.pushed())
    Serial << "   pushed " << "\n";
    return;
  */


  if (isCarMoving())
    return;

  if (InputButton.released())
  {
    if (inhibit_first)
    {
      inhibit_first = 0;
      return;
    }

    if (isCarLocked())
      if (state == STATE_BOOT_LOCKED)
        mode = MODE_IDLE;
      else
        mode = MODE_CLOSING;
    else
    {
      // button pressed, car is unlocked, several cases:
      if (mode == MODE_MANUAL_STOP)
      {
        if (mode_when_stopped_was == MODE_CLOSING)
          mode = MODE_OPENING;
        else if (mode_when_stopped_was == MODE_OPENING)
          mode = MODE_CLOSING;
        else
          mode = MODE_IDLE;
      }
      else
      {

        if (mode != MODE_IDLE)
        {
          mode_when_stopped_was = mode;
          mode = MODE_MANUAL_STOP;
        }
        else if (state == STATE_AT_TOP_END)
          mode = MODE_CLOSING;
        else
          // in all other case, a pressed button is associated to open
          mode = MODE_OPENING;
      }

    }
  } // if lock_cmd_button

  if (InputRemoteButton.released())
  {
    Serial << "    $$$$$$  REMOTE \n";
    if (mode != MODE_IDLE)
    {
      mode = MODE_MANUAL_STOP;
    }
    else
    {
      if (state == STATE_AT_TOP_END)
        mode = MODE_CLOSING;
      else
        mode = MODE_OPENING;
    }
  }

  if (old_mode != mode)
  {
    Serial << "New mode is " ;
    switch (mode)
    {
      case MODE_IDLE : Serial << " IDLE \n"; break;
      case MODE_OPENING : Serial << " MODE_OPENING \n"; motion_started = millis(); break;
      case MODE_CLOSING : Serial << " MODE_CLOSING \n"; motion_started = millis(); break;
      case MODE_SAFETY_CLOSING : Serial << " MODE_SAFETY_CLOSING \n"; break;
      case MODE_MANUAL_STOP: Serial << " MODE_MANUAL_STOP\n"; break;
    }
  }

  old_mode = mode;
}


boolean isCarLocked()
{
  //TODO read dedicated input signal and define
  // if car is locked

  return false;
}

boolean isMotorRunning()
{
 //return true;
  bool in = digitalRead(MOTOR_RUNNING);
  Serial << "in " << in <<"\n";
  return in;
}

boolean isCarMoving()
{
  //TODO read dedicated input signal and define
  // if car is moving

  return false;
}

void TestServo()
{

  if (power)
    PowerDrivesOn();
  else
    PowerDrivesOff();
  return;


  if (pos <= 0)
    pos = 0;
  if  (pos >= 180)
    pos = 180;
  if (!off_servo)
    myservo.write(pos);

}


void SetServo(int position_target)
{
  static int current_target = -1;

  if (position_target != current_target)
  {
    Serial << "sservo go to  target" << position_target << " \n";
    //return;
  }
  current_target = position_target;


  if (!myservo.attached())
    myservo.attach(PIN_PWM_SERVO);
  myservo.write(position_target);
  servo_stopped = false;
}

void EvaluateState()
{
  static short count_debounce = 0;
  //Read Servo feedback
  current_pos = ScalePosition();
  if (show_position)
    Serial << "Current position " << current_pos << " \n";

  boolean sw1, sw2;
  sw1 = digitalRead (PIN_LOCK_SW1);
  sw2 = digitalRead (PIN_LOCK_SW2);

  if (show_switches)
    Serial << "sw1 " << sw1 << "  sw2 " << sw2 << "\n";

  state = STATE_SWINGING;
  if (sw1)
  {
    state = STATE_ENGAGED;
    //Serial << "Engaged ! \n";
  }


  if (sw2)
  {
    state = STATE_BOOT_LOCKED;
    //Serial << "Boot locked ! \n";
  }


  if (!sw1 && !sw2)
  {
    if ((abs (current_pos - SERVO_POSITION_TOP_END) <=  POSITION_TOLERANCE - 4)  || (current_pos <= (SERVO_POSITION_TOP_END)))
    {

      //Serial << "xount = " << count_debounce << "position = " << current_pos << "\n";
      count_debounce ++;
      if (count_debounce >= 10)
      {
        state = STATE_AT_TOP_END;
        count_debounce = 10; // bound counter

      }

    }
    else count_debounce --;
    if (count_debounce < 0)
      count_debounce = 0;
  }



  old_sw1 = sw1;
  old_sw2 = sw2;

  if (old_state != state)
  {
    Serial << "  ==== New state is " ;
    switch (state)
    {
      case STATE_BOOT_LOCKED : Serial << " STATE_BOOT_LOCKED \n"; break;
      case STATE_ENGAGED : Serial << " STATE_ENGAGED \n"; break;
      case STATE_SWINGING : Serial << " STATE_SWINGING \n";  break;
      case STATE_AT_TOP_END : Serial << " STATE_AT_TOP_END \n"; break;
    }

  }


  old_state = state;

}



unsigned int ScalePosition ()
{
  //raw reading 1022 = top = 180°
  // raw reading 980 = bottom = 0°
  static int pos_average_steps = 0;
  int j = 0;
  bool displayout = false;

  //1024 - anain

  float scaledpos = analogRead (PIN_SERVO_FEEDBACK) / 1024.0 * 5.0; //in volts

  raw_position [pos_average_steps] = scaledpos;
  pos_average_steps++;

  if (pos_average_steps >= ANALOG_AVERAGING_STEPS)
  {
    pos_average_steps = 0;
    displayout = true;
  }

  //compute average
  float average = 0;
  for (j = 0; j < ANALOG_AVERAGING_STEPS; j++)
  {
    average = average + raw_position [j];
  }
  average = average / ANALOG_AVERAGING_STEPS;

  unsigned int output;
  float x = average;
  //polynomial approximation in use with 10 turn potentiometer
  average = 88.1593538351 * x * x * x * x - 869.9926535361 * x * x * x + 3208.9184567625 * x * x - 5385.4931352597 * x + 3698.1106094017;
  output = (unsigned int) (average );
  //if (displayout) Serial <<" Ana reading " << scaledpos <<"average "<<  average << "  pos corresponds to " <<output<< "\n";
  return output;


}

void ReadKeyboardCmds()
{

  //Check for manual commands
  if (Serial.available() > 0)
  {
    Serial << "keyboard in \n";

    char rx_byte = Serial.read();       // get the character


    // ?
    if (rx_byte == '?')
    {
      Serial << "B,b = bottom, T,t = top ; P,p = +5, M,m = -5;  C,c = current\n" ;
    }


    switch (rx_byte)
    {
      case 'M':
      case 'm': show_mode = !show_mode ; break;
      case 'U':
      case 'u': Serial << "Unlock cam\n"; PowerDrivesOn(); UnlockCam(); PowerDrivesOff(); break;
      case 'L':
      case 'l': Serial << "Lock cam\n"; PowerDrivesOn(); LockCam(); PowerDrivesOff(); break;
      case 'P':
      case 'p': Serial << "+5 \n" ; pos = pos + 5; break;
      case 'T':
      case 't': Serial << "go to top end \n" ; pos = 180; break;
      case 'B':
      case 'b': Serial << "go to bottom end \n" ; pos = 0; break;
      case 'D':
      case 'd': Serial << "show US distance \n"; show_us_distance = ! show_us_distance; break;
      case 'C':
      case 'c': Serial << "show current \n"; show_current_measure = ! show_current_measure; break;
      case 'Z':
      case 'z': Serial << "show position \n"; show_position = ! show_position; break;
      case 'S':
      case 's' : Serial << " STOP STOP at " << ScalePosition() << "\n"; StopServo(); break;
      case 'a':
      case 'A' : Serial << " cam motor positive \n"; OutMotor (MOTOR_CAM, 0.5); delay (1000); OutMotor (MOTOR_CAM, 0); break;
      case 'q':
      case 'Q' : Serial << " cam motor negative \n"; OutMotor (MOTOR_CAM, -0.5); delay (1000); OutMotor (MOTOR_CAM, 0); break;
      case 'w':
      case 'W' : Serial << " unlock motor  \n"; OutMotor (MOTOR_UNLOCKER, 0.5); delay (1000); OutMotor (MOTOR_UNLOCKER, 0); break;
      case '1' : mode = MODE_CLOSING; Serial << "closing keyboard command \n"; break;
      case '2' : mode = MODE_OPENING; Serial << "opening keyboard command \n"; break;
      case '3' : show_switches = !show_switches; break;
      case  'n':
      case 'N':
        Serial << "power " << (power ? "ON" : "OFF") << "\n"; power = !power; break;

      case 'X':
      case 'x' : off_servo = !off_servo;
        if (off_servo) myservo.detach();
        else myservo.attach(9);
        Serial << " SERVO AXIS IS " <<  (off_servo ? "OFF" : "ON") << "\n";  break;


    }

  }
}






void CurrentProtection()
{
  int j;
  static short int skip = 0;
  int anain = analogRead(PIN_CURRENT_SENSE);
  raw_current [current_av_steps] = ADCValueToCurrent(anain) ;
  current_av_steps++;

  if (current_av_steps >= ANALOG_AVERAGING_STEPS)
    current_av_steps = 0;

  //compute average
  float average = 0;
  for (j = 0; j < ANALOG_AVERAGING_STEPS; j++)
  {
    average = average + raw_current [j];
  }
  average = average / ANALOG_AVERAGING_STEPS;

  float current_limit = CURRENT_LIMIT;

  if (current_pos >= SERVO_POSITION_ENGAGEMENT_INCREASE_CURRENT)
  {
    current_limit += CURRENT_EXTRA_ALLOWANCE_LOCK;
  }

  if (show_current_measure)
  {
    if (skip >= 30)
    {
      Serial << " current measure " << fabs(average) << " (A)  LIMIT : " << current_limit << "\n";
      skip = 0;
    }
    skip = skip + 1;
  }


  unsigned long deltat;
  if ((fabs(average) >= current_limit) && (state == STATE_SWINGING) && (mode != MODE_OPENING) )
  {
    overcurrent_cnt++;

    if (overcurrent_cnt >= OVERCURRENT_CONSECUTIVE_STEPS)
    {

      deltat = millis() - motion_started;
      //Inhibit first two seconds after servo has started moving
      if (deltat >= 2000)
      {
        Serial << "##### current limit reached " << fabs(average) << " (A) ; current position is " << current_pos << "\n";
        StopServo();
        if (mode == MODE_CLOSING)
          //      mode = MODE_SAFETY_CLOSING;
          //      else
          mode == MODE_IDLE;

        //delay (500);
      }
    }
  }
  else
    overcurrent_cnt = 0;


}


float ADCValueToCurrent (long int adc_in)
{

  float temp = adc_in;
  float mv = (temp / 1023.0) * 5000.0;

  return ((mv - 2500) / MV_PER_AMP);

}



bool FootSwitchSwing()
{
  //return false;
  static bool filter_one = true;
  now_millis = millis();
  if ((now_millis - old_millis) <= 30)
  {
    return false;
  }
  old_millis = now_millis;

  dist = UltrasonDistance();

  if ( (cycle != 0) && (now_millis - time_sig > 2000) )
  { cycle = 0;
    Serial << "   ****   RESET !!\n";
    filter_one = true;
  }



  if (abs (dist - old_dist) >= 1)
  {
   //Serial << " ## change ; dist : " << dist <<"  old_dist: " <<old_dist <<"\n";
    time_sig = now_millis;
    switch (cycle)
    {
      case 0: if (( dist <= DIST_LOW)&& (dist >10)) //test > 10 required for noisy measures on gravel
        {
          cycle++;
          t1 = time_sig;
          Serial << "t1 : " << t1 << "\n";
        } break;
      case 1: if ( dist >= DIST_HIGH)
        {
          if (filter_one)
            filter_one = false;
          else
          {
            filter_one = true;
            cycle++; t2 = time_sig; Serial << "t2 : " << t2 << "  t2-t1: " << t2-t1 <<"\n";
          }
        }
        break;

      case 2: if (( dist <= DIST_LOW)&& (dist >10))  //test > 10 required for noisy measures on gravel
        {
          cycle++;
          t3 = time_sig;
          Serial << "t3 : " << t3 << "  t3-t2: " << t3-t2 << "\n";
        } break;
      case 3: if ( dist >= DIST_HIGH)
        { if (filter_one)
          {
            filter_one = false;
          }
          else
          {
            filter_one = true;
            cycle++; t4 = time_sig; Serial << "t4 : " << t4 << "  t4-t3: " << t4-t3 << "\n";
          }
        }
        break;
    }
  }

    if (show_us_distance)  Serial << "Swing ; distance : " << dist <<" cycle:" <<cycle <<" Mot-or on: " << digitalRead(MOTOR_RUNNING) <<"\n";
  
  if (cycle == 4)
  {
    Serial << "t2 - t1: ==" << t2 - t1 << "== ; t3 - t2: **" << t3 - t2 << "**  t4 - t3: %%" << t4 - t3 << "%%\n";
    cycle = 0;
    if  ( ( (t2 - t1) <= 650)  && ((t2 - t1) >= 80) &&
          //( (t3 - t2) <= 1500) && ((t3 - t2) >= 200) &&
          ( (t4 - t3) <= 650) && ((t4 - t3) >= 80) )
    {
      Serial << "GOT IT !!!!!! \n";
      return true;
    }
  }

  old_dist = dist;
  return false;
}

#define FOOT_KEEP_DELAY 400


float f_dist;
#define US_DIST_AVERAGING_STEPS 5
int raw_dist[US_DIST_AVERAGING_STEPS];



int UltrasonDistance()
{
  int j = 0;
  static int dist_av_steps = 0;
  int _dist = sonar.ping_cm();    

  raw_dist [dist_av_steps] = _dist ;
  dist_av_steps++;

  if (dist_av_steps >= US_DIST_AVERAGING_STEPS)
    dist_av_steps = 0;

  //compute average
  float f_dist = 0;
  for (j = 0; j < US_DIST_AVERAGING_STEPS; j++)
  {
    f_dist = f_dist + raw_dist [j];
  }
  f_dist = f_dist / US_DIST_AVERAGING_STEPS;
  
  return  (int) f_dist;

}



bool FootSwitchKeep()
{
  now_millis = millis();

  if ((now_millis - old_millis) <= 30)
  {
    return false;
  }
  old_millis = now_millis;

  dist = UltrasonDistance();


  if (show_us_distance)  Serial << "distance : " << dist << "\n";

  if (abs (dist - old_dist) >= 5)
  {
    time_sig = now_millis;
    if ( dist <= DIST_LOW)
    {
      if (cycle == 0)
      {
        t1 = time_sig;
        Serial << "t1 : " << t1 << "\n";
        cycle = 1;
      }
    }
    else
    {
      Serial << "Foot removed, counter reset !!!!!  distance is " << dist << "old " << old_dist << "\n";
      cycle = 0;
      old_dist = dist;
      return false;
    }
  }

  if (cycle == 1)
  {
    if ((now_millis - t1) >= FOOT_KEEP_DELAY)
    {
      Serial << "GOT IT !!!!!! \n";
      old_dist = dist;
      cycle = 0;
      return true;
    }
    else
      Serial << "Counting .... dt is " << now_millis - t1 << "\n";
  }

  old_dist = dist;
  return false;
}



