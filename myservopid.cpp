#include "myservopid.h"


#ifndef __MATH_H
#include <math.h>
#endif

#ifndef ARDUINO_STREAMING
#include <Streaming.h>
#endif 
#include <analogWrite.h>

MyServoPID::MyServoPID(unsigned short PWMPIN, unsigned short DIRPIN, unsigned short signal_pin, double Kp, double Ki, double Kd ) : m_PID( &m_pid_measure, &m_pid_output, &m_setpoint, Kp, Ki, Kd,P_ON_E, REVERSE,400)
{
    StopServo();
    
    is_stopped = false;
    is_breaked = false;
    m_PID.SetMode(AUTOMATIC);
    m_pid_measure = 1122.0;
    m_PID.SetSampleTime (10);
    m_PID.SetOutputLimits (-3200,3200);

    m_signal_pin = signal_pin;
    pinMode (m_signal_pin, INPUT);
    m_dir_pin = DIRPIN;
    m_pwm_pin = PWMPIN;
    
    
    int j;
    for (j = 0; j < ANALOG_AVERAGING_STEPS; j++)
    {
      m_raw_signal [j] = 0;
    }
       

}

MyServoPID::~MyServoPID()
{

}

unsigned short MyServoPID::GetPosition()
{
m_current_position = ReadFilteredPosition();
return m_current_position;
}

void MyServoPID::StopServo()
{
    digitalWrite (m_dir_pin, 0);
    analogWrite (m_pwm_pin, 0);
    is_stopped = true;
}

void MyServoPID::Break()
{
    digitalWrite (m_dir_pin, 0);
    analogWrite (m_pwm_pin, 0);
    is_breaked = true;
}

void MyServoPID::SetPosition (double pos)
{
    is_stopped = false;
    is_breaked = false;
    
    m_setpoint = pos;

}


unsigned short MyServoPID::ReadFilteredPosition()
{
   
  static unsigned short average_steps = 0;
  unsigned short anain = analogRead (m_signal_pin);
  m_raw_signal [average_steps] =  anain ;
  average_steps++;

  if (average_steps >= ANALOG_AVERAGING_STEPS)
    average_steps = 0;

  //compute average
  unsigned short average = 0;
  int j;
  for (j = 0; j < ANALOG_AVERAGING_STEPS; j++)
  {
    average = average + m_raw_signal [j];
  }
  average = average / ANALOG_AVERAGING_STEPS;

  //Serial << " raw " << anain <<"  average: " <<average <<"\n";

  return  average;

}

double MyServoPID::FilterOutput(double unfiltered)
{
 
  static unsigned short average_steps = 0;
  
  m_raw_output [average_steps] =  unfiltered;
  average_steps++;

  if (average_steps >= OUTPUT_AVERAGING_STEPS)
    average_steps = 0;

  //compute average
  double average = 0.0;
  int j;
  for (j = 0; j < OUTPUT_AVERAGING_STEPS; j++)
  {
    average = average + m_raw_output [j];
  }
  average = average / OUTPUT_AVERAGING_STEPS;

  return  average;

}

void MyServoPID::Compute(char* output_string)
{
    
    m_current_position = ReadFilteredPosition();
   // Serial << "m_current_position is : " << m_current_position<< " setpoint:"<<m_setpoint<< " \n";
    
    if ((is_stopped)||(is_breaked))
            return;
    m_pid_measure = (double) m_current_position;

   m_PID.Compute();

 
    //Serial << "output is : " << _output <<" \n";
    unsigned short ushort_speed_cmd = (unsigned short) fabs (FilterOutput(m_pid_output));
    float pwm_f = abs(ushort_speed_cmd) / 3200.0;
    unsigned short pwm = (unsigned short) (pwm_f * 255.0);
   //Serial << "ushort_speed_cmd is " << ushort_speed_cmd << " pwm is : " << pwm <<" \n";
    
   bool dir = (m_pid_output > 0? 1 : 0);
   Output (dir, pwm)  ;  


}

void MyServoPID::Output(bool _dir, unsigned short _pwm)
{
   
  if (_dir )
    {
            digitalWrite (m_dir_pin, 0);
            analogWrite (m_pwm_pin, _pwm);
    }
    else
    {
         digitalWrite (m_dir_pin, 1);
          analogWrite (m_pwm_pin, _pwm);
    } 
}
