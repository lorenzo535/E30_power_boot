#include "myservopid.h"


#ifndef __MATH_H
#include <math.h>
#endif

#ifndef ARDUINO_STREAMING
#include <Streaming.h>
#endif 

MyServoPID::MyServoPID(unsigned short RXPIN, unsigned short TXPIN, unsigned short signal_pin, unsigned short PIN_3_3, double Kp, double Ki, double Kd ) : m_PID( &m_pid_measure, &m_pid_output, &m_setpoint, Kp, Ki, Kd,P_ON_E, REVERSE,400)
{
    m_softserial.begin(9600, RXPIN,TXPIN, SWSERIAL_8N1, false, 256);
    StopServo();
    
    is_stopped = false;
    is_breaked = false;
    m_PID.SetMode(AUTOMATIC);
    m_pid_measure = 1122.0;
    m_PID.SetSampleTime (10);
    m_PID.SetOutputLimits (-3200,3200);

    m_signal_pin = signal_pin;
    pinMode (m_signal_pin, INPUT);
    pinMode (PIN_3_3, OUTPUT);

    digitalWrite (PIN_3_3, HIGH);
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
    m_softserial.println("X");
    is_stopped = true;
}

void MyServoPID::Break()
{
    m_softserial.println("B");
    is_breaked = true;
}

void MyServoPID::SetPosition (double pos)
{
    is_stopped = false;
    is_breaked = false;
    m_softserial.println("GO");
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
    //Serial << "m_current_position is : " << m_current_position<<" \n";
    
    if ((is_stopped)||(is_breaked))
            return;
    m_pid_measure = (double) m_current_position;

   m_PID.Compute();

  // Serial << "output is : " << m_pid_output <<" \n";
    unsigned short ushort_speed_cmd = (unsigned short) fabs (FilterOutput(m_pid_output));
    if (m_pid_output <= 0)
        sprintf (output_string, "F%04d",ushort_speed_cmd );
    else
        sprintf (output_string, "R%04d",ushort_speed_cmd );
   m_softserial.println("GO");
   m_softserial.println(output_string);


}

