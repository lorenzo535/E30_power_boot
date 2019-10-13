#ifndef MYSERVOPID_H
#define MYSERVOPID_H

#include <SoftwareSerial.h>
#define ANALOG_AVERAGING_STEPS 10
#define OUTPUT_AVERAGING_STEPS 10
#include "PID_v1l.h"

class MyServoPID
{
public:
    MyServoPID(unsigned short TXPIN, unsigned short RXPIN,unsigned short SIGNALPIN, unsigned short PIN_3_3, double Kp, double Ki, double Kd );
    ~MyServoPID();

    void StopServo();
    void Break();
    void SetPosition (double pos);
    void Compute(char* output_string);
    unsigned short GetPosition();
    unsigned short ReadFilteredPosition();

private:
    double FilterOutput(double unfiltered);
    double m_Kp, m_Ki, m_Kd;
    unsigned short m_tx_pin, m_rx_pin;
    SoftwareSerial m_softserial;
    PID m_PID;
    double m_setpoint, m_pid_output, m_pid_measure;
    bool is_stopped, is_breaked;
    unsigned short m_current_position;
    unsigned short m_signal_pin;
    unsigned short m_raw_signal[ANALOG_AVERAGING_STEPS];
    double m_raw_output[OUTPUT_AVERAGING_STEPS];
    


};

#endif // MYSERVOPID_H
