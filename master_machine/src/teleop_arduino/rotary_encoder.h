#ifndef __ROTARY_ENCODER_H__
#define __ROTARY_ENCODER_H__

// #define CALCULATE_SPEED

//#include "WProgram.h"
#include "Arduino.h"

class Rotary_Encoder 
{
  public:
    Rotary_Encoder( int8_t PinA, int8_t PinB) : pin_a (PinA), pin_b(PinB) 
    {
      // set pin a and b to be input
      pinMode(pin_a, INPUT);
      pinMode(pin_b, INPUT);
      // and turn on pull-up resistors
      digitalWrite(pin_a, HIGH);
      digitalWrite(pin_b, HIGH);
#ifdef CALCULATE_SPEED
      previousTime = millis();
#endif      
    };

#ifdef CALCULATE_SPEED
    Rotary_Encoder( int8_t PinA, int8_t PinB, int signal_revolution) : pin_a (PinA), pin_b(PinB), signalPerRevolution(signal_revolution) 
    {
      // set pin a and b to be input
      pinMode(pin_a, INPUT);
      pinMode(pin_b, INPUT);
      // and turn on pull-up resistors
      digitalWrite(pin_a, HIGH);
      digitalWrite(pin_b, HIGH);
    };

    Rotary_Encoder( int8_t PinA, int8_t PinB, int signal_revolution, float wheel_radius) : pin_a (PinA), pin_b(PinB), signalPerRevolution(signal_revolution), wheelRadius(wheel_diameter)
    {
      // set pin a and b to be input
      pinMode(pin_a, INPUT);
      pinMode(pin_b, INPUT);
      // and turn on pull-up resistors
      digitalWrite(pin_a, HIGH);
      digitalWrite(pin_b, HIGH);
    };
#endif    

    void update () 
    {
        
        if (digitalRead(pin_a)) digitalRead(pin_b) ? position++ : position--;
        else digitalRead(pin_b) ? position-- : position++;
#ifdef CALCULATE_SPEED        
        if(position >= signalPerRevolution)
        {
            nowTime = millis();
            deltaTime = nowTime -  previousTime;
        }
#endif       
    };

    long int getPosition () {
      return position;
    };

    void setPosition ( const long int p) {
      position = p;
    };

#ifdef CALCULATE_SPEED
    double getAngularSpeed() //in Hz
    {
        return (6.2831/deltaTime);
    }

    double getLinearSpeed() // in m/s
    {
        return (6.2831 * wheelRadius/deltaTime);
    }

    void calculateSpeed()
    {

    }
#endif    

  private:
    long int position;
    int8_t pin_a;
    int8_t pin_b;
#ifdef CALCULATE_SPEED    
    float wheelRadius;
    int signalPerRevolution;
    unsigned long previousTime, nowTime, deltaTime;
#endif 
};   

#endif // __ROTARY_ENCODER_H__
