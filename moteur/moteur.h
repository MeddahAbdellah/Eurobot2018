#ifndef eurobot2017_H
#define eurobot2017_H
#include "mbed.h"
class Moteur
{
private:
    PwmOut m_vitesseIn1;// controle la vitesse du moteur 
    PwmOut m_vitesseIn2;
public:
Moteur(PinName vitesseIn1,PinName vitesseIn2); // IN1 = 1 IN2 = 0 avance , IN1 = 0 IN2 = 1 recule , other cases X

void controle (float vitesse);

void stop();

};

#endif
