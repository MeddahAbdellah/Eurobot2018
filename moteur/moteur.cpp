#include "moteur.h"

Moteur::Moteur(PinName vitesseIn1,PinName vitesseIn2) : m_vitesseIn1(vitesseIn1) , m_vitesseIn2(vitesseIn2)
{m_vitesseIn1.period_us(20);
 m_vitesseIn2.period_us(20);
 m_vitesseIn1=0;
 m_vitesseIn2=0;
}

void Moteur::controle(float vitesse)
{ 
if(vitesse>=0){
    m_vitesseIn1.write(vitesse);
    m_vitesseIn2.write(0.0f);
    }
    else if(vitesse<0){
    m_vitesseIn1.write(0.0f);
    m_vitesseIn2.write(abs(vitesse));
        }
}

void Moteur::stop()
{
    m_vitesseIn1=0;
    m_vitesseIn2=0;
}


