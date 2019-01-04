/**
 * @author BERTELONE Benjamin
 *
 * @section DESCRIPTION
 * 
 */

#include "Odometry.h"

extern Serial logger;


Odometry::Odometry(QEI *qei_left, QEI *qei_right, float radius_left, float radius_right, float v)
{
    m_qei_left = qei_left;
    m_qei_right = qei_right;
    this->radius_left = radius_left;
    this->radius_right = radius_right;
    this->delta_right = 0;
    this->delta_left = 0;
    
    m_distPerTick_left = ((float)radius_left)/((float)qei_left->getPulsesPerRev())*2.0f*3.1415f;
    m_distPerTick_right = ((float)radius_right)/((float)qei_right->getPulsesPerRev())*2.0f*3.1415f;    
    m_v = v;
    
    m_pulses_left = qei_left->getPulses();
    m_pulses_right = qei_right->getPulses();
    
    setPos(0,0,0);
    setVit(0,0,0);
    setVitPhi(0,0);
    dt = 0.1;
    
    initoffset = false;
    offsetVx = 0.0;
    offsetVy = 0.0;
    
    // Vitesse du moteur gauche et droit
    m_vitLeft = 0;
    m_vitRight = 0;
    
    updater.attach(this, &Odometry::update, Ts);
}

void Odometry::setPos(float x, float y, float theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}

void Odometry::setVit(float Vx, float Vy, float W)
{
    this->Vx = Vx;
    this->Vy = Vy;
    this->W = W;
}

void Odometry::setVitPhi(float phi_r, float phi_l)
{
    this->phi_r = phi_r;
    this->phi_l = phi_l;
}    

void Odometry::setX(float x)
{
    this->x = x;
}

void Odometry::setY(float Y)
{
    this->y = y;
}

void Odometry::setTheta(float theta)
{
    this->theta = theta;
}

void Odometry::reset()
{
    setPos(0,0,0);
    setVit(0,0,0);
    setVitPhi(0,0);
    m_pulses_left = m_qei_left->getPulses();
    m_pulses_right = m_qei_right->getPulses();
    initoffset = false;
}

void Odometry::update()
{
    this->dt = Ts;
    
    delta_left = m_qei_left->getPulses() - m_pulses_left;
    m_pulses_left = m_qei_left->getPulses();
    delta_right = m_qei_right->getPulses() - m_pulses_right;
    m_pulses_right = m_qei_right->getPulses();
    
    m_vitLeft = m_distPerTick_left*delta_left/dt;
    m_vitRight = m_distPerTick_right*delta_right/dt;
    
    float deltaS = (m_distPerTick_left*delta_left + m_distPerTick_right*delta_right) / 2.0f;
    float deltaTheta = (m_distPerTick_right*delta_right - m_distPerTick_left*delta_left) / m_v;
    
    float dx = deltaS*cos(theta);;
    float dy = deltaS*sin(theta);
    
    x += dx;
    y += dy;
    theta += deltaTheta;
    //update velocity
    //dt = time(NULL)-dt;    
    //dt = timer.read_ms()*1e3;
    //pc.printf("%f secondes",dt);
        
    if(!initoffset)
    {
        offsetVx = dx/dt;
        offsetVy = dy/dt;
        initoffset = true;
    }
    
    Vx = dx/dt-offsetVx;
    Vy = dy/dt-offsetVy;
    W = deltaTheta/dt;            
    //timer.stop();
    //timer.reset();
    //timer.start();
}



   
    