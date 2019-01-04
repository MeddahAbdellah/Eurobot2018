/**
 * @author BERTELONE Benjamin
 *
 * @section DESCRIPTION
 * 
 */

#include "Odometry2.h"
#include "defines.h"

Odometry2::Odometry2(QEI *qei_left, QEI *qei_right, float radius_left, float radius_right, float v)
{
    m_qei_left = qei_left;
    m_qei_right = qei_right;
    m_v = v;
    
    m_distPerTick_left = radius_left/qei_left->getPulsesPerRev()*2*PI;
    m_distPerTick_right = radius_right/qei_right->getPulsesPerRev()*2*PI;
    
    m_pulses_left = qei_left->getPulses();
    m_pulses_right = qei_right->getPulses();
    
    setPos(0.0f,0.0f,0.0f);
    
    // Vitesse du moteur gauche et droit
    m_vitLeft = 0;
    m_vitRight = 0;
}

void Odometry2::setPos(float x, float y, float theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}

void Odometry2::setX(float x)
{
    this->x = x;
}

void Odometry2::setY(float y)
{
    this->y = y;
}

void Odometry2::setTheta(float theta)
{
    this->theta = theta;
}

void Odometry2::update(float dt)
{
    int delta_left = m_qei_left->getPulses() - m_pulses_left;
    m_pulses_left = m_qei_left->getPulses();
    int delta_right = m_qei_right->getPulses() - m_pulses_right;
    m_pulses_right = m_qei_right->getPulses();
    
    m_distLeft = m_pulses_left*m_distPerTick_left;
    m_distRight = m_pulses_right*m_distPerTick_right;
    
    m_vitLeft = m_distPerTick_left*delta_left/dt;
    m_vitRight = m_distPerTick_right*delta_right/dt;
    
    float deltaS = (m_distPerTick_left*delta_left + m_distPerTick_right*delta_right) / 2.0f;
    float deltaTheta = (m_distPerTick_right*delta_right - m_distPerTick_left*delta_left) / m_v;
    
    float dx = deltaS*cos(theta);
    float dy = deltaS*sin(theta);
    
    x += dx;
    y += dy;
    theta += deltaTheta;
    
    while(theta > PI) theta -= 2*PI;
    while(theta <= -PI) theta += 2*PI;
}


