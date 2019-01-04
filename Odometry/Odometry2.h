#ifndef ODOMETRY2_H
#define ODOMETRY2_H

#include "mbed.h"
#include "QEI.h"

class Odometry2
{
    public:
        Odometry2(QEI *qei_left, QEI *qei_right, float radius_left, float radius_right, float v);
        
        void setPos(float x, float y, float theta);
        void setX(float x);
        void setY(float y);
        void setTheta(float theta);
        
        float getX() {return x;}
        float getY() {return y;}
        float getTheta() {return theta;} // ]-PI;PI]
        
        float getVitLeft() {return m_vitLeft;}
        float getVitRight() {return m_vitRight;}
        
        float getDistLeft() {return m_distLeft;}
        float getDistRight() {return m_distRight;}
        
        void setDistLeft(float dist) {m_distLeft = dist;}
        void setDistRight(float dist) {m_distRight = dist;}
        
        void update(float dt);
        
        int getPulsesLeft(void) {return m_pulses_left;}
        int getPulsesRight(void) {return m_pulses_right;}
    
    private:
        QEI* m_qei_left;
        int m_pulses_left;
        QEI* m_qei_right;
        int m_pulses_right;
        
        float x, y, theta;
        float m_vitLeft, m_vitRight;
        float m_distLeft, m_distRight;
        
        float m_distPerTick_left, m_distPerTick_right, m_v;
};

#endif
