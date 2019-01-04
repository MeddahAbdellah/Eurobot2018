#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "mbed.h"
#include "QEI.h"
#include "defines2.h"
#define Ts 0.01
#define PI 3.14159265359


extern Serial logger;

class Odometry
{
    public:
        Odometry(QEI *qei_left, QEI *qei_right, float radius_left=RAYONGAUCHE, float radius_right=RAYONDROITE, float v=DISTANCEENTRELESENCODEURS );
        
        void setPos(float x, float y, float theta);
        void setVit(float Vx, float Vy, float W);
        void setVitPhi(float phi_r, float phi_l);
        void setX(float x);
        void setY(float Y);
        void setTheta(float theta);
        
        float getX() {return x;}
        float getY() {return y;}
        float getTheta() {return theta;}
        
        void reset();
        
        float getVx()   {return Vx;}
        float getVy()   {return Vy;}
        float getW()   {return W;}
        float getPhiright()
        {
            phi_r = (m_distPerTick_right/radius_right)*(delta_right/dt);             
            //logger.printf("phi r = %f \r\n", phi_r);
            return phi_r;
        }
        
        float getPhileft() 
        {
            phi_l = (m_distPerTick_left/radius_left)*(delta_left/dt);            
            //logger.printf("phi  = %f \r\n", phi_l);
            return phi_l;
        }
        
        float getVitLeft() {return m_vitLeft;}
        float getVitRight() {return m_vitRight;}
        
        void update();
    
    private:
        QEI* m_qei_left;
        int m_pulses_left;
        QEI* m_qei_right;
        int m_pulses_right;
        int delta_right;
        int delta_left;
        float radius_left;
        float radius_right;
        
        volatile float x, y, theta;
        volatile float m_vitLeft, m_vitRight;
        volatile float offsetVx, offsetVy;
        bool initoffset;
        volatile float Vx,Vy,W;
        volatile float phi_r,phi_l;
        volatile float dt;
        Timer timer;
        
        float m_distPerTick_left, m_distPerTick_right, m_v;
        
        Ticker updater;
};

#endif
