/*
******			Auteur : Meddah Abdellah
******
******			L'objet Robot permet après avoir fait les bons branchements de contrôler le déplacement du robot
******			(évitement et déblocage automatique inclus), vous aurez sans doute besoin de changer les paramètres
******			qui se trouvent dans le fichier defines2.h si vous changer de plateforme roulante.
******			Les Méthode qui constituent cette classent sont :
******			-aller : qui envoie le robot vers une position (x,y) sans lui imposé un angle précis d'arrivé,
******							 cette méthode prend essentiellement en paramètre la position vers la quelle doit se
******							 rendre le robot en coordonnées cartésiennes absolu.
******							 Elle prend en paramètre facultatif:
******										 -l'activation de l'évitement
******										 -la tolérance à l'arrivé au point demandé
******										 -le temps de régulation après le quelle si le robot n'arrive pas à sa destination
******											il abandonne le déplacement, pour éviter les blocages.
******										 -mode de déplacement en COURBE ou DIRECT.
******			-tourner: qui fait tourner le robot sur lui-même vers un angle absolu.
******								cette méthode prend essentiellement en paramètre l'angle que doit avoir le robot.
******								elle prend le même paramètre facultatif que la méthode aller. Sauf pour le mode de rotation
******								qui est soit NORMAL quand le robot tourne sur lui-même ou MARCHEENTOURNANT quand le robot doit
******								tourner en déplacement.
******			-positionner : qui prend trois paramètre essentielle x,y,theta et qui est une combinaison de aller puis tourner.
******										 elle prend les mêmes paramètres facultatifs qu’aller.
******			-avancer : qui prend en paramètre une distance soit positif ou négatif , pour faire avancer ou reculer le robot
******								 elle prend les mêmes paramètres facultatifs qu’aller.
******			-d'autres méthode aussi qui sont mentionné en bas et qui portent leur noms.
******
******      IMPORTANT: la classe robot doit être accompagné des objets : QEI, Odometry,moteur,mbed
*/
#ifndef ROBOT_H
#define ROBOT_H

#include "mbed.h"
#include "moteur.h"
#include "QEI.h"
#include "Odometry.h"
#include "defines2.h"

class Robot
{
    public:
        Robot(PinName encodeurDroiteA,PinName encodeurDroiteB,//pins de l'encodeur de la roue droite
              PinName encodeurGaucheA,PinName encodeurGaucheB,//pins de l'encodeur de la roue gauche
              PinName moteurDroiteIn1,PinName moteurDroiteIn2,//pins du moteur Droite
              PinName moteurGaucheIn1,PinName moteurGaucheIn2);//pins du moteur Gauche

              void setPostion(float x ,float y,float theta){m_odometrie->setPos(x,y,theta);}
              float getPosX(){return m_odometrie->getX();}
              float getPosY(){return m_odometrie->getY();}
              float getAngle(){return m_odometrie->getTheta();}


              void aller(float xConsigne,float yConsigne,bool activerEvitement=true,float tolerence=TOLERENCEDEPLACEMENT,int timeToStopReg=TIMETOSTOPREGULATION,int modeDeDeplacement=DIRECT);
              void tourner(float thetaC,bool activerEvitement=true, int modeDeRotation=NORMAL,float tolerence=TOLERENCEROTATION,int timeToStopReg=TIMETOSTOPREGULATION); // theta <-[-pi,pi]; and if thetaActuel n'est pas donné on le sera dans le corp de la fonction
              void avancer(float distanceConsigne,bool activerEvitement=true,float tolerence=TOLERENCEDEPLACEMENT,int timeToStopReg=TIMETOSTOPREGULATION);
              void positionner(float xC,float yC, float thetaC,bool activerEvitement=true,float tolerence=TOLERENCEDEPLACEMENT,int timeToStopReg=TIMETOSTOPREGULATION,int modeDeDeplacement=DIRECT);
              void vibrerEnRotation(int nombreDeVibrations=1);
              void vibrerEnTranslation(int nombreDeVibrations=1);
              void bloquerSurPlace();//le robot stop est résiste au déplacement
              void stop();
        virtual ~Robot();
    private :
    Moteur m_moteurG;
    Moteur m_moteurD;
    QEI m_qeiD;
    QEI m_qeiG;

    Odometry* m_odometrie;
    void vitesseAngulaireVoulu(float& vitesseAngulaireVouluDroite, float& vitesseAngulaireVouluGauche);
};

#endif // ROBOT_H
