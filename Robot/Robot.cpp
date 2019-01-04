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
******      IMPORTANT: la classe robot doit être accompagné des objets : QEI, Odometry,moteur,mbe
*/
#include "Robot.h"
#include "Odometry.h"
#include "QEI.h"
#include "math.h"
/*
e : erreur
C: Consigne
w: vitesseAngulaire
D: Droite
G: Gauche
eWiD : ewIntegralD
eWiG : ewIntegralG
eAtheta : eAncienneTheta
eTheta: eTheta
vC : vitesseC
thetaC : thetaC
thetaAct: thetaActuel

    */
float sign(float x)
{
    if( x>=0) return 1.0f;
    return -1.0f;
}
Serial arduinoEvitement(p28,p27,115200);// communication série avec arduino
char posObstacle='a';
bool stopRegulation=false;
Timeout timeToStop;// timeout pour arreter la régulation
Robot:: Robot(PinName encodeurDA,PinName encodeurDB,
              PinName encodeurGA,PinName encodeurGB,
              PinName moteurDIn1,PinName moteurDIn2,
              PinName moteurGIn1,PinName moteurGIn2):m_moteurG(moteurGIn1,moteurGIn2),
    m_moteurD(moteurDIn1,moteurDIn2),
    m_qeiD(encodeurDA,encodeurDB,NC,pulsationsParRevolution),
    m_qeiG(encodeurGA,encodeurGB,NC,pulsationsParRevolution)
{
    m_odometrie=new Odometry(&m_qeiG,&m_qeiD,RAYONGAUCHE,RAYONDROITE,DISTANCEENTRELESENCODEURS);/*Creation d'un objet odometrie , pour lui passer les QEI , les rayons et v
*/

}
void stopRegulateur() //stopper la régulation
{
    stopRegulation=true;
}
void Robot::aller(float xC,float yC,bool activerEvitement,float tolerence,int timeToStopReg,int modeDeDeplacement)
{

    /* le robot tourne d'abord puis il avance ainsi le PID ne causera des oscillations que sur un seul axe*/
    float eAncienneTheta=0.0, wD=0.0, wG=0.0, wC=0.0,eTheta=0.0,
          eThetaDerive=0.0,thetaVoulu=0.0,thetaAct=0.0,thetaC=0.0;
    bool calculDerive=false;

    float ePosX=0.0, ePosY=0.0,ePosXDerive=0.0,posXVoulu=0.0,ePosYDerive=0.0, posYVoulu=0.0,
          eAncienneX=0.0, eAncienneY=0.0,vC=0.0,inverseur=0.0;
    stopRegulation=false;
    timeToStop.attach(&stopRegulateur,timeToStopReg);// on attache la fonction arret de régulation , et on donne en paramétre le temps pour l'arret

    do
    {
        thetaC=atan2(yC-this->getPosY(),xC-this->getPosX());//calcul de thetaC
        thetaAct=this->getAngle();
        if(abs(thetaC-this->getAngle())>PI/2)
        {
            thetaAct+=sign(thetaC-this->getAngle())*PI;
        }
        if(abs(thetaC-thetaAct)>=PI/3)
        {
            tourner(thetaC,activerEvitement,MARCHEENTOURNANT);
        }
        eTheta=thetaC-thetaAct;             //e = C - actuel
        if(calculDerive)
        {
            eThetaDerive=(eTheta-eAncienneTheta)/Ts;     //calcul de la derive=f(t+t0)-f(t)/Ts
        }
        calculDerive=true;
        eAncienneTheta=eTheta;                           //sauvegarde de l'e pour une prochaine utilisation(calcul de la derive)
        thetaVoulu=kp_phi*eTheta+kd_phi*eThetaDerive;//l'equation du PID
        wC=(L*thetaVoulu)/(2*R);
        //PID sur theta fin

        //PID sur axe de mouvement a inserer ici
        ePosX=xC-this->getPosX();             //e = C - actuel
        ePosXDerive=(ePosX-eAncienneX)/Ts;     //calcul de la derive=f(t+t0)-f(t)/Ts
        eAncienneX=ePosX;                           //sauvegarde de l'e pour une prochaine utilisation(calcul de la derive)
        posXVoulu=kp_v*ePosX+kd_v*ePosXDerive;//l'equation du PID

        ePosY=yC-this->getPosY();             //e = C - actuel
        ePosYDerive=(ePosY-eAncienneY)/Ts;     //calcul de la derive=f(t+t0)-f(t)/Ts
        eAncienneY=ePosY;                           //sauvegarde de l'e pour une prochaine utilisation(calcul de la derive)
        posYVoulu=kp_v*ePosY+kd_v*ePosYDerive;//l'equation du PID
        vC=sqrt((posXVoulu*posXVoulu)+(posYVoulu*posYVoulu));//calcul de la vitesse conseigne

        //PID sur axe de mouvement fin
        printf("wC: %f\r\n",wC);
        wD=((vC/R)+wC);//affectation de la vitesse afin de la réguler
        wG=((vC/R)-wC);// pour tourner les vitesse doivent etre inversé
        // Limitation de vitesse
        if(modeDeDeplacement==DIRECT)
        {
            if(abs(wD)>VITESSEMAX && eTheta>=0)
            {
                wD=sign(wD)*VITESSEMAX;   //Ne pas dépasser la vitesse max
                wG=wD-2*wC;
                if(abs(wG)>VITESSEMAX)
                {
                    wG=sign(wG)*VITESSEMAX;    //Ne pas dépasser la vitesse max
                }
            }
            if(abs(wG)>VITESSEMAX && eTheta<0)
            {
                wG=sign(wG)*VITESSEMAX;   //Ne pas dépasser la vitesse max
                wD=wG+2*wC;
                if(abs(wD)>VITESSEMAX)
                {
                    wD=sign(wD)*VITESSEMAX;    //Ne pas dépasser la vitesse max
                }
            }
        }
        else if(modeDeDeplacement==COURBE)
        {
            if(abs(wG)>VITESSEMAX)
            {
                wG=sign(wG)*VITESSEMAX;    //Ne pas dépasser la vitesse max
            }
            if(abs(wD)>VITESSEMAX)
            {
                wD=sign(wD)*VITESSEMAX;    //Ne pas dépasser la vitesse max
            }
        }
        // Limitation de vitesse Fin
        if(abs(thetaC-this->getAngle())>(PI/2))// si l'angle est entre le robot et la position cible le GO HERE
        {
            inverseur=-wD;
            wD=-wG;
            wG=inverseur;
        }
        //Evitement Debut
        if(activerEvitement)//zone d'évitment
        {
            if(abs(atan2(yC-this->getPosY(),xC-this->getPosX())-this->getAngle())>PI/2)
            {
                arduinoEvitement.printf("3");
            }
            if(abs(atan2(yC-this->getPosY(),xC-this->getPosX())-this->getAngle())<PI/2)
            {
                arduinoEvitement.printf("0");
            }
            if(arduinoEvitement.readable())
            {
                posObstacle=arduinoEvitement.getc();
            }
            if(abs(atan2(yC-this->getPosY(),xC-this->getPosX())-this->getAngle())>PI/2 && (posObstacle=='4'|| posObstacle=='5'|| posObstacle=='6'))
            {
                arduinoEvitement.printf("3");
                wD=0.0;
                wG=0.0;
            }
            else if(abs(atan2(yC-this->getPosY(),xC-this->getPosX())-this->getAngle())<PI/2 && (posObstacle=='1'|| posObstacle=='2'|| posObstacle=='3'))
            {
                arduinoEvitement.printf("0");
                wD=0.0;
                wG=0.0;
            }
        }
        //Evitement Fin
//apel de PID sur Moteurs
        this->vitesseAngulaireVoulu(wD,wG);// les vitesse sont envoyé par référence de tels façon a se qu'elle soient modifié a l'interieur de la méthode
//PID sur Moteurs fin
        m_moteurD.controle(wD);//affectation des vitesses au moteurs
        m_moteurG.controle(wG);
    }
    while(abs(sqrt(ePosX*ePosX+ePosY*ePosY))>tolerence && !stopRegulation); //condition d'arret , ( a*a est plus facile a calculer que pow(a,2.0) pour un microcontroleur)
    this->stop();
    timeToStop.detach();
}

void Robot::tourner(float thetaC,bool activerEvitement,int modeDeRotation,float tolerence,int timeToStopReg)
{
    float eAncienne=0.0, wD=0.0, wG=0.0, wC=0.0,eTheta=0.0,
          eThetaDerive=0.0,thetaVoulu=0.0,thetaAct=0.0;
    stopRegulation=false;
    timeToStop.attach(&stopRegulateur,timeToStopReg);// on attache la fonction arret de régulation , et on donne en paramétre le temps pour l'arret
    do
    {
//PID sur theta a inserer ici
        if(modeDeRotation==NORMAL)
        {
            thetaAct=this->getAngle();
        }
        else if(modeDeRotation==MARCHEENTOURNANT)
        {
            thetaAct=this->getAngle();
            if(abs(thetaC-this->getAngle())>PI/2)
            {
                thetaAct+=sign(thetaC-this->getAngle())*PI;
            }
        }
        eTheta=thetaC-thetaAct;             //e = C - actuel
        eThetaDerive=(eTheta-eAncienne)/Ts;     //calcul de la derive=f(t+t0)-f(t)/Ts
        eAncienne=eTheta;                           //sauvegarde de l'e pour une prochaine utilisation(calcul de la derive)
        thetaVoulu=kp_phi*eTheta+kd_phi*eThetaDerive;//l'equation du PID
        wC=(L*thetaVoulu)/(2*R);
        printf("eTheta: %f\r\n",eTheta);
        //protection contre les dépassement
        if(abs(wC)>VITESSEROTATIONMAX)
        {
            wC=sign(wC)*VITESSEROTATIONMAX;   //Ne pas dépasser la vitesse max
        }
        //protection contre les dépassement fin
//PID sur theta fin
        wD=wC;//affectation de la vitesse afin de la réguler
        wG=-wC;// pour tourner les vitesse doivent etre inversé
//apel de PID sur Moteurs
        this->vitesseAngulaireVoulu(wD,wG);// les vitesse sont envoyé par référence de tels façon a se qu'elle soient modifié a l'interieur de la méthode
//PID sur Moteurs fin

        //Evitement Debut
        if(activerEvitement)//zone d'évitment
        {
            if(abs(thetaC-this->getAngle())<0)
            {
                arduinoEvitement.printf("3");
            }
            if(abs(thetaC-this->getAngle())>0)
            {
                arduinoEvitement.printf("0");
            }
            if(arduinoEvitement.readable())
            {
                posObstacle=arduinoEvitement.getc();
            }
            if(abs(thetaC-this->getAngle())<0 && (posObstacle=='4'|| posObstacle=='5'|| posObstacle=='6'))
            {
                arduinoEvitement.printf("3");
                this->stop();
                wD=0.0;
                wG=0.0;
            }
            else if(abs(thetaC-this->getAngle())>0 && (posObstacle=='1'|| posObstacle=='2'|| posObstacle=='3'))
            {
                arduinoEvitement.printf("0");
                this->stop();
                wD=0.0;
                wG=0.0;
            }
        }
        //Evitement Fin

        m_moteurD.controle(wD);//affectation des vitesses au moteurs
        m_moteurG.controle(wG);
        //wait(Ts);
    }
    while(abs(eTheta)>tolerence && !stopRegulation); //refaire jusqu'a atteindre l'e voulu
    this->stop();
    timeToStop.detach();
}
//Regulation PID des moteurs , une fonction interne a u cpp de la la classe robot
void Robot::vitesseAngulaireVoulu(float& wVouluD, float& wVouluG)
{
    /*le PID se fait de la meme façon pour les deux roues sauf que l'odometrie rends des
                                                                                                               résultats différents a chaque fois pour chaque roue*/
    float ewD=0.0,ewG=0.0;
    static float eWiD=0.0,eWiG=0.0;
    float wActuelD = (this->m_odometrie->getVitRight()+(l*this->m_odometrie->getW()))/R;//récuperer la vitesse angulaire actuel a partir de l'objet odometrie et l'affecter a robot
    float wActuelG = (this->m_odometrie->getVitLeft()-(l*this->m_odometrie->getW()))/R;
    ewD = wVouluD-wActuelD;//e = C - actuel
    ewG = wVouluG-wActuelG;
    eWiD +=Ts*ewD;// calcul d'integral
    eWiG +=Ts*ewG;
    if(abs(ki_w*eWiD/12)>1.0f)//anti WindUp
    {
        eWiD=(sign(eWiD)/ki_w)*12;
    }
    if(abs(ki_w*eWiG/12)>1.0f)//anti WindUp
    {
        eWiG=(sign(eWiG)/ki_w)*12;
    }
    wVouluD=(kp_w*ewD+ki_w*eWiD)/12;//l'equation du PID et calcul du pwm
    wVouluG=(kp_w*ewG+ki_w*eWiG)/12;

}
void Robot::avancer(float distanceConsigne,bool activerEvitement,float tolerence,int timeToStopReg) //avance d'une distance précise
{
    this->aller(this->getPosX()+distanceConsigne*cos(this->getAngle()),this->getPosY()+distanceConsigne*sin(this->getAngle()),activerEvitement,tolerence,timeToStopReg);
}
void Robot::positionner(float xC,float yC, float thetaC,bool activerEvitement,float tolerence,int timeToStopReg,int modeDeDeplacement)  // combinaison de aller et tourner
{
    this->aller(xC,yC,activerEvitement,tolerence,timeToStopReg,modeDeDeplacement);
    this->tourner(thetaC);
}
void Robot::vibrerEnRotation(int nombreDeVibrations) // faire vibrer le robot en rotation
{
    this->tourner(this->getAngle()+ 4*PI/180,true,NORMAL,0.01,1);
    for(int i=0; i<nombreDeVibrations; i++)
    {
        this->tourner(this->getAngle()- 8*PI/180,true,NORMAL,0.01,1);
        this->tourner(this->getAngle()+ 8*PI/180,true,NORMAL,0.01,1);
    }
    this->tourner(this->getAngle()- 4*PI/180,true,NORMAL,0.01,1);
}
void Robot::vibrerEnTranslation(int nombreDeVibrations) // faire vibrer le robot en translation
{
    for(int i=0; i<nombreDeVibrations; i++)
    {
        this->avancer(-2.0f,true,1.0f,1);
        this->avancer(2.0f,true,1.0f,1);
    }
}
void Robot::bloquerSurPlace() // bloquer le Robot sur place
{
    float wD=0.0,wG=0.0;
    this->vitesseAngulaireVoulu(wD,wG);
    m_moteurD.controle(wD);
    m_moteurG.controle(wG);
}
void Robot::stop()
{
    m_moteurD.stop();
    m_moteurG.stop();
}

Robot::~Robot()
{
    delete m_odometrie;// on détruit l'objet odometrie
}

