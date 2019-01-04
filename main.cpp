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

#include <mbed.h>
#include "defines2.h"
#include "Robot.h"
#include "Odometry.h"
#include "QEI.h"
#include "math.h"

Robot monRobot(p13,p14,p11,p12,p23,p21,p25,p24);//IN1=p21(->b sur la carte de AitSaid) IN2=p23 IN3=p24 ,IN4=p25
/*pour les encodeur 13 14 et 11 12 pour la cobinaison bleu blanc  blanc bleu et le moteur G au 11
                                 moteur Droite , le plus a droite du mbed , l'autre c'est le gauche*/
Serial arduinoMega(p9,p10,115200);
DigitalIn start(p6);
Timeout match;
void waitForArduino()/* cette fonction permet à l'mbed d'attendre une commande de l'arduino , ca remplace le wait car
                     celui ci ne marche pas bien vu le nombre important d'interruption */
{
    char recievedFromArduino='a';
    while(recievedFromArduino!='f')
    {
        if(arduinoMega.readable())
        {
            recievedFromArduino=arduinoMega.getc();
        }
    }
    while(arduinoMega.readable())
    {
        arduinoMega.getc();
    }
}
void stopMatch()// arreter le match
{
    monRobot.stop();
    arduinoMega.printf("s");// ces caractére servent de flags ou signe pour communiquer avec l'arduino
    arduinoMega.printf("9");// il est conseillé d'utilisé un seul caractére pour éviter l'encombrement
    while(1);
}
int main()
{

    while(arduinoMega.readable())// vider les registre du Serial
    {
        arduinoMega.getc();
    }
    while(start == 0) {}// attendre le signal de départ

    match.attach(&stopMatch,100);
    arduinoMega.printf("6");
    monRobot.avancer(-87.097);// avant panneau
    monRobot.tourner(PI/2);
    monRobot.avancer(-32.503,true,TOLERENCEDEPLACEMENT,2); // interrupteur du panneau
    monRobot.avancer(10.0f);
    monRobot.tourner(6*PI/12);
    monRobot.aller(-60.781,80.715,true,2.0);//avant avant abeille
    monRobot.tourner(PI/3);
    monRobot.positionner(-8.781,126.715,-0.542*PI/180);//avant abeille
    arduinoMega.printf("1");//dire a l'arduino de déployer
    waitForArduino();
    monRobot.tourner(PI/2);//abeille
    monRobot.tourner(5*PI/12,false);
    monRobot.positionner(-35.957,-0.0500,-121.431*PI/180);//avant recuperation_chateau
    monRobot.positionner(-11.762,35.863, -128.755*PI/180,false,TOLERENCEDEPLACEMENT,2);//recuperation_chateau
    monRobot.vibrerEnRotation(1);
    arduinoMega.printf("0");//dire a l'arduino qu'on est entrein de récuperer
    waitForArduino();
    arduinoMega.printf("4");// dire a l'arduino de charger le canon
    monRobot.avancer(10.0,true,2);
    monRobot.positionner(-72.840,78.368,110.247*PI/180);//tir_chateau_8V
    arduinoMega.printf("2");//dire a l'arduino d'ouvrire la vane pour tirer
    waitForArduino();
    monRobot.aller(-87.097,0.0);// avant panneau
    monRobot.tourner(PI/2);
    monRobot.avancer(-32.503,true,TOLERENCEDEPLACEMENT,2); // interrupteur du panneau
    monRobot.avancer(10.0f);
    monRobot.positionner(-218.196,78.368,-PI/2,true,1.0);	//avant1 recuperation_epuration
    monRobot.positionner(-218.196,97.368,-PI/2,true,1.0);	//avant1 recuperation_epuration
    monRobot.positionner(-218.196,122.964,-37.762*PI/180,false);	//avant2 recuperation_epuration
    monRobot.positionner(-223.788,127.269,-37.264*PI/180);
    monRobot.vibrerEnRotation(2);
    arduinoMega.printf("8");//dire a l'arduino qu'on est entrein de récuperer
    waitForArduino();
    monRobot.avancer(5.0,true,1.0);
    arduinoMega.printf("5");// dire a l'arduino de charger le canon
    monRobot.tourner(-PI/2);//avant avant tire
    monRobot.avancer(20,true,2.0);//avant tire , le troisiemme paramétre est la tolerence
    monRobot.positionner(-217.009,118.189,-165.947*PI/180);  //tir epuration_8V
    arduinoMega.printf("3");//dire a l'arduino de tirer a la station d'épuration


}


