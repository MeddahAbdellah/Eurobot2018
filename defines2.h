
/*
******			Auteur : Meddah Abdellah
******			
******			L'objet Robot permet aprés avoir fait les bons branchement de controller le déplacement du robot 
******			(evitement et débloquage automatique inclus) , vous aurez sans doute besoin de changer les paramétres
******			qui se trouvent dans le fichier defines2.h si vous changer de plateforme roulante.
******			Les Méthode qui constituent cette classent sont :
******			-aller : qui envoie le robot vers une position (x,y) sans lui imposé un angle précis d'arrivé,
******							 cette méthode prend essentiellement en parmétre la position vers la quelle doit se
******							 rendre le robot en coordonées carthésienne absolu. 
******							 Elle prend en paramétre facultatif:
******										 -l'activation de l'évitement 
******										 -la tolérence à l'arrivé au point demandé
******										 -le temps de régulation aprés le quelle si le robot n'arrive pas à sa destination
******											il abandonne le déplacement, pour eviter les bloquage.
******										 -mode de déplacement en COURBE ou DIRECT.
******			-tourner: qui fait tourner le robot sur lui meme vers un angle absolue.
******								cette méthode prend éssentiellement en paramétre l'angle que doit avoir le robot.
******								elle prend les meme paramétre facultatif que la méthode aller. Sauf pour le mode de rotation
******								qui est soit NORMAL quand le robot tourne sur lui meme ou MARCHEENTOURNANT quand le robot doit
******								tourner en déplacement.
******			-positionner : qui prend trois paramétre essentielle x,y,theta et qui est une combinaison de aller puis tourner.
******										 elle prend les meme paramétre facultatifs que aller.
******			-avancer : qui prend en paramétre une distance soit positif ou négatif , pour faire avancer ou reculer le robot 
******								 elle prend les meme paramétre facultatifs que aller.
******			-d'autres méthode aussi qui sont mentionné en bas et qui portent leur noms.										
*/

// Constante de travaille
#define PI 3.14159265359
#define NORMAL 5000  // constante arbitraire
#define MARCHEENTOURNANT 1122 // constante arbitraire
#define DIRECT 1523 //constante arbitraire
#define COURBE 1447 //constante arbitraire
/*** Odometry ***/
#define pulsationsParRevolution 2048
#define RAYONGAUCHE 2.6f
#define RAYONDROITE 2.6f
#define DISTANCEENTRELESENCODEURS 31.2f
#define Ts 0.01
#define VITESSEMAX 20
#define VITESSEROTATIONMAX 20

/* Regulation Vitesse linéaire du robot */
#define kp_v 3.2f
#define ki_v 0.0f
#define kd_v 0.11f//0.12

/* Regulation Orientation du robot */
#define kp_phi 27.2f//28.0
#define ki_phi 0.0f
#define kd_phi 1.242f//1.25

/* Regulation Vitesse des moteurs */
#define kp_w 1.6f
#define ki_w 9.5f
#define kd_w 0.0f
/* Paramtere des moteurs*/
#define R 3.35f
#define L 19.2f
#define l 5.9f
#define TOLERENCEROTATION 0.001
#define TOLERENCEDEPLACEMENTROTATION 0.1
#define TOLERENCEDEPLACEMENT 0.1f
#define TOLERENCESTOP 0.0f
#define TOLERENCEEVITEMENT 3.0f
#define TIMETOSTOPREGULATION 20

/*Parametre de L'evitement*/
#define n 10

#define poidsGeneral 100
#define poids_but 5
#define poids_obst 10
#define influ_obst 3
#define poids_mur 5
#define influ_mur 1

#define TOLERENCE_I_J 0.3
#define unite_x 20
#define unite_y 30
