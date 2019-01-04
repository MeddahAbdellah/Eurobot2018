# Eurobot2018
Bibliothèque complète qui permet de contrôler un robot mobile à 2 roues.
L'objet Robot permet après avoir fait les bons branchements de contrôler le déplacement du robot (évitement et déblocage automatique inclus), vous aurez sans doute besoin de changer les paramètres qui se trouvent dans le fichier defines2.h si vous changer de plateforme roulante.
### Declaration de l'objet

	Robot(PinName encodeurDA,PinName encodeurDB,PinName encodeurGA,PinName encodeurGB,
	      PinName moteurDIn1,PinName moteurDIn2,PinName moteurGIn1,PinName moteurGIn2);

### aller(x,y,facultatifs...)

	monRobot.aller(float xConsigne,float yConsigne,bool activerEvitement=true,float tolerence=TOLERENCEDEPLACEMENT,
		       int timeToStopReg=TIMETOSTOPREGULATION,int modeDeDeplacement=DIRECT);
	
qui envoie le robot vers une position (x,y) sans lui imposé un angle précis d'arrivé,
cette méthode prend essentiellement en paramètre la position vers la quelle doit se
rendre le robot en coordonnées cartésiennes absolu.
Elle prend en paramètre facultatif:<br>
-l'activation de l'évitement.<br>
-la tolérance à l'arrivé au point demandé.<br><br>
-le temps de régulation après le quelle si le robot n'arrive pas à sa destination il abandonne le déplacement, pour éviter les blocages.<br>
-mode de déplacement en COURBE ou DIRECT.<br>

### tourner(theta,facultatifs...)

	monRobot.tourner(float thetaC,bool activerEvitement=true, int modeDeRotation=NORMAL,float tolerence=TOLERENCEROTATION,
	                 int timeToStopReg=TIMETOSTOPREGULATION);	

qui fait tourner le robot sur lui-même vers un angle absolu.
cette méthode prend essentiellement en paramètre l'angle que doit avoir le robot. elle prend le même paramètre facultatif que la méthode aller. <br>
-Sauf pour le mode de rotation qui est soit NORMAL quand le robot tourne sur lui-même ou MARCHEENTOURNANT quand le robot doit tourner en déplacement.
			
### positionner(x,y,theta,facultatifs...)

	monRobot.positionner(float xC,float yC, float thetaC,bool activerEvitement=true,float tolerence=TOLERENCEDEPLACEMENT,
	                     int timeToStopReg=TIMETOSTOPREGULATION,int modeDeDeplacement=DIRECT);

qui prend trois paramètre essentielle x,y,theta et qui est une combinaison de aller puis tourner.elle prend les mêmes paramètres facultatifs qu’aller.

### avancer(distance,facultatifs...)

	monRobot.avancer(float distanceConsigne,bool activerEvitement=true,float tolerence=TOLERENCEDEPLACEMENT,
			 int timeToStopReg=TIMETOSTOPREGULATION);
	
qui prend en paramètre une distance soit positif ou négatif , pour faire avancer ou reculer le robot elle prend les mêmes paramètres facultatifs qu’aller.

### vibrerEnRotation(nombreDeVibrations)

	monRobot.vibrerEnRotation(int nombreDeVibrations=1);
	
qui fait vibrer le robot autour de son axe de rotation.

### vibrerEnTranslation(nombreDeVibrations)

	monRobot.vibrerEnTranslation(int nombreDeVibrations=1);

qui fait virbrer le root en translation.

### bloquerSurPlace()

	monRobot.bloquerSurPlace();

qui block le robot sur place (ne le laisse pas bouger meme si des force externes y sont appliqué sur lui);

### stop();

	monRobot.stop();

qui éteint les moteurs.

## Note

      IMPORTANT: la classe robot doit être accompagné des objets : QEI, Odometry,moteur,mbed
