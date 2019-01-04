# Eurobot2018
Bibliothèque complète qui permet de contrôler un robot mobile à 2 roues.
			L'objet Robot permet après avoir fait les bons branchements de contrôler le déplacement du robot
			(évitement et déblocage automatique inclus), vous aurez sans doute besoin de changer les paramètres
  		qui se trouvent dans le fichier defines2.h si vous changer de plateforme roulante.
			Les Méthode qui constituent cette classent sont :
			-aller : qui envoie le robot vers une position (x,y) sans lui imposé un angle précis d'arrivé,
							 cette méthode prend essentiellement en paramètre la position vers la quelle doit se
							 rendre le robot en coordonnées cartésiennes absolu.
							 Elle prend en paramètre facultatif:
										 -l'activation de l'évitement
										 -la tolérance à l'arrivé au point demandé
										 -le temps de régulation après le quelle si le robot n'arrive pas à sa destination
											il abandonne le déplacement, pour éviter les blocages.
										 -mode de déplacement en COURBE ou DIRECT.
			-tourner: qui fait tourner le robot sur lui-même vers un angle absolu.
								cette méthode prend essentiellement en paramètre l'angle que doit avoir le robot.
								elle prend le même paramètre facultatif que la méthode aller. Sauf pour le mode de rotation
								qui est soit NORMAL quand le robot tourne sur lui-même ou MARCHEENTOURNANT quand le robot doit
								tourner en déplacement.
			-positionner : qui prend trois paramètre essentielle x,y,theta et qui est une combinaison de aller puis tourner.
										 elle prend les mêmes paramètres facultatifs qu’aller.
			-avancer : qui prend en paramètre une distance soit positif ou négatif , pour faire avancer ou reculer le robot
								 elle prend les mêmes paramètres facultatifs qu’aller.
			-d'autres méthode aussi qui sont mentionné en bas et qui portent leur noms.

      IMPORTANT: la classe robot doit être accompagné des objets : QEI, Odometry,moteur,mbe
