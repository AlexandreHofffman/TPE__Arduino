# TPE__Arduino

***

## Configuration du système

* Pour utiliser l'accéléromètre, modifier la valeur suivante
```C++
const boolean accelero = true;
```

* Modifier le temps choisi pour déterminer la vitesse du vélo (5000 par défaut) :
```C++
const unsigned int timerValue = 5000;
```

* Choisir le diametre de la roue du vélo utilisé (en cm) :
```C++
const float diametreRoue = 31.85;
```
* Choisir le temps pour le rafraichissement des données provenant de l'accéléromètre :
```C++
const int tauxDeRafraichissement = 5;
```

* Activer les communications via le moniteur série (désactivé par défaut):
```C++
const boolean serial = false;
```

* Choisir les pins sur lesquels seront branchés les DEL ( _Attention, les pin pour les DEL rouge et blanche doivent être compatible avec les signaux MLI_ ):
```C++
int pinLedRouge = 11; // /!\ Signal MLI
int pinLedBlanc = 10; // /!\ Signal MLI
int pinLedStop = 9;
int pinLedLat = 8;
```

* Choisir les pins sur lesquels seront branchés les capteurs :
```C++
int pinPhotosensor = A0; // /!\ Nécessite entrée analogique
int pinHallSensor = 2;
int pinFrein = 13;
```

* Choisir les temps nécessaire à la détection de l'arrêt et du redémarrage (en millisecondes):
```C++
int tempsFreinValue = 2000; // Si aucun aimant n'a été détecté depuis 2 sec, le SESA considère le vélo à l'arrêt
int tempsDepartValue = 1000;
```

* Choisir le temps de clignotement des DELS (en millisecondes) :
```C++
int blinkValue = 330;
```

* Choisir la valeur de la temporisation au déclenchement lors de la lecture des données provenant de l'accéléromètre (à multiplier par 10 millisecondes): 
```C++
int tempoValue = 20;
```

***

## Changelog

* **[1.0.4]** : réorganisation des classes et ajout description configuration.

* **[1.0.3]** : Ajout parti configuration et correction bugs mineurs.

* **[1.0.2]** : Ajout des commentaires.

* **[1.0.0]** : Nouvelle version. Intégration de l'ensemble des composants.

* **[0.5.0]** : création de la classe axe pour a gestion de données provenant de l'accléromètre (acquisition, lissage, temporisation).

* **[0.4.0]** : correction stabilité.

* **[0.3.0]** : premiers essais intégration accéléromètre.

* **[0.2.0]** : ajout de l'ensemble des fichiers.

* **[0.1.0]** : création du dépôt.
