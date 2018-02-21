/*-------------------------INFOS-------------------------*\
	AUTEUR : Tahitoa L
	PROJET : prgm de commande systeme eclairage SESA
	VERSION : 1.0.2
\*-------------------------------------------------------*/

//config

const boolean accelero = true;

//config.end

class analogSensor
{
	private:
		byte pin;
		int currentValue;
		int savedValue;
		int currentState;
		int previousState;
		int seuilHaut;
		int seuilBas;
		boolean hasChanged;
		boolean hasRising;
	public:
		analogSensor(byte aPin);
		void setUp(int aSeuilBas,  int aSeuilHaut);
		int getValue();
		int getState();
		void setPreviousState();
		int stateHasChanged();
		int stateHasRising();
};

class digitalSensor
{
	private:
		byte pin;
		int modeInverse;
		int previousState;
		int currentState;
		boolean hasRising;
		boolean hasChanged;
	public:
		digitalSensor(byte aPin);
		void setUp(int aModeInverse);
		int getState();
		void setPreviousState();
		int stateHasChanged();
		int stateHasRising();
};

class lampe
{
	private:
		byte pin;
		byte value;
		void refresh();
	public:
		lampe(byte aPin);
		void setUp();
		void switchOn();
		void switchOff();
		void setInstensity(int intensityRequested);
};

class binaryLampe
{
	private:
		byte pin;
		byte state;
	public:
		binaryLampe(byte aPin);
		void setUp();
		void switchOn();
		void switchOff();
		int getState();
};

class timer
{
	private:
		unsigned int delayValue;
		unsigned int delayCurrent;
		unsigned long initTime;
		unsigned long currentMillis;
		boolean end;
	public:
		timer(int aDelayValue);
		void init();
		int timeIsUp();
		int getDelay();
};

class hysteresis
{
	private:
		int savedValue;
		boolean currentState;
		int seuilHaut;
		int seuilBas;
	public:
		hysteresis();
		void setUp(int aSeuilBas,  int aSeuilHaut);
		void setValue(int value);
		int getState();
};



analogSensor::analogSensor(byte aPin)
{
	pin = aPin;
}

void analogSensor::setUp(int aSeuilBas, int aSeuilHaut)
{
	seuilHaut = aSeuilHaut;
	seuilBas = aSeuilBas;
	currentState = 0;
}

int analogSensor::getValue()
{
	currentValue = analogRead(pin);
	return currentValue;
}

int analogSensor::getState()
{
	savedValue = getValue();
	if(currentValue <= (seuilBas) && currentState != 0)
	{
		currentState = 0;
	}
	else if(currentValue > (seuilHaut) && currentState != 1)
	{
		currentState = 1;
	}
	return currentState;
}

void analogSensor::setPreviousState()
{
	previousState = currentState;
}

int analogSensor::stateHasChanged()
{
	if (previousState == currentState)
	{
		hasChanged = false;
		return 0;
	}
	else
	{
		hasChanged = true;
		return 1;
	}
}

int analogSensor::stateHasRising()
{
	if (previousState < currentState)
	{
		hasRising = true;
		return 1;
	}
	else
	{
		hasRising = false;
		return 0;
	}
}

digitalSensor::digitalSensor(byte aPin)
{
	pin = aPin;
}

void digitalSensor::setUp(int aModeInverse)
{
	if (aModeInverse == 1)
	{
		modeInverse = 1;
	}
	else
	{
		modeInverse = 0;
	}
	pinMode(pin, INPUT_PULLUP);
}

int digitalSensor::getState()
{
	if (digitalRead(pin) == HIGH)
	{
		currentState = 1;
	}
	else
	{
		currentState = 0;
	}
	if (modeInverse == 1)
	{
		if (currentState == 0)
		{
			currentState = 1;
		}
		else
		{
			currentState = 0;
		}
	}
	if (currentState == 1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void digitalSensor::setPreviousState()
{
	previousState = currentState;
}

int digitalSensor::stateHasChanged()
{
	if (previousState == currentState)
	{
		hasChanged = false;
		return 0;
	}
	else
	{
		hasChanged = true;
		return 1;
	}
}

int digitalSensor::stateHasRising()
{
	if (previousState == 0 && currentState == 1)
	{
		hasRising = true;
		return 1;
	}
	else
	{
		hasRising = false;
		return 0;
	}
}


lampe::lampe(byte aPin)
{
	pin = aPin;
}

void lampe::setUp()
{
	pinMode(pin, OUTPUT);
	switchOff();
}

void lampe::switchOn()
{
	//digitalWrite(pin, HIGH);
	value = 255;
	refresh();
}

void lampe::switchOff()
{
	//digitalWrite(pin, LOW);
	value = 0;
	refresh();
}

void lampe::setInstensity(int intensityRequested)
{
	if (intensityRequested <= 1023 && intensityRequested >= 0)
	{
		value = intensityRequested;
		refresh();
	}
}

void lampe::refresh()
{
	byte valueDemande = value;
	analogWrite(pin, value);
}

binaryLampe::binaryLampe(byte aPin)
{
	pin = aPin;
}

void binaryLampe::setUp()
{
	pinMode(pin, OUTPUT);
	switchOff();
}

void binaryLampe::switchOn()
{
	digitalWrite(pin, HIGH);
	state = 1;
}

void binaryLampe::switchOff()
{
	digitalWrite(pin, LOW);
	state = 0;
}

int binaryLampe::getState()
{
	return int(state);
}

timer::timer(int aDelayValue)
{
	delayValue = aDelayValue;
}

void timer::init()
{
	end = false;
	initTime = millis();
}

int timer::timeIsUp()
{
	currentMillis = millis();
	if(currentMillis - initTime >= delayValue)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int timer::getDelay()
{
	delayCurrent = millis() - initTime;
	return delayCurrent;
}

class axe
{
  private:
    int lastValue;
    int currentValue;
    int lastDiff;
    int currentDiff;
    int moyenne;
    int conservTimes;
    int conservCounter;
    int seuilHaut;
    int seuilBas;
    boolean currentState;
  public:
    axe(int aSeuilBas, int aSeuilHaut, int aConservTimes);
    void setValue(int value);
    int getState(); // + refresh
};

axe::axe(int aSeuilBas, int aSeuilHaut, int aConservTimes)
{
  seuilBas = aSeuilBas;
  seuilHaut = aSeuilHaut;
  conservTimes = aConservTimes;
  lastValue = 0;
  currentValue = 0;
  lastDiff = 0;
  currentDiff = 0;
  moyenne = 0;
  conservCounter = 0;
  currentState = false;
}

void axe::setValue(int value)
{
  lastValue = currentValue;
  currentValue = value;
  lastDiff = currentDiff;
  currentDiff = currentValue - lastValue;
  moyenne = abs((lastDiff + currentDiff) / 2);
}

int axe::getState()
{
  if (moyenne > seuilHaut && !currentState)
  {
    currentState = true;
  }
  else if (moyenne < seuilBas && currentState)
  {
    currentState = false;
  }

  if (currentState)
  {
    conservCounter = conservTimes;
  }
  if (conservCounter > 0)
  {
    conservCounter--;
    return 1;
  }
  else
  {
    return 0;
  }
}

#include <Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

const unsigned int timerValue = 5000;
const float diametreRoue = 31.85;
const int tauxDeRafraichissement = 5;
boolean serialDebug = true;
boolean stop = false;
boolean stopVitesse = false;
boolean vitesse0;
boolean mouvement = false;
int testState;
int aimantCounter;
int savedState;
int savedValue;
int savedStateFrein;
int savedDelay;
float currentDistance;
float currentVitesse;
float distanceRoue;
boolean blinkOn = false;

const boolean serial = false;


// Création des objets
lampe ledRouge(11);
lampe ledBlanc(10);
binaryLampe ledStop(9);
binaryLampe ledLat(8);
analogSensor photoSensor(A0);
digitalSensor aimantVitesse(2);
digitalSensor frein(13);
timer tempsVitesse(timerValue); //Temps defini pour la mesure de vitesse
timer tempsFrein(2000); // Minuteur permettant de dire que le vélo est à l'arrêt si aucun aimant n'est passé devant le capeteur pendant plus de 2 secondes
timer tempsDepart(1000);
timer blink(330); // Minuteur pour le clignotement des lumières
timer tempsAccelero(10);
axe axeX(1400, 1600, 20);
axe axeY(1400, 1600, 20);
axe axeZ(800, 1000, 20);


void setup()
{
	ledRouge.setUp();
	ledBlanc.setUp();
	ledStop.setUp();
	ledLat.setUp();
	photoSensor.setUp(715, 745);
	aimantVitesse.setUp(2);
	frein.setUp(1);
	distanceRoue = diametreRoue * PI / 100;
	blink.init();
	tempsAccelero.init();
	vitesse0 = false;
	blinkOn = false;
	if (accelero)
	{
		Wire.begin();
	  Wire.beginTransmission(MPU_addr);
	  Wire.write(0x6B);  // PWR_MGMT_1 register
	  Wire.write(0);     // set to zero (wakes up the MPU-6050)
	  Wire.endTransmission(true);
	}
	if (serial)
	{
		Serial.begin(9600);
		Serial.println("Fin du setUp !");
	}
	Serial.begin(9600);
}

void loop()
{
	stop = false;
	tempsVitesse.init();
	if (stopVitesse == true) // Permet d'éviter une boucle sans délai lorsque le vélo est à l'arrêt
	{
		tempsFrein.init();
	}
	stopVitesse = false;
	aimantCounter = 0;

	// ComptageVitesse.begin
	while(tempsVitesse.timeIsUp() == 0 && !stop)
	{
		// Lecture des données provenant de l'accéléromètre.begin
		if (accelero && tempsAccelero.timeIsUp() == 1)
		{
			Wire.beginTransmission(MPU_addr);
		  Wire.write(0x3B);  // commence à l'inscription 0x3B (ACCEL_XOUT_H)
		  Wire.endTransmission(false);
		  Wire.requestFrom(MPU_addr,14,true);  // demander un total de 14 inscriptions
		  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
		  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
		  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
		  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
		  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
		  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
		  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
			// Prétraitement des données
			axeX.setValue(AcX);
		  axeY.setValue(AcY);
		  axeZ.setValue(AcZ);
			if (axeX.getState() == 1 || axeY.getState() == 1 || axeZ.getState() == 1) // Si un mouvement détecté sur l'un des trois axe
			{
				mouvement = true;
				Serial.println("Mouvement detecte");
			}
			else
			{
				mouvement = false;
				Serial.println("AUCUN mvt detecte");
			}
		}
		// Lecture des données provenant de l'accléromètre.end

		// Sauvegarde des données pour détecter les fronts
		aimantVitesse.setPreviousState();
		photoSensor.setPreviousState();
		frein.setPreviousState();
		delay(tauxDeRafraichissement);
		savedState = aimantVitesse.getState();
		savedValue = photoSensor.getState();
		savedStateFrein = frein.getState();
		// Vélo à l'arrêt.begin
		if (tempsFrein.timeIsUp() == 1 || mouvement == false)
		{
			if (serial)
			{
				Serial.println("Le velo est a l'ARRET !");
			}
			stop = true;
			currentDistance = 0;
			currentVitesse = 0;
			stopVitesse = true;
			aimantCounter = 0;
		}
		// Vélo à l'arrêt.end

		// Vélo redémarre.begin
		if (vitesse0)
		{
			if (tempsDepart.timeIsUp() == 1)
			{
				tempsDepart.init();
				if (aimantCounter > 0 || mouvement == true)
				{
					stop = true;
					aimantCounter = 1; // Pour ne pas repasser dans l'etat v0 si le vel est en mvt mais que l'aimant n'a pas encore ete detcete
				}
			}
		}
		// Vélo redémarre.end

		// Détection front montant aimant.begin
		if (aimantVitesse.stateHasRising() == 1)
		{
			aimantCounter++;
			if (serial)
			{
				Serial.println("Aimant detecte");
			}
			tempsFrein.init();
		}
		// Détection front montant aimant.end

		// Détection front photorésistance.begin
		if (photoSensor.stateHasChanged() == 1)
		{
			stop = true;
		}
		// Détection front photorésistance.end

		// Détection front capteur frein.begin (sans sortir de la boucle de comptage de vitesse)
		if (frein.stateHasChanged() == 1)
		{
			if (frein.getState() == 1)
			{
			ledRouge.switchOn();
			}
			else
			{
				if (photoSensor.getState() == 0)
				{
					ledRouge.setInstensity(30);
				}
				else
				{
					ledRouge.setInstensity(3);
				}
			}
		}
		// Détection front capteur frein.end

		//Clignotement.begin
		if (blink.timeIsUp() == 1)
		{
			blink.init();
			blinkOn = !blinkOn;
			if (blinkOn)
			{
				if (vitesse0)
				{
					ledStop.switchOn();	// Clignotement feux de détresse
				}
				ledLat.switchOn(); // Clignotement des éclairages latéraux
			}
			else
			{
				ledStop.switchOff(); // Clignotement feux de détresse
				ledLat.switchOff(); // Clignotement des éclairages latéraux
			}
		}
		//Clignotement.end
	}
	// ComptageVitesse.end


	if (stop == true) // Si le comptage de la vitesse a été interrompu (-> on ne peut donc pas déterminer vitesse du vélo)
	{
		if (serial)
		{
			Serial.println("Boucle vitesse arretee. Changement de l'etat des lumieres...");
		}
	}
	else // Détermination de la vitesse du vélo
	{
		if (serial)
		{
			Serial.print("La roue a tourne : ");
			Serial.print(aimantCounter);
			Serial.println(" fois.");
		}
		currentDistance = distanceRoue * aimantCounter; // Calcul de la distance parcourue durant les 5 dernières secondes
  		currentVitesse = (currentDistance * 10000) / 3600; // Calcul de la vitesse moyenne durant les 5 dernières secondes
	}
	if (serial)
	{
		Serial.print("La vitesse est de ");
		Serial.print(currentVitesse);
		Serial.println(" km/h.");
		Serial.print("Il fait ");
	}

	// Modification fonctionnement éclairage.begin
	if (photoSensor.getState() == 1)
	{
		if (serial)
		{
			Serial.print("JOUR");
		}
		ledBlanc.setInstensity(8); //Affectation du mode veilleuse à l'éclairage avant
	}
	else if (photoSensor.getState() == 0 && currentVitesse < 15)
	{
		if (serial)
		{
			Serial.print("NUIT");
		}
		ledBlanc.setInstensity(100); // Affectation du mode feux position à l'éclairage avant
	}
	else if (photoSensor.getState() == 0)
	{
		ledBlanc.switchOn(); // Affectation du mode feux de route à l'éclairage avant
	}
	if (photoSensor.getState() == 1)
	{
		if (frein.getState() == 0)
		{
			ledRouge.setInstensity(3); // Affectation du mode veilleuse à l'éclairage arrière
		}
	}
	else
	{
		if (frein.getState() == 0)
		{
			ledRouge.setInstensity(30); // Affectation du mode feux position à l'éclairage arrière
		}
	}
	// Modification fonctionnement éclairage.end


	if (aimantCounter == 0) // Si aucun aimant n'a été détecté durant la boucle, le vélo est à l'arrêt
	{
		vitesse0 = true;
	}
	else
	{
		vitesse0 = false;
	}
	if (serial)
	{
		Serial.println("");
	}
}
