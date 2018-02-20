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
  Serial.println(moyenne);
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
boolean serialDebug = true;
boolean stop = false;
boolean stopVitesse = false;
boolean vitesse0;
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

lampe ledRouge(11);
lampe ledBlanc(10);
binaryLampe ledStop(9);
binaryLampe ledLat(8);
analogSensor photoSensor(A0);
digitalSensor aimantVitesse(2);
digitalSensor frein(13);
timer tempsVitesse(timerValue); //Temps defini pour la mesure de vitesse
timer tempsFrein(2000); // Minuteur permettant de dire que le vélo est à l'arrêt si aucun aimant n'est passé devant le capeteur pendant plus de 2 secondes
timer blink(330); // Minuteur pour le clignotement des lumières
timer tempsAccelero(10);
axe axeX(1400, 1600, 15);
axe axeY(1400, 1600, 15);
axe axeZ(800, 1000, 15);

void setup()
{
	//AcceleroSetup.begin
	  Wire.begin();
	  Wire.beginTransmission(MPU_addr);
	  Wire.write(0x6B);  // PWR_MGMT_1 register
	  Wire.write(0);     // set to zero (wakes up the MPU-6050)
	  Wire.endTransmission(true);
	//AcceleroSetup.end
	ledRouge.setUp();
	ledBlanc.setUp();
	ledStop.setUp();
	ledLat.setUp();
	photoSensor.setUp(715, 745); 
	aimantVitesse.setUp(2);
	frein.setUp(1);
	Serial.begin(9600);
	Serial.println("Fin du setUp !");
	distanceRoue = diametreRoue * PI / 100;
	blink.init();
	vitesse0 = false;
	blinkOn = false;
}

void loop()
{
	stop = false;
	tempsVitesse.init();
	if (stopVitesse == true)
	{
		tempsFrein.init();
	}
	stopVitesse = false;
	aimantCounter = 0;
	while(tempsVitesse.timeIsUp() == 0 && !stop)
	{
		//AcceleroRead.begin
		if (tempsAccelero.timeIsUp() == 1)
		{
		  Wire.beginTransmission(MPU_addr);
		  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
		  Wire.endTransmission(false);
		  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
		  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
		  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
		  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
		  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
		  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
		  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
		  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
		}
		//AcceleroRead.end
		axeX.setValue(AcX);
		axeY.setValue(AcY);
		axeZ.setValue(AcZ);

		aimantVitesse.setPreviousState();
		photoSensor.setPreviousState();
		frein.setPreviousState();
		delay(5);
		savedState = aimantVitesse.getState();
		savedValue = photoSensor.getState();
		savedStateFrein = frein.getState();
		if (tempsFrein.timeIsUp() == 1) //|| axeX.getState() == 0 || axeY.getState() == 0 || axeZ.getState() == 0)
		{
			Serial.println("Le velo est a l'ARRET !");
			stop = true;
			currentDistance = 0;
			currentVitesse = 0;
			stopVitesse = true;
			aimantCounter = 0;
		}
		if (aimantVitesse.stateHasRising() == 1)
		{
			aimantCounter++;
			Serial.println("Aimant detecte");
			tempsFrein.init();
		}
		if (photoSensor.stateHasChanged() == 1)
		{
			stop = true;
		}
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
					ledRouge.setInstensity(3);
				}
				else
				{
					ledRouge.setInstensity(3);
				}
			}
		}
		if (blink.timeIsUp() == 1)
		{
			blink.init();
			blinkOn = !blinkOn;
			if (blinkOn)
			{
				if (vitesse0)
				{
					ledStop.switchOn();	// 
				}
				ledLat.switchOn(); //Clignotement des éclairages latéraux
			}
			else
			{
				ledStop.switchOff();
				ledLat.switchOff(); //Clignotement des éclairages latéraux
			}
		}
	}
	if (stop == true)
	{
		Serial.println("Boucle vitesse arretee. Changement de l'etat des lumieres...");
	}
	else
	{
		Serial.print("La roue a tourne : ");
		Serial.print(aimantCounter);
		Serial.println(" fois.");
		currentDistance = distanceRoue * aimantCounter; // Calcul de la distance parcourue durant les 5 dernières secondes
  		currentVitesse = (currentDistance * 10000) / 3600; // Calcul de la vitesse moyenne durant les 5 dernières secondes
	}
	Serial.print("La vitesse est de ");
	Serial.print(currentVitesse);
	Serial.println(" km/h.");
	Serial.print("Il fait ");
	if (photoSensor.getState() == 1)
	{
		Serial.print("JOUR");
		ledBlanc.setInstensity(8); //Affectation du mode veilleuse à l'éclairage avant
	}
	else if (photoSensor.getState() == 0 && currentVitesse < 15)
	{
		Serial.print("NUIT");
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
	if (aimantCounter == 0)
	{
		vitesse0 = true;
	}
	else
	{
		vitesse0 = false;
	}
	Serial.println("");
}
