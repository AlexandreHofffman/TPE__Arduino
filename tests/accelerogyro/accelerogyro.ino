class hysteresis
{
	private:
		int savedValue;
		boolean currentState;
		int seuilHaut;
		int seuilBas;
	public:
		hysteresis(int aSeuilBas,  int aSeuilHaut);
		void setValue(int value);
		int getState();
};

hysteresis::hysteresis(int aSeuilBas,  int aSeuilHaut)
{
	seuilHaut = aSeuilHaut;
  seuilBas = aSeuilBas;
  currentState = false;
}

void hysteresis::setValue(int value)
{
	savedValue = value;
}

int hysteresis::getState()
{
	if (savedValue > seuilHaut && !currentState)
	{
		currentState = true;
	}
	if (savedValue < seuilBas && currentState)
	{
		currentState = false;
	}

	return int(currentState);
}

class conserv
{
  private:
    unsigned int count;
    unsigned int nb;
  public:
    conserv(int aNb);
    void setLastState(boolean state);  
    int getState();
    void refresh();
};

/*
Entrée :
  entre les valeurs VRAI ou FAUX après hysteresis (haut et bas)
Traitement :
  enregistrer la différence pécédente
  sauvegarder l'état précédent
  lire la nouvelle valeur
  faire la différence
  faire une moyenne sur les deux dernieres entrées
  SI la moyenne est en dehors des bornes prévues
  ALORS passer à l'état vrai
  SINON état faux
  conserver cet état sur 20 boucles (20 * 5 millisecondes => 100 millisecondes)
Sortie:
  état conservé VRAI ou FAUX
*/
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


axe axeX(1400, 1600, 20);
axe axeY(1400, 1600, 20);
axe axeZ(800, 1000, 20);

#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
}
void loop(){
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
  axeX.setValue(AcX);
  axeY.setValue(AcY);
  axeZ.setValue(AcZ);

  /*Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.println(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);*/
  

  if (axeX.getState() == 1)
  {
    digitalWrite(3, HIGH);
  }
  else
  {
    digitalWrite(3, LOW);
  }
  if (axeY.getState() == 1)
  {
    digitalWrite(4, HIGH);
  }
  else
  {
    digitalWrite(4, LOW);
  }
  if (axeZ.getState() == 1)
  {
    digitalWrite(5, HIGH);
  }
  else
  {
    digitalWrite(5, LOW);
  }

  delay(50);
  
}
