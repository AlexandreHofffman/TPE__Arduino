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

conserv::conserv(int aNb)
{
  nb = aNb;
}

void conserv::setLastState(boolean state)
{
  if (state)
  {
    count = nb;
  }
}

int conserv::getState()
{
  if (count > 0)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void conserv::refresh()
{
   if (count > 0)
  {
    count--;
  }
}

hysteresis hautX(19000, 19500);
hysteresis basX(14200, 14700);
hysteresis hautY(1500, 2000);
hysteresis basY(-2000, -1500);
hysteresis hautZ(2500, 2800);
hysteresis basZ(-2800, 2500);
conserv axeX(25);
conserv axeY(25);
conserv axeZ(25);

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
  /*Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);*/
  Serial.println(AcZ);
  hautX.setValue(AcX);
  basX.setValue(AcX);
  hautY.setValue(AcY);
  basY.setValue(AcY);
  hautZ.setValue(AcZ);
  basZ.setValue(AcZ);
  axeX.refresh();
  axeY.refresh();
  axeZ.refresh();
 
  if (hautY.getState() == 1 || basY.getState() == 0)
  {
  	axeY.setLastState(true);
  }
  else
  {
  	axeY.setLastState(false);
  }
  if (hautX.getState() == 1 || basX.getState() == 0)
  {
    axeX.setLastState(true);
  }
  else
  {
    axeX.setLastState(false);
  }
  if (hautZ.getState() == 1 || basZ.getState() == 0)
  {
    axeZ.setLastState(true);
  }
  else
  {
    axeZ.setLastState(false);
  }

  if (axeY.getState() == 1)
  {
    digitalWrite(4, HIGH);
  }
  else
  {
    digitalWrite(4, LOW);
  }
  if (axeX.getState() == 1)
  {
    digitalWrite(3, HIGH);
  }
  else
  {
    digitalWrite(3, LOW);
  }
  if (axeZ.getState() == 1)
  {
    digitalWrite(5, HIGH);
  }
  else
  {
    digitalWrite(5, LOW);
  }
  delay(20);
}
