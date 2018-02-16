byte val;
const byte pin = 3;
const boolean mode_fade = true;
int i = 0;

void setup()
{
	pinMode(pin, OUTPUT);
}

void loop()
{
	if (!mode_fade)
	{
		val = analogRead(A0) / 4;
		analogWrite(pin, val);
		delay(50);
	}
	else
	{
		while(i < 255)
		{
			i++;
			analogWrite(pin, i);
			delay(5);
		}
		delay(2000);
		while(i > 0)
		{
			i--;
			analogWrite(pin, i);
			delay(5);
		}
		delay(2000);
	}
}
	