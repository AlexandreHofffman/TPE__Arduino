byte val;

void setup()
{
	pinMode(3, OUTPUT);
}

void loop()
	
{
	val = analogRead(A0) / 4;
	analogWrite(3, val);
	delay(50);
}
