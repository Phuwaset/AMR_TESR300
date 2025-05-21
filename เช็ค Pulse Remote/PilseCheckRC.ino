#define CH2 13
#define CH3 14
#define CH4 15
#define CH7 17

void setup() 
{
  Serial.begin(115200);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH7, INPUT);
}
void loop() 
{
  unsigned long ch2Pulse = pulseIn(CH2, HIGH, 50000);
  unsigned long ch3Pulse = pulseIn(CH3, HIGH, 50000);
  unsigned long ch4Pulse = pulseIn(CH4, HIGH, 50000);
  unsigned long ch7Pulse = pulseIn(CH7, HIGH, 50000);

  Serial.print("  CH2: ");
  Serial.print(ch2Pulse);
  Serial.print("  CH3: ");
  Serial.print(ch3Pulse);
  Serial.print("  CH4: ");
  Serial.print(ch4Pulse);
  Serial.print("  CH7: ");
  Serial.println(ch7Pulse);

  delay(50);
}
