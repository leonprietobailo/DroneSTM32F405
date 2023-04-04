#define pin LED_BUILTIN
TIM_TypeDef *Instance = TIM2;
HardwareTimer *MyTim = new HardwareTimer(Instance);
int potenciometro;
int tempo=500000;

void setup()
{
  pinMode(pin, OUTPUT);

  MyTim->setMode(2, TIMER_OUTPUT_COMPARE); 
  MyTim->setOverflow(tempo, MICROSEC_FORMAT);
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();
}

void loop()
{
  //potenciometro = analogRead(PA1);
  //tempo=map(potenciometro,10,1023,100000,100);
  //MyTim->setOverflow(tempo, MICROSEC_FORMAT);
}

void Update_IT_callback()
{ 
  digitalWrite(pin, !digitalRead(pin));
}
