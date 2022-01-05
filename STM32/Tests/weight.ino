#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 7, 6, 5, 4, 3);
 
#define DT A0
#define SCK A1
#define sw 2
 
long sample=0;
float val=0;
long count=0;
int c=0;
 
unsigned long readCount(void)
{
unsigned long Count = 0;
unsigned char i;
pinMode(DT, OUTPUT);
digitalWrite(DT,HIGH);
digitalWrite(SCK,LOW);

pinMode(DT, INPUT);
while(digitalRead(DT));
for (i=0;i<24;i++)
{
digitalWrite(SCK,HIGH);
Count=Count<<1;
digitalWrite(SCK,LOW);
if(digitalRead(DT))
Count++;
}
digitalWrite(SCK,HIGH);
Count=Count^0x800000;
digitalWrite(SCK,LOW);
return(Count);
}
 
void setup()
{
  Serial.begin(115200);
  Serial.println("VA-U Thrust Setup");
pinMode(SCK, OUTPUT);
pinMode(sw, INPUT_PULLUP);
lcd.begin(16, 2);
lcd.print(" Weight ");
lcd.setCursor(0,1);
lcd.print(" Measurement ");
delay(1000);
lcd.clear();
//calibrate();
val = -106.39;//-107.58;
sample = 8552510;//8547787;
}
 
void loop()
{
count= readCount();
int w=(((count-sample)/val)-2*((count-sample)/val));
lcd.setCursor(0,0);
lcd.print("Measured Weight");
lcd.setCursor(0,1);
//lcd.print(w);
//char* W = itoa(w-c);
if (w-c >= 0)
{
  Serial.println(w-c);
}
else
{
  Serial.println(0);
}
//lcd.print(w-c,"g ");
 
if(digitalRead(sw)==0)
{
//val=0;
//sample=0;
c = w;
//count=0;
//calibrate();
}
}
 
void calibrate()
{
lcd.clear();
lcd.print("Calibrating...");
delay(500);
lcd.setCursor(0,1);

lcd.print("Please Wait...");
delay(2000);
for(int i=0;i<100;i++)
{
count=readCount();
sample+=count;
}

sample/=100;
lcd.clear();
lcd.print("Put 200g & wait");
//delay(2000);
count=0;
while(count<100)
{
count=readCount();
count=sample-count;
}
lcd.clear();
lcd.print("Please Wait....");
delay(2000);
for(int i=0;i<100;i++)
{
count=readCount();
val+=sample-count;
}
val=val/100.0;
val=val/121.0; // put here your calibrating weight
lcd.clear();
lcd.print(val);
lcd.setCursor(0,1);
lcd.print(sample);
delay(2000);
lcd.clear();
}
