#include <LiquidCrystal.h>
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
void setup(){ lcd.begin(16, 2);
  Serial.begin(9600); // open serial port, set the baud rate to 9600 bps
}
void loop(){int val;
      val=analogRead(0);   //connect mic sensor to Analog 0
      Serial.print("Sound=");
      Serial.println(val,DEC);//print the sound value to serial        
      lcd.setCursor(0, 0);
lcd.print("Sound=");
lcd.print(val);
lcd.print("   ");
delay(1);}
