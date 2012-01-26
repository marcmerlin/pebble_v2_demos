// Example use of LCD3Wire library
// Almost a carbon-copy of LCD4BitExample.pde

#include <LCD3Wire.h> 

const int PIN_LCD_STROBE         =  2;  // Out: LCD IC4094 shift-register strobe
const int PIN_LCD_DATA           =  3;  // Out: LCD IC4094 shift-register data
const int PIN_LCD_CLOCK          =  4;  // Out: LCD IC4094 shift-register clock
const int PIN_LCD_BACKLIGHT      =  5;  // Out: LCD backlight (PWM)

// Arduino pins
#define LCD_LINES 2  // number of lines in your display

#define DOUT_PIN  PIN_LCD_DATA  // Dout pin
#define STR_PIN   PIN_LCD_STROBE  // Strobe pin
#define CLK_PIN   PIN_LCD_CLOCK  // Clock pin

#define LED_PIN   13 // we'll use the debug LED to output a heartbeat

//create object to control an LCD.  
LCD3Wire lcd = LCD3Wire(LCD_LINES, DOUT_PIN, STR_PIN, CLK_PIN); 

//some messages to display on the LCD
char msgs[6][15] = {"apple", "banana", "pineapple", "mango", "watermelon", "pear"};
int NUM_MSGS = 6;

void setup() { 
  lcd.init();
  
  //optionally, now set up our application-specific display settings, overriding whatever the lcd did in lcd.init()
  //lcd.commandWrite(0x0F);//cursor on, display on, blink on.  (nasty!)
  
  pinMode(LED_PIN, OUTPUT);  
}

void loop() {  
  digitalWrite(LED_PIN, HIGH);  //light the debug LED
  pinMode(PIN_LCD_BACKLIGHT, OUTPUT);
  analogWrite(PIN_LCD_BACKLIGHT, 80);

  //pick a random message from the array
  int pick = random(NUM_MSGS);
  char* msg = msgs[pick];
  
  lcd.clear();
  lcd.print(msg);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  
  //print some dots individually
  for (int i=0; i<3; i++){
    lcd.print('.');
    delay(100);
  }
  //print something on the display's second line. 
  if(LCD_LINES>1){
    lcd.cursorTo(2, 0);  //line=2, x=0.
    lcd.print("Score: 6/7");
    delay(1000);
  }
  
  //scroll entire display 20 chars to left, delaying 50ms each inc
  lcd.leftScroll(20, 50);
}


