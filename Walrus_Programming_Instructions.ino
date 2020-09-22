
//#include <NewPing.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <splash.h>

/*
   "Bump-N-Go" By LeRoy Miller Dec 2017
   for Keyes L-298P Shield.

   Uses the switches to detect bumps, moves the robot backwards
   and turns in the robot left or right, based on switch pressed
   starts the robot moving forward again.

   If your Motors move in the wrong direction, switch the wires
   for that motor.
   Program waits for the RIGHT switch to be pressed before
   moving the motors. Once the "Dance" is done, the program
   waits again for the RIGHT switch to be pressed, and starts
   over again.
*/

#define OLED_RESET 15
Adafruit_SSD1306 display(OLED_RESET);
#define SSD1306_LCDHEIGHT 64
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

#define RIGHTSWITCH 5 //D5 Blue Connector Marked "G"
#define LEFTSWITCH 6 //D6 Blue Connector Marked "B"

#define ECHOPIN 7
#define TRIGPIN 8

#define BUZZER 9 //D4

#define M1SPD 10 //D10 PWM M1 speed pin
#define M2SPD 11 //D11 PWM M2 speed pin
#define M2DIR 13 //D13 PWM M2 direction
#define M1DIR 12 //D12 M1 direction

long duration;
int distance;
int lowspd = 100; //PWM for stall of motors
//changing this speed will change how fast or slow your robot goes
//This is PWM the motors will run at.
int setspd = 150; 
int printIncrement = 0;

//constructor for the lcd screen
LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
  //printing starting message that will continue to appear until a switch is hit
  //do not change any of this besides the message
  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  lcd.init();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(1, 0);
  //change the "hello everyone" and "konichiwaa" to a different message if you would like
  lcd.print("hello everyone");
  lcd.setCursor(1, 1);
  lcd.print("konichiwaa");
  delay(5000);

  //allows you to run your serial monitor to see if your sensor is working correctly
  Serial.begin(9600);
  Serial.println("Starting Up...");
  pinMode(LEFTSWITCH, INPUT_PULLUP);
  pinMode(RIGHTSWITCH, INPUT_PULLUP);
  pinMode(M1SPD, OUTPUT);
  pinMode(M2SPD, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  stop();

  //to start the robot the right switch needs to be hit
  //the robot will then dance and move as normal
  while (digitalRead(RIGHTSWITCH) == HIGH) {
    delay(1);
    //Waiting for RIGHTSWITCH Button to be pushed to continue
  }
}

void loop() {
  delay(100);
  //getting and printing the distance to an object in the serial monitor
  float ultraSonicDistanceCm = getUltrasonicDistanceInCm();
  Serial.print("Distance (cm): ");
  Serial.println(ultraSonicDistanceCm);
  setSpd(setspd, setspd); //set the PWM for motor1 and motor2
  //can be used to set different speeds and cause turns
  //this is not a direction control.
  Forward();
  if (digitalRead(LEFTSWITCH) == LOW || digitalRead(RIGHTSWITCH) == LOW) {
    int left = digitalRead(LEFTSWITCH);
    int right = digitalRead(RIGHTSWITCH);
    for (int i = 0; i < 2; i++) {
      delay(100);
    }
    Backward();
    delay(1000);
    if (left == 0) {
      Right();
      delay(500);
    }
    if (right == 0) {
      Left();
      delay(500);
    }
    //can change this message to say whatever you would like after one of the switches is hit
    if (digitalRead(RIGHTSWITCH) == HIGH || digitalRead(LEFTSWITCH) == HIGH) {
      lcd.clear();
      lcd.print("GET OUT THE WAY");
    }
    if (digitalRead(RIGHTSWITCH) == HIGH && digitalRead(LEFTSWITCH) == HIGH) {
      stop();
    }

    if(distance < 1000 && digitalRead(RIGHTSWITCH) == LOW && digitalRead(LEFTSWITCH) == LOW){
      Backward();
      Right();
  
    }
      
  }

  Forward();

  //start over

}

void stop() {
  analogWrite(M1SPD, 0);
  digitalWrite(M1DIR, LOW);
  analogWrite(M2SPD, 0);
  digitalWrite(M2DIR, LOW);
}

void setSpd(int m1, int m2) {
  if (m1 < lowspd) {
    m1 = lowspd;
  }
  if (m2 < lowspd) {
    m2 = lowspd;
  }
  if (m1 > 255) {
    m1 = 255;
  }
  if (m2 > 255) {
    m2 = 255;
  }
  analogWrite(M1SPD, m1);
  analogWrite(M2SPD, m2);
}
void Forward() {
  digitalWrite(M1DIR, LOW);
  digitalWrite(M2DIR, LOW);
}

void Backward() {
  digitalWrite(M1DIR, HIGH);
  digitalWrite(M2DIR, HIGH);
}

void Left() {
  digitalWrite(M1DIR, HIGH);
  digitalWrite(M2DIR, LOW);
}

void Right() {
  digitalWrite(M1DIR, LOW);
  digitalWrite(M2DIR, HIGH);
}

void playCharge() {
  int note[] = {262, 262, 392, 523, 392, 523};
  int duration[] = {100, 100, 100, 300, 100, 300};
  for (int thisNote = 0; thisNote < 6; thisNote ++) {
    // play the next note:
    tone(BUZZER, note[thisNote]);
    // hold the note:
    delay(duration[thisNote]);
    // stop for the next note:
    noTone(BUZZER);
    delay(25);
  }
}
float getUltrasonicDistanceInCm () {
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);
  pinMode(ECHOPIN, INPUT);
  
  float duration = pulseIn(ECHOPIN, HIGH);
  float cm = (duration / 2) / 29.1;


  return cm;
}
