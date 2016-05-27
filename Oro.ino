#include "Servo.h"
#include "Wire.h"
#define DS1307_ADDRESS 0x68
byte zero = 0x00; //workaround for issue #527

int One = 21;
int Two = 20;
int Three = 19;
int Four = 18;
int Five = 17;
int Six = 16;
int Seven = 15;
int Eight = 14;
int Nine = 13;
int Ten = 12;
int Eleven = 11;
int Twelve = 10;

Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
void setup(){
 Wire.begin();
 Serial.begin(9600);
 setDateTime(); //MUST CONFIGURE IN FUNCTION
 myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
 
 /*
 pinMode(One, OUTPUT);
 pinMode(Two, OUTPUT);
 pinMode(Three, OUTPUT);
 pinMode(Four, OUTPUT);
 pinMode(Five, OUTPUT);
 pinMode(Six, OUTPUT);
 pinMode(Seven, OUTPUT);
 pinMode(Eight, OUTPUT);
 pinMode(Nine, OUTPUT);
 pinMode(Ten, OUTPUT);
 pinMode(Eleven, OUTPUT);
 pinMode(Twelve, OUTPUT);
 */
 
}

void loop(){
 printDate();
 HourLed(printDate());
 MinutesServo(printDate());
 delay(1000);
}

void setDateTime(){

 byte second =      45; //0-59
 byte minute =      45; //0-59
 byte hour =        18; //0-23
 byte weekDay =     4; //1-7
 byte monthDay =    14; //1-31
 byte month =       3; //1-12
 byte year  =       13; //0-99

 Wire.beginTransmission(DS1307_ADDRESS);
 Wire.write(zero); //stop Oscillator

 Wire.write(decToBcd(second));
 Wire.write(decToBcd(minute));
 Wire.write(decToBcd(hour));
 Wire.write(decToBcd(weekDay));
 Wire.write(decToBcd(monthDay));
 Wire.write(decToBcd(month));
 Wire.write(decToBcd(year));

 Wire.write(zero); //start

 Wire.endTransmission();

}

byte decToBcd(byte val){
// Convert normal decimal numbers to binary coded decimal
 return ( (val/10*16) + (val%10) );
}

byte bcdToDec(byte val)  {
// Convert binary coded decimal to normal decimal numbers
 return ( (val/16*10) + (val%16) );
}

int printDate(){

 // Reset the register pointer
 Wire.beginTransmission(DS1307_ADDRESS);
 Wire.write(zero);
 Wire.endTransmission();

 Wire.requestFrom(DS1307_ADDRESS, 7);

 int second = bcdToDec(Wire.read());
 int minute = bcdToDec(Wire.read());
 int hour = bcdToDec(Wire.read() & 0b111111); //24 hour time
 int weekDay = bcdToDec(Wire.read()); //0-6 -> sunday - Saturday
 int monthDay = bcdToDec(Wire.read());
 int month = bcdToDec(Wire.read());
 int year = bcdToDec(Wire.read());
 
 return hour;
/*
 //print the date EG   3/1/11 23:59:59
 Serial.print(month);
 Serial.print("/");
 Serial.print(monthDay);
 Serial.print("/");
 Serial.print(year);
 Serial.print(" ");
 Serial.print(hour);
 Serial.print(":");
 Serial.print(minute);
 Serial.print(":");
 Serial.println(second);
*/
}
/*
int HourLed(int hour){
	if (hour < 13) {
		digitalWrite(hour, HIGH);
	}
	if (hour >= 13){
		hour = hour-12;
		digitalWrite(hour, HIGH);
	}
}
*/

int MinutesServo(int minute){
	
	int TotalDegrees = 180;
	int pos = TotalDegrees/minute;
	
	myservo.write(pos);
}

int HourLed(int hour){
	// Hour setting for LEDS
//Set LED for One
	if (hour = 1 or 13){
 		digitalWrite(One, HIGH);	
	}
 	else{
 		digitalWrite(One, LOW);
 	}
 //Set LED for Two
 	if (hour = 2 or 14){
 		digitalWrite(Two, HIGH);	
	}
 	else{
 		digitalWrite(Two, LOW);
 	}
 //Set LED for Three
 	if (hour = 3 or 15){
 		digitalWrite(Three, HIGH);	
	}
 	else{
 		digitalWrite(Three, LOW);
 	}
 //Set LED for Four
 	if (hour = 4 or 16){
 		digitalWrite(Four, HIGH);	
	}
 	else{
 		digitalWrite(Four, LOW);
 	}
 //Set LED for Five
 	if (hour = 5 or 17){
 		digitalWrite(Five, HIGH);	
	}
 	else{
 		digitalWrite(Five, LOW);
 	}
//Set LED for Six
	if (hour = 6 or 18){
 		digitalWrite(Six, HIGH);	
	}
 	else{
 		digitalWrite(Six, LOW);
 	}
//Set LED for Seven
	if (hour = 7 or 19){
 		digitalWrite(Seven, HIGH);	
	}
 	else{
 		digitalWrite(Seven, LOW);
 	}
//Set LED for Eight
	if (hour = 8 or 20){
 		digitalWrite(Eight, HIGH);	
	}
 	else{
 		digitalWrite(Eight, LOW);
 	}
//Set LED for Nine
	if (hour = 9 or 21){
 		digitalWrite(Nine, HIGH);	
	}
 	else{
 		digitalWrite(Nine, LOW);
 	}
//Set LED for Ten
	if (hour = 10 or 22){
 		digitalWrite(Ten, HIGH);	
	}
 	else{
 		digitalWrite(Ten, LOW);
 	}
//Set LED for Eleven
	if (hour = 11 or 23){
 		digitalWrite(Eleven, HIGH);	
	}
 	else{
 		digitalWrite(Eleven, LOW);
 	}
//Set LED for Twelve
	if (hour = 12 or 0){
 		digitalWrite(Twelve, HIGH);	
	}
 	else{
 		digitalWrite(Twelve, LOW);
 	}
}
