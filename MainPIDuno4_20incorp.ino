#include <pid_reg3.h>
#include <max6675.h>

PIDREG3 pwmPid;
/*
int thermoDO = 12;
int thermoCS = 11;
int thermoCLK = 10;
int power = 13;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
*/
int analogPin = 0;     // potentiometer wiper (middle terminal) connected to analog pin 3
                       // outside leads to ground and +5V
int val = 0;           // variable to store the value read
float Temp;

//Average Stuff
const int numReadings = 5;
int readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int inputPin = A0;

int pwmPin = 5;
int vccPin = 3;
int gndPin = 2;


float setPoint = 69; // Setpoint in deg C. 
float currentTemp;
float percentDuty;

char cmdByte;
float inputVal;

void pid_reg3_calc(PIDREG3 *v)

{	

    // Compute the error
    v->Err = v->Ref - v->Fdb;
    // Compute the proportional output
    v->Up = v->Kp*v->Err;
    // Compute the integral output
    v->Ui = v->Ui + v->Ki*v->Up + v->Kc*v->SatErr;
    // integral term = oldIntegralTerm + Ki*PorportionalTerm + Kc*(difference between out and presat output)
	// in our case, Kc is always zero
	// so where is the antiwindup code?
	// I want to limit the integral term the total range
	if (v->Ui > (v->OutMax))
		v->Ui = v->OutMax;
	if (v->Ui < (v->OutMin))
		v->Ui = v->OutMin;

    // Compute the derivative output
    v->Ud = v->Kd*(v->Up - v->Up1);
    // Compute the pre-saturated output
    v->OutPreSat = v->Up + v->Ui + v->Ud;     
    // Saturate the output
    if (v->OutPreSat > v->OutMax)                   
      v->Out =  v->OutMax;
    else if (v->OutPreSat < v->OutMin)
      v->Out =  v->OutMin;  
    else
      v->Out = v->OutPreSat;                   
    // Compute the saturate difference
    v->SatErr = v->Out - v->OutPreSat;     
    // Update the previous proportional output 
    v->Up1 = v->Up; 
}

void setup() {
  
  pwmPid.Kd = 1;
  pwmPid.Ki = 0;
  pwmPid.Kp = 15;
  pwmPid.Kc = 0;
  pwmPid.OutMax = 1;
  pwmPid.OutMin = 0;
  pwmPid.Ui = 0;
  pwmPid.Out = 0;
  pwmPid.Ref = setPoint;
  
  
  Serial.begin(9600);
  // use Arduino pins 
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;  
  
  pinMode(vccPin, OUTPUT); digitalWrite(vccPin, HIGH);
  pinMode(gndPin, OUTPUT); digitalWrite(gndPin, LOW);
  //pinMode(power, OUTPUT); digitalWrite(power, HIGH);
  
  Serial.println("MAX6675 test");
  Serial.println("Temp  , Duty ,  P    ,   I   ,   D    , PreSat, Sat,  Ref");
  // wait for MAX chip to stabilize
  delay(100);
}

void loop() {
  // subtract the last reading:
  total= total - readings[index];         
  // read from the sensor:  
  readings[index] = analogRead(inputPin); 
  // add the reading to the total:
  total= total + readings[index];       
  // advance to the next position in the array:  
  index = index + 1;                    

  // if we're at the end of the array...
  if (index >= numReadings)              
    // ...wrap around to the beginning: 
    index = 0;                           

  // calculate the average:
  average = total / numReadings;         
  // send it to the computer as ASCII digits
  Serial.println(average);  
  Temp = (float) ((average*0.0693)-4.9673); 
  
  delay(1);
  
  /*
  // basic readout test, just print the current temp
    val = analogRead(analogPin);    // read the input pin
    //Temp = (float) ((((val+68)-4)/16*(1023-222) + 222)*.000911);
    Temp = (float) ((val*0.0693)-4.9673);
    Serial.println(Temp);
    Serial.println(val);
    */
/*




void loop() {
  ;        // delay in between reads for stability            
}
*/

   if(Serial.available()){
     cmdByte = Serial.read();
     
     switch(cmdByte){
     case 'p':
       Serial.println("Please input proportional value:");
       while(!Serial.available()){}
       pwmPid.Kp = Serial.parseFloat();
     break;  
     case 'i':
       Serial.println("Please input integral value:");
       while(!Serial.available()){}
       pwmPid.Ki = Serial.parseFloat();
     break;  
     case 'd':
       Serial.println("Please input derivative value:");
       while(!Serial.available()){}
       pwmPid.Kd = Serial.parseFloat();
     break;  
     case 't':
       Serial.println("Please input the temperature set point (C):");
       while(!Serial.available()){}
       pwmPid.Ref = Serial.parseFloat();
     break;  
     case 'm':
       Serial.println("Please input the max PWM cycle (0 to 255):");
       while(!Serial.available()){}
       pwmPid.OutMax = (int) Serial.parseFloat();
     break;  
     
     case 'r':
       Serial.print("kp: ");
       Serial.print(pwmPid.Kp,5);
       Serial.print(", ki: ");
       Serial.print(pwmPid.Ki,5);
       Serial.print(", kd: ");
       Serial.print(pwmPid.Kd,5);
       Serial.print(", Set Point: ");
       Serial.print(pwmPid.Ref,5);
       Serial.print(", Saturated Pulse %: ");
       Serial.println(100.0* (float) pwmPid.OutMax/ 255.0);
     break;  
     } //end switch
   } //end if
   
   currentTemp = Temp;
   pwmPid.Fdb = currentTemp;
   pid_reg3_calc(&pwmPid);
   analogWrite(pwmPin, pwmPid.Out);
   
   percentDuty = 100.0* (float) pwmPid.Out/ 255.0;
   
   /* Recent
   Serial.print(currentTemp);   //C =
   Serial.print(" ,");          
   Serial.print(percentDuty); //Serial.print(", PWM Duty % = ");
   Serial.print(" ,");
   Serial.print(pwmPid.Up); //Porportional Output
   Serial.print(" ,");
   Serial.print(pwmPid.Ui); //Integral Output
   Serial.print(" ,");
   Serial.print(pwmPid.Ud); //Differential Output
   Serial.print(" ,");
   Serial.print(pwmPid.OutPreSat); //PreSaturated Output
   Serial.print(" ,");
   Serial.print(pwmPid.SatErr); //Saturated difference
   Serial.print(" ,");
   Serial.println(pwmPid.Ref,5); //Serial.print(", Ref = ");
   */
   /*
   Serial.print(pwmPid.Kp,5); //Serial.print(", P Control = ");
   Serial.print(" ,");
   Serial.print(pwmPid.Ki,5); //Serial.print(", I Control = ");
   Serial.print(" ,");
   Serial.print(pwmPid.Kd,5); //Serial.print(", D Control = ");
   Serial.print(" ,");
   */
   
   
   
   delay(1000);
}

