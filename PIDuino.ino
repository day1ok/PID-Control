#include <Adafruit_MAX31855.h>
#include <pid_reg3.h>
#include <max6675.h>

// Global defines.
#define NUM_PID_CHANNELS 2
#define KW_HEATER_MAX_PWM 120           // Max duty cycle of 255 for 1kW heater. 
#define BOWL_HEATER_MAX_PWM 1           // Max duty cycle of 255. 
unsigned int pwmPin[2]= {5,6};          // Assign PWM output pins. 0 = bowl heater, 1 = 1kW heater. 

unsigned int thermoDO[3] = {13,8,2};    // Digital output lines for TCs.
unsigned int thermoCS[3] = {12,9,4};    // Chip select lines for TCs. 
unsigned int thermoCLK[3] = {11,10,3};  // Clock lines for TCs. 
unsigned int obFillPin = 14;            // Input line for Oyster Bay (OB) Fill signal from LMM (Liquid Mixing Module). 5 Minute Cycles (Hold at 5v while fill is happening).
unsigned int powerLed = 17;             // Connected to green LED on Box.
unsigned int readyLed = 18;             // Connected to yellow LED on Box.
unsigned int errorLed = 19;             // Connected to red LED on Box.
unsigned int valveRelayPin = 7;         // Valve relay pin. 
unsigned int valveOpenTime = 6000;      // Time to hold the valve open when collecting new. In milliseconds! 

// Assign Initial Set Points. 
float setPoint[2] = {55,72}; // Setpoint in deg C. 0 = bowl temp, 1 = fluid temp in 1kW heater. 
// TODO: when setting set points over serial, the setPoint variable is not updated. Just the instance of the setpoint for the pid_reg3 type. This will cause the valve control logic to function
//improperly as that code checks on the setPoint global variable.

PIDREG3 pwmPid[2];  

// Initialize TCs. 
MAX6675 kwThermo(thermoCLK[0], thermoCS[0], thermoDO[0]);            
Adafruit_MAX31855 bowlThermo(thermoCLK[1], thermoCS[1], thermoDO[1]);
Adafruit_MAX31855 kwExternalThermo(thermoCLK[2], thermoCS[2], thermoDO[2]);  

// Variable to store temperature readings. 
float currentTemp[3];   // 0 = bowl temp, 1 = fluid temp after 1kW heater, 2 = external 1kW heater temp. 

// Set warmup threshold for bowl temperature. 
// This magic number was found experimentally. Bowl temp fluctuations of +- 1.0C lead to window fluctuations of +- 0.1C. 
float bowlTempThreshold = 1.0;
float fluidTempThreshold = 1.0;
float heaterTempSafetyThreshold = 140.0; //Shut off 1kW heater if its external temperature is > 140. 

// TODO: Add high temperature flush line. 

// Logic variables. 0 = False, > 0 = True. 
unsigned int bowlReady = 0;      // Will be used to determine if bowl temperature is within bowlThreshold of the bowl setpoint.
unsigned int fluidTempReady = 0; // Will be used to determine if fluid temp exiting heater is within fluidTempThreshold of fluid temp setpoint. 
unsigned int fillingTank = 0;    // Is the oyster bay being filled or not. 
unsigned int newCycle = 1;       // Is the current fill cycle new and ready to heated/ measured, or already been heated/ measured. 

void setup() {
  
  // Set pin modes. 
  pinMode(obFillPin, INPUT);        // Set up to receive HIGH/ LOW signal indicating OB fill.  
  pinMode(valveRelayPin, OUTPUT);   // Set up to output HIG/ LOW signal to control valve relay. 
  digitalWrite(valveRelayPin, LOW); // Make sure valve is closed when starting up.
  pinMode(pwmPin[0], OUTPUT);       // Set PWM pins to output. 
  pinMode(pwmPin[1], OUTPUT);
  
  
  // Flow Cell Heater Setting
  pwmPid[0].Kd = 1;
  pwmPid[0].Ki = 0;
  pwmPid[0].Kp = 15;
  pwmPid[0].Kc = 0;
  pwmPid[0].OutMax = BOWL_HEATER_MAX_PWM;
  pwmPid[0].OutMin = 0;
  pwmPid[0].Ui = 0;
  pwmPid[0].Out = 0;
  pwmPid[0].Ref = setPoint[0];
  
  // 1kW Heater settings
  pwmPid[1].Kd = 1;
  pwmPid[1].Ki = 0;
  pwmPid[1].Kp = 15;
  pwmPid[1].Kc = 0;
  pwmPid[1].OutMax = KW_HEATER_MAX_PWM;
  pwmPid[1].OutMin = 0;
  pwmPid[1].Ui = 0;
  pwmPid[1].Out = 0;
  pwmPid[1].Ref = setPoint[1];

  Serial.begin(115200);
  
  // TODO: Fix formatting. 
  Serial.println("Temp  , Duty ,  P    ,   I   ,   D    , PreSat, Sat,  Ref");

  delay(100);   // wait for MAX chip to stabilize
}

void loop() {
  unsigned int pwmChannel;
  digitalWrite(powerLed, HIGH);
 
  // Check incoming serial to change PID parameters.  
  if(Serial.available()){
    pwmChannel = (int) Serial.parseFloat();
    Serial.flush();
    if(pwmChannel > (NUM_PID_CHANNELS-1)){
      // Received an invalid channel. 
      Serial.println("Please input a channel number 0 or 1: "); 
    }
    else{
      // Channel is valid. Change setting. 
      change_pid_param(&pwmPid[0], pwmChannel); // Pass pointer to first element of pwmPid array, but whole array is modified by function--determined by channel. 
    }
  }
     
  // Make Thermocouple readings.  
  read_thermocouples(&pwmPid[0], currentTemp);
  
  // Check bowl status, fluid temp after 1kW heater status, and Oyster Bay Fill Status. 
  set_status_flags(currentTemp);
  
  // Set 1kW heater MAX output on/ off based value of status flags. Also performs safety check of external 1kW heater temp, which is currentTemp[2].
  set_kw_heater(&pwmPid[1], currentTemp[2]); 
  
  // Update PID settings. 
  pid_reg3_calc(&pwmPid[0]); // Pass pointer to first element of pwmPid array, but whole array is modified by function. 
  
  // Write PWM values to pins. 
  analogWrite(pwmPin[0], pwmPid[0].Out);
  analogWrite(pwmPin[1], pwmPid[1].Out);
  
  // Open or close valve. 
  control_valve(); 
  
  // Print current readings/ settings over serial. 
  print_current_status(pwmPid, currentTemp);
  
  // Delay 1 second. 
  delay(100);
  
}  // End of main loop. 


void pid_reg3_calc(PIDREG3 *v)
{	
    //Loop over the number of PID channels. 
    for(int i=0; i<NUM_PID_CHANNELS; i++){
      // Compute the error
      v[i].Err = v[i].Ref - v[i].Fdb;
      // Compute the proportional output
      v[i].Up = v[i].Kp*v[i].Err;
      // Compute the integral output
      v[i].Ui = v[i].Ui + v[i].Ki*v[i].Up + v[i].Kc*v[i].SatErr;
      // integral term = oldIntegralTerm + Ki*PorportionalTerm + Kc*(difference between out and presat output)
  	// in our case, Kc is always zero
  	// so where is the antiwindup code?
  	// I want to limit the integral term the total range
  	if (v[i].Ui > (v[i].OutMax))
  		v[i].Ui = v[i].OutMax;
  	if (v[i].Ui < (v[i].OutMin))
  		v[i].Ui = v[i].OutMin;
  
      // Compute the derivative output
      v[i].Ud = v[i].Kd*(v[i].Up - v[i].Up1);
      // Compute the pre-saturated output
      v[i].OutPreSat = v[i].Up + v[i].Ui + v[i].Ud;     
      // Saturate the output
      if (v[i].OutPreSat > v[i].OutMax)                   
        v[i].Out =  v[i].OutMax;
      else if (v[i].OutPreSat < v[i].OutMin)
        v[i].Out =  v[i].OutMin;  
      else
        v[i].Out = v[i].OutPreSat;                   
      // Compute the saturate difference
      v[i].SatErr = v[i].Out - v[i].OutPreSat;     
      // Update the previous proportional output 
      v[i].Up1 = v[i].Up; 
    }
}

void change_pid_param(PIDREG3 *v, int channel)
{
   char cmdByte; 
  
   Serial.println("Please input which setting you would like to change: ");
   Serial.flush();
   while(!Serial.available()){}
   cmdByte = Serial.read();
   
   switch(cmdByte){
     case 'p':
       Serial.println("Please input proportional value:");
       while(!Serial.available()){}
       v[channel].Kp = Serial.parseFloat();
     break;  
     case 'i':
       Serial.println("Please input integral value:");
       while(!Serial.available()){}
       v[channel].Ki = Serial.parseFloat();
     break;  
     case 'd':
       Serial.println("Please input derivative value:");
       while(!Serial.available()){}
       v[channel].Kd = Serial.parseFloat();
     break;  
     case 't':
       Serial.println("Please input the temperature set point (C):");
       while(!Serial.available()){}
       v[channel].Ref = Serial.parseFloat();
     break;  
     case 'm':
       Serial.println("Please input the max PWM cycle (0 to 255):");
       while(!Serial.available()){}
       v[channel].OutMax = (int) Serial.parseFloat();
     break;  
     default:
       Serial.println("Available commands are: p,i,d,t,m.");
       Serial.println("Please re-enter the desired channel and re-try command.");    
     break;    
   } //End switch. 
}

void print_current_status(PIDREG3 v[],float temps[]){
  float percentDuty;
  
  for (int i = 0 ; i<NUM_PID_CHANNELS; i++)   
  {
     percentDuty = 100.0* (float) v[i].Out/ 255.0;
     
     Serial.print(temps[i]);   //C 
     Serial.print(" ,");          
     Serial.print(percentDuty); //Serial.print(", PWM Duty % = ");
     Serial.print(" ,");
     Serial.print(v[i].Up); //Porportional Output
     Serial.print(" ,");
     Serial.print(v[i].Ui); //Integral Output
     Serial.print(" ,");
     Serial.print(pwmPid[i].Ud); //Differential Output
     Serial.print(" ,");
     Serial.print(v[i].OutPreSat); //PreSaturated Output
     Serial.print(" ,");
     Serial.print(v[i].SatErr); //Saturated difference
     Serial.print(" ,");
     Serial.print(v[i].Ref,5); //Serial.print(", Ref = ");
     Serial.print(" ,");
  }
  Serial.print(temps[2]);  // Print 1kW external temp. 
  Serial.print(", ");
  Serial.print("bowlReady: ");
  Serial.print(bowlReady);
  Serial.print(", ");
  Serial.print("fluidTempReady: ");
  Serial.print(fluidTempReady);
  Serial.print(", ");
  Serial.print("newCycle: ");
  Serial.print(newCycle);
  Serial.print(", ");
  Serial.print("fillingTank: ");
  Serial.print(fillingTank);
  Serial.print("\n");
}

void set_status_flags(float temps[]){
  
  // Check to see if bowl is ready
  if( (temps[0] <= setPoint[0] + bowlTempThreshold) && (temps[0] >= setPoint[0] - bowlTempThreshold)){
    // Then we are within setpoint. 
    bowlReady = 1;
    digitalWrite(readyLed, HIGH);
  
  }
  else{
    bowlReady = 0;
  }
  
 
  
  // Check to see if 1kW heater has heated fluid to within setpoint. 
  if( (temps[1] <= setPoint[1] + fluidTempThreshold) && (temps[1] >= setPoint[1] - fluidTempThreshold)){
    // Then we are within setpoint. 
    fluidTempReady = 1;
  }
  else{
    fluidTempReady = 0;
  }
  
  // Check input for obFill signal. HIGH = filling, LOW = not filling. 
  fillingTank = digitalRead(obFillPin); 
  if(!fillingTank){newCycle = 1;}  // When not filling tank, the next cycle will be new, fresh fluid. 
                                   // Do not set newCycle = 0 until the valve is opened for measurement @ SRSensor.  
}

void set_kw_heater(PIDREG3 *v, float temp){
  if(temp > heaterTempSafetyThreshold){
  // Then the 1kW heater is too hot, and we need to shut it down for safety.  
    v->OutMax = 0;
    digitalWrite(errorLed, HIGH);
  } 
  else{
  // 1kW heater temp is OK. Operate normally. 
    if( bowlReady && fillingTank && newCycle){
    // The flow cell is at set point, we are receiving a fill command from the LMM, and this a New Cycle, meaning we haven't 
    // made a heated fluid/ measurement yet. So we need to heat up the fluid with the 1kW heater. 
      // Set max PWM duty limit to non-zero MAX. 
      v->OutMax = KW_HEATER_MAX_PWM;
    }
    else{
    // We are not ready, so set PWM duty limit to 0.  
      v->OutMax = 0;
    }  
  }
}

void control_valve(void){  
  if( bowlReady && fluidTempReady && fillingTank && newCycle){
  // Then the bowl is at setpoint, the fluid temp after 1kW heater is at setpoint, the OB tank is filling so there is flow, and it is a fresh cycle. 
  // The valve should open to allow the fresh cycle to inundate the SRSensor. 
    digitalWrite(valveRelayPin, HIGH);  // Open valve. 
    delay(valveOpenTime);               // Warning, PID control will not update during this period. 
    digitalWrite(valveRelayPin, LOW);   // Close valve.
    newCycle = 0;                       // Indicate that we have made a measurement. 
                                        // A new cycle will be initiated when set_status_flags() detects LOW on obFillPin.   
  }
  else{
  // Some ready conditions not met; ensure the valve is closed. 
    digitalWrite(valveRelayPin, LOW);   // Close valve.
  }
}

void read_thermocouples(PIDREG3 *v, float *temps){
  // Make Thermocouple readings.   
  temps[0] = bowlThermo.readCelsius();  //Flow Cell
  temps[1] = kwThermo.readCelsius();   //1 kW F Temp
  temps[2] = kwExternalThermo.readCelsius();  //1 kW Normal Temp
  
  // Update PID feedback variables. 
  v[0].Fdb = temps[0];
  v[1].Fdb = temps[1];
}



