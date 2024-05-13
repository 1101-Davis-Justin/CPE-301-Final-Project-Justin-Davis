//Name: Justin Davis
//Date: 5/10/2024
//CPE 301 Final Project

//Libraries
#include <dht.h> //This is the Sensor Library
#include <Stepper.h> //This is the Motor Library
#include <LiquidCrystal.h> //This is the Display Library (Used in Lab 6)
#include <DS3231.h>

#define RDA 0x80 //?? From Previous Lab 5 UART
#define TBE 0x20 //?? From Previous Lab 5 UART

// UART Pointers (From Lab 7)
volatile unsigned char *myUCSR0A  = (unsigned char *) 0x00C0;
volatile unsigned char *myUCSR0B  = (unsigned char *) 0x00C1; 
volatile unsigned char *myUCSR0C  = (unsigned char *) 0x00C2; 
volatile unsigned int  *myUBRR0   = (unsigned int *) 0x00C4; 
volatile unsigned char *myUDR0    = (unsigned char *) 0x00C6;

//ADC Pointers (From Lab 7)
volatile unsigned char *my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char *my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char *my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int *my_ADC_DATA = (unsigned int*) 0x78;


//Liquid Crystal Stuff (Lab 6)
// LCD pins <--> Arduino pins
const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5; //May need to Change
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//Motor Stuff
const int stepsPerRevolution = 2038; //Might need to change
Stepper myStepper = Stepper(stepsPerRevolution, 38, 40, 39, 41); //IN1-IN3-IN2-IN4 for proper step sequence

//Sensor Stuff
dht DHT;
#define DHT11_PIN 7
int temperature = 1;
int currentTemp;
int temperatureThreshold = 25;


//WaterLevelStuff
int waterLevel;
int waterLevelThreshold = 50;

//Data Direction Registers

//Port A  this is for the buttons
volatile unsigned char *portDDRA = (unsigned char *) 0x21;
//This is the Data Direction Register for Port A
volatile unsigned char *portA =    (unsigned char *) 0x22;
//This is the output data register for port A.
volatile unsigned char *pinA =    (unsigned char *) 0x20;
//This is the input pin address for port A.

//Port B (This will be for the LEDs Light)
volatile unsigned char *portDDRB = (unsigned char *) 0x24;
//This is the Data Direction Register for Port B
volatile unsigned char *portB =    (unsigned char *) 0x25;
//This is the output data register for port B.
volatile unsigned char *pinB =    (unsigned char *) 0x23;
//This is the input pin address for port B.

//Port H (Currently Used for the L293D and the FAN)
volatile unsigned char *portDDRH = (unsigned char *) 0x101;
//This is the Data Direction Register for Port C
volatile unsigned char *portH =    (unsigned char *) 0x102;
//This is the output data register for port C.
volatile unsigned char *pinH =    (unsigned char *) 0x100;
//This is the input pin address for port C.

//Fan Stuff
int enA = 9; //PH6 //Output 0100 0000 0xBF Input = 1, Output is 0 So to set this to an output do 
// int in1 = 8; //PH5 //Output to set to output is  0010 0000 0x20
// int in2 = 6; //PH3 //Output to set to output is 0000 1000 0x08

//Button Stuff
enum State {DisabledState, IdleState, RunningState, ErrorState, NothingState};
State currentState = DisabledState;
State previousState = NothingState;
bool restartButton = false;
bool stopButton = false;


//Millis Stuff
static unsigned long previousMillis = 0;
static unsigned long interval = 5000; // Adjust as needed

//Time Stuff
DS3231 rtc(SDA, SCL);

//ISR Stuff
bool isStart = false;

//Delay Stuff
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

void setup()
{

  U0init(9600); //Setup the UART
  lcd.begin(16, 2); // set up number of columns and rows (Lab 6)
  adc_init(); //Setup the ADC
  rtc.begin();
  *portDDRB = 0xFF; // 0x0F Makes the pins 53, 52, 51, and 50 outputs of port B an output.
  *portDDRA &= 0xF8;// Makes the first 3 pins an input pin Should be pin 22, 23, and 24
  *portDDRH = 0x68; // Makes the pins PH3, PH5, and PH6 outputs

  attachInterrupt(digitalPinToInterrupt(18), startButton, CHANGE); //AttachInterrupt Pins are 2,3,18
  temperature = DHT.read11(DHT11_PIN);
  displayTemperature();
  
}


void loop()
{
  unsigned long currentMillis = millis(); 
  if (currentMillis - previousMillis >= interval)
  {   
    previousMillis = currentMillis;
    updateCurrentState();
    if (currentState == DisabledState)
    {
      disabledState();
      allStates();
    } 
    else if (currentState == IdleState) 
    {
      idleState();
      allStates();
    } 
    else if (currentState == RunningState)
    {
      runningState();
      allStates();
    } 
    else if (currentState == ErrorState)
    {
      errorState();
    }
    
  }
}
/* DEBUGGING CODE
void printStatuses()
{
  Serial.print("Here is the waterLevel: ");
  Serial.println(waterLevel);
  Serial.print("Here is the WaterLevelThreshold: ");
  Serial.println(waterLevelThreshold);

  Serial.print("Here is the temperature: ");
  Serial.println(int(DHT.temperature));
  
  Serial.print("Here is the temperatureThreshold: ");
  Serial.println(temperatureThreshold);

  Serial.print("Here is the humidiuty: ");
  Serial.println(int(DHT.humidity));

  Serial.print("Here is RestartStatus: ");
  Serial.println(restartButton);
  
  Serial.print("Here is StopButtonStatus: ");
  Serial.println(stopButton);

  Serial.print("Here is CurrentState: ");
  Serial.println(currentState);
  Serial.println("\n\n");
}
*/
void updateCurrentState()
{
  waterLevel = adc_read(0);
  temperature = DHT.read11(DHT11_PIN);
  currentTemp = int(DHT.temperature);
  //printStatuses();
  checkRestart();
  checkStop();
  
  if(currentState != previousState)
  {
    if(currentState == DisabledState)
    {

      if(isStart == true)
      {
        stopButton = false;
        currentState = IdleState;
      }
    }
    else if (currentState == IdleState)
    {

      restartButton = false;
      if(waterLevel <= waterLevelThreshold)
      {
        Serial.println("It appears that waterLevel <= waterLevelThreshold(IdleState)");
        currentState = ErrorState;
      }

      
      if(currentTemp > temperatureThreshold)
      {
        Serial.println("It appears that temperature > temperatureThreshold(IdleState)");
        currentState = RunningState;
        
      }
      if(stopButton == true)
      {
        Serial.println("It appears that stopButton == true");
        currentState = DisabledState;
        isStart = false;
      }
    }
    else if (currentState == RunningState)
    {
 
      if(currentTemp <= temperatureThreshold)
      {
        Serial.println("It appears that temperature <= temperatureThreshold (RunningState)");
        currentState = IdleState;
        directionControl(false);
        //printDateAndTime();
      }
      if(waterLevel < waterLevelThreshold)
      {
        Serial.println("It appears that waterLevel < waterLevelThreshold (RunningState)");
        currentState = ErrorState;
        directionControl(false);
        printDateAndTime();
      }
      if(stopButton == true)
      {
        Serial.println("It appears that stopButton == true (RunningState)");
        currentState = DisabledState;
        directionControl(false);
        printDateAndTime();
        isStart = false;
      }
    }
    else if (currentState == ErrorState)
    {
      
      if(restartButton == true)
      {
        Serial.println("It appears that restartButton == true (ErrorState)");
        currentState = IdleState;
      }
      if(stopButton == true)
      {
        Serial.println("It appears that stopButton == true (ErrorState)");
        currentState = DisabledState;
        isStart = false;
      }
    }
  }
}
void checkStop()
{
  if(*pinA & (1<<1))
  {
    stopButton = true;
  }
}

void checkRestart()
{
  if(*pinA & (1<<2))
  {
    restartButton = true;
  }
}
void setStepperSpeed(int speed)
{
  myStepper.setSpeed(speed);
}
void disabledState()
{
  //Serial.print("Welcome to disabledState");
  *portB |= 0x01; //Sets pin 53 High
  *portB &= 0xF1;
  directionControl(false); //turns fans off
}
void idleState()
{
  *portB |= 0x02; // Sets pin 52 High
  *portB &= 0xF2;
  setStepperSpeed(10);
  myStepper.step(-stepsPerRevolution);
  
}
void runningState()
{
  *portB |= 0x04; //Set pin 51 High
  *portB &= 0xF4;
  directionControl(true);
  setStepperSpeed(10);
  myStepper.step(-stepsPerRevolution);
}
void errorState()
{
  *portB |= 0x08; //Set pin 50 High
  *portB &= 0xF8;
  setStepperSpeed(0);
  displayError();
}
void allStates() //Keeps updating the properties all states share
{
  updateTemperature();
  displayTemperature();
  if(isStart == false)
  {
    currentState = DisabledState;
  }
}
void startButton()
{
  if (isStart == false)
  {
    isStart = true;
  } 
}
void printDateAndTime()
{
  String dateStr = rtc.getDateStr(FORMAT_LONG, FORMAT_MIDDLEENDIAN, '.');
  String timeStr = rtc.getTimeStr();
  U0putchar('D');
  U0putchar('a');
  U0putchar('t');
  U0putchar('e');
  U0putchar(':');
  U0putchar(' ');
  for (int i = 0; i < dateStr.length(); ++i)
  {
    U0putchar(dateStr.charAt(i));
  }
  U0putchar('\n');

  U0putchar('T');
  U0putchar('i');
  U0putchar('m');
  U0putchar('e');
  U0putchar(':');
  U0putchar(' ');
  for (int i = 0; i < timeStr.length(); ++i)
  {
    U0putchar(timeStr.charAt(i));
  }
  U0putchar('\n');
}

void directionControl(bool onOrOff) {
	// Set motors to maximum speed
	// For PWM maximum possible values are 0 to 255
	analogWrite(enA, 255); //Is okay to use this function

  if(onOrOff == true)
  {
  // Turn on motor A
	//digitalWrite(in1, HIGH);
  *portH |= 0x20; //PH5
  *portH &= ~0x08; //PH3
	//digitalWrite(in2, LOW);
  }
  else
  {
	// Turn off motors
	// digitalWrite(in1, LOW);
  *portH &= 0x20; //PH5
  *portH &= ~0x08; //PH3
	// digitalWrite(in2, LOW);
  }
}
void updateTemperature()
{
  temperature = DHT.temperature;
  waterLevel = adc_read(0);
}
void displayTemperature(){
    // Display Temperature
    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print("Temp: ");
    lcd.print(int(DHT.temperature));
    lcd.print((char)223);
    lcd.print("C");
    // Display Humidity
    lcd.setCursor(0,1);
    lcd.print("Humi: ");
    lcd.print(int(DHT.humidity));
    lcd.print("%");
}
void displayError()
{
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("Water level is ");
  lcd.setCursor(0,1);
  lcd.print("too low");
}

/*
 * UART FUNCTIONS (From Lab 7)
 */
void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}


/*
*ADC Functions (From LAB 7)
*/
unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}


void my_delay(unsigned int delay) //Lab 4
{
  // calc period
  double period = 1.0/double(delay);
  // 50% duty cycle
  double half_period = period/ 2.0f;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  unsigned int ticks = half_period / clk_period;
  // stop the timer
  *myTCCR1B &= 0xF8;
  // set the counts
  *myTCNT1 = (unsigned int) (65536 - ticks);
  // start the timer
  * myTCCR1A = 0x0;
  * myTCCR1B |= 0b00000001;
  // wait for overflow
  while((*myTIFR1 & 0x01)==0); // 0b 0000 0000
  // stop the timer
  *myTCCR1B &= 0xF8;   // 0b 0000 0000
  // reset TOV           
  *myTIFR1 |= 0x01;
}