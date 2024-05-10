#include <RC_Receiver.h>
#include <SoftwareSerial.h>

//RC-pins
#define CH1 2 //GRØNN
#define CH2 3 //GRÅ
#define CH3 4 //HVIT
//motor 1
#define PWM1 5 //HVIT_RØD
#define INA1 6 //HVIT_BLÅ
#define INB1 7 //HVIT_SVART
//motor 2
#define INA2 8  //GUL_BLÅ
#define INB2 9  //GUL_SVART
#define PWM2 10 //GUL_RØD
//Software serial pins
#define RX_PIN 11 //HVIT
#define TX_PIN 12 //GUL 

RC_Receiver receiver(CH1, CH2, CH3);
SoftwareSerial mySerial(RX_PIN, TX_PIN);

//---REGULERING------------------------

//target_verdier
float target_speed = 1.1; //------Stopp fartsreg 1.1
float target_angle = 1.1; //------Stopp autopilot 1.1
float current_speed = 0.0;
float current_angle = 0.0;

//tidl.verdier
float prev_current_speed = 0.0;
float prev_current_angle = 0.0;
float prev_target_speed = 0.0;
float prev_target_angle = 0.0;

//PI fartsregulering
double f_Kp = 5;
double f_Ki = 0.005;
int f_konst = 25;
float f_reg = 0.0;

double f_error, f_prev_error, f_integral;
unsigned long f_prev_time = 0;

//PI autopilot
double a_Kp = 0.3;
double a_Ki = 0.0001;
int a_konst = 25;
float a_reg = 0.0;


int a_retning = 0;
int prev_a_retning = 1;

double a_error, a_prev_error, a_integral;
unsigned long a_prev_time = 0;

//felles tid mellom iterasjoner
unsigned long dt = 100; 



//---RC KONTROLLER-----------------------------------

int minMax[8][2] = 
{ 
	{1050,2050}, 
	{1010,2020}, 
  {1050,2005}
};

int styring= 0;
int gass= 0;
int mode= 0;
int prev_styring = 0;
int prev_gass = 0;
int prev_mode = 0;

//---Motordriver-------------------------------------

int MotorSpeed1 = 0;
int MotorSpeed2 = 0;
 
// Motor retning - 0 = bak, 1 = fram
int MotorDir = 1;



void setup(){
  
  Serial.begin(9600);
  mySerial.begin(9600);

  receiver.setMinMax(minMax);
  
  Serial.println("Ready");

  pinMode(PWM1, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);

  delay(1000);
}

void loop()
{  
//---RC KONTROLLER--------------------------------------------------------------------------
  
  lesRCinput();

 if (mode == 1){
  
    MotorSpeed1 = abs(gass) + styring;
    MotorSpeed2 = abs(gass) - styring;
        
    if (gass >= 0) {
      //Fram
      MotorDir = 1;
      //Serial.println(" Fram");
    } 
    else if (gass <= -5){
      //Bak
      MotorDir = 0;
      //Serial.println(" Bak");
    }
  }

//---STYRING VIA REGULERING OG PYTHON INPUTS FRA SERIELL--------------------------------------------------------------------------

  else if (mode == 0) {
    
    lesPCinput(); //Leser input fra seriell

    if (target_speed >= 0){
      //Fram
      MotorDir = 1;
      //Serial.println("Fram");
    }
    else{
      //Bak
      MotorDir = 0;
      //Serial.println("Bak");
      target_angle = 1.1;
    }
    
    //---FARTSREGULERING--------------------------------------------------------------------------  

      //Timer fartsregulering
    unsigned long f_cur_time = millis();
    unsigned long f_timer = f_cur_time - f_prev_time;

    //Nullstill integral ved ny target
    if (target_speed != prev_target_speed) {
      f_integral = 0;
      prev_target_speed = target_speed;
    }
    
    if (current_speed != abs(target_speed) && target_speed != 0) {
      // Sjekker timer
      if (f_timer >= dt && f_timer != 0) {

        // regner ut avviket
        f_error = target_speed - current_speed;

        // integrasjon trapes regel
        f_integral += 0.5 * (f_error + f_prev_error) * f_timer;

        // PI utregning
        f_reg = f_Kp * f_error + f_Ki * f_integral + f_konst;
        f_reg = constrain(f_reg, 0, 200);

        // Oppdatter error og timer
        f_prev_error = f_error;
        f_prev_time = f_cur_time;

        // PI verdier for debug
        Serial.print("f_P: ");
        Serial.print(f_Kp * f_error);
        Serial.print("\t f_I: ");
        Serial.println(f_Ki * f_integral);
      }
    }
    else{
      // Når fart er større en target
      f_reg = 0;
     
      // Nullstill integral
      f_integral = 0;
    }

    if (target_speed == 1.1){ //----------STOPPKODE 1.1
      f_reg = 0;
      f_integral = 0;
      Serial.println("Fartsregulering AV");
    }

    MotorSpeed1 = f_reg;
    MotorSpeed2 = f_reg;
    
    

    //---AUTOPILOT--------------------------------------------------------------------------   

      //Timer autopilot
    unsigned long a_cur_time = millis();
    unsigned long a_timer = a_cur_time - a_prev_time;

    // Regn ut korteste retning
    int v_avstand = target_angle - current_angle;

    if (v_avstand > 180) {
      v_avstand -= 360; 

    }
    else if (v_avstand < -180) {
      v_avstand += 360; 
    }

    if (v_avstand > 0) {
      a_retning = 1; // Med klokken
    } 
    else if (v_avstand < 0) {
      a_retning = -1; // Mot klokken
    } 
    else {
      a_retning = 0; 
    }

    if (a_retning != prev_a_retning) {
      a_integral = 0;
      prev_a_retning = a_retning;
    }
    
    /*
    // Print retning for debug
    Serial.print("v_avstand = ");
    Serial.println(v_avstand);
    Serial.print("Direction to turn: ");

    if (a_retning == 1) {
      Serial.println("Clockwise");
    } 
    else if (a_retning == -1) {
      Serial.println("Counterclockwise");
    } 
    else {
      Serial.println("No need to turn");
    }
    */
    
    //----NULLSTILL VED NY TARGET VINKEL--------
  
    if (target_angle != prev_target_angle) {
      a_integral = 0;
      prev_target_angle = target_angle;
    }

    //---REGULERING----------
    
    if (current_angle != target_angle && abs(v_avstand) > 2) {
      // Sjekker timer
      if (a_timer >= dt && a_timer != 0) {
        
        a_error = abs(v_avstand);

        // integrasjon trapes regel
        a_integral += 0.5 * (a_error + a_prev_error) * a_timer;

        // PI utregning
        a_reg = a_Kp * a_error + a_Ki * a_integral + a_konst;
        a_reg = constrain(a_reg, 0, 100);

       // Oppdatter error og timer
        a_prev_error = a_error;
        a_prev_time = a_cur_time;

        // PI verdier for debug
        Serial.print("a_P: ");
        Serial.print(a_Kp * a_error);
        Serial.print("\t a_I: ");
        Serial.println(a_Ki * a_integral);
      }
    }
    else {
      // Når vinkel = target
      a_reg = 0;
     
      // Nullstill integral
      a_integral = 0;
    }

    if (target_angle == 1.1){ //----------STOPPKODE 1.1
      a_reg = 0;
      a_integral = 0;
      Serial.println("Autopilot AV");
    }

  //----AUTOPILOT MOTORKONTROLL--------  
    if (a_retning == 1){
        MotorSpeed1 += a_reg;
        MotorSpeed2 -= a_reg;
    }
    else if (a_retning == -1) {
      MotorSpeed1 -= a_reg;
      MotorSpeed2 += a_reg;
    }
    else {
      MotorSpeed1 = f_reg;
      MotorSpeed2 = f_reg;
    }
    
  }

//---MOTORSTYRING--------------------------------------------------------------------------

  MotorSpeed1 = constrain(MotorSpeed1, 0, 255); //Begrenser verdier innenfor PWM 
  MotorSpeed2 = constrain(MotorSpeed2, 0, 255); 

  //Kjører motore
  if (MotorSpeed1 >= 25){ 
  mControlA(MotorSpeed1, MotorDir);
  }
  else{
    mControlA(0, MotorDir);
    }

  if (MotorSpeed2 >= 25){ 
    mControlB(MotorSpeed2, MotorDir);
  }
  else{
    mControlB(0, MotorDir);
    }
  
  //PWM verdier for debug
  Serial.print("Motor 1 Speed = ");
  Serial.print(MotorSpeed1);
  Serial.print(" | Motor 2 Speed = ");
  Serial.println(MotorSpeed2);

  delay(100);
}



//---MOTOR 1--------------------------------------------------------------------------

void mControlA(int mspeed, int mdir) {
 
  // Velger motorretning (HIGH-LOW) 
  if (mdir == 0) {
    // Bak
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, HIGH);
  } else {
    // Fram
    digitalWrite(INA1, HIGH);
    digitalWrite(INB1, LOW);
  }
  analogWrite(PWM1, mspeed);
}
 
//---MOTOR 2--------------------------------------------------------------------------

void mControlB(int mspeed, int mdir) {
 
  // Velger motorretning (HIGH-LOW) 
  if (mdir == 0) {
    // Bak
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, HIGH);
  } else {
    // Fram
    digitalWrite(INA2, HIGH);
    digitalWrite(INB2, LOW);
  }
  analogWrite(PWM2, mspeed);
}

void lesRCinput() {
  
  int n_mode = map(receiver.getRaw(3), 1100, 1900, 0, 1);
  
  if (n_mode == prev_mode) {
    mode = n_mode; 
  }
  prev_mode = n_mode;

  //Sjekk kun de andre verdiene om båren er i RC-modus
  if (mode == 1) {

    int n_styring = map(receiver.getRaw(1), 1050, 2050, -150, 150);
    int n_gass = map(receiver.getRaw(2), 1010, 2020, -200, 200);
  

    // Rc verdier sjekkes for å unngå feilbeskjed
    int styring_endr = n_styring - prev_styring;
    int gass_endr = n_gass - prev_gass;

    if (abs(styring_endr) <= 200) {
      styring = n_styring;
    }
    prev_styring = styring;

    if (abs(gass_endr) <= 255) {
      gass = n_gass;
    }
    prev_gass = gass;
  }

  /*
  // RC-verdier for debug
  Serial.print("Ch1 styring = ");
  Serial.print(styring);
 
  Serial.print(" Ch2 gass = ");
  Serial.println(gass);

  Serial.print(" Ch3 mode = ");
  Serial.println(mode);
  */

}

void lesPCinput() {
  // Check if there is data available to read from SoftwareSerial
  while (mySerial.available() > 0) {
    // Read the incoming string from SoftwareSerial
    String receivedString = mySerial.readStringUntil('\n');
    
       
    //Serial.println("Received String: " + receivedString);
    
    //Sjekk at in streng har 3 komma
    int commaCount = 0;
    for (int i = 0; i < receivedString.length(); i++) {
      if (receivedString.charAt(i) == ',') {
        commaCount++;
      }
    }
    if (commaCount != 3) {
      continue;
    }
    
    // Parse the received string and extract target_speed, target_angle, current_speed, and current_angle
    int commaIndex1 = receivedString.indexOf(',');
    int commaIndex2 = receivedString.indexOf(',', commaIndex1 + 1);
    int commaIndex3 = receivedString.indexOf(',', commaIndex2 + 1);
    if (commaIndex1 != -1 && commaIndex2 != -1 && commaIndex3 != -1) {

      target_speed = receivedString.substring(0, commaIndex1).toFloat();
      target_speed = constrain(target_speed, -2, 7);

      target_angle = receivedString.substring(commaIndex1 + 1, commaIndex2).toFloat();
      target_angle = constrain(target_angle, 0, 360);

      current_speed = receivedString.substring(commaIndex2 + 1, commaIndex3).toFloat();

      current_angle = receivedString.substring(commaIndex3 + 1).toFloat();

      /*
      // Sjekk at det ikke er store endringer i verdier for å unngå feilbeskjed
      if (abs(new_current_speed - prev_current_speed) < 4.0) {
        current_speed = new_current_speed;
        prev_current_speed = new_current_speed;
      }

      if (abs(new_current_angle - prev_current_angle) < 100.0) {
        current_angle = new_current_angle;
        prev_current_angle = new_current_angle;
      }
      */

      current_speed = constrain(current_speed, 0, 7);
      current_angle = constrain(current_angle, 0, 360);

      
      Serial.print("Target Speed: ");
      Serial.println(target_speed);
      Serial.print("Target Angle: ");
      Serial.println(target_angle);
      Serial.print("Current Speed: ");
      Serial.println(current_speed);
      Serial.print("Current Angle: ");
      Serial.println(current_angle);
    }
  }
}