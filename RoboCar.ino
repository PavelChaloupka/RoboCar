/*
PINS Receiver
o ... Ground
o...  Ucc (5V)
o...  B pin 4,10,11,12
o...  D pin 4,10,11,12
o...  A pin 4,10,11,12
o...  C pin 4,10,11,12
o...  rele

Click on the button on the transmiter causes on pin log "1" 
 
 */
// Volne
// 1,2,  
/*
 HC-SR04 zapojte následovně:
 VCC to arduino 5v
 GND to arduino GND
 Echo to Arduino pin 7
 Trig to Arduino pin 8
 */
 
 // 1 nastavit na plny vykon
 // po stisku nastavit trvani 3s
 // pokud vpred a stisknu vzad pak zastavit

// original
const int PORT_LEFT_WHEEL_FRONT = 3; // PWM 3, 5, 6, 9
const int PORT_LEFT_WHEEL_BACK = 5; // PWM 3, 5, 6, 9
const int PORT_RIGHT_WHEEL_FRONT = 6; // PWM 3, 5, 6, 9
const int PORT_RIGHT_WHEEL_BACK = 9; // PWM 3, 5, 6, 9

const int MOTOR_OKAMZITA_BRZDA = -1000;

#define echoPin 7 // Echo Pin Ultrazvuk
#define trigPin 8 // Trigger Pin Ultrazvuk

#define inputLeft 12 // Vstup pro povel vlevo
#define inputFront 11 // Vstup pro povel vpred
#define inputRight 4 // Vstup pro povel vpravo
#define inputBack  10 // Vstup pro povel vzad

const int ULTRASONIC_maximumRange = 45; // maximální hodnota vydalenost v centimetrech
const int ULTRASONIC_minimumRange = 7; // minimální hodnota v centimetrech
const int ULTRASONIC_kriticRange = 4; // minimální hodnota v centimetrech
long ULTRASONIC_duration, ULTRASONIC_distance; // Vypočítání vzdálenosti
byte ULTRASONIC_State_distance = 0; // 0 prekazka velmi blizko 1 prekazka blizko mohu jet 2 Volno

const int DIODA_INFO = 13; // Na desce 
const int MAX_long_pulse = 49*5;

const int MAX_duration_separate_drive = 21;

#define  STEP_DRIVE_MOTOR 14

const int PERIODE_SWITCH_CORNERING = 14; //Pri otaceni pojizdime dopredu a pak dozadu
int step_switch_cornering = 0;
bool switch_cornering = false;
bool switching_cornering = false; // ma li se otaceni prepinat

const int DIRECTION_NONE     = 0; 
const int DIRECTION_FRONT   = 1; 
const int DIRECTION_LEFT = 3; 
const int DIRECTION_RIGHT = 2; 
const int DIRECTION_FRONT_LEFT = 13; 
const int DIRECTION_FRONT_RIGHT = 12; 
const int DIRECTION_BACK_LEFT  = 43; 
const int DIRECTION_BACK_RIGHT  = 42; 
const int DIRECTION_BACK    = 4; 
const int DIRECTION_STOP    = -1;

const int MOVEMENT_NIC           = 0; 
const int MOVEMENT_FRONT         = 1; 
const int MOVEMENT_LEFT       = 3; 
const int MOVEMENT_RIGHT       = 2; 
const int MOVEMENT_FRONT_LEFT = 13; 
const int MOVEMENT_FRONT_RIGHT = 12; 
const int MOVEMENT_BACK_LEFT  = 43; 
const int MOVEMENT_BACK_RIGHT  = 42; 
const int MOVEMENT_BACK          = 4; 
const int MOVEMENT_STOP          = -1; 

byte Value_LEFT= 0;
byte Value_FRONT = 0;
byte Value_BACK = 0;
byte Value_RIGHT = 0;
byte Value_Stop = 0;

byte Value_FRONT_LEFT = 0;
byte Value_FRONT_RIGHT = 0;
byte Value_BACK_LEFT  = 0;
byte Value_BACK_RIGHT  = 0;

byte Actual_Value_LEFT= 0;
byte Actual_Value_FRONT = 0;
byte Actual_Value_BACK = 0;
byte Actual_Value_RIGHT = 0;

byte Actual_Value_FRONT_LEFT = 0;
byte Actual_Value_FRONT_RIGHT = 0;
byte Actual_Value_BACK_LEFT  = 0;
byte Actual_Value_BACK_RIGHT  = 0;
byte Actual_Value_Stop = 0;

byte Old_Value_LEFT= 0;
byte Old_Value_FRONT = 0;
byte Old_Value_BACK = 0;
byte Old_Value_RIGHT = 0;

byte Old_Value_FRONT_LEFT = 0;
byte Old_Value_FRONT_RIGHT = 0;
byte Old_Value_BACK_LEFT  = 0;
byte Old_Value_BACK_RIGHT  = 0;
byte Old_Value_Stop = 0;

byte Grent_Old_Value_LEFT= 0;
byte Grent_Old_Value_FRONT = 0;
byte Grent_Old_Value_BACK = 0;
byte Grent_Old_Value_RIGHT = 0;

byte Grent_Old_Value_FRONT_LEFT = 0;
byte Grent_Old_Value_FRONT_RIGHT = 0;
byte Grent_Old_Value_BACK_LEFT  = 0;
byte Grent_Old_Value_BACK_RIGHT  = 0;
byte Grent_Old_Value_Stop = 0;

byte DIRECTION_historie = DIRECTION_NONE;

int step_separate_drive = 0;

int Number_mode = 0;
byte Old_number_mode = 0;
int Position_long_pulse = 0;

int State_Left_wheel_front = 0; // Aktualni nastavene stavy motoru
int State_Left_wheel_back = 0;
int State_Right_wheel_front = 0;
int State_Right_wheel_back = 0;

int State_Left_wheel = 0; // Aktualni nastavene stavy motoru
int State_Right_wheel = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.

  pinMode(trigPin, OUTPUT); // Ultrazvuk
  pinMode(echoPin, INPUT);  // Ultrazvuk
  pinMode(inputLeft, INPUT);  // Vstup z prijimace
  pinMode(inputFront, INPUT);  // Vstup z prijimace
  pinMode(inputRight, INPUT); // Vstup z prijimace
  pinMode(inputBack, INPUT);   // Vstup z prijimace
  
  pinMode(DIODA_INFO, OUTPUT); // Diode on board 
  pinMode(PORT_LEFT_WHEEL_FRONT, OUTPUT);  
  pinMode(PORT_RIGHT_WHEEL_FRONT, OUTPUT);
  pinMode(PORT_LEFT_WHEEL_BACK, OUTPUT);
  pinMode(PORT_RIGHT_WHEEL_BACK, OUTPUT);
  
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);  
}

double Measure_distance_ultrasonic()
// vraci vzdalenost v cm
{
 digitalWrite(trigPin, LOW);
 delayMicroseconds(2); 
 
 digitalWrite(trigPin, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin, LOW);
 long duration = pulseIn(echoPin, HIGH,28000); // TimeOut v micros 28000 4,5m a zpet
 
 //vypočítání v cm
 long distance = duration/58.2;

 //Serial.print(distance);
 //Serial.println(" cm");
 return (distance);
} 

void Ultrasonic_Get_state_front()
{

ULTRASONIC_distance = Measure_distance_ultrasonic();
if (ULTRASONIC_distance < ULTRASONIC_kriticRange)
    ULTRASONIC_State_distance = 0;
else if (ULTRASONIC_distance < ULTRASONIC_minimumRange)
    ULTRASONIC_State_distance = 1;
else if (ULTRASONIC_distance < ULTRASONIC_maximumRange)    
    ULTRASONIC_State_distance = 2;
else 
    ULTRASONIC_State_distance = 3;

//ULTRASONIC_State_distance = 2;    
}  


void read_input()
{

  Grent_Old_Value_LEFT= Old_Value_LEFT;
  Grent_Old_Value_FRONT = Old_Value_FRONT;
  Grent_Old_Value_BACK = Old_Value_BACK;
  Grent_Old_Value_RIGHT = Old_Value_RIGHT;
  Grent_Old_Value_FRONT_LEFT = Old_Value_FRONT_LEFT;
  Grent_Old_Value_FRONT_RIGHT = Old_Value_FRONT_RIGHT;
  Grent_Old_Value_BACK_LEFT  = Old_Value_BACK_LEFT;
  Grent_Old_Value_BACK_RIGHT  = Old_Value_BACK_RIGHT;
  Grent_Old_Value_Stop = Old_Value_Stop;
  
  Old_Value_LEFT= Actual_Value_LEFT;
  Old_Value_FRONT = Actual_Value_FRONT;
  Old_Value_BACK = Actual_Value_BACK;
  Old_Value_RIGHT = Actual_Value_RIGHT;
  
  Old_Value_FRONT_LEFT= Actual_Value_FRONT_LEFT;
  Old_Value_FRONT_RIGHT = Actual_Value_FRONT_RIGHT;
  Old_Value_BACK_LEFT= Actual_Value_BACK_LEFT;
  Old_Value_BACK_RIGHT = Actual_Value_BACK_RIGHT;
  
  Old_Value_Stop = Actual_Value_Stop;
  
// dopocitane hodnoty vynulujeme (dopocitac\vaji se jen pri vice stisknutich
  Actual_Value_Stop = Actual_Value_FRONT_LEFT= Actual_Value_FRONT_RIGHT = Actual_Value_BACK_LEFT= Actual_Value_BACK_RIGHT = 0;

  if (digitalRead(inputLeft) == HIGH)      Actual_Value_LEFT = 1;
  else                                      Actual_Value_LEFT = 0;
  if (digitalRead(inputFront) == HIGH)      Actual_Value_FRONT   = 1;
  else                                      Actual_Value_FRONT   = 0;
  if (digitalRead(inputBack) == HIGH)       Actual_Value_BACK    = 1;
  else                                      Actual_Value_BACK    = 0;
  if (digitalRead(inputRight) == HIGH)     Actual_Value_RIGHT = 1;
  else                                      Actual_Value_RIGHT = 0;

  if (Actual_Value_LEFT+ Actual_Value_FRONT + Actual_Value_BACK + Actual_Value_RIGHT >= 2)
  {
      // uz vime ze to bude dopocitana hodnota
      Actual_Value_LEFT= Actual_Value_FRONT = Actual_Value_BACK = Actual_Value_RIGHT = 0;
      bool m_Stop = false;
      
      if (Actual_Value_FRONT == 1) //vpred a do boku obslouyeno ostatni brzda
      {
          if (Actual_Value_BACK == 0)
              m_Stop = true;
          else
          {
              if ((Actual_Value_LEFT== 1) && (Actual_Value_RIGHT != 1))
                  Actual_Value_FRONT_LEFT= 1;
              else if ((Actual_Value_LEFT!= 1) && (Actual_Value_RIGHT == 1))
                  Actual_Value_FRONT_RIGHT = 1;
              else    
                  m_Stop = true;
          }
      }

      if (m_Stop)   Actual_Value_Stop = 1;
  }
  

// vyhodnoceni
// DOLEVA
  if (Value_LEFT== 0)
  {
      if ((Grent_Old_Value_LEFT+ Old_Value_LEFT+ Actual_Value_LEFT) >= 2)
          Value_LEFT= 1;
  }
  else if (Value_LEFT== 1)
  {
      if ((Grent_Old_Value_LEFT+ Old_Value_LEFT+ Actual_Value_LEFT) == 0)
          Value_LEFT= 0;
  }
// VPRED  
  if (Value_FRONT == 0)
  {
      if ((Grent_Old_Value_FRONT + Old_Value_FRONT + Actual_Value_FRONT) >= 2)
          Value_FRONT = 1;
  }
  else if (Value_FRONT == 1)
  {
      if ((Grent_Old_Value_FRONT + Old_Value_FRONT + Actual_Value_FRONT) == 0)
          Value_FRONT = 0;
  }

// VZAD  
  if (Value_BACK == 0)
  {
      if ((Grent_Old_Value_BACK + Old_Value_BACK + Actual_Value_FRONT) >= 2)
          Value_BACK = 1;
  }
  else if (Value_BACK == 1)
  {
      if ((Grent_Old_Value_BACK + Old_Value_BACK + Actual_Value_BACK) == 0)
          Value_BACK = 0;
  }

// DOPRAVA  
  if (Value_RIGHT == 0)
  {
      if ((Grent_Old_Value_RIGHT + Old_Value_RIGHT + Actual_Value_RIGHT) >= 2)
          Value_RIGHT = 1;
  }
  else if (Value_RIGHT == 1)
  {
      if ((Grent_Old_Value_RIGHT + Old_Value_RIGHT + Actual_Value_RIGHT) == 0)
          Value_RIGHT = 0;
  }

// VPRED DOLEVA
  if (Value_FRONT_LEFT== 0)
  {
      if ((Grent_Old_Value_FRONT_LEFT+ Old_Value_FRONT_LEFT+ Actual_Value_FRONT_LEFT) >= 2)
          Value_FRONT_LEFT= 1;
  }
  else if (Value_FRONT_LEFT== 1)
  {
      if ((Grent_Old_Value_FRONT_LEFT+ Old_Value_FRONT_LEFT+ Actual_Value_FRONT_LEFT) == 0)
          Value_FRONT_LEFT= 0;
  }
// VPRED DOPRAVA
  if (Value_FRONT_RIGHT == 0)
  {
      if ((Grent_Old_Value_FRONT_RIGHT + Old_Value_FRONT_RIGHT + Actual_Value_FRONT_RIGHT) >= 2)
          Value_FRONT_RIGHT = 1;
  }
  else if (Value_FRONT_RIGHT == 1)
  {
      if ((Grent_Old_Value_FRONT_RIGHT + Old_Value_FRONT_RIGHT + Actual_Value_FRONT_RIGHT) == 0)
          Value_FRONT_RIGHT = 0;
  }

// VZAD DOLEVA
  if (Value_BACK_LEFT== 0)
  {
      if ((Grent_Old_Value_BACK_LEFT+ Old_Value_BACK_LEFT+ Actual_Value_FRONT) >= 2)
          Value_BACK_LEFT= 1;
  }
  else if (Value_BACK_LEFT== 1)
  {
      if ((Grent_Old_Value_BACK_LEFT+ Old_Value_BACK_LEFT+ Actual_Value_BACK_LEFT) == 0)
          Value_BACK_LEFT= 0;
  }

// VZAD DOPRAVA
  if (Value_BACK_LEFT== 0)
  {
      if ((Grent_Old_Value_BACK_LEFT+ Old_Value_BACK_LEFT+ Actual_Value_BACK_LEFT) >= 2)
          Value_BACK_LEFT= 1;
  }
  else if (Value_BACK_LEFT== 1)
  {
      if ((Grent_Old_Value_BACK_LEFT+ Old_Value_BACK_LEFT+ Actual_Value_BACK_LEFT) == 0)
          Value_BACK_LEFT= 0;
  }

  if (Value_Stop == 0)
  {
      if ((Grent_Old_Value_Stop + Old_Value_Stop + Actual_Value_Stop) >= 2)
          Value_Stop = 1;
  }
  else if (Value_Stop == 1)
  {
      if ((Grent_Old_Value_Stop + Old_Value_Stop + Actual_Value_Stop) == 0)
          Value_Stop = 0;
  }
           
}


void Read_setup()
{
// 1 Vpred, 2 Doprava, 3 Doleva 4 Vzad  0 nic
  if (Value_LEFT> 0) {
      if ((Number_mode != DIRECTION_NONE) && (Number_mode != DIRECTION_STOP) && (Number_mode != DIRECTION_LEFT))
        Number_mode = DIRECTION_STOP;  // B nejprve musime zastavit!
      else  
        Number_mode = DIRECTION_LEFT;  // B
      step_separate_drive = 0;
  } else if (Value_FRONT > 0) {
      if ((Number_mode != DIRECTION_NONE) && (Number_mode != DIRECTION_STOP) && (Number_mode != DIRECTION_FRONT) && (Number_mode != DIRECTION_FRONT_LEFT)&& (Number_mode != DIRECTION_FRONT_RIGHT))
        Number_mode = DIRECTION_STOP;  // B nejprve musime zastavit!
      else  
        Number_mode = DIRECTION_FRONT;  // A
      step_separate_drive = 0;
  } else if (Value_BACK > 0) {
      if ((Number_mode != DIRECTION_NONE) && (Number_mode != DIRECTION_STOP) && (Number_mode != DIRECTION_BACK) && (Number_mode != DIRECTION_BACK_LEFT)&& (Number_mode != DIRECTION_BACK_RIGHT))
        Number_mode = DIRECTION_STOP;  // B nejprve musime zastavit!
      else  
        Number_mode = DIRECTION_BACK;  // D
      step_separate_drive = 0;
  } else if (Value_RIGHT > 0) {
      if ((Number_mode != DIRECTION_NONE) && (Number_mode != DIRECTION_STOP) && (Number_mode != DIRECTION_RIGHT))
        Number_mode = DIRECTION_STOP;  // B nejprve musime zastavit!
      else  
        Number_mode = DIRECTION_RIGHT; // C
      step_separate_drive = 0;


 //-----------------------------------------------------------
  } else if (Value_FRONT_LEFT> 0) {
      if ((Number_mode != DIRECTION_NONE) && (Number_mode != DIRECTION_STOP) && (Number_mode != DIRECTION_FRONT_LEFT) && (Number_mode != DIRECTION_FRONT)&& (Number_mode != DIRECTION_FRONT_RIGHT))
        Number_mode = DIRECTION_STOP;  // B nejprve musime zastavit!
      else  
        Number_mode = DIRECTION_FRONT_LEFT;  // B
      step_separate_drive = 0;
  } else if (Value_FRONT_RIGHT > 0) {
      if ((Number_mode != DIRECTION_NONE) && (Number_mode != DIRECTION_STOP) && (Number_mode != DIRECTION_FRONT_LEFT) && (Number_mode != DIRECTION_FRONT)&& (Number_mode != DIRECTION_FRONT_RIGHT))
        Number_mode = DIRECTION_STOP;  // B nejprve musime zastavit!
      else  
        Number_mode = DIRECTION_FRONT_RIGHT;  // A
      step_separate_drive = 0;
  } else if (Value_BACK_LEFT> 0) {
      if ((Number_mode != DIRECTION_NONE) && (Number_mode != DIRECTION_STOP) && (Number_mode != DIRECTION_BACK_LEFT) && (Number_mode != DIRECTION_BACK)&& (Number_mode != DIRECTION_BACK_RIGHT))
        Number_mode = DIRECTION_STOP;  // B nejprve musime zastavit!
      else  
        Number_mode = DIRECTION_BACK_LEFT;  // D
      step_separate_drive = 0;
  } else if (Value_BACK_RIGHT > 0) {
      if ((Number_mode != DIRECTION_NONE) && (Number_mode != DIRECTION_STOP) && (Number_mode != DIRECTION_BACK_LEFT) && (Number_mode != DIRECTION_BACK)&& (Number_mode != DIRECTION_BACK_RIGHT))
        Number_mode = DIRECTION_STOP;  // B nejprve musime zastavit!
      else  
        Number_mode = DIRECTION_BACK_RIGHT; // C
      step_separate_drive = 0;
 
  } else if (Value_Stop > 0) {
        Number_mode = DIRECTION_STOP; 
      step_separate_drive = 0;
  } else {  // Stop nebo nic
      if (step_separate_drive >= MAX_duration_separate_drive)
         Number_mode = DIRECTION_NONE; 
      else
         step_separate_drive++;   
  }

  if (Number_mode != Old_number_mode)
  {
    Old_number_mode = Number_mode;
    Serial.print("Number_mode = ");  
    switch (Number_mode)
    {
      case 1 : Serial.print(" Vpred ");  
               break;
      case 2 : Serial.print(" Doprava ");  
               break;
      case 3 : Serial.print(" Doleva ");  
               break;
      case 4 : Serial.print(" Vzad ");  
               break;
      case -1 : Serial.print(" stop ");  
               break;
      }
    Serial.print(" step_separate_drive=");
    Serial.print(step_separate_drive);
    Serial.print(" Ciclo reyimu=");
    Serial.println(Number_mode);  
  }

  //int sensorValue = analogRead(DOLEVA);
  // print out the value you read:
  //Serial.println(sensorValue);  
}

void Setup_switch_cornering()
{
  if (step_switch_cornering >= PERIODE_SWITCH_CORNERING)
  {
      switch_cornering = !switch_cornering;
      step_switch_cornering = 0;
      
      //Serial.print("Prepinac = ");  
      //Serial.println(switch_cornering);  
  }
  else
      step_switch_cornering++;
}

int Transforming_direction_movement(int par_direction)
{
  if (switching_cornering)
  {
      switch (par_direction)
      {
          case DIRECTION_STOP :    return (MOVEMENT_STOP);
          case DIRECTION_NONE :     return (MOVEMENT_NIC);
          case DIRECTION_FRONT :   return (MOVEMENT_FRONT);
          case DIRECTION_LEFT: 
                              Setup_switch_cornering();
                              if (switch_cornering)
                                  return (MOVEMENT_BACK_LEFT);
                              else    
                                  return (MOVEMENT_FRONT_LEFT);
          case DIRECTION_RIGHT : 
                              Setup_switch_cornering();
                              if (switch_cornering)
                                  return (MOVEMENT_BACK_RIGHT);
                              else    
                                  return (MOVEMENT_FRONT_RIGHT);
          case DIRECTION_BACK : return (MOVEMENT_BACK);
      }
  }
  else
  {
      switch (par_direction)
      {
          case DIRECTION_STOP    : return (MOVEMENT_STOP);
          case DIRECTION_NONE     : return (MOVEMENT_NIC);
          case DIRECTION_FRONT   : return (MOVEMENT_FRONT);
          case DIRECTION_LEFT : return (MOVEMENT_LEFT);
          case DIRECTION_RIGHT : return (MOVEMENT_RIGHT);
          case DIRECTION_BACK    : return (MOVEMENT_BACK);
      }
  }
  return (MOVEMENT_NIC);  
}

void Control_motor(int New_Valua, int *State, int PORT_FRONT, int PORT_BACK)
// Vpred kladna hodnota 
// Vzad zaporna
{

//#define  STEP_DRIVE_MOTOR 5  
// 
  // New_Valua = par_Left_wheel_front
  // State = State_Left_wheel_front
  // PORT = PORT_LEFT_WHEEL_FRONT
 if (New_Valua == MOTOR_OKAMZITA_BRZDA)
 {
     (* State) = 0;
 }
 else 
 {
    if ((* State) != New_Valua) // pokud jsou stejne je vse jiz nastaveno 
    {
        if (abs((* State) - New_Valua) < STEP_DRIVE_MOTOR)
            (* State) = New_Valua;
        else 
        {
            if ((* State) < New_Valua) 
                (* State) += STEP_DRIVE_MOTOR;
            else    
                (* State) -= STEP_DRIVE_MOTOR;
        } 

        byte m_front = 0;
        byte m_back  = 0;

        if ((* State) >= 0)
        {
          if ((* State) >= 255) m_front = 255;
          else                 m_front = (* State);
        }
        else
        {
          int m_State = -(* State);       
          if (m_State >= 255) m_back = 255;
          else               m_back = m_State;
        }

        if (m_front == 0)  // to mensi napred, aby nedoslo ke zkratu, tim ze je jeste hodnota tam, kde ma byt jiz 0 
        {
            digitalWrite(PORT_FRONT, LOW);
            
            if (m_back == 255) digitalWrite(PORT_BACK, HIGH);
            else if (m_back == 0) digitalWrite(PORT_BACK, LOW);
            else analogWrite(PORT_BACK, m_back);
        }
        else
        {
            digitalWrite(PORT_BACK, LOW);

            if (m_front == 255) digitalWrite(PORT_FRONT, HIGH);
            else if (m_front == 0) digitalWrite(PORT_FRONT, LOW);
            else analogWrite(PORT_FRONT, m_front);
        }
    }
 }

}

void Control_all_motors(int par_Left_wheel, int par_Right_wheel)
{
// Plynuly prechod na nastavenou hodnotu

    Control_motor(par_Left_wheel, &State_Left_wheel, PORT_LEFT_WHEEL_FRONT, PORT_LEFT_WHEEL_BACK);
    Control_motor(par_Right_wheel, &State_Right_wheel, PORT_RIGHT_WHEEL_FRONT, PORT_RIGHT_WHEEL_BACK);
  
}

int Permissible_speed_front(byte Pozadovana_rychlost)
{
  switch (ULTRASONIC_State_distance)
  {
      case 0:return(MOTOR_OKAMZITA_BRZDA); // rychly stop
      case 1:return(0); // pozvolny stop
      case 2:return(Pozadovana_rychlost/2); // zpomaleni
      case 3:return(Pozadovana_rychlost); // rychly stop
  }
  return (0);
}

void Make_the_move(int Smer) // 0 stat 1 Vpred 2 Doleva 3 Doprava 4 Vzad
{
//  Control_all_motors(int par_Left_wheel_front, int par_Left_wheel_back, int par_Right_wheel_front, int par_Right_wheel_back)
//Control_all_motors(-10, 255, -100, -10);
//const int MOVEMENT_NIC           = 0; 
//const int MOVEMENT_FRONT         = 1; 
//const int MOVEMENT_FRONT_LEFT = 13; 
//const int MOVEMENT_FRONT_RIGHT = 12; 
//const int MOVEMENT_BACK_LEFT  = 43; 
//const int MOVEMENT_BACK_RIGHT  = 42; 
//const int MOVEMENT_BACK          = 4;
  switch(Transforming_direction_movement(Smer))
  {
    case MOVEMENT_FRONT: // Rovne Vsechna kola
                      Control_all_motors(Permissible_speed_front(255), Permissible_speed_front(255));
                      break;
    case MOVEMENT_RIGHT: // Doprava Vsechna kola
                      Control_all_motors(255, -255);
                      break;
    case MOVEMENT_LEFT: // Doleva v
                      Control_all_motors(-255, 255);
                      break;
    case MOVEMENT_FRONT_RIGHT : // Doprava Leva kola
                      Control_all_motors(Permissible_speed_front(255), 0);
                      break;
    case MOVEMENT_FRONT_LEFT: // Doleva Prava kola
                      Control_all_motors(0, Permissible_speed_front(255));
                      break;
    case MOVEMENT_BACK_RIGHT : // Doprava Prava kola
                      Control_all_motors(0, -255);
                      break;
    case MOVEMENT_BACK_LEFT: // Doleva Leva kola
                      Control_all_motors(-255, 0);
                      break;
    case MOVEMENT_BACK: // Vzad
                      Control_all_motors(-255, -255);
                      break;
    case MOVEMENT_STOP: // zastavit
                      Control_all_motors(MOTOR_OKAMZITA_BRZDA, MOTOR_OKAMZITA_BRZDA);
                      break;
    default : // nic vypnuto MOVEMENT_NIC
                      Control_all_motors(0, 0);
  }

}

void Make_the_blink() // 0 stat 1 Vpred 2 Doleva 3 Doprava 4 Vzad
{
   
  if(( Value_LEFT+ Value_FRONT + Value_BACK + Value_RIGHT + Value_Stop + 
       Value_FRONT_LEFT+ Value_FRONT_RIGHT + Value_BACK_LEFT+ Value_BACK_RIGHT) > 0)
      digitalWrite(DIODA_INFO, HIGH);
  else    
  {
            if (Position_long_pulse < 70)
            {
              digitalWrite(DIODA_INFO, HIGH);
            }
            else if (Position_long_pulse < 105)
            {
              if (Position_long_pulse & 00001) 
                  digitalWrite(DIODA_INFO, HIGH);
              else  
                  digitalWrite(DIODA_INFO, LOW);

            }
            else 
              digitalWrite(DIODA_INFO, LOW);
  }
  Position_long_pulse++;

  if (Position_long_pulse > MAX_long_pulse)
      Position_long_pulse = 0;
  
}

// the loop function runs over and over again forever
void loop() {
 
  read_input();
  Read_setup();

  if((ULTRASONIC_State_distance == 0) || (Number_mode != DIRECTION_NONE))
      Ultrasonic_Get_state_front();
  
  Make_the_move(Number_mode);
  Make_the_blink();


  delay(28);

}

