// Authors: 
// Last Updated: 8-21-17
// Program Name: HelenBrains
// Intended Purpose: Take input from the sensors of a wheeled robot to maneuver through an obstacle course to and from a goal state, and then up an incline
//
// Noted Software Bugs: 
//    1. Unable to maneuver around dead ends with current configuration. 
//    2. Unable to correct for drifting as a result of Hardware Issues with current configuration
//
// Noted Hardware Issues: 
//    1. Tires tend to slip/become loose, leading to inconsistent performance. Must look into getting new tires. 
//    2. Ultrasonic Sensor prone to innacurracies if other Ultrasonic sensors are active around it. 
//    

#include <motordriver_4wd.h>
#include <seeed_pwm.h>
#include <ChainableLED.h>

int counter = 0;
//-------- PIE constants

const double PIE    = 3.14159265;
const double PIE_O2 = PIE/2.0;
const double PIE2 = PIE*2.0;

//-------- LED crap

#define NUM_LEDS  1
ChainableLED leds(A0, A1, NUM_LEDS);

//-------- Ultrasonic Sensor carp

const int pingpin = 11; 

//-------- motor control 

void TurnLeft90();
void TurnRight90();
void Straight( int speed, int dirn );

//-------- dead reckoning 

// ticks per rotation
#define TPR 72
 
// robot measurements (mm)
#define RW    42.5  // radius wheel
#define D     158.0

// robot config variables
double x = 100.0, y = 100.0, dx = 0.0, dy = 0.0;
double theta =  PI/2.0;

// encoder variables
volatile long left_encoder_count = 0, right_encoder_count = 0;   
int left_dirn = 1, right_dirn = 1;


//-------- robot state 

enum {WANDER, GOHOME, INCLINE, DISCO} state;

enum {FWD, REV, BACK} wander_state;

enum {R, L} incline_state;

enum {r, l,rr,ll} back_state;

//-------- Grid state
// -1 = unvisited
// 0 = visited and empty
// 99 = visited and occupied
char grid[5][5];

//-------- model of environment 

double LEFT = 0.0;
double RIGHT = 1480.0;
double BOTTOM = 0.0;
double TOP = 1480.0;


//-------- servo stuffs
int neckPos = 0;
int servoPin=13;
long dir=-1;
int i=0;

void Pan(int pos){
  for(int i=0; i<100;i++)
  {
    digitalWrite(servoPin,HIGH);
    delayMicroseconds(pos);
    digitalWrite(servoPin,LOW);
    delayMicroseconds(20000-pos);
  }
}

//======================================================================================
// setup
//======================================================================================
void setup()
{
    digitalWrite(A2, INPUT_PULLUP);
    incline_state = R;
    back_state = r;
    Serial.begin(9600);
    for(int j = 0; j<5; j++ ){
      for(int k = 0; k<5; k++){
        grid[j][k] = '?';
      }
    }
    grid[0][0] = 'o';
    leds.init();
    MOTOR.init();
    Pan(1300);
    attachInterrupt(0, LeftEncoder, CHANGE);
    attachInterrupt(1, RightEncoder, CHANGE);
        
    // go straight
    leds.setColorRGB(0, 100, 200, 0);  // green
    delay(50);
    Straight( 20, 1 ); 
    
    state = WANDER;
    wander_state = FWD;
    
    //Serial.begin(9600);
}

//======================================================================================
// Loop
//======================================================================================
void loop()
{
  switch(state) {
    case WANDER:
      Wander();
      break;
    case GOHOME:
      GoHomeHelen();
      break;
    case INCLINE:
      MovingOnUp();
      break;
    case DISCO:
      BreakItDown();
    default:
      Serial.print("Oops\n");
  }
}


//======================================================================================
// TurnLeft90
//======================================================================================
void
TurnLeft90()
{
    right_encoder_count = left_encoder_count = 0;
    
    left_dirn = -1; right_dirn = 1;
    MOTOR.setSpeedDir1(35, DIRR); MOTOR.setSpeedDir2(35, DIRR); 
    while (right_encoder_count < 64)
    {
      delayMicroseconds(1);
    }

    MOTOR.setSpeedDir1(0, DIRF); MOTOR.setSpeedDir2(0, DIRR);
    theta = fmod(theta + PIE_O2, PIE2);
    delay(280);
}

//======================================================================================
// TurnRight90
// dirn is 1 for right, -1 for left
//======================================================================================
void
TurnRight90()
{
    right_encoder_count = left_encoder_count = 0;
    
    left_dirn = 1; right_dirn = -1;
    MOTOR.setSpeedDir1(35, DIRF); MOTOR.setSpeedDir2(35, DIRF); 
    while (left_encoder_count < 71)
    {
      delayMicroseconds(1);
    }

    MOTOR.setSpeedDir1(0, DIRF); MOTOR.setSpeedDir2(0, DIRR);
    theta = fmod(theta - PIE_O2, PIE2);
    delay(280);
}

//======================================================================================
// Straight
// dirn is 1 for fwd, -1 for bwd
//======================================================================================
void
Straight( int speed, int dirn )
{
    //---- setup encoder variables
    left_dirn = dirn; right_dirn = dirn;
    
    if (speed == 0)       //-- stop
    {
      MOTOR.setSpeedDir1(0, DIRF); MOTOR.setSpeedDir2(0, DIRR);   return;
    }
    else if (dirn == 1)   //-- fwd
    {
      MOTOR.setSpeedDir1(speed-0.1*speed, DIRF); MOTOR.setSpeedDir2(speed, DIRR); 
    }
    else                  //-- bwd
    {
      MOTOR.setSpeedDir1(speed-0.1*speed, DIRR); MOTOR.setSpeedDir2(speed, DIRF); 
    }
}

//======================================================================================
// Ping
// dirn is 1 for fwd, -1 for bwd
//======================================================================================
int Ping(int pingPin)
{
  long duration, cm;
  pinMode(pingPin,OUTPUT);
  digitalWrite(pingPin,LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin,HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin,LOW);
  pinMode(pingPin,INPUT);
  duration = pulseIn(pingPin,HIGH);
  cm = duration/29.0/2.0;
  delay(10);
  return cm;
}


//======================================================================================
// Interrupt Service Routines for encoders
//======================================================================================
void LeftEncoder()
{
  left_encoder_count = left_encoder_count + left_dirn;
}

void RightEncoder()
{
  right_encoder_count = right_encoder_count + right_dirn;
}

//======================================================================================
// Print Grid
//======================================================================================
void printGrid() {
  for(int i=0; i<5; i++){
    for(int j=0; j<5; j++){
      Serial.print(grid[i][j]); Serial.print(" ");
    }
  Serial.print("\n");
  }
}

//======================================================================================
// Wander
//======================================================================================
void Wander() {
  // If we have reached our goal state, DISCO FEVER~
  if(x >1300 && y > 1250) {
    printGrid();
    delay(50);
    Straight(0, 0);
    wander_state = FWD;
    state = GOHOME;
    int dance = 0;
    while(dance<50) {
      dance++;
      leds.setColorRGB(0, random(256), random(256), random(256));
      leds.setColorHSB(0, random(1, 1000)/1000.0, random(1, 1000)/1000.0, random(1, 1000)/1000.0);
      delay(100);
    } 
    TurnRight90();
    //delay(500);
    TurnRight90();
    //delay(500);
    Straight(20,1);
  }

  // Checking for obstructions
  int dist_cm = Ping(pingpin);
  //Serial.println(dist_cm);

  // If Obstruction detected, determine where the obstruction is and mark it on the internal grid.
  // Once obstruction is marked, determine where to turn based on where the goal is, where the robot is, and the current direction of travel
  // Once direction to turn is determined, turn in that direction, and start going forward once you have turned 90 degrees


  if(dist_cm < 12)
  {
    
    int gx = (int) (x + PIE * RW * cos(theta) * ((double)((left_encoder_count+36) + (right_encoder_count+36)) / TPR))/300;
    int gy = (int) (y + PIE * RW * sin(theta) * ((double)((left_encoder_count+36) + (right_encoder_count+36)) / TPR))/300;\

    if(gx<5 && gy <5 && gx>=0 && gy>=0){
      grid[gx][gy]='X';
    }
    
    Straight(0, 0);
    leds.setColorRGB(0, 255, 0, 0);
    leds.setColorHSB(0, 0.5,  1.0, 0.5);
    delay(100);

    gx = (int)x/300;
    gy = (int)y/300;


    switch((int)theta){
      case 0:
        if(x>y){
          if(x<800){
            TurnRight90();
            back_state = r;
            Straight(20,1);
          }
          else{
            TurnLeft90();
            back_state = l;
            Straight(20,1);
          }
        }
        else if(y>x){
          if(y<1200){
            TurnLeft90();
            back_state = l;
            Straight(20,1);
          }
          else{
            TurnRight90();
            back_state = r;
            Straight(20,1);
          }
        }
        else{
          TurnLeft90();
          back_state = r;
          Straight(20,1);
        }
        break;
      case (int)PIE_O2:
        if(x>y){
          if(x>1300){
            TurnLeft90();
            back_state = l;
            Straight(20,1);
          }
          else{
            TurnRight90();
            back_state = r;
            Straight(20,1);
          }
        }
        else if(y>x){
          if(y>1200){
            TurnLeft90();
            back_state = l;
            Straight(20,1);
          }
          else{
            TurnRight90();
            back_state = r;
            Straight(20,1);
          }
        }
        else{
          TurnLeft90();
          back_state = l;
          Straight(20,1);
        }
        break;
      case (int)PIE:
        if(x>y){
          if(x<1200){
            TurnRight90();
            back_state = r;
            Straight(20,1);
          }
          else{
            TurnRight90();
            TurnRight90();
            back_state = rr;
            Straight(20,1);
          }
        }
        else{
          TurnRight90();
          back_state = r;
          Straight(20,1);
        }
        break;
      case (int)(PIE_O2*-1):
        if(y>x){
          if(y<1200){
            TurnLeft90();
            back_state = l;
            Straight(20,1);
          }
          else{
            TurnLeft90();
            TurnLeft90();
            back_state = ll;
            Straight(20,1);
          }
        }
        else{
          TurnLeft90();
          back_state = l;
          Straight(20,1);
        }
        break;
      default:
        break;
    }
    leds.setColorRGB(0,0,100,0);
  }

  delay(50);
  
  //---- update robot config (x,y,theta)
  dx = PIE * RW * cos(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  x = x + dx;
  
  dy = PIE * RW * sin(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  y = y + dy;

  // mark the grid spot as empty if no obstruction is detected
  if(dist_cm>12){
    grid[(int)(x/300)][(int)(y/300)] = 'o';
  }
  
  //Every 20 loops, print the current state of the internal grid map
  counter++;
  if(counter % 20 == 0)
    printGrid();
  
  //reset encoder counts to avoid incrementing before an actual tick from the wheel occurs
  right_encoder_count = left_encoder_count = 0;
  
  //---- a simple two-state behavior to stay in the box
  
  if ((wander_state == FWD) && (x >= RIGHT || x <= LEFT || y >= TOP || y <= BOTTOM))
  {
    //---- stop
    Straight( 0, 0 );    
    delay(100);

    //---- back up
    leds.setColorRGB(0, 100, 0, 0);  // red
    Straight( 15, -1 );    
    delay(650);

    //---- update state
    wander_state = REV;    
  }
  else
  if ((wander_state == REV) && (x < RIGHT && x > LEFT && y < TOP && y > BOTTOM))
  {
    //---- stop
    Straight( 0, 0 );    

    //---- turn right or left 90
    leds.setColorRGB(0, 0, 0, 100);

    if(theta <= PIE && theta >= PIE_O2){
      TurnRight90();
      Straight(20,1);
    }
    else{
      TurnLeft90();
      Straight(20,1);
    }
    //---- update robot config (theta)
    //theta  = fmod(theta - PIE_O2, PIE2);
    delay(100);

    //---- go straight
    leds.setColorRGB(0, 0, 100, 0);  // green
    //Straight( 20, 1 );   

    //---- update state
    wander_state = FWD;    
  }
  dist_cm = 100000;
}

//======================================================================================
// GO Home
//======================================================================================
void GoHomeHelen() {

  // If we have reached our goal state, DISCO FEVER~ (flash the leds with random colors)
  // Else, continue with the loop
  if(x <=200 && y <= 200) {
    delay(150);
    Straight(0, 0);
    state = INCLINE;
    int dance = 0;
    while(dance<50) {
      dance++;
      leds.setColorRGB(0, random(256), random(256), random(256));
      leds.setColorHSB(0, random(1, 1000)/1000.0, random(1, 1000)/1000.0, random(1, 1000)/1000.0);
      delay(100);
    } 

    int val = analogRead(A2);

    //Turn toward the given incline
    while(val>75){
    MOTOR.setSpeedDir1(15, DIRF);
    MOTOR.setSpeedDir2(15, DIRF);
    val = analogRead(A2);
    }
    
    delay(100);
    
    //stop 
    Straight(0,0);
    leds.setColorRGB(0,125,235,84);
    delay(2000);
    
    //head straight 
    leds.setColorRGB(0,12,84,235);
    Straight(16,1);
    delay(1000);
    
    //start drfiting to the left
    MOTOR.setSpeedDir1(8, DIRF);
    MOTOR.setSpeedDir2(15, DIRR);
    
    wander_state = BACK;
    left_encoder_count = 0;
  }
  

  dx = PIE * RW * cos(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  x = x + dx;
  
  dy = PIE * RW * sin(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  y = y + dy;
  
  right_encoder_count = left_encoder_count = 0;
  
  // Check if grid spot in front of the robot is untraveled
  int gx = (int) (x + PIE * RW * cos(theta) * ((double)((left_encoder_count+30) + (right_encoder_count+30)) / TPR))/300;
  int gy = (int) (y + PIE * RW * sin(theta) * ((double)((left_encoder_count+30) + (right_encoder_count+30)) / TPR))/300;
    
  if(gx<5 && gy <5 && gx>=0 && gy>=0)
  {
    if(grid[gx][gy]=='?'){
      Straight(0, 0);
      leds.setColorRGB(0, 255, 0, 0);
      leds.setColorHSB(0, 0.5,  1.0, 0.5);
      delay(100);
      if(theta <= 0 && theta >= -PIE_O2){
        TurnRight90();
        Straight(20,1);
      }
      else{
        TurnLeft90();
        Straight(20,1);
      }
    }
  }
  
  delay(50);
  
  //---- update robot config (x,y,theta)
  dx = PIE * RW * cos(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  x = x + dx;
  
  dy = PIE * RW * sin(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  y = y + dy;
  
  
  //print the internal grid map once every 20 loops
  counter++;
  if(counter % 20 == 0)
    printGrid();
  
  right_encoder_count = left_encoder_count = 0;
  
  //---- a simple two-state behavior to stay in the box
  
  if ((wander_state == FWD) && (x >= RIGHT || x <= LEFT || y >= TOP || y <= 165.0))
  {
    //---- stop
    Straight( 0, 0 );    
    delay(100);

    //---- back up
    leds.setColorRGB(0, 100, 0, 0);  // red
    Straight( 15, -1 );    
    delay(500);

    //---- update state
    wander_state = REV;    
  }
  else
  if ((wander_state == REV) && (x < RIGHT && x > LEFT && y < TOP && y > 165.0))
  {
    //---- stop
    Straight( 0, 0 );    

    //---- turn right 90
    leds.setColorRGB(0, 0, 0, 100);
    if(theta <= 0 && theta >= -PIE_O2){
        TurnRight90();
        Straight(20,1);
      }
      else{
        TurnLeft90();
        Straight(20,1);
      }
    //---- update robot config (theta)
    //theta  = fmod(theta - PIE_O2, PIE2);
    delay(100);

    //---- go straight
    leds.setColorRGB(0, 0, 100, 0);  // green
    //Straight( 25, 1 );   

    //---- update state
    wander_state = FWD;    
  }
}


//==========================================
// Incline
//==========================================
void MovingOnUp(){
  //rotate right until we sense a light powerful enough to move toward it
  int val = analogRead(A2);
  Serial.println(val);
  if (val > 75)
  {
    if (incline_state == L)
    {
      MOTOR.setSpeedDir1(15, DIRF);
      MOTOR.setSpeedDir2(8, DIRR);
      incline_state = R;
      delay(1000);
    }
    else
    if (incline_state == R)
    {
      MOTOR.setSpeedDir1(8, DIRF);
      MOTOR.setSpeedDir2(15, DIRR);
      incline_state = L;
      delay(1000);      
    }
  }
  if(left_encoder_count >= 375){
    state = DISCO;
    Straight(0,0);
    delay(5000);
  }
}

//========================================================
// Dance Time!!!!
//========================================================
void BreakItDown(){
  leds.setColorRGB(0,random(255),random(255),random(255));
  leds.setColorHSB(0,random(1, 1000)/1000.0, random(1, 1000)/1000.0, random(1, 1000)/1000.0);
}

