#include<SoftwareSerial.h>
#include <QMC5883LCompass.h>// QMC5883L Compass Library
#include <Wire.h>// I2C Library

//bluetooth
SoftwareSerial BT(11 , 10);//11接TXD , 10接RXD
QMC5883LCompass compass;


//pins for motor
const int PWMA = 2;
const int AIN1 = 15;
const int AIN2 = 14;
//add pin
const int BIN1 = 16;
const int BIN2 = 17;
const int PWMB = 3;

//pins for HSC SR04
const int trigPin_front = 19;
const int echoPin_front = 18;
const int trigPin_left = 22;
const int echoPin_left = 23;
const int trigPin_right = 26;
const int echoPin_right = 27;

const int thres = 15;
const int N = 10;

char intmap[N][N] = {  //internal map, stored inside arduino
    {'?', '?', '?', '?', '?', '?', '?', '?', '?', '?'},
    {'?', '?', '?', '?', '?', '?', '?', '?', '?', '?'},
    {'?', '?', '?', '?', '?', '?', '?', '?', '?', '?'},
    {'?', '?', '?', '?', '?', '?', '?', '?', '?', '?'},
    {'?', '?', '?', '?', '?', '?', '?', '?', '?', '?'},
    {'?', '?', '?', '?', '?', '?', '?', '?', '?', '?'},
    {'?', '?', '?', '?', '?', '?', '?', '?', '?', '?'},
    {'?', '?', '?', '?', '?', '?', '?', '?', '?', '?'},
    {'?', '?', '?', '?', '?', '?', '?', '?', '?', '?'},
    {'?', '?', '?', '?', '?', '?', '?', '?', '?', '?'},
};

class vec{//self made vector class
    public:
    int x, y;
};

void forward(){
  //go foward
  unsigned long start = millis();
  compass.read();
  int init_dir = compass.getAzimuth();
  while((millis() - start) < 800){
    compass.read();
    double error = (compass.getAzimuth()- init_dir); //positive --> turn right , neg --> turn left
    double powerCorrection = error * 10;//scale it up to make a significant impact to motor rpm
    int basespeed = 150; //a base speed to reduce "jerkiness"
    double vL = basespeed + powerCorrection;
    double vR = basespeed - powerCorrection;
    //makes sure that vR and vL doesn't exceed max value
    //vL = (int)((double)vL*0.8);//compensation
    //vR = (int)((double)vR*1.3);//compensation
    if(vL > 255) vL = 255;
    if(vL < -255) vL = -255;
    if(vR > 255) vR = 255;
    if(vR < -255) vR = -255;
    motorWriting((int)vL , (int)vR);//anologWrite can only take in integer type
  }
  motorWriting(0,0);  
}

void right(){
  compass.read();
  int start_az = compass.getAzimuth();
  //right
  int caz = compass.getAzimuth();
  motorWriting(70, -70);//turn right
  while((caz - start_az) <= 90){;//turn until 90deg(azimuth is oriented clockwise)
    compass.read();
    caz = compass.getAzimuth();
    BT.println(caz);
    if(caz <= 90) caz += 360;//dealing with errors from branch cut
  }
  forward();
  motorWriting(0,0);
}

void left(){
  compass.read();
  int start_az = compass.getAzimuth();
  //left
  int caz = compass.getAzimuth();  
  motorWriting(-70, 70);//turn right
  while((caz - start_az) >= -90){;//turn until 90deg
    compass.read();
    caz = compass.getAzimuth();
    if(caz >= 270) caz -= 360;//dealing with errors from branch cut

  }
  forward();
  motorWriting(0,0);
}

void backward(){
  int start_az = compass.getAzimuth();
  //back
  int caz = compass.getAzimuth();
  motorWriting(-70, 70);
  while((caz - start_az) >= -180){;//turn until 90deg
  caz = compass.getAzimuth();
  if(caz >= 180) caz -= 360;//dealing with errors from branch cut
  }
  forward();
  motorWriting(0,0);
}

class car{
    public:
    car():x(1), y(1){//initialization
        ori.x = 1;
        ori.y = 0;
    }
    int x, y;//position of car
    vec ori;//orientation of car
    void autoctrl(int df, int dl, int dr){
        //decides which way to turn according to its surroundings
        int ty = ori.y;
        int tx = ori.x; //store the old orientation        
        
        int DoF = 3;
        if(dr<=thres) DoF--;
        if(df<=thres) DoF--;
        if(dl<=thres) DoF--;
        if(DoF == 3){//continue foward
            ori.y = ty;
            ori.x = tx;
            forward();
        }
        else if(DoF == 0){//turn backward
            ori.y = -ty;
            ori.x = -tx;
            backward();
        }
        else if(intmap[y-ori.x][x+ori.y] == '?'){//turn left uncharted area
            ori.y = -tx;
            ori.x = ty;
            left();
        }
        else if(intmap[y+ori.x][x-ori.y] == '?'){//turn right uncharted area
            ori.y = tx;
            ori.x = -ty;
            right();
        }
        else if(intmap[y+ori.y][x+ori.x] == '?'){//go foward uncharted area
            ori.y = ty;
            ori.x = tx;
            forward();
        }
        else{
            int r =random(0,DoF);// r = 0 or 1;
            //cout<<"DoF = "<<DoF<<endl;
            //cout<<"r = "<<r<<endl;
            if(df >= thres){
                if(r == 0){
                    ori.y = ty;
                    ori.x = tx; 
                    forward();                   
                }
                r--;
            }
            if(dl >= thres){
                if(r == 0){
                    ori.y = -tx;
                    ori.x = ty;
                    left();
                }
                r--;
            }
            if(dr >= thres){
                if(r == 0){
                    ori.y = tx;
                    ori.x = -ty;
                    right();
                }
                r--;
            }

        }

        y += ori.y;
        x += ori.x;
        
        //add motor function
  
        
        //goes in designated direction
        x = (x < 0)? 0 : x;//boundary check
        x = (x > N-1)? N-1 : x;
        y = (y < 0)? 0 : y;
        y = (y > N-1)? N-1 : y;
    }
};
car mycar;

void sonar(int& df, int& dl, int& dr, int y, int x){
    df = getdist(trigPin_front, echoPin_front);
    dr = getdist(trigPin_right, echoPin_right);
    dl = getdist(trigPin_left, echoPin_left);
    if(df <= thres){//in front of car
        intmap[y+mycar.ori.y][x+mycar.ori.x] = 'X';
        //map and intmap are global, so passing in as argument was not necessary
    }
    //else intmap[y+mycar.ori.y][x+mycar.ori.x] = ' ';
    if(dl <= thres){//left side of car
        intmap[y-mycar.ori.x][x+mycar.ori.y] = 'X';
    }
    //else intmap[y-mycar.ori.x][x+mycar.ori.y] = ' ';
    if(dr <= thres){//right side of car
        intmap[y+mycar.ori.x][x-mycar.ori.y] = 'X';
    }
    //else intmap[y+mycar.ori.x][x-mycar.ori.y] = ' ';
    intmap[y][x] = ' ';
}

void setup() {
  // put your setup code here, to run once:
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode (AIN2 , OUTPUT);
  pinMode (BIN1 , OUTPUT);
  pinMode (BIN2 , OUTPUT);
  pinMode (PWMA , OUTPUT);
  pinMode (PWMB , OUTPUT);
  //pinmodes for HCSR04
  pinMode(trigPin_left , OUTPUT);
  pinMode(echoPin_left , INPUT);
  pinMode(trigPin_right , OUTPUT);
  pinMode(echoPin_right , INPUT);
  pinMode(trigPin_front , OUTPUT);
  pinMode(echoPin_front , INPUT);
  //RFID and BT
  Serial.begin(9600);
  BT.begin(9600);
  Wire.begin();
  compass.init();//initialize compass
  compass.setCalibration(-3890, 0, -170, 1000, -8047, 0);;//calibration
  compass.setSmoothing(10, true);//smooth the values to provide consistent readings
}

void loop() {

  right();
  delay(1000);
  forward();
  delay(1000);
  
  /*
  static bool finished = false;
  if (finished) return;
  // put your main code here, to run repeatedly:
  int df, dl, dr;
  for (int i=0;i<10;++i){sonar(df, dl, dr, mycar.y, mycar.x);}//get the "distance" on each side of the car
  mycar.autoctrl(df, dl, dr);//controls the next step the car will take
  
  if(BT.available()){
    BT.read();//take the input out of buffer
    Serial.println("transmitting map");
    BT.println(N);//give the dimension of the map
    for(int i=0;i<N;++i){
      for(int j=0;j<N;++j){        
        BT.print('[');
        BT.print(intmap[i][j]);   
        BT.print(']');
      }
      BT.println();//formatting output
    }
    BT.println();
    finished = true;
  }
  */
  
}


int getdist(int trig , int echo){
  long duration;
  int distance;
  // Clears the trigPin
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echo, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // returns distance
  return distance;
}

void motorWriting(int vL , int vR){
  //acquire the magnitude of vL and vR
  int absvL = vL;
  int absvR = vR;
  if(vL < 0){
    absvL = -vL;
  }
  if(vR < 0){
    absvR = -vR;
  }
  //magniture decides speed
  analogWrite(PWMA , absvR);
  analogWrite(PWMB , absvL);
  //uses the sign of vL and rL to determine the direction 
  if(vL < 0){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else{
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  if(vR < 0){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else{
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

}
