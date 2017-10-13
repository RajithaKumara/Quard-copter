/*
  (A)                    (B)
    x                   x
      x               x
        x           x
          x       x
            x   x
              x
            x   x
          x       x
        x           x
      x               x
    x                   x 
  (D)                    (C)


|-----------|
| OUTPUT    |
|-----------|
| int A=9;  |
| int B=6;  |
| int C=5;  |
| int D=3;  |
|-----------|


             (CH3)                    (CH2)
               x                        x
               x                        x
               x                        x
     (CH1)x x x x x x(CH1)    (CH4)x x x x x x(CH4)
               x                        x
               x                        x
               x                        x
             (CH3)                    (CH2)

|--------------|
|    INPUT     |
|--------------|
| int CH3=2;   |
| int CH1=7;   |
| int CH4=10;  |
| int CH2=11;  |
|--------------|

                        valY=CH2
                          |   |
                          |   |
                          |   |
                          |   |
                      Q   | PQ|   P
                          |   |
                          |   |
                          |   |
                          |   |
                          |   |
                          |   |
__________________________|___|__________________________
                    QR    |   |   PS    valX=CH4
__________________________|___|__________________________
                          |   |
                          |   |
                          |   |
                          |   |
                      R   | RS|   S
                          |   |
                          |   |
                          |   |
                          |   |
                          |   |
                          |   |
                          |   |


*/



#include <I2Cdev.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 accelgyro;
int16_t ax, ay, az;
int minVal=265; //minimum value from GY-521 gyro sensor
int maxVal=402; //maximum value from GY-521 gyro sensor
double x;
double y;
//double z;

//declare output pins to ESCs contol signals
int A=9;
int B=6;
int C=5;
int D=3;

//declare input pins from receiver signals
int ch4=10;
int ch3=2;
int ch2=11;
int ch1=7;

//for temparary hold pulse input from 4 channels
int val_tX=0; 
int val_tY=0;
int val_tT=0;
int val_tO=0;

//for temparary store maped values of above four
int val_X=0;
int val_Y=0;
int val_T=0;
int val_O=0;

int LED=13; //declare inbuild LED
float gap=100.0;
int min_throttle=1200; //minimum throttle value supply from arduino to the ESCs
int max_throttle=1500;  //maximum throttle value supply from arduino to the ESCs
int min_throttle_trans=1200; //minimum throttle value from transmeeter, this value can be differ from one to another transmeter
int max_throttle_trans=1800; //maximum throttle value from transmeeter, this value can be differ from one to another transmeter
int tolerance=25; //transmeter signals are not static, always change. so tolerance use to that purpose
bool arming; //this is a swith for transmeter. arduino send signals to ESCs if only this variable set true. this use only for safty.
int arming_confirm;
unsigned long time;
String latitude_home,longitude_home;
double x_axis,y_axis;
int min_angle=2;
int autoBalance_constant=1;
int array_count=0;
int array_valT[]={0,0,0,0,0};

void setup() {
  pinMode(A,OUTPUT);
  pinMode(B,OUTPUT);
  pinMode(C,OUTPUT);
  pinMode(D,OUTPUT);
  pinMode(LED,OUTPUT);
  pinMode(ch4,INPUT);
  pinMode(ch3,INPUT);
  pinMode(ch2,INPUT);
  pinMode(ch1,INPUT);
  Wire.begin();
  Serial.begin(9600);
  accelgyro.initialize();
  accelgyro.getAcceleration(&ax, &ay, &az);
  arming=false; // arming for safty. until arming true, arduino neglect transmeter controls. 
  arming_confirm=1;
//  time=0;
  
  digitalWrite(LED,HIGH);
  setESC(max_throttle_trans);     //set ESC maximum value
  digitalWrite(LED,LOW);
  
  delay(800);
  
  digitalWrite(LED,HIGH);
  setESC(min_throttle);   //set ESC minumum value
  setESC(min_throttle);
  digitalWrite(LED,LOW);
  
//  time=micros();
//  time=micros()-time;
//  Serial.println(time);
  delay(1000);
  
}
void loop() {
  time=micros();
  val_tX=pulseIn(ch4,HIGH,28500); //ch4 horizontal (right stick)
  val_tT=pulseIn(ch3,HIGH,28500); //Throttle
  val_tY=pulseIn(ch2,HIGH,28500); //ch2 vertical (right stick)
  val_tO=pulseIn(ch1,HIGH,28500); //ch1 horizontal (left stick)  rotate Z axis
  time=micros()-time;
  Serial.print("time==>");
  Serial.println(time);

  val_T=map(val_tT,min_throttle_trans,max_throttle_trans,min_throttle,max_throttle); //convert transmeter signal speed range into constant range
  val_O=val_tO;
  val_X=val_tX;
  val_Y=val_tY;

  Serial.print("valX=>");
  Serial.print(val_X);
  Serial.print(" valY=>");
  Serial.print(val_Y);
  Serial.print(" valT=>");
  Serial.print(val_T);
  Serial.print(" valO=>");
  Serial.println(val_O);

  if (arming==false){   //to send singnal to ESC arm 3 times or arm 3 seconds
    if ((val_X<1330) && (val_T<1210) && (val_Y<1350) && (val_O>1680)){
      delay(1000);
      if (arming_confirm==3){
        if ((val_X<1330) && (val_T<1210) && (val_Y<1350) && (val_O>1680)){
          arming=true; 
          Serial.print(" arming====true");
        }
      }else if(arming_confirm==2){
        if ((val_X<1330) && (val_T<1210) && (val_Y<1350) && (val_O>1680)){
          arming_confirm=3;
          Serial.print(" arming==>3");
        }
      }else if(arming_confirm==1){
        arming_confirm=2;
        Serial.print(" arming==>2");
      }
    }
  }else{
    if (val_T>max_throttle){
      val_T=max_throttle;
    }
    if (val_T<min_throttle){
      val_T=min_throttle;
    }
    array_valT[array_count]=val_T;
    array_count++;
    if (array_count==5){
      array_count=0;
    }
    move4(val_X,val_Y,val_T,val_O);
  }
//  time=micros()-time;
//  Serial.print("time==>");
//  Serial.println(time);
  //delay(2000);
}

void control(int valA,int valB,int valC,int valD){
  Serial.print(" valA=>");
  Serial.print(valA);
  Serial.print(" valB=>");
  Serial.print(valB);
  Serial.print(" valC=>");
  Serial.print(valC);
  Serial.print(" valD=>");
  Serial.println(valD);
}

void move4(int valX,int valY,int valT,int valO){  //this is the main function speed contoll of four motors
  if(valO==0 && valX==0 && valY==0 && valT==1200){ //if No Signal from transmeter
    Serial.print("{");
    for (int k=0;k<5;k++){
      Serial.print(array_valT[k]);
      Serial.print(",");
    }
    Serial.print("}");
    int valT_max=0;
    int num1=0;
    int num2=0;
    int num_max=0;
    int i=0;
    int j=0;
    for (i=0;i<4;i++){
      num1=array_valT[i];
      for (j=i+1;j<5;j++){
        num2=array_valT[j];
        num_max=max(num1,num2);
        valT_max=max(valT_max,num_max);
      }
    }
    return2Home(valT_max);
    
//    control(min_throttle,min_throttle,min_throttle,min_throttle);
//    pulseOutAll25(min_throttle);
//    delayMicroseconds(min_throttle);
//    pulseOutAll25(min_throttle);
//    delayMicroseconds(min_throttle);
//    pulseOutAll25(min_throttle);
//    delayMicroseconds(min_throttle);
//    pulseOutAll25(min_throttle);
  }
  else if ((valO-1500)<=tolerance && (valO-1500)>=-tolerance){                           //left horizontal handle middle point
    
    if ((valX-1500)<=tolerance && (valX-1500)>=-tolerance && (valY-1500)<=tolerance && (valY-1500)>=-tolerance){  //AREA---> CENTER
      control(valT,valT,valT,valT);
      Serial.println("  _AREA CENTER_  ");
      pulseOutAll25(valT);
      delayMicroseconds(valT);
      pulseOutAll25(valT);
        
//      if(valT>1270){
//        autoBalance(valT);
//      }
//      else{
//        control(valT,valT,valT,valT);
//        Serial.println("  _AREA CENTER_  ");
//        pulseOutAll25(valT);
//        delayMicroseconds(valT);
//        pulseOutAll25(valT);
//      }
    }
    else if ((valY-1500)<=tolerance && (valY-1500)>=-tolerance && (valX-1500)>tolerance){       //AREA---> PS
      int dif=(valX-1500)*(gap/360.0);
      Serial.println("  _AREA PS_  ");
      control(valT+dif,valT,valT,valT+dif);               // this method use for serial print only
      controlAll(B,C,A,D,valT,dif);
    }
    else if ((valX-1500)<=tolerance && (valX-1500)>=-tolerance && (valY-1500)>tolerance){       //AREA---> PQ
      int dif=(valY-1500)*(gap/310.0);
      Serial.println("  _AREA PQ_  ");
      control(valT,valT,valT+dif,valT+dif);               // this method use for serial print only
      controlAll(A,B,C,D,valT,dif);
    }
    else if ((valY-1500)<=tolerance && (valY-1500)>=-tolerance && (valX-1500)<tolerance){       //AREA---> QR
      int dif=(1500-valX)*(gap/340.0);
      Serial.println("  _AREA QR_  ");
      control(valT,valT+dif,valT+dif,valT);               // this method use for serial print only
      controlAll(A,D,B,C,valT,dif);
    }
    else if ((valX-1500)<=tolerance && (valX-1500)>=-tolerance && (valY-1500)<tolerance){       //AREA---> RS
      int dif=(1500-valY)*(gap/320.0);
      Serial.println("  _AREA RS_  ");
      control(valT+dif,valT+dif,valT,valT);               // this method use for serial print only
      controlAll(C,D,A,B,valT,dif);
    }
    
//######################################################################################
    
    else if ((valX-1500)>tolerance && (valY-1500)>tolerance){                       //AREA---> P
      int difY=(valY-1500)*(gap/310.0);
      int difX=(valX-1500)*(gap/360.0);
      Serial.println("  _AREA P_  ");
      control(valT+difX,valT,valT+difY,valT+difX+difY);               // this method use for serial print only
      controlAll_2(B,A,C,D,valT,difX,difY);
    }
    else if ((valX-1500)<tolerance && (valY-1500)>tolerance){                       //AREA---> Q
      int difY=(valY-1500)*(gap/310.0);
      int difX=(1500-valX)*(gap/340.0);
      Serial.println("  _AREA Q_  ");
      control(valT,valT+difX,valT+difX+difY,valT+difY);               // this method use for serial print only
      controlAll_2(A,B,D,C,valT,difX,difY);
    }
    else if ((valX-1500)<tolerance && (valY-1500)<tolerance){                       //AREA---> R
      int difY=(1500-valY)*(gap/320.0);
      int difX=(1500-valX)*(gap/340.0);
      Serial.println("  _AREA R_  ");
      control(valT+difY,valT+difX+difY,valT+difX,valT);               // this method use for serial print only
      controlAll_2(D,C,A,B,valT,difX,difY);
    }
    else if ((valX-1500)>tolerance && (valY-1500)<tolerance){                       //AREA---> S
      int difY=(1500-valY)*(gap/320.0);
      int difX=(valX-1500)*(gap/360.0);
      Serial.println("  _AREA S_  ");
      control(valT+difX+difY,valT+difY,valT,valT+difX);               // this method use for serial print only
      controlAll_2(C,D,B,A,valT,difX,difY);
    }
  }
//######################################################################################
  else if((valO-1500)>tolerance){                                           //left horizontal handle ==>
    int difO=(1500-valO)*(gap/375.0);
    control(valT-(difO/2),valT+difO,valT-(difO/2),valT+difO);               // this method use for serial print only
    controlAll(A,C,B,D,valT-(difO/2),1.5*difO);  
  }
  else if((valO-1500)<tolerance){                                           //left horizontal handle <==
    int difO=(valO-1500)*(gap/370.0);
    control(valT+difO,valT-(difO/2),valT+difO,valT-(difO/2));               // this method use for serial print only
    controlAll(B,D,A,C,valT-(difO/2),1.5*difO);
  }
}

void setESC(int val){ //maximum time for this method~= 900 milli seconds, this method set ESC maximum value and minimum value
  int count=0;
  while (count<20){
    pulseOutAll25(val); 
    delayMicroseconds(val);
    count++;
  }
}

void controlAll(int first,int second,int third,int fourth,int val,int dif){ // two motors have same speed and other two also same speed but differ from first two
  int count=0;
  while (count<25){
    digitalWrite(first,HIGH);
    digitalWrite(second,HIGH);
    digitalWrite(third,HIGH);
    digitalWrite(fourth,HIGH);
    delayMicroseconds(val);
    digitalWrite(first,LOW);
    digitalWrite(second,LOW);
    delayMicroseconds(dif);
    digitalWrite(third,LOW);
    digitalWrite(fourth,LOW);

    delayMicroseconds(val);
    count++;
  }
}

void controlAll_2(int valT_pin,int difX_pin,int difY_pin,int dif_X_Y_pin,int val,int dif_X,int dif_Y){ // all four motors have different speeds
  int count=0;
  if (dif_X>dif_Y){
    while (count<25){
      digitalWrite(valT_pin,HIGH);
      digitalWrite(difX_pin,HIGH);
      digitalWrite(difY_pin,HIGH);
      digitalWrite(dif_X_Y_pin,HIGH);
      delayMicroseconds(val);
      digitalWrite(valT_pin,LOW);
      delayMicroseconds(dif_Y);
      digitalWrite(difY_pin,LOW);
      delayMicroseconds(dif_X-dif_Y);
      digitalWrite(difX_pin,LOW);
      delayMicroseconds(dif_Y);
      digitalWrite(dif_X_Y_pin,LOW);

      delayMicroseconds(val);
      count++;
    }
  }else{
    while (count<25){
      digitalWrite(valT_pin,HIGH);
      digitalWrite(difX_pin,HIGH);
      digitalWrite(difY_pin,HIGH);
      digitalWrite(dif_X_Y_pin,HIGH);
      delayMicroseconds(val);
      digitalWrite(valT_pin,LOW);
      delayMicroseconds(dif_X);
      digitalWrite(difX_pin,LOW);
      delayMicroseconds(dif_Y-dif_X);
      digitalWrite(difY_pin,LOW);
      delayMicroseconds(dif_X);
      digitalWrite(dif_X_Y_pin,LOW);

      delayMicroseconds(val);
      count++;
    }
  }
}

void pulseOutAll(int val){ //pulse out same speed for all 4 motors at one time, time for this method~= val micro seconds.
  digitalWrite(A,HIGH);
  digitalWrite(B,HIGH);
  digitalWrite(C,HIGH);
  digitalWrite(D,HIGH);
  delayMicroseconds(val);
  digitalWrite(A,LOW);
  digitalWrite(B,LOW);
  digitalWrite(C,LOW);
  digitalWrite(D,LOW);
}

void pulseOutAll25(int val){ //25 pulse out at continuously, maximum (max_throttle*2*25)micro seconds spend for this method
  int count=0;
  while (count<25){
    pulseOutAll(val); //maximum (max_throttle)micro seconds spend for this line
    delayMicroseconds(val);   //maximum (max_throttle)micro second spend for this line
    count++;
  }
}

void return2Home(int valT){
  Serial.print("~~~Return to home~~~ valT==>");
  Serial.println(valT);
  delay(2000);
  long return2Home_count=0;
  while (return2Home_count<500){
    autoBalance(valT);
    return2Home_count++;
  }
  Serial.println("~~~Check again~~~");
  val_T=pulseIn(ch3,HIGH,28000); //Throttle
  if ((1100<val_T) && (val_T<1800)){
    Serial.println("Signal detected...");
  }else{
    while (true){
      Serial.print(".");
      delay(2000);
    }
  }

//  latitude_home=7.000255;
//  longitude_home=80.004562;
//  
//  String latitude_now,longitude_now;
//  
//  if (longitude_home<longitude_now){
//    //right
//    if (latitude_home<longitude_now){
//      //top
//      
//    }else{
//      //bottom
//      
//    }
//  }else{
//    //left
//    if (latitude_home<longitude_now){
//      //top
//      
//    }else{
//      //bottom
//      
//    }
//  }
  
}

void autoBalance(int valT){
  accelgyro.getAcceleration(&ax, &ay, &az);
  int xAng = map(ax,minVal,maxVal,-90,90);
  int yAng = map(ay,minVal,maxVal,-90,90);
  int zAng = map(az,minVal,maxVal,-90,90);
  
  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  //z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  if (x<180){
    x_axis=-x;
  }else{
    x_axis=(360-x);
  }
  if (y<180){
    y_axis=-y;
  }else{
    y_axis=(360-y);
  }
  
//  int x_axis=0;//x axis from sensor
//  int y_axis=0;//y axis from sensor
  Serial.print("  autoBalance => ");
  
  if((-min_angle<=x_axis) && (x_axis<=min_angle) && (-min_angle<=y_axis) && (y_axis<=min_angle)){//centre
    //(-min_angle<=x_axis<=min_angle) && (-min_angle<=y_axis<=min_angle)
    control(valT,valT,valT,valT);
    Serial.println("  _AREA CENTER_  ");
    pulseOutAll25(valT);
    delayMicroseconds(valT);
    pulseOutAll25(valT);
  }
  else if((-min_angle<=x_axis) && (x_axis<=min_angle) && (y_axis>min_angle)){// down to PS
    Serial.println("PS");
    int dif=y_axis*autoBalance_constant;
    Serial.println("  _AREA QR_  ");
    control(valT,valT+dif,valT+dif,valT);
    controlAll(A,D,B,C,valT,dif);
  }
  else if((-min_angle<=x_axis) && (x_axis<=min_angle) && (-min_angle>y_axis)){// down to QR
    Serial.println("QR");
    int dif=-y_axis*autoBalance_constant;
    Serial.println("  _AREA PS_  ");
    control(valT+dif,valT,valT,valT+dif);
    controlAll(B,C,A,D,valT,dif);
  }
  else if((-min_angle<=y_axis) && (y_axis<=min_angle) && (x_axis>min_angle)){// down to PQ
    Serial.println("PQ");
    int dif=x_axis*autoBalance_constant;
    Serial.println("  _AREA RS_  ");
    control(valT+dif,valT+dif,valT,valT);
    controlAll(C,D,A,B,valT,dif);
  }
  else if((-min_angle<=y_axis) && (y_axis<=min_angle) && (-min_angle>x_axis)){// down to RS
    Serial.println("RS");
    int dif=-x_axis*autoBalance_constant;
    Serial.println("  _AREA PQ_  ");
    control(valT,valT,valT+dif,valT+dif);
    controlAll(A,B,C,D,valT,dif);
  }
//###########################################################################################################
  else if((x_axis>min_angle) && (y_axis>min_angle)){//down to P
    Serial.println("P");
    int difY=x_axis*autoBalance_constant;
    int difX=y_axis*autoBalance_constant;
    Serial.println("  _AREA R_  ");
    control(valT+difY,valT+difX+difY,valT+difX,valT);
    controlAll_2(D,C,A,B,valT,difX,difY);
  }
  else if((x_axis>min_angle) && (y_axis<-min_angle)){//down to Q
    Serial.println("Q");
    int difY=x_axis*autoBalance_constant;
    int difX=-y_axis*autoBalance_constant;
    Serial.println("  _AREA S_  ");
    control(valT+difX+difY,valT+difY,valT,valT+difX);
    controlAll_2(C,D,B,A,valT,difX,difY);
  }
  else if((x_axis<-min_angle) && (y_axis<-min_angle)){//down to R
    Serial.println("R");
    int difY=-x_axis*autoBalance_constant;
    int difX=-y_axis*autoBalance_constant;
    Serial.println("  _AREA P_  ");
    control(valT+difX,   valT,   valT+difY,   valT+difX+difY);
    controlAll_2(B,A,C,D,valT,difX,difY);
  }
  else if((x_axis<-min_angle) && (y_axis>min_angle)){//down to S
    Serial.println("S");
    int difY=-x_axis*autoBalance_constant;
    int difX=y_axis*autoBalance_constant;
    Serial.println("  _AREA Q_  ");
    control(valT,valT+difX,valT+difX+difY,valT+difY);
    controlAll_2(A,B,D,C,valT,difX,difY);
  }
}


