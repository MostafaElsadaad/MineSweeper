#include<ros.h>
#include<geometry_msgs/Vector3.h>
#include<Encoder.h>
#include <std_msgs/Int8.h>
#include "beginner_tutorials/motion.h"
#include <using_markers/nav.h>

#define Dir_Right_M  8
#define PWM_Right_M  5
#define Dir_Left_M  6
#define PWM_Left_M  7

#define RDetector 9
#define LDetector 13
#define Buzzerr 12

#define SuctionA 51
#define SuctionB 49
#define MechanismDir 47
#define MechanismPwm 45
#define MechanismUpSwitch 27
#define MechanismDownSwitch 28
#define Proximity 4

using_markers::nav msg_nav;

int speedRight = 0;
int speedLeft = 0;
bool R2Bug = false;

void motionCb( const beginner_tutorials::motion& msg) {
  // y = map(FB,0, 1000,0 ,255);
  //--------------Motion ROS--------------//

  if(msg.FB == 0) {//forward
    //  detachInterrupt( Interrupt_zero );
    //  detachInterrupt( Interrupt_one );
    digitalWrite(Dir_Right_M , HIGH);
    digitalWrite( Dir_Left_M , HIGH);
        analogWrite( PWM_Left_M , msg.RT);
        analogWrite( PWM_Right_M , msg.RT);
    
  }
  else  if(msg.FB == 1000){//backward
    //   attachInterrupt( Interrupt_zero , Right_encoder , RISING);
    //  attachInterrupt( Interrupt_one , Left_encoder , RISING);
    //  Direction = forward ;
    digitalWrite(Dir_Right_M , LOW);
    digitalWrite( Dir_Left_M , LOW);
    analogWrite( PWM_Left_M , msg.RT);
    analogWrite( PWM_Right_M , msg.RT);
  }
  else  if(msg.RL == 0){//right
    //   detachInterrupt( Interrupt_one );
    //  detachInterrupt( Interrupt_zero);
    //  Direction = right ;
    digitalWrite(Dir_Right_M , HIGH);
    digitalWrite( Dir_Left_M , LOW);
    analogWrite( PWM_Left_M , msg.RT);
    analogWrite( PWM_Right_M , msg.RT);
  }
  else if(msg.RL == 1000) {//left
    //   detachInterrupt( Interrupt_one );r
    //  detachInterrupt( Interrupt_zero);
    //  Direction = left ;
    digitalWrite(Dir_Right_M , LOW);
    digitalWrite( Dir_Left_M , HIGH);
    analogWrite( PWM_Left_M , msg.RT );
    analogWrite( PWM_Right_M , msg.RT);
  }
  else{
    // detachInterrupt( Interrupt_zero );
    // detachInterrupt( Interrupt_one );
    digitalWrite(Dir_Right_M , LOW);
    digitalWrite( Dir_Left_M , LOW);
    analogWrite( PWM_Left_M , LOW);
    analogWrite( PWM_Right_M , LOW);
  }

  //----------End of motion ROS----------//

  //--------------Mechanism--------------//
  if (msg.FB2 > 500) { //Mechanism down
      digitalWrite(MechanismDir , 1);
      analogWrite(MechanismPwm , 140);
    }
 else if (msg.FB2 < 500) { //up
    digitalWrite(MechanismDir , 0);
      analogWrite(MechanismPwm , 140);
  } 
  else analogWrite(MechanismPwm , 0);
  //if(msg.LT == 0 || msg.FB2 == 500) {
    //analogWrite(MechanismPwm , 0);
    //}
  //--------------Mechanism End--------------//

  //--------------Suction--------------//
  if (msg.X == 1) { //Suction on
    digitalWrite(SuctionA , 1);
    digitalWrite(SuctionB , 0);
  }
  if (msg.A == 1) { //Suction off
    digitalWrite(SuctionA , 0);
    digitalWrite(SuctionB , 0);
  }
  //--------------Suction End--------------//
  
}

ros::Subscriber<beginner_tutorials::motion> sub("motion", &motionCb );
ros::Publisher mine("mine_detection", &msg_nav);


ros::NodeHandle nh;
geometry_msgs::Vector3 data;
ros::Publisher pos("pos", &data);
int real_data[4];
Encoder l_encoder(2, 3);
Encoder r_encoder(20, 21);
long r_oldPosition = -999;
long l_oldPosition = -999;

bool prev = LOW;
byte flaag = 0;
unsigned long Htimer;
signed long detectionTimer;
void setup()
{
  //erial.begin(57600);
  nh.initNode();
  nh.advertise(pos);
  nh.advertise(mine);
  nh.subscribe(sub);

  
  pinMode(Dir_Right_M , OUTPUT);
  pinMode(PWM_Right_M , OUTPUT);
  pinMode(Dir_Left_M, OUTPUT);
  pinMode(PWM_Left_M , OUTPUT);
  pinMode(RDetector , INPUT);
  pinMode(LDetector , INPUT);
  pinMode(Proximity , INPUT_PULLUP);
  pinMode(Buzzerr , OUTPUT);
  digitalWrite(Buzzerr, 0);

  pinMode(MechanismDir , OUTPUT);
  pinMode(MechanismPwm , OUTPUT);
  pinMode(SuctionA , OUTPUT);
  pinMode(SuctionB , OUTPUT);
  digitalWrite(SuctionA , 0);
  digitalWrite(SuctionB , 0);
  pinMode(MechanismUpSwitch , INPUT_PULLUP);
  pinMode(MechanismDownSwitch , INPUT_PULLUP);

}
void loop()
{

  
  if (millis() - detectionTimer <= 3000) {
    if (digitalRead(Proximity) == LOW) {
      msg_nav.mine_z = 1;   //Above ground
      msg_nav.mine = 1;
      detectionTimer = -3001;
    } else {
      msg_nav.mine_z = 2;   //Under ground
      msg_nav.mine = 1;
    }
  }

  if (msg_nav.mine == 1 && millis() - detectionTimer > 3000) {
    mine.publish(&msg_nav);
    msg_nav.mine = 0;
    flaag = 0;
  }else {
      msg_nav.mine = 0;
      mine.publish(&msg_nav);
    }
    
  bool detectorReading = digitalRead(LDetector) || digitalRead(RDetector);
  if (detectorReading == HIGH && prev == LOW) {
    prev = HIGH;
  } else if (detectorReading == HIGH && prev == HIGH) {
    if (millis() - Htimer > 190) {
      digitalWrite( Buzzerr , HIGH);
      msg_nav.mine = 1;
      if(flaag == 0){  
         detectionTimer = millis();//proximity timer
         flaag = 1;
      }
    } else {
    digitalWrite( Buzzerr , LOW);
    msg_nav.mine = 0;
    }
  } else if (detectorReading == LOW && prev == LOW) {
    digitalWrite( Buzzerr , LOW);
    msg_nav.mine = 0;
    Htimer = millis();
  } else if (detectorReading == LOW && prev == HIGH) {
    digitalWrite( Buzzerr , LOW);
    msg_nav.mine = 0;
    prev = LOW;
  }
//    
//  if (detectorReading == HIGH && prev == LOW) {
//    prev = HIGH;
//  } else if (detectorReading == HIGH && prev == HIGH) {
//    if (millis() - Htimer > 200) {
//      msg_nav.mine = 1;
//      digitalWrite(Buzzerr , 1);
//      detectionTimer = millis();
//    } else {
//      msg_nav.mine = 0;
//       digitalWrite(Buzzerr , 0);
//    }
//  } else if (detectorReading == LOW && prev == LOW) {
//    Htimer = millis();
//    msg_nav.mine = 0;
//     digitalWrite(Buzzerr , 0);
//  } else if (detectorReading == LOW && prev == HIGH) {
//    prev = LOW;
//    msg_nav.mine = 0;
//     digitalWrite(Buzzerr , 0);
//  }
  long r_newPosition = r_encoder.read();
  long l_newPosition = l_encoder.read();
  if (r_newPosition != r_oldPosition) {
    r_oldPosition = r_newPosition;
    data.x = r_newPosition;
  }
  if (l_newPosition != l_oldPosition) {
    l_oldPosition = l_newPosition;
    data.y = l_newPosition;
  }
  pos.publish(&data);
  nh.spinOnce();
}
