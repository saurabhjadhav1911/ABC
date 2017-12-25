#include <Servo.h>
#define lcolor 255
#define lcolorb 1
#define condist 27
#define spt 125
#define mKp 1
#define mKd 1
#define mKi 1
#define maxs 1000
#define kdst 7
#define kdsf 7
#define kds2 15
#define r1 4//D4
#define r2 7//D5
#define sr 6//D6
#define l1 13//B0
#define l2 12//D7
#define sl 5//B1
#define consKpt 1.38
#define consKit 0.0005
#define consKdt 0.0017
#define consKpf 0.561
#define consKif 0.01
#define consKdf 0.075
#define endp 10
#define mid 3.0
#define lspdmax 236
#define rspdmax 255
#define lspdcon 140
#define rspdcon 140
#define lspdmin 160
#define rspdmin 160
#define stopdist 20
#define n 9
#define m 9
#define nk 5
#define INT 0
#define thres 500
#define midst 10
#define dla 2
#define dlb 4
#define dra 3
#define drb 8

String line = "", data = "", t;
char c;
boolean flag = false, sendflag = true;
int camposition[3] = {
  0, 90, 180};
int T, G = 0;
float V;
int gservozero1 = 105, gservozero2 = 85, armservozero = 50, camservozero = 90;
Servo cam_servo, gripper_servo1, gripper_servo2, arm_servo;

int pr[2], negh[8][2] = {
  { 
    -1, 0                                                  }
  , { 
    -1, 1                                                  }
  , {
    0, 1                                                  }
  , {
    1, 1                                                  }
  , {
    1, 0                                                  }
  , { 
    1, -1                                                  } 
  , {
    0, -1                                                  }
  , { 
    -1, -1                                                  }
};
int nx, ny, ul, ur, ts[6], sm, endX = 5, endY = 2, startX = 3 , startY = 6 , address, cpm = 10, L, R;
byte ori, co, currx, curry, minvalue, xn, ym, oi, rspd = rspdmin, lspd = lspdmin, rsp, lsp;
unsigned long int rr, ll, dr, dl, prevdist, d;

float Kp = 50, Ki = 0, Kd = 10, P, I, D, sen[6] , input, pinput, output, l, r, rad, krad, turnsen, dis = 3,  dino = 0.0, turndino = 0.0;
boolean trial, finalr, endr = false, resetconditionsflag = 0, zeroresetconditionsflag = 0, tur = 0, bc, finals = false, speedfg = true, speedtg = true, endc = false;
byte con, prevcon, pcon, conset, ee, kds = kdst, lspdt = 160, rspdt = 160;
long mean[4][2];
float distmean[4];
unsigned long pdl, pdr, condl, condr, ddl, ddr, endd = 0;


void motor(int left, int right) //drive motor//
{
  lm(left);
  rm(right);
}

void lm(int lsp)//
{
  if (lsp >= 0) {

    digitalWrite(l1, 1);
    digitalWrite(l2, 0);

    //PORTB |= 0b00000001;
    //PORTD &= 0b01111111;
    analogWrite(sl, lsp);
  }
  else {

    digitalWrite(l1, 0);
    digitalWrite(l2, 1);
    //PORTD |= 0b10000000;
    //PORTB &= 0b11111110;
    analogWrite(sl, -lsp);
  }
}
void stopp()//
{
  PORTD &= 0b01001111;
  PORTB &= 0b01111110;
}
void rm(int rsp)//
{
  if (rsp >= 0) {

    digitalWrite(r1, 1);
    digitalWrite(r2, 0);

    //PORTD |= 0b00010000;
    //PORTD &= 0b11011111;
    analogWrite(sr, rsp);
  }
  else {
    digitalWrite(r1, 0);
    digitalWrite(r2, 1);
    //PORTD |= 0b00100000;
    //PORTD &= 0b11101111;

    analogWrite(sr, -rsp);
  }
}

void stepmotor(int steps, int left, int right)
{
  for (int z = 0; z < steps; z++)
  {

    rr = dr;
    ll = dl;
    ul = 1;
    ur = 1;
    do {
      motor((left * ul), (right * ur));
      if ((dr - rr) > ( 1)) ur = 0;
      if ((dl - ll) > ( 1)) ul = 0;
    }
    while (ur || ul);
  }
}

void leftstepmotor(int stepsl, int leftm)
{
  for (int z = 0; z < stepsl; z++)
  {
    ll = dl;
    ul = 1;
    do {
      motor((leftm * ul), 0);
      if ((dl - ll) > ( 1)) ul = 0;
    }
    while ( ul);
  }
}

void rightstepmotor(int stepsr, int rightm)
{
  for (int z = 0; z < stepsr; z++)
  {
    rr = dr;
    ur = 1;
    do {
      motor(0, (rightm * ur));
      if ((dr - rr) > ( 1)) ur = 0;
    }
    while ( ur);
  }
}
void spl()//
{

  if (digitalRead(dla))
  {
    if (digitalRead(dlb))
    {
      dl++;
    }
    else {
      dl--;
    }
  }
  else {
    if (digitalRead(dlb))
    {
      dl--;
    }
    else {
      dl++;
    }
  }
}
void spr()//
{

  if (digitalRead(dra))
  {
    if (digitalRead(drb))
    {
      dr++;
    }
    else {
      dr--;
    }
  }
  else {
    if (digitalRead(drb))
    {
      dr--;
    }
    else {
      dr++;
    }
  }
}
void drive_motors(int Ls, int Rs)
{
  if (L == 0) rightstepmotor(abs(Rs), (Rs / abs(Rs)));
  if (R == 0) leftstepmotor(abs(Ls), (Ls / abs(Ls)));

  stepmotor(abs((Ls + Rs) / 2), (Ls / abs(Ls)), (Rs / abs(Rs)));

}

void sense()
{
  sen[5] = lcolorb ^ (analogRead(A0) < thres);
  sen[4] = lcolorb ^ (analogRead(A2) < thres);
  sen[3] = lcolorb ^ (analogRead(A3) < thres);
  sen[2] = lcolorb ^ (analogRead(A4) < thres);
  sen[1] = lcolorb ^ (analogRead(A5) < thres);
  sen[0] = lcolorb ^ (analogRead(A1) > 600);

  c = 0;

  bitWrite(c, (7), sen[0]);
  bitWrite(c, (6), sen[1]);
  bitWrite(c, (5), sen[2]);
  bitWrite(c, (4), sen[3]);
  bitWrite(c, (3), sen[4]);
  bitWrite(c, (2), sen[5]);
  //c ^= lcolor;
  //c &= 0b11110000;
  //Serial.print(dl);
  //Serial.print("\t");
  if ((c == 0b01111100) && (!endc)) {
    endc = 1;
    endd = dl + dr;
  }
  if (((dl + dr - endd) > 15) && (c == 0b01111100) && endc) endr = true;

  //Serial.print(dr);
  //Serial.print("\t");
  //Serial.println();
}
void move_cam(int inpd)
{
  cam_servo.write(camposition[inpd]);
}
void move_gripper(int inpd)
{
  gripper_servo1.write(gservozero1 + inpd);
  gripper_servo2.write(gservozero2 - inpd);
}
void move_arm(int inpd)
{
  arm_servo.write(armservozero + inpd);
}
void turn(byte tu) //turning at node
{ 
  Serial.println(tu);
  tur = 1;

  switch (tu) {
    case (1):
    { 
      stepmotor(kds - 1, lspdt, (-rspdt));
      sense();
      while ((bitRead(c, 7)))
      { 
        stepmotor(1, lspdt, (-rspdt));
        /*Serial.print(F("dl"));
         Serial.print(dl);
         Serial.print(F("dr"));
         Serial.println(dr);*/
        sense();
      }
      break;
    }
    case (2):
    { 
      stepmotor(2 * kds, lspdt, (-rspdt));
      sense();
      while ((bitRead(c, 7)))
      { 
        stepmotor(1, lspdt, (-rspdt));
        /*Serial.print(F("dl"));
         Serial.print(dl);
         Serial.print(F("dr"));
         Serial.println(dr);*/
        sense();
      }
      break;
    }
    case (3):
    { 
      stepmotor(3 * kds, lspdt, (-rspdt));
      sense();
      while ((bitRead(c, 7)))
      { 
        stepmotor(1, lspdt, (-rspdt));
        /*Serial.print(F("dl"));
         Serial.print(dl);
         Serial.print(F("dr"));
         Serial.println(dr);*/
        sense();
      }
      break;
    }
    case (7):
    { 
      stepmotor(kds - 1, (-lspdt), rspdt );
      sense();
      while ((bitRead(c, 7)))
      { 
        stepmotor(1, (-lspdt), rspdt );
        /*Serial.print(F("dl"));
         Serial.print(dl);
         Serial.print(F("dr"));
         Serial.println(dr);*/
        sense();
      }
      break;
    }
    case (6):
    { 
      stepmotor(2 * kds, (-lspdt), rspdt );
      sense();
      while ((bitRead(c, 7)))
      { 
        stepmotor(1, (-lspdt), rspdt );
        /*Serial.print(F("dl"));
         Serial.print(dl);
         Serial.print(F("dr"));
         Serial.println(dr);*/
        sense();
      }
      break;
    }
    case (5):
    { 
      stepmotor(3 * kds, (-lspdt), rspdt );
      sense();
      while ((bitRead(c, 7)))
      { 
        stepmotor(1, (-lspdt), rspdt );
        /*Serial.print(F("dl"));
         Serial.print(dl);
         Serial.print(F("dr"));
         Serial.println(dr);*/
        sense();
      }
      break;
    }
    case (4):
    {
      stepmotor(4 * kds, lspdt , -rspdt);
      sense();
      while ((bitRead(c, 7)))
      { 
        stepmotor(1, lspdt, (-rspdt));
        /*Serial.print(F("dl"));
         Serial.print(dl);
         Serial.print(F("dr"));
         Serial.println(dr);*/
        sense();
      }
      break;
    }
  }
  /*  if (tu != 0)
   { dis = 5;
   do
   {
   sense();
   gen();
   
   }
   while (!(bitRead(c, 2)));
   motor(0, 0);
   input = 0;
   pinput = 0;
   I = 0;
   prevdist = dl + dr;
   Serial.print(F("s");
   do
   {
   sense();
   gen();
   Serial.print(F("\t");
   Serial.print(dl);
   Serial.print(F("\t");
   Serial.print(dr);
   }
   while (((dl + dr - prevdist) < 3) || (!(bitRead(c, 2))) ); //|| (bitRead(c, 1)) || (bitRead(c, 2)) || (bitRead(c, 3))
   }*/
  Serial.print(F("s"));
  stopp();//motor(0, 0);
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  ori = sori(tu);
  tur = 0;
  dis = 3;
}
void process_string(String instruction)
{
  int i = 0;
  if (instruction[i] != '(') {
    while (i < instruction.length())
    {

      switch (instruction[i])
      {
        case ('G'):
        {
          i++;
          t = "";
          while (instruction[i] != ' ')
          {
            t += instruction[i];
            i++;
          }
          G = t.toInt();
          break;
        }
        case ('L'):
        {
          t = "";
          i++;
          while (instruction[i] != ' ')
          {
            t += instruction[i];
            i++;
          }
          L = t.toInt();
          break;
        }
        case ('R'):
        {
          t = "";
          i++;
          while (instruction[i] != ' ')
          {
            t += instruction[i];
            i++;
          }
          R = t.toInt();
          drive_motors(L, R);
          break;
        }
        case ('C'):
        {
          t = "";
          i++;
          while (instruction[i] != ' ')
          {
            t += instruction[i];
            i++;
          }

          move_cam(t.toInt());
          break;
        }
        case ('A'):
        {
          i++;
          t = "";
          while (instruction[i] != ' ')
          {
            t += instruction[i];
            i++;
          }

          move_gripper(t.toInt());
          break;
        }
        case ('J'):
        {
          i++;
          t = "";
          while (instruction[i] != ' ')
          {
            t += instruction[i];
            i++;
          }

          move_arm(t.toInt());
          break;
        }
        case ('T'):
        {
          i++;
          t = "";
          while (instruction[i] != ' ')
          {
            t += instruction[i];
            i++;
          }
          T = t.toInt();
          turn()
            break;
        }
        case ('V'):
        {
          i++;
          t = "";
          while (instruction[i] != ' ')
          {
            t += instruction[i];
            i++;
          }
          V = t.toFloat();
          break;
        }
      }
      i++;
    }
    switch(G)
    {
      /*
  
       ################## G commands ##################
       
       from master to robot 
       ...
       
       from robot to master
       
       120 - lines present L F R N
       
       ################################################
       
       */

      case(0): //      0 - start
      {
        start_flag=true;
        break;
      }
      case(1): //      1 - stop

      {
        start_flag=false;
        brake();
        break;
      }


      //        8 - led on
      case(8):
      {
        digitalWrite(led_pin,HIGH);
        break;
      }

      //        9 - led off
      case(9):
      {
        digitalWrite(led_pin,LOW);
        break;
      }


      //      10 - line follower testing mode
      case(10):
      {
        lfr_mode=true;
        break;
      }

      //      11 - test drive motors forward with max speed
      case(11):
      {
        motor(125, 125);
        delay(2000);

        motor(-125, -125);
        delay(1000);
      }

      //      21 - delay miliseconds
      case(21):
      {
        delay(V);
        break;
      }

      //22 - delay microseconds
      case(80):
      
      {
        Kp=V;
        break;
      }
      //        81 - set I
      case(81):
      {
        Ki=V;
        break;
      }
      //        82 - set D
      case(82):
      {
        Kd=V;
        break;
      }
    }
  }

}
drive_motors(L, R);
void do_action()
{
  Serial.println("Start next");
}
int digitalPinToInterrupt(int pin)
{
  int intpin;
  switch(pin)
  {
    case(2):
    {


      intpin=0;
      break;

    }
    case(3):
    {
      intpin=1;
      break;
    }
    case(21):
    {
      intpin=2;
      break;
    }
    case(20):
    {
      intpin=3;
      break;
    }
    case(19):
    {
      intpin=4;
      break;
    }
    case(18):
    {
      intpin=5;
      break;
    }
  }
  return(intpin);
}
void setup()
{
  attachInterrupt(digitalPinToInterrupt(dla), spl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(dra), spr, CHANGE);

  Serial.begin(115200);

  //######################## servopins ############################
  cam_servo.attach(8);
  gripper_servo1.attach(11);
  gripper_servo2.attach(10);
  arm_servo.attach(9);
  //####################################################
  //pidcheck();
  //generate();
  //floodfill();
  //mazeprint();
  //trialmaze();

  //floodfill();
  //mazeprint();
  // trialmaze();
  /*

   pinMode(12, INPUT_PULLUP);
   pinMode(endp, INPUT_PULLUP);
   pinMode(13, 1);
   pinMode(l1, 1);
   pinMode(l2, 1);
   pinMode(sl, 1);
   pinMode(r1, 1);
   pinMode(r2, 1);
   pinMode(sr, 1);
   //setmode();
   while (digitalRead(12) == 1)
   {}
   //resetconditions();
   delay(500);
   digitalWrite(13, 0);
   digitalWrite(13, 1);
   digitalWrite(13, 0);
   //stepmotor(1, lspd, rspd)
   //motor(lspd, rspd);
   //delay(2000);
   //pidcheck();
   */
  cam_servo.write(camservozero);
  gripper_servo1.write(gservozero1);
  gripper_servo2.write(gservozero2);
  arm_servo.write(armservozero);
}
void loop()
{
  Serial.print(dl);
  Serial.print("\t");
  Serial.println(dr);
}

























