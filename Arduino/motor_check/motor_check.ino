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

String line = "", data = "", t;
char c;
boolean flag = false, sendflag = true;
int camposition[3] = {0, 90, 180};
int T, G = 0;
int gservozero1 = 105, gservozero2 = 85, armservozero = 50, camservozero = 90;
Servo cam_servo, gripper_servo1, gripper_servo2, arm_servo;

int pr[2], negh[8][2] = {{ -1, 0}, { -1, 1}, {0, 1}, {1, 1}, {1, 0}, { 1, -1} , {0, -1}, { -1, -1}};
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
  dl++;
}
void spr() //interupt function right rotary encoder//
{
  dr++;
}
void drive_motors(int Ls, int Rs)
{
  if (L == 0) rightstepmotor(abs(Rs), (Rs / abs(Rs)));
  if (R == 0) leftstepmotor(abs(Ls), (Ls / abs(Ls)));

  stepmotor(abs((Ls + Rs) / 2), (Ls / abs(Ls)), (Rs / abs(Rs)));

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
            break;
          }
      }
      i++;
    }

  }

}
void do_action()
{
  Serial.println("Start next");
}

void setup()
{
  attachInterrupt(0, spl, HIGH);
  attachInterrupt(1, spr, HIGH);

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
  motor(125, 125);
  delay(2000);

  motor(-125, -125);
  delay(1000);
}
