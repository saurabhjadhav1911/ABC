#include <Servo.h>
#define lcolor 255
#define lcolorb 1
#define condist 27
#define spt 125
#define mKp 1
#define mKd 1
#define mKi 1
#define maxs 1000
#define kdst 15
#define kdsf 7
#define kds2 15
#define r1 5//D4
#define r2 7//D5
#define sr 6//D6
#define l1 2//B0
#define l2 4//D7
#define sl 3//B1
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
#define thres 300
#define midst 10
#define dla 18
#define dlb 19
#define dra 20
#define drb 21
#define led_pin 13
#define lfr_mode 10
#define sensor_mode 8
#define NODE 9
#define INV 10
#define PNT 11


String line = "", data = "", t;
char cc;
byte c;
boolean flag = false, sendflag = true, start_flag = false;
int camposition[3] = {
  0, 90, 180
};
int T, G = 0;
float V;
int gservozero1 = 105, gservozero2 = 85, armservozero = 50, camservozero = 90, mode = 0;
Servo cam_servo, gripper_servo1, gripper_servo2, arm_servo;

int pr[2], negh[8][2] = {
  {
    -1, 0
  }
  , {
    -1, 1
  }
  , {
    0, 1
  }
  , {
    1, 1
  }
  , {
    1, 0
  }
  , {
    1, -1
  }
  , {
    0, -1
  }
  , {
    -1, -1
  }
};
int nx, ny, ul, ur, ts[6], sm, endX = 5, endY = 2, startX = 3 , startY = 6 , address, cpm = 10, L, R;
byte ori, co, currx, curry, minvalue, xn, ym, oi, rspd = 255, lspd = 255, lspdprev, rspdprev, rsp, lsp;
unsigned long int rr, ll, dr, dl, prevdist;

float Kp = 50, Ki = 0, Kd = 10, P, I, D, sen[6] , psen[6], input, pinput, output, l, r, rad, krad, turnsen, dis = 3,  dino = 0.0, turndino = 0.0;
boolean trial, finalr, endr = false, resetconditionsflag = 0, zeroresetconditionsflag = 0, tur = 0, bc, finals = false, speedfg = true, speedtg = true, endc = false;
byte con, prevcon, pcon, conset, ee, kds = kdst, lspdt = 160, rspdt = 160, kdss = kdst;
float distmean[6][4];
unsigned long pdl, pdr, condl, condr, ddl[6], ddr[6], endd = 0;


void motor(int left, int right) //drive motor//
{
  lm(left);
  rm(right);
  Serial3.print("motor drive");
  Serial3.print(left);
  Serial3.print(right);
}

void lm(int lsp)//
{
  if (lsp > 0) {

    digitalWrite(l1, 1);
    digitalWrite(l2, 0);

    //PORTB |= 0b00000001;
    //PORTD &= 0b01111111;
    analogWrite(sl, lsp);
  }
  if (lsp < 0)
  { digitalWrite(l1, 0);
    digitalWrite(l2, 1);
    //PORTD |= 0b10000000;
    //PORTB &= 0b11111110;
    analogWrite(sl, -lsp);
  }
  if (lsp == 0)
  { digitalWrite(l1, 0);
    digitalWrite(l2, 0);
    //PORTD |= 0b10000000;
    //PORTB &= 0b11111110;
  }
}
void stopp()//
{
  motor(0, 0);
}
void rm(int rsp)//
{
  if (rsp > 0) {

    digitalWrite(r1, 1);
    digitalWrite(r2, 0);

    //PORTD |= 0b00010000;
    //PORTD &= 0b11011111;
    analogWrite(sr, rsp);
  }
  if (rsp < 0) {
    digitalWrite(r1, 0);
    digitalWrite(r2, 1);
    //PORTD |= 0b00100000;
    //PORTD &= 0b11101111;

    analogWrite(sr, -rsp);
  }
  if (rsp == 0) {
    digitalWrite(r1, 0);
    digitalWrite(r2, 0);
    //PORTD |= 0b00100000;
    //PORTD &= 0b11101111;

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

      int lsq = (left * ul);
      int rsq = (right * ur);
      motor(lsq, rsq);
      //Serial3.print();
      if (left > 0) {
        if ((dl - ll) > ( 0)) ul = 0;
      }
      else
      { if ((ll - dl) > ( 0)) ul = 0;
      }
      if (right > 0) {
        if ((dr - rr) > ( 0)) ur = 0;
      }
      else
      { if ((rr - dl) > ( 0)) ur = 0;
      }

    }
    while (ur || ul);
  }
  stopp();
}
double mean[6][4][2];
int pi, qi, ri, si;
int limit(int in, int low, int high)
{
  if (in < low) return (low);
  if (in > high) return (high);
  return (in);
}
byte edge, sum, cn, cnode, cnnode, cnodeg;
int ti;
unsigned long conndl, conndr;
boolean conditions2()
{
  // sensor 0 is active low and all others are active high
  Serial3.println("conditions");
  if ((psen[0]) && (!sen[0]))
  {
    pi++;
    ddl[pi] = dl;
    ddr[pi] = dr;
    Serial3.println(F("edge centerfront####################################################################"));
    pi = limit(pi, 0, 5);
  }
  if ((!psen[1]) && sen[1])
  {
    qi++;
    ddl[qi] = dl;
    ddr[qi] = dr;
    Serial3.println(F("edge left####################################################################"));
    qi = limit(qi, 0, 5);
    condl = dl;
    conset = 1;
  }
  if ((!psen[5]) && sen[5])
  {
    ri++;
    ddl[ri] = dl;
    ddr[ri] = dr;
    Serial3.println(F("edge right####################################################################"));
    ri = limit(ri, 0, 5);
    condr = dr;
    conset = 1;
  }
  if (((!(sen[2] && sen[3] && sen[4])) && sen[1] && sen[5]) & (!cnode))
  {
    cn += 1;
    if (cn > 1)
    {
      con = INV;
      Serial3.println(F("INV ####################################################################"));

    }
    cnode = 1;
    cnodeg += 1;
    si++;
    ddl[si] = dl;
    ddr[si] = dr;
    Serial3.println(F("node +++++ ####################################################################"));
  }
  if ((((sen[2] || sen[3] || sen[4])) && (!sen[1]) && (!sen[5])) & (!cnnode)  & cnode)
  {
    cn += 1;
    Serial3.println(F("edge node ---- ####################################################################"));
    cnnode = 1;
    cnode -= 1;
    conndl = dl;
    conndr = dr;
  }
  if (((dl + dr - conndl - conndr) > 130)&cnnode)
  {
    Serial3.println(F("NODE ####################################################################"));
    con = NODE;
  }
  if (((dl + dr - condl - condr) > 130) & (!cnnode)&conset)
  {
    Serial3.println(F("POINT ####################################################################"));
    con = PNT;
  }
  mean[pi][0][0] += (dl - ddl[pi] + dr - ddr[pi]) * (1 - sen[0]);
  mean[qi][1][0] += 2 * (dl - ddl[qi] ) * (sen[1]);
  mean[ri][3][0] += 2 * (dr - ddr[ri]) * (sen[5]);
  mean[si][2][0] += (dr - ddr[si] + dl - ddl[si]) * (cnode);

  mean[pi][0][1] += (1 - sen[0]);
  mean[qi][1][1] += (sen[1]);
  mean[ri][3][1] += (sen[5]);
  mean[si][2][1] += (cnode);
  con = 0;

  for (int j = 0; j < 6; j++)
  {
    for (int i = 0; i < 4; i++)
    {
      bitWrite(con, 7 - i, (mean[j][i][1] != 0));
      distmean[j][i] = (float)mean[j][i][0] / mean[j][i][1];
      Serial3.print(distmean[j][i]);
      Serial3.print(" ");
    }
    Serial3.print("    ");
  }
}
void resetconditions()
{
  cnode = 0;
  cnnode = 0;
  cnodeg = 0;
  con = 0;
  conset = 0;
}
/*
  Serial3.print("u-l=");
  uml = distmean[0] - distmean[1];
  Serial3.print(uml);
  Serial3.print("\tu-r=");
  umr = distmean[0] - distmean[3];
  Serial3.print(umr);
  Serial3.print("\tl-m=");
  lmm = distmean[1] - distmean[2];
  Serial3.print(lmm);
  Serial3.print("\tr-m=");
  rmm = distmean[3] - distmean[2];
  Serial3.print(rmm);
  Serial3.println(con, BIN);
  distreach = ((dl + dr - condl - condr) > condist);
  Serial3.print(F("dl"));
  Serial3.print(dl);
  Serial3.print(F("dr"));
  Serial3.println(dr);
  /*
  if ((dl + dr) > 144)
  {
  switch (con & 0b01110000)
  {
  case (0b01100000 )://left 3p
    {
    if ((midst > lmm) && ((c & 0b00111000) == 0) && distreach) //90(sen[2] == 0) && (sen[3] == 0) && (sen[4] == 0)
    {
      stopp();
      Serial3.println(F("90left"));
      setnode();
      setline(true, 0b00001010);
      return true;
      //delay(5000);
    }
    if ((midst < lmm) && ((c & 0b00111000) == 0)) //45
    {
      stopp();
      setnode();
      Serial3.println(F("45left"));
      setline(true, 0b00001001);
      //delay(5000);
      return true;
    }
    if (((midst > lmm) && (c & 0b00111000) != 0) && distreach) //90+front
    {
      stopp();
      Serial3.println(F("90lefttfront"));
      setnode();
      setline(true, 0b10001010);conditions2
      //delay(5000);
      return true;
    }
    break;
    }
  case (0b00110000 )://right 3p
    {
    if ((midst > rmm) && ((c & 0b00111000) == 0) && distreach) //90(sen[2] == 0) && (sen[3] == 0) && (sen[4] == 0)
    {

      stopp();
      Serial3.println(F("90right"));
      setnode();
      setline(true, 0b00101000);
      // delay(5000);
      return true;
    }
    if ((midst < rmm) && ((c & 0b00111000) == 0)) //45
    {

      stopp();
      Serial3.println(F("45right"));
      setnode();
      setline(true, 0b01001000);
      //delay(5000);
      return true;
    }
    if ((midst > rmm) && ((c & 0b00111000) != 0) && distreach) //90+front
    {

      stopp();
      Serial3.println(F("90rightfront"));
      setnode();
      setline(true, 0b10101000);
      //delay(5000);
      return true;
    }
    break;

    }
  case (0b01110000 )://both 5p
    {
    if ((midst < rmm) && (midst < lmm) && ((c & 0b00111000) == 0)) //45
    {
      stopp();
      Serial3.println(F("Ycom"));
      setnode();
      setline(true, 0b01001001);
      //delay(5000);
      return true;
    }
    if ((midst < rmm) && (midst > lmm) && ((c & 0b00111000) == 0)) //45
    {
      stopp();
      Serial3.println(F("45R 90L"));
      setnode();
      setline(true, 0b01001010);
      //delay(5000);
      return true;
    }
    if ((midst > rmm) && (midst < lmm) && ((c & 0b00111000) == 0)) //45
    {
      stopp();
      Serial3.println(F("45L 90R"));
      setnode();
      setline(true, 0b00101001);
      //delay(5000);
      return true;
    }
    if ((midst > rmm) && (midst > lmm) && ((c & 0b01111100) == 0b00000000) && distreach) //90+front
    {
      stopp();
      Serial3.println(F("leftright"));
      setnode();
      setline(true, 0b00101010);
      //delay(5000);
      return true;
    }
    if ((midst > rmm) && (midst > lmm) && ((c & 0b00111000) != 0) && distreach) //90+front
    {
      stopp();
      Serial3.println(F("leftrightfront"));
      setnode();
      setline(true, 0b10101010);
      //delay(5000);
      return true;
    }
    break;
    }
  };
  }
  if ((conset == 2) && ((con & 0b01010000) == 0b00000000) && ((c & 0b11111100) == 0b10000000)) {

  stopp();
  Serial3.println(F("back"));
  setnode();
  setline(true, 0b00001000);
  //delay(5000);
  return true;
  }*/
/*
  pdl = dl;
  pdr = dr;
  }
  return false;
  }
  return false;}*/
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
    while (ul);
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
      dl--;
    }
    else {
      dl++;
    }
  }
  else {
    if (digitalRead(dlb))
    {
      dl++;
    }
    else {
      dl--;
    }
  }
}
void spr()//
{

  if (digitalRead(dra))
  {
    if (digitalRead(drb))
    {
      dr--;
    }
    else {
      dr++;
    }
  }
  else {
    if (digitalRead(drb))
    {
      dr++;
    }
    else {
      dr--;
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
  sen[4] = lcolorb ^ (analogRead(A1) < thres);
  sen[3] = lcolorb ^ (analogRead(A2) < thres);
  sen[2] = lcolorb ^ (analogRead(A3) < thres);
  sen[1] = lcolorb ^ (analogRead(A4) < thres);
  sen[0] = lcolorb ^ (analogRead(A5) < thres);

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
  /*if ((c == 0b01111100) && (!endc)) {
    endc = 1;
    endd = dl + dr;
    }
    if (((dl + dr - endd) > 15) && (c == 0b01111100) && endc) endr = true;
  */
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
    case (0):
      {
        stepmotor(kdss, lspd, rspd);
        sense();
        while ((bitRead(c, 7)))
        {
          stepmotor(1, lspd, (-rspd));
          /*Serial.print(F("dl"));
            Serial.print(dl);
            Serial.print(F("dr"));
            Serial.println(dr);*/
          sense();
        }
        break;
      }
    case (1):
      {
        stepmotor(kds - 1, lspd, (-rspd));
        sense();
        while ((bitRead(c, 7)))
        {
          stepmotor(1, lspd, (-rspd));
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
        stepmotor(2 * kds, lspd, (-rspd));
        sense();
        while ((bitRead(c, 7)))
        {
          stepmotor(1, lspd, (-rspd));
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
        stepmotor(kds - 1, (-lspd), rspd );
        sense();
        while ((bitRead(c, 7)))
        {
          stepmotor(1, (-lspd), rspd );
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
  tur = 0;
  dis = 3;
}
void gen()
{
  //Serial3.println("inside gen");
  dino = (sen[2] + sen[3] + sen[4]);

  if (dino != 0) {
    input = (((sen[2] + 2 * sen[3] + 3 * sen[4]) / dino) - 2);
    //Serial3.print("input");
    //Serial3.println(input);
  }
  P = Kp * input;
  I += Ki * input;
  D = Kd * (input - pinput);
  pinput = input;
  output = P + I + D;
  rad = abs(output);
  krad = (dis - (rad)) / (dis + (rad));

  if (output >= 0)
  {
    sm = rspd - (int)rad;
    lm(lspd);
    rm(sm);
    //delay(1);
    /*Serial.print("\t left motor=");
      Serial.print(lspd);
      Serial.print("\t right motor=");
      Serial.print(sm);
    */
  }
  else
  {

    sm = lspd - (int)rad;

    lm(sm);
    rm(rspd);
    //delay(1);
    /* Serial.print("\t left motor=");
      Serial.print(sm);
      Serial.print("\t right motor=");
      Serial.print(rspd);
    */
  }


}
void brake()
{

}
int lw, rw;
int sori(int ch) //calculate orientation due to turns//
{
  return (( ori + ch) % 4);
}
void process_string(String instruction)
{
  int i = 0;
  instruction += ' ';
  //Serial3.print("inside process string");
  //Serial3.println(instruction);
  if (instruction[i] != '(') {
    while (i < instruction.length())
    {
      //Serial.println("gda");
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
        case ('V'):
          {
            i++;
            t = "";
            while (instruction[i] != ' ')
            {
              Serial3.println("ivr");
              t += instruction[i];
              i++;
            }
            V = t.toFloat();
            break;
          }
      }
      i++;
    }
    //Serial3.print("read string");
    //Serial3.println(G);
    switch (G)
    {
      /*

        ################## G commands ##################

        from master to robot
        ...

        from robot to master

        120 - lines present L F R N

        ################################################

      */
      case (0): //      0 - start
        {
          start_flag = true;
          break;
        }
      case (1): //      1 - stop

        {
          start_flag = false;
          brake();
          break;
        }
      //        8 - led on
      case (8):
        {
          digitalWrite(led_pin, HIGH);
          break;
        }

      //        9 - led off
      case (9):
        {
          digitalWrite(led_pin, LOW);
          break;
        }


      //      10 - line follower testing mode
      case (10):
        {
          mode = lfr_mode;
          sense();
          for (int i; i < 6; i++)
          {
            psen[i] = sen[i];
          }
          //Serial3.print("lfrmode");
          //Serial3.println(mode);
          break;
        }

      //      11 - test drive motors forward with max speed
      case (11):
        {
          motor(lspd, rspd);
          delay(2000);

          motor(-lspd, -rspd);
          delay(1000);
          break;
        }

      //      21 - delay miliseconds
      case (21):
        {
          delay(V);
          break;
        }
      //22 - delay microseconds

      case (30):

        {
          turn(T);
          break;
        }
      case (32):
        {
          Serial3.println("motor drives");
          lspd = lspdprev;
          rspd = rspdprev;
          if (L > 0) {
            lw = lspd;
          }
          else {
            lw = -lspd;
          }
          if (R > 0) {
            rw = rspd;
          }
          else {
            rw = -rspd;
          }
          dstepmotor(lw, rw);
          stopp();
          break;
        }
      case (70):
        {
          lspd = L;
          rspd = R;
          lspdprev = L;
          rspdprev = R;
          break;
        }
      case (80):

        {
          Kp = V;
          //Serial3.print("P=");
          //Serial3.println(Kp);
          break;
        }
      //        81 - set I
      case (81):
        {
          Ki = V;
          //Serial3.print("I=");
          //Serial3.println(Ki);
          break;
        }
      //        82 - set D
      case (82):
        {
          Kd = V;
          //Serial3.print("D=");
          //Serial3.println(Kd);
          break;
        }
    }
  }
}
void do_action()
{
  Serial.println("Start next");
}
/*int digitalPinToInterrupt(int pin)
  {
  int intpin;
  switch (pin)
  {
    case (2):
      {


        intpin = 0;
        break;

      }
    case (3):
      {
        intpin = 1;
        break;
      }
    case (21):
      {
        intpin = 2;
        break;
      }
    case (20):
      {
        intpin = 3;
        break;
      }
    case (19):
      {
        intpin = 4;
        break;
      }
    case (18):
      {
        intpin = 5;
        break;
      }
  }
  return (intpin);
  }*/
int prevd;
void dstepmotor(int l, int r)
{
  int d;
  if (abs(l) == abs(r))
  {

    int le = lspd * (l / abs(l));
    int rig = rspd * (r / abs(r));
    Serial3.print("le");
    Serial3.print(le);
    Serial3.print("ri");
    Serial3.println(rig);
    stepmotor((abs(l) + abs(r)) / 2, le , rig);
    Serial3.println("inside stepmotor");
  }
  else
  {
    if (abs(l) > abs(r))
    {
      leftstepmotor(1, l * lspd / abs(l));
      d = (int)(abs(r) * abs(r) / abs(l));
      rightstepmotor((d - prevd), r * rspd / (abs(r)));
      prevd = d;
    }
    else
    {
      rightstepmotor(1, r * rspd / abs(r));
      d = (int)(abs(l) * abs(l) / abs(r));
      leftstepmotor((d - prevd), l * lspd / (abs(l)));
      prevd = d;
    }
  }
}
void setup()
{
  attachInterrupt(digitalPinToInterrupt(dla), spl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(dra), spr, CHANGE);

  Serial.begin(115200);
  Serial3.begin(115200);

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


  //pinMode(12, INPUT_PULLUP);
  //pinMode(endp, INPUT_PULLUP);
  //pinMode(13, 1);
  pinMode(l1, 1);
  pinMode(l2, 1);
  pinMode(sl, 1);
  pinMode(r1, 1);
  pinMode(r2, 1);
  pinMode(sr, 1);
  /*//setmode();
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
void mode_based_operation_loop()
{
  Serial3.println("inside mode based o l");
  switch (mode)
  {
    case (lfr_mode):
      {
        for (int i; i < 6; i++)
        {
          psen[i] = sen[i];
        }
        sense();

        //Serial3.println("sensed");
        Serial3.println(c, BIN);
        boolean kc = conditions2();

        Serial3.println(c);
        if (c == 0)
        {
          lspd = 0;
          rspd = 0;
        }
        else
        {
          lspd = lspdprev;
          rspd = rspdprev;
        }
        gen();
        break;
      }

  }
}

void sensor_check()
{
  Serial.print("j");
}
void responce(String inp)
{ inp += "|";
  Serial.print(inp);
}
void loop()
{
  if (Serial.available() > 0)
  {
    cc = Serial.read();
    if (cc == '|') {
      process_string(data);
      //responce(data);
      data = "";
    }
    else {
      data += cc;
    }
  }
  mode_based_operation_loop();
  Serial3.print("dl");
  Serial3.print(dl);
  Serial3.print("dl");
  Serial3.println(dl);
}
