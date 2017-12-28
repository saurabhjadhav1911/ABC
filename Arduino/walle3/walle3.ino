#include<EEPROM.h>
#include<QueueArray.h>
#define lcolor 255
#define lcolorb 0 //white line =0
#define condist 25
#define spt 125
#define mKp 1
#define mKd 1
#define mKi 1
#define maxs 1000
#define kdst 5
#define kdsf 6
#define kds2 9
#define r1 4//D4
#define r2 5//D5
#define sr 6//D6
#define l1 8//B0
#define l2 7//D7
#define sl 9//B1
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
#define lspdcon 175
#define rspdcon 175
#define lspdmin 200
#define rspdmin 200
#define stopdist 10
#define n 9
#define m 9
#define INT 0
#define thres 500
#define midst 100
int pr[2], negh[8][2] = {{ -1, 0}, { -1, 1}, {0, 1}, {1, 1}, {1, 0}, { 1, -1} , {0, -1}, { -1, -1}};
int nx, ny, ul, ur, t[6], sm, endX = 5, endY = 2, startX = 3 , startY = 6 , address, cpm = 10;;
byte ori, c, currx, curry, minvalue, xn, ym, oi, rspd = rspdmin, lspd = lspdmin, rsp, lsp;
unsigned long int rr, ll, dr, dl, prevdist, d;
float Kp = 150, Ki = 0, Kd = 40, P, I, D, sen[6] , input, pinput, output, l, r, rad, krad, turnsen, dis = 3,  dino = 0.0, turndino = 0.0;
boolean trial, finalr, endr = false, resetconditionsflag = 0, zeroresetconditionsflag = 0, tur = 0, bc, finals = false, speedfg = true, speedtg = true, endc = false;
byte con, prevcon, pcon, conset, ee, kds = kdst, lspdt = lspdmin + 15, rspdt = lspdmin + 15;
long mean[4][2], savedl = 0, savedr = 0;
byte edge, sum;
float distmean[4];
unsigned long pdl, pdr, condl, condr, ddl, ddr, endd = 0;
byte point[n][m][9];
QueueArray <byte> turnque;
byte leftcount = 0, rightcount = 0, midcount = 0;

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
  //Serial.print(dr);
  //Serial.print("\t");
  //Serial.println();
}
float uml, umr, lmm, rmm;
boolean distreach;
/*int leftright(int tp)
  {
  if(rightside) return((tp+4)%4)
  }*/
void savepath()
{
  turnque.enqueue(6);
  turnque.enqueue(2);
  turnque.enqueue(0);
  turnque.enqueue(2);
  turnque.enqueue(2);

  turnque.enqueue(6);
  turnque.enqueue(1);
  turnque.enqueue(0);
  turnque.enqueue(1);
  turnque.enqueue(6);
  turnque.enqueue(2);
  turnque.enqueue(6);

  //turnque.enqueue(0);
}
void savepath2()
{
  turnque.enqueue( 6 );
  turnque.enqueue( 2 );
  turnque.enqueue( 2 );
  turnque.enqueue( 6 );
  turnque.enqueue( 0 );
  turnque.enqueue( 6 );
  turnque.enqueue( 2 );
  turnque.enqueue( 2 );
  turnque.enqueue( 6 );
  turnque.enqueue( 6 );
  turnque.enqueue( 2 );
  turnque.enqueue( 6 );
  turnque.enqueue( 6 );
  turnque.enqueue( 2 );
  turnque.enqueue( 2 );
  turnque.enqueue( 6 );
  turnque.enqueue( 6 );
  turnque.enqueue( 2 );
  turnque.enqueue( 0 );
  turnque.enqueue( 0 );
  turnque.enqueue( 0 );
  turnque.enqueue( 6 );
  turnque.enqueue( 2 );
  turnque.enqueue( 6 );//small
  turnque.enqueue( 6 );
  turnque.enqueue( 2 );
  turnque.enqueue( 2 );
  turnque.enqueue( 2 );
  turnque.enqueue( 6 );
  turnque.enqueue( 0 );
  turnque.enqueue( 6 );
  turnque.enqueue( 2 );
  turnque.enqueue( 6 );
  turnque.enqueue( 2 );
}
void speedprofile(int dstatic)//
{
  if ((dstatic))
  {
    d = dl + dr + dstatic;
  }
  if ((d - dl - dr) < stopdist)
  {
    lspdt = map(((d - dl - dr)), stopdist, 0, lspdmax, lspdmin);
    rspdt = map(((d - dl - dr)), stopdist, 0, rspdmax, rspdmin);
  }
}
boolean conditions4()
{
  // sensor 0 is active low and all others are active high
  if ((sen[1] || sen[5] || ((sen[2] || sen[3] || sen[4]) == 0)) && (!conset))
  { conset = 2 - (byte)((sen[2] || sen[3] || sen[4]) == 0);
    condl = dl;
    condr = dr;
    lspd = lspdcon;
    rspd = rspdcon;
    //stepmotor(1, -lspdmin, -rspdmin);
  }
  if ((conset) && ((dl != pdl) || (dr != pdr))) {
    ddl = dl - condl;
    ddr = dr - condr;
    mean[0][0] += (ddl + ddr) * (sen[0]);
    mean[1][0] += 2 * ddl * (sen[1]);
    mean[3][0] += 2 * ddr * (sen[5]);
    mean[2][0] += (ddr + ddl) * (sen[3]);

    mean[0][1] += (sen[0]);
    mean[1][1] += (sen[1]);
    mean[3][1] += (sen[5]);
    mean[2][1] += (sen[3]);
    con = 0;
    for (int i = 0; i < 4; i++)
    {
      bitWrite(con, 7 - i, (mean[i][1] != 0));
      distmean[i] = (float)mean[i][0] / mean[i][1];
      Serial.print(distmean[i]);
      Serial.print("\t");
    }
    Serial.print("u-l=");
    uml = distmean[0] - distmean[1];
    Serial.print(uml);
    Serial.print("\tu-r=");
    umr = distmean[0] - distmean[3];
    Serial.print(umr);
    Serial.print("\tl-m=");
    lmm = distmean[1] - distmean[2];
    Serial.print(lmm);
    Serial.print("\tr-m=");
    rmm = distmean[3] - distmean[2];
    Serial.print(rmm);
    Serial.println(con, BIN);
    distreach = ((dl + dr - condl - condr) > condist);
    Serial.print(F("dl"));
    Serial.print(dl);
    Serial.print(F("dr"));
    Serial.println(dr);
    switch (con & 0b01110000)
    {
      case (0b01100000 )://left 3p
        {
          if (((c & 0b00111000) == 0) && distreach) //90(sen[2] == 0) && (sen[3] == 0) && (sen[4] == 0)
          {
            stopp();
            Serial.println(F("90left"));
            //setnode();
            //setline(true, 0b00001010);
            turn(turnque.dequeue());//turn(6);
            return true;
            //delay(5000);
          }

          if (((c & 0b00111000) != 0) && ((c & 0b01000100) == 0) && distreach) //90+front
          {
            stopp();
            Serial.println(F("90lefttfront"));
            turn(turnque.dequeue());
            //setnode();
            //setline(true, 0b10001010);
            //delay(5000);
            return true;
          }
          break;
        }
      case (0b00110000 )://right 3p
        {
          if (((c & 0b00111000) == 0) && distreach) //90(sen[2] == 0) && (sen[3] == 0) && (sen[4] == 0)
          {

            stopp();
            Serial.println(F("90right"));
            //setnode();
            //setline(true, 0b00101000);
            // delay(5000);
            turn(turnque.dequeue());
            //turn(2);
            return true;
          }

          if (((c & 0b00111000) != 0) && ((c & 0b01000100) == 0) && distreach) //90+front
          {

            stopp();
            Serial.println(F("90rightfront"));
            turn(turnque.dequeue());
            //setnode();
            //setline(true, 0b10101000);
            //delay(5000);
            return true;
          }
          break;

        }
      case (0b01110000 )://both 5p
        {
          if (((c & 0b00111000) == 0b00000000) && distreach) //90+front
          {
            stopp();
            Serial.println(F("leftright"));
            //setnode();
            //setline(true, 0b00101010);
            //delay(5000);
            turn(turnque.dequeue());
            return true;
          }
          if (((c & 0b00111000) != 0) && ((c & 0b01000100) == 0) && distreach) //90+front
          {
            stopp();
            Serial.println(F("leftrightfront"));
            turn(turnque.dequeue());
            //setnode();
            //setline(true, 0b10101010);
            //delay(5000);
            return true;
          }
          break;
        }
    };
    if (((dl + dr) < 10) && ((c & 0b11111100) == 0b10000000)) {
      stopp();
      Serial.println(F("back"));
      //setnode();
      //setline(true, 0b00001000);
      //delay(5000);
      turn(turnque.dequeue());
      return true;

    }
    pdl = dl;
    pdr = dr;
  }
  return false;
}
boolean conditions()
{
  // sensor 0 is active low and all others are active high
  if ((sen[1] || sen[5] || ((sen[2] || sen[3] || sen[4]) == 0)) && (!conset))
  { conset = 2 - (byte)((sen[2] || sen[3] || sen[4]) == 0);
    stopp();
    condl = dl;
    condr = dr;
    lspd = lspdcon;
    rspd = rspdcon;

    Serial.print(dl);
    Serial.print("\t");
    Serial.print(dr);
    Serial.print("\t");
    //stepmotor(1, -lspdmin, -rspdmin);
  }
  edge = edge ^ c;
  sum |= (c & 0b11111100);
  if ((conset))// && ((dl != pdl) || (dr != pdr)))
  {
    //con = 0;

    bitWrite(con, 7, ((bitRead(con, 7)) || sen[0]));
    bitWrite(con, 6, ((bitRead(con, 6)) || sen[1]));
    bitWrite(con, 5, ((bitRead(con, 5)) || sen[2] || sen[3] || sen[4]));
    bitWrite(con, 4, ((bitRead(con, 4)) || sen[5]));

    if ((dl != pdl) || (dr != pdr))
    {
      ddl = dl - condl;
      ddr = dr - condr;
      mean[0][0] += (ddl + ddr) * (sen[0]);
      mean[1][0] += 2 * ddl * (sen[1]);
      mean[3][0] += 2 * ddr * (sen[5]);
      mean[2][0] += (ddr + ddl) * (sen[3]);

      mean[0][1] += (sen[0]);
      mean[1][1] += (sen[1]);
      mean[3][1] += (sen[5]);
      mean[2][1] += (sen[3]);

      for (int i = 0; i < 4; i++)
      {
        distmean[i] = (float)mean[i][0] / mean[i][1];
        //Serial.print(distmean[i]);
        //Serial.print("\t");
      }

      Serial.print("u-l=");
      uml = distmean[0] - distmean[1];
      Serial.print(uml);
      Serial.print("\tu-r=");
      umr = distmean[0] - distmean[3];
      Serial.print(umr);
      Serial.print("\tl-m=");
      lmm = distmean[1] - distmean[2];
      Serial.print(lmm);
      Serial.print("\tr-m=");
      rmm = distmean[3] - distmean[2];
      Serial.print(rmm);
      Serial.println(con, BIN);
      distreach = ((dl + dr - condl - condr) > condist);
      //Serial.print(F("dl"));
      //Serial.print(dl);
      //Serial.print(F("dr"));
      //Serial.println(dr);
      //Serial.print(F("edge"));
      //Serial.println(edge, BIN);
      pdl = dl;
      pdr = dr;
    }
    switch (con & 0b01110000)
    {
      case (0b01100000)://left 3p
        {
          if ( ((c & 0b01111100) == 0) && distreach && (lmm < midst)) //90(sen[2] == 0) && (sen[3] == 0) && (sen[4] == 0)
          {
            stopp();
            Serial.println(F("90left"));
            leftcount += 1;
            ////setnode();
            ////setline(true, 0b00001010);
            turn(7);
            return true;

            //delay(5000);
          }
          /*if (((c & 0b00111000) != 0) && distreach) //90+front
            {
            stopp();
            Serial.println(F("90leftfront"));
            ////setnode();
            ////setline(true, 0b10001010);
            //delay(5000);
            //turn(turnque.dequeue());
            return true;
            }*/
          break;
        }
      case (0b00110000)://right 3p
        {
          if ( ((c & 0b01111100) == 0) && distreach)//. && (rmm < midst)) //90(sen[2] == 0) && (sen[3] == 0) && (sen[4] == 0)
          {
            stopp();
            Serial.println(F("90right"));
            ////setnode();
            turn(1);
            ////setline(true, 0b00101000);
            // delay(5000);
            rightcount += 1;
            return true;
          }
          /*if ((midst > rmm) && ((c & 0b00111000) != 0) && distreach) //90+front
            {
            stopp();
            Serial.println(F("90rightfront"));
            ////setnode();
            ////setline(true, 0b10101000);
            //delay(5000);
            //turn(turnque.dequeue());
            return false;
            }*/
          break;
        }
      case (0b01110000)://both 5p
        {
          /*if (((c & 0b01111100) == 0b00000000) && distreach) //90+front
            {
            stopp();
            Serial.println(F("leftright"));
            //delay(5000);
            turn(turnque.dequeue());
            return true;
            }
            if (((c & 0b00111000) != 0) && distreach) //90+front
            {*/
          //kds = 3;
          if (distreach) { ////90
            stopp();
            Serial.println(F("both"));
            ////setnode();
            //////setline(true, 0b10101010);
            //delay(5000);
            midcount += 1;
            if (!turnque.isEmpty())
            {
              turn(turnque.dequeue());
            }
            kds = kdst;
            return true;
          }
          break;
        }
    };
  }
  /* if ((conset == 2) && ((con & 0b01010000) == 0b00000000) && ((c & 0b11111100) == 0b10000000)) {

     stopp();
     Serial.println(F("back"));
     ////setnode();
     ////setline(true, 0b00001000);
     //delay(5000);
     turn(0);
     return true;
    }*/
  return false;
}
byte prevc = 0;
void gen()
{ dino = (sen[2] + sen[3] + sen[4]);
  if (dino != 0) {
    input = (((sen[2] + 2 * sen[3] + 3 * sen[4]) / dino) - 2);
    //Serial.print("input");
    //Serial.println(input);
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
    //sm = map(sm, 0, rspd, 40, rspd);
    lm(lspd);
    rm(sm);
    delay(1);
    /*Serial.print("\t left motor=");
      Serial.print(lspd);
      Serial.print("\t right motor=");
      Serial.print(sm);
    */
  }
  else
  {

    sm = lspd - (int)rad;
    //sm = map(sm, 0, lspd, 40, lspd);
    lm(sm);
    rm(rspd);
    delay(1);
    /* Serial.print("\t left motor=");
      Serial.print(sm);
      Serial.print("\t right motor=");
      Serial.print(rspd);
    */
  }
}

boolean conditions2()
{

  // sensor 0 is active low and all others are active high
  if ((sen[1] || sen[5] || ((sen[2] || sen[3] || sen[4]) == 0)) && (!conset))
  { conset = 2 - (byte)((sen[2] || sen[3] || sen[4]) == 0);
    condl = dl;
    condr = dr;
    lspd = lspdcon;
    rspd = rspdcon;
    //stepmotor(1, -lspdmin, -rspdmin);
  }

  Serial.print(F("edge"));
  Serial.println(edge, BIN);
  Serial.print(F("c"));
  Serial.println(c, BIN);
  edge = prevc ^ c;
  sum |= (c & 0b11111100);
  Serial.print(F("edge"));
  Serial.println(edge, BIN);
  if ((conset) && ((dl != pdl) || (dr != pdr))) {
    ddl = dl - condl;
    ddr = dr - condr;
    mean[0][0] += (ddl + ddr) * (sen[0]);
    mean[1][0] += 2 * ddl * (sen[1]);
    mean[3][0] += 2 * ddr * (sen[5]);
    mean[2][0] += (ddr + ddl) * (sen[3]);

    mean[0][1] += (sen[0]);
    mean[1][1] += (sen[1]);
    mean[3][1] += (sen[5]);
    mean[2][1] += (sen[3]);
    con = 0;
    for (int i = 0; i < 4; i++)
    {
      bitWrite(con, 7 - i, (mean[i][1] != 0));
      distmean[i] = (float)mean[i][0] / mean[i][1];
      Serial.print(distmean[i]);
      Serial.print("\t");
    }
    Serial.print("u-l=");
    uml = distmean[0] - distmean[1];
    Serial.print(uml);
    Serial.print("\tu-r=");
    umr = distmean[0] - distmean[3];
    Serial.print(umr);
    Serial.print("\tl-m=");
    lmm = distmean[1] - distmean[2];
    Serial.print(lmm);
    Serial.print("\tr-m=");
    rmm = distmean[3] - distmean[2];
    Serial.print(rmm);
    Serial.println(con, BIN);
    distreach = ((dl + dr - condl - condr) > condist);
    Serial.print(F("dl"));
    Serial.print(dl);
    Serial.print(F("dr"));
    Serial.println(dr);

    {

      if (conset && ((sum & 0b01000100)^edge))
        switch (sum & 0b01000100)
        {
          case (0b01000000 )://left 3p
            {
              if (((c & 0b00111000) == 0)) //90(sen[2] == 0) && (sen[3] == 0) && (sen[4] == 0)
              {
                stopp();
                Serial.println(F("90left"));
                ////setnode();
                ////setline(true, 0b00001010);
                turn(7);
                return true;
                //delay(5000);
              }
              if (((c & 0b00111000) != 0))//90+front
              {
                stopp();
                Serial.println(F("90leftfront"));
                ////setnode();
                ////setline(true, 0b10001010);
                //delay(5000);
                //turn(turnque.dequeue());
                return true;
              }
              break;
            }
          case (0b00000100)://right 3p
            {
              if (((c & 0b00111000) == 0)) //90(sen[2] == 0) && (sen[3] == 0) && (sen[4] == 0)
              {
                stopp();
                Serial.println(F("90right"));
                ////setnode();
                turn(1);
                ////setline(true, 0b00101000);
                // delay(5000);
                return true;
              }
              if (((c & 0b00111000) != 0)) //90+front
              {
                stopp();
                Serial.println(F("90rightfront"));
                ////setnode();
                ////setline(true, 0b10101000);
                //delay(5000);
                //turn(turnque.dequeue());
                return false;
              }
              break;

            }
          case (0b01000100 )://both 5p
            {
              if (((c & 0b01111100) == 0b00000000) && distreach) //90+front
              {
                stopp();
                Serial.println(F("leftright"));
                //delay(5000);
                turn(turnque.dequeue());
                return true;
              }
              if (((c & 0b00111000) != 0) && distreach) //90+front
              {
                stopp();
                Serial.println(F("leftrightfront"));
                ////setnode();
                //////setline(true, 0b10101010);
                //delay(5000);
                turn(turnque.dequeue());
                return true;
              }
              break;
            }
        };
    }
    if ((conset == 2) && ((con & 0b01010000) == 0b00000000) && ((c & 0b11111100) == 0b10000000)) {
      stopp();
      Serial.println(F("back"));
      ////setnode();
      ////setline(true, 0b00001000);
      //delay(5000);
      turn(0);
      return true;
    }
    pdl = dl;
    pdr = dr;
  }
  prevc = c;
  return false;
}
int sori(int ch) //calculate orientation due to turns//
{
  return (( ori + ch) % 8);
}
int globalori = 0;
void spl()//
{
  dl++;
  savedl++;
}
void spr() //interupt function right rotary encoder//
{
  dr++;
  savedr++;
}
void motor(int left, int right) //drive motor//
{
  lm(left);
  rm(right);
}

void lm(int lsp)//
{
  if (lsp >= 0) {
    /*
      digitalWrite(l1, 1);
      digitalWrite(l2, 0);*/

    PORTB |= 0b00000001;
    PORTD &= 0b01111111;
    analogWrite(sl, lsp);
  }
  else {
    /*
      digitalWrite(l1, 0);
      digitalWrite(l2, 1);*/
    PORTD |= 0b10000000;
    PORTB &= 0b11111110;
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
    /*
      digitalWrite(r1, 1);
      digitalWrite(r2, 0);*/

    PORTD |= 0b00010000;
    PORTD &= 0b11011111;
    analogWrite(sr, rsp);
  }
  else {
    //digitalWrite(r1, 0);
    //digitalWrite(r2, 1);
    PORTD |= 0b00100000;
    PORTD &= 0b11101111;
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
void turn(byte tu) //turning at node
{ Serial.println(tu);
  tur = 1;

  switch (tu) {
    case (1):
      { stepmotor(kds + 3, lspdt, (-rspdt));
        sense();
        while ((bitRead(c, 7)))
        { stepmotor(1, lspdt, (-rspdt));
          /*Serial.print(F("dl"));
            Serial.print(dl);
            Serial.print(F("dr"));
            Serial.println(dr);*/
          sense();
        }
        break;
      }
    case (2):
      { stepmotor(2 * kds, lspdt, (-rspdt));
        sense();
        while ((!bitRead(c, 4)) || (bitRead(c, 7)))
        { stepmotor(1, lspdt, (-rspdt));
          /*Serial.print(F("dl"));
            Serial.print(dl);
            Serial.print(F("dr"));
            Serial.println(dr);*/
          sense();
        }
        break;
      }
    case (9):
      { stepmotor(13, lspdt, (-rspdt));
        sense();
        /*while ((!bitRead(c, 4)) || (bitRead(c, 7)))
          { stepmotor(1, lspdt, (-rspdt));
          /*Serial.print(F("dl"));
            Serial.print(dl);
            Serial.print(F("dr"));
            Serial.println(dr);
          sense();
          }*/
        break;
      }
    case (10):
      { stepmotor(13, (- lspdt), (rspdt));
        sense();
        /*while ((!bitRead(c, 4)) || (bitRead(c, 7)))
          { stepmotor(1, lspdt, (-rspdt));
          /*Serial.print(F("dl"));
            Serial.print(dl);
            Serial.print(F("dr"));
            Serial.println(dr);
          sense();
          }*/
        break;
      }
    case (3):
      { stepmotor(3 * kds, lspdt, (-rspdt));
        sense();
        while ((bitRead(c, 7)))
        { stepmotor(1, lspdt, (-rspdt));
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
        stepmotor(4, (lspdt), rspdt );///iosadsdfbusbdc
        stepmotor(kds + 1, (-lspdt), rspdt );
        sense();
        while ((!bitRead(c, 4)) || (bitRead(c, 7)))
        { stepmotor(1, (-lspdt), rspdt );
          /*Serial.print(F("dl"));
            Serial.print(dl);
            Serial.print(F("dr"));
            Serial.println(dr);*/
          sense();
        }
        break;
      }
    case (6):
      { stepmotor(2 * kds, (-lspdt), rspdt );
        sense();
        while ((!bitRead(c, 4)) || (bitRead(c, 7)))
        { stepmotor(1, (-lspdt), rspdt );
          /*Serial.print(F("dl"));
            Serial.print(dl);
            Serial.print(F("dr"));
            Serial.println(dr);*/
          sense();
        }
        break;
      }
    case (5):
      { stepmotor(3 * kds, (-lspdt), rspdt );
        sense();
        while ((bitRead(c, 7)))
        { stepmotor(1, (-lspdt), rspdt );
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
        { stepmotor(1, lspdt, (-rspdt));
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
void resetconditions()
{
  for (int ci = 0; ci < 4; ci++)
  {
    mean[ci][0] = 0;
    mean[ci][1] = 0;
  }
  con = 0;
  conset = 0;
  endr = 0;
  edge = 0;
  sum = 0;
  endc = 0;

  dl = 0;
  dr = 0;
  lspd = lspdmin;
  rspd = rspdmin;

}
boolean psy = true;
boolean setmode() {
  while (1)
  {
    digitalWrite(13, (psy));
    if (digitalRead(12) == 0)
      break;
    delay(1000);
    psy = !psy;
  }
  return psy;
}
byte tyu;
/*
  void trialrun() {
  set();
  trial = true;
  finalr = false;
  pr[0] = startX - negh[ori][0];
  pr[1] = startY - negh[ori][1];
  ori = 0;
  ////setline(true, 0b10000000);
  mazeprint();
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  delay(1000);
  do
  { sense();
    if (dl + dr < 170) {
      sen[0] = 0;
      c &= 0b0111111;
    }
    if (conditions())
    {
      motor(-255, -255);
      delay(30);
      stopp();
      resetconditions();
      digitalWrite(13, 1);
      floodfill();
      mazeprint();
      tyu = ((dir() - ori) + 8) % 8;
      turn(tyu);
      dl = 0;
      dr = 0;
      digitalWrite(13, 0);
      pinput = 0;
    }
    else {
      if (conset) {

        stepmotor(1, lspd, rspd);
        //stopp();
        cpm--;
      }
      }


    gen();
  } while ((!endr));
  if (endr) {
    digitalWrite(13, 1);
    stopp();
    Serial.println(F("endr"));
    pr[0] += negh[ori][0];
    pr[1] += negh[ori][1];
    Serial.println(F("endr"));
    endX = pr[0];
    endY = pr[1];
    Serial.println(F("endr"));
    clearline();
    Serial.println(F("endr"));
    mazeprint();
    Serial.println(F("endr"));
    floodfill();
    mazeprint();
    complete();
    while (1)
    {}
  }
  }
  int p = 0;
*/
/*
  void path()
  {
  while(end)
  sense();
  if (conditions())
  {
    turn()
  }
  gen();
  }*/
void setup()
{
  attachInterrupt(0, spl, HIGH);
  attachInterrupt(1, spr, HIGH);

  Serial.begin(57600);

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
  //resetconditions();*/
  delay(500);
  digitalWrite(13, 0);
  digitalWrite(13, 1);
  digitalWrite(13, 0);
  //pidcheck();
  savepath();
}
int sti = 0;
void loop() {
  sense();
  for (int u = 7; u >= 0; u--) Serial.print(bitRead(c, u));
  Serial.println();
  /*Serial.print("leftdist ");
    Serial.print(savedl);
    Serial.print("rightdist ");
    Serial.print(savedr);*/
  /*if (conditions4())
  {
    resetconditions();

    //delay(500);
  }*/
  /*if (conset)
    {
    stopp();
    stepmotor(1, lspd, rspd);
    }
    else*/
  gen();
}
