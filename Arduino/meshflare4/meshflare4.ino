#include<EEPROM.h>
#include<QueueArray.h>
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
#define ARROWLEFT true
#define ARROWRIGHT false
#define TFLOGO 111
#define NOTFLOGO 222
#define END 1
#define NODE 2
#define BLOCK 3
#define PIT 4
#define REDPIT 5
#define DEP 6


void floodfill();
//void recfloodfill(byte fx, byte fy);
void qfloodfill();
void generate();
void gen2();
void gen();
void sense();
void trialmaze();
void speedprofile(int dstatic);
void set(); //initialisation of variables;
void setline( boolean line , byte lines); //set lines are present or not at left front and right;
void mazeprint(); //print the maze in serial monitor;
void spl();
void spr(); //interupt function right rotary encoder;
void motor(int left, int right); //drive motor;
void lm(int lsp);
void stopp();
void rm(int rsp);
void setf();
void sett();
void complete();
void getlines();
void stepmotor(int steps, int left, int right);
void leftstepmotor(int stepsl, int leftm);
void rightstepmotor(int stepsr, int rightm);
void turn(byte tu);// turning at node;
void clearline();
void setnode();
void resetconditions();
void finalrun() ;
void trialrun() ;
//void setup();
//void loop() ;




int pr[2], negh[8][2] = {{ -1, 0}, { -1, 1}, {0, 1}, {1, 1}, {1, 0}, { 1, -1} , {0, -1}, { -1, -1}};
int nx, ny, ul, ur, t[6], sm, endX = 5, endY = 2, startX = 3 , startY = 6 , address, cpm = 10;;
byte ori, c, currx, curry, minvalue, xn, ym, oi, rspd = rspdmin, lspd = lspdmin, rsp, lsp;
unsigned long int rr, ll, dr, dl, prevdist, d;
float Kp = 50, Ki = 0, Kd = 10, P, I, D, sen[6] , input, pinput, output, l, r, rad, krad, turnsen, dis = 3,  dino = 0.0, turndino = 0.0;
boolean trial, finalr, endr = false, resetconditionsflag = 0, zeroresetconditionsflag = 0, tur = 0, bc, finals = false, speedfg = true, speedtg = true, endc = false;
byte con, prevcon, pcon, conset, ee, kds = kdst, lspdt = 160, rspdt = 160;
long mean[4][2];
float distmean[4];
unsigned long pdl, pdr, condl, condr, ddl, ddr, endd = 0;

class Android
{
public:
  Android();
  void send(string data);
  void listen();
  boolean getarrow();
  int checkpicture();
  int checkend();

};

byte point[n][m][nk];
QueueArray <byte> quex;
QueueArray <byte> quey;
void floodfill()
{
  for (int i = 0; i < n; i++)
  { for (int j = 0; j < m; j++)
  { point[i][j][nk] = 255;
  }
  }
  point[endX][endY][8] = 0;
  //mazeprint();
  qfloodfill();
  //recfloodfill(endX, endY);
  /*
  point[endX][endY][8] = 0;
  xn = pr[0] ;//+ negh[ori][0];
  ym = pr[1] ;//+ negh[ori][1];
  if (!stackline.isEmpty()) {//new notlines discovered
    stackx.push(xn);
    stacky.push(ym);
    Serial.println(F("push currentnode to stack"));
  }
  while (!stackline.isEmpty())//nebours of new notlines
  {
    oi = stackline.pop();
    stackx.push(xn + negh[oi][0]);
    stacky.push(ym + negh[oi][1]);
    //Serial.print(F("push nodes of notlines 3 currentnode to stack"));
    //Serial.println(10 * (xn + negh[oi][0]) + (ym + negh[oi][1]));
  }
  while (!stackx.isEmpty())//while main stack is not empty
  {
    currx = stackx.pop();//pop curr from main stack
    curry = stacky.pop();
    minvalue = 255;

    for (byte nei = 0; nei < 8; nei++)//min negbours of curr
    {
    if (lineconnect(currx, curry, nei))
    {
      stackmin.push(nei);
      //Serial.println(F("push connected nebours"));

      if ((point[(currx + negh[nei][0])][curry + negh[nei][1]][8]) < minvalue) minvalue = point[currx + negh[nei][0]][curry + negh[nei][1]][8];
    }
    }
    if (!(point[currx][curry][8] == (1 + minvalue)))//is curr is 1 more than open nebours
    {
    while (!stackmin.isEmpty())
    {
      point[currx][curry][8] = 1 + minvalue;
      byte nei = stackmin.pop();
      if (((currx + negh[nei][0]) > -1) && ((curry + negh[nei][1]) > -1) && ((currx + negh[nei][0]) < n) && ((curry + negh[nei][1]) < m))
      {
      stackx.push(currx + negh[nei][0]);
      stacky.push(curry + negh[nei][1]);
      //Serial.print(F("push connected nebours"));
      //Serial.println(10 * (currx + negh[nei][0]) + (curry + negh[nei][1]));
      }
    }
    }

  }
  */
}
byte value, fnx, fny, fx, fy;
/*void recfloodfill(byte fx, byte fy)
  {
  if (!(fx > -1) && (fy > -1) && (fx < n) && (fy < m)) return;
  value = point[fx][fy][8] + 1;
  Serial.println(F(""));
  for (byte nei = 0; nei < 8; nei++)//min negbours of curr
  {

  if (lineconnect(fx, fy, nei))
  {
    fnx = fx + negh[nei][0];
    fny = fy + negh[nei][1];
    if (value < point[fnx][fny][8]) {
    Serial.println(point[fnx][fny][8]);
    point[fnx][fny][8] = value;
    mazeprint();
    recfloodfill(fnx, fny);
    }
  }
  }
  }*/
void qfloodfill()
{
  quex.enqueue(endX);
  quey.enqueue(endY);
  while (!quex.isEmpty())
  {
  fx = quex.pop();
  fy = quey.pop();
  //Serial.print(10*fy+fx);
  if ((fx > -1) && (fy > -1) && (fx < n) && (fy < m)) {
    value = point[fx][fy][8] + 1;
    //Serial.print(F("dequed"));
    //Serial.println(value);
    for (byte nei = 0; nei < 8; nei++)//min negbours of curr
    {
    //Serial.println(F("inside nebour loop"));
    if (lineconnect(fx, fy, nei))
    {
      //Serial.println(F("lineconnected"));
      fnx = fx + negh[nei][0];
      fny = fy + negh[nei][1];
      //Serial.print(point[fnx][fny][8]);
      if (value < point[fnx][fny][8]) {
      //Serial.println(F("minfound"));
      point[fnx][fny][8] = value;
      //Serial.println(point[fnx][fny][8]);
      //mazeprint();
      quex.enqueue(fnx);
      quey.enqueue(fny);
      }
    }
    }
  }
  }
}
void generate()//
{
  for (int i = 0; i < n; i++)
  { for (int j = 0; j < m; j++)
  { //point[i][j][8] = 255; // (abs(i - endX) + abs(j - endY)) - min(abs(j - endY), abs(i - endX));
    for (int k = 0; k < 8; k++)
    {
    if (((i + negh[k][0]) > -1) && ((j + negh[k][1]) > -1) && ((i + negh[k][0]) < n) && ((j + negh[k][1]) < m)) {
      point[i][j][k] = 10;
    }
    else point[i][j][k] = 0;

    }
  }
  }

}
void gen2()
{ dino = (sen[2] + sen[3] + sen[4]);
  if (dino != 0) {
  input = (((sen[2] + 2 * sen[3] + 3 * sen[4]) / dino) - 2);
  Serial.print("input");
  Serial.println(input);
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

void gen()
{ dino = (sen[2] + sen[3] + sen[4]);
  if (dino != 0) {
  input = (((sen[2] + 2 * sen[3] + 3 * sen[4]) / dino) - 2);
  Serial.print("input");
  Serial.println(input);
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
float uml, umr, lmm, rmm;
boolean distreach;
boolean conditions()
{

  // sensor 0 is active low and all others are active high
  if ((sen[1] || sen[5] || sen[0] || ((sen[2] || sen[3] || sen[4]) == 0)) && (!conset))
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
  if ((dl + dr) > 144)
  {
    switch (con & 0b01110000)
    {
    case (0b01100000 )://left 3p
      {
      if ((midst > lmm) && ((c & 0b00111000) == 0) && distreach) //90(sen[2] == 0) && (sen[3] == 0) && (sen[4] == 0)
      {
        stopp();
        Serial.println(F("90left"));
        setnode();
        setline(true, 0b00001010);
        return true;
        //delay(5000);
      }
      if ((midst < lmm) && ((c & 0b00111000) == 0)) //45
      {
        stopp();
        setnode();
        Serial.println(F("45left"));
        setline(true, 0b00001001);
        //delay(5000);
        return true;
      }
      if (((midst > lmm) && (c & 0b00111000) != 0) && distreach) //90+front
      {
        stopp();
        Serial.println(F("90lefttfront"));
        setnode();
        setline(true, 0b10001010);
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
        Serial.println(F("90right"));
        setnode();
        setline(true, 0b00101000);
        // delay(5000);
        return true;
      }
      if ((midst < rmm) && ((c & 0b00111000) == 0)) //45
      {

        stopp();
        Serial.println(F("45right"));
        setnode();
        setline(true, 0b01001000);
        //delay(5000);
        return true;
      }
      if ((midst > rmm) && ((c & 0b00111000) != 0) && distreach) //90+front
      {

        stopp();
        Serial.println(F("90rightfront"));
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
        Serial.println(F("Ycom"));
        setnode();
        setline(true, 0b01001001);
        //delay(5000);
        return true;
      }
      if ((midst < rmm) && (midst > lmm) && ((c & 0b00111000) == 0)) //45
      {
        stopp();
        Serial.println(F("45R 90L"));
        setnode();
        setline(true, 0b01001010);
        //delay(5000);
        return true;
      }
      if ((midst > rmm) && (midst < lmm) && ((c & 0b00111000) == 0)) //45
      {
        stopp();
        Serial.println(F("45L 90R"));
        setnode();
        setline(true, 0b00101001);
        //delay(5000);
        return true;
      }
      if ((midst > rmm) && (midst > lmm) && ((c & 0b01111100) == 0b00000000) && distreach) //90+front
      {
        stopp();
        Serial.println(F("leftright"));
        setnode();
        setline(true, 0b00101010);
        //delay(5000);
        return true;
      }
      if ((midst > rmm) && (midst > lmm) && ((c & 0b00111000) != 0) && distreach) //90+front
      {
        stopp();
        Serial.println(F("leftrightfront"));
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
    Serial.println(F("back"));
    setnode();
    setline(true, 0b00001000);
    //delay(5000);
    return true;
  }
  pdl = dl;
  pdr = dr;
  }
  return false;
}
void trialmaze()
{ ori = 0;

  pr[0] = startX;
  pr[1] = startY;
  setline(true, 0b10000000);
  mazeprint();

  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b10101000);
  floodfill();
  mazeprint();

  ori = 0 ;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b00101010 );
  floodfill();
  mazeprint();

  ori = 6 ;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b00001000 );
  floodfill();
  mazeprint();

  ori = 2 ;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b10101000 );
  floodfill();
  mazeprint();

  ori = 4 ;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b10001010 );
  floodfill();
  mazeprint();

  ori = 2 ;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b00101000 );
  floodfill();
  mazeprint();

  ori = 4 ;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b10001000 );
  floodfill();
  mazeprint();


  ori = 4;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b00101000 );
  floodfill();
  mazeprint();

  ori = 6 ;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b10001010 );
  floodfill();
  mazeprint();
  ori = 6 ;

  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b10001010 );
  floodfill();
  mazeprint();

  ori = 6;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b00101000 );
  floodfill();
  mazeprint();

  ori = 0;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b00101001 );
  mazeprint();

  ori = 7;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b01001000 );
  mazeprint();

  ori = 0 ;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b00101010 );
  floodfill();
  mazeprint();

  ori = 6;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b00001001 );
  floodfill();
  mazeprint();

  ori = 5;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b00001001 );
  floodfill();
  mazeprint();

  ori = 4;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b10001010 );
  floodfill();
  mazeprint();


  ori = 4;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b10001010 );
  floodfill();
  mazeprint();

  ori = 4;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b10001000 );
  floodfill();
  mazeprint();

  ori = 4;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b00001010 );
  floodfill();
  mazeprint();

  ori = 2;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b10001000 );
  floodfill();
  mazeprint();

  ori = 2;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b10001010 );
  floodfill();
  mazeprint();

  ori = 0;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b10101000 );
  floodfill();
  mazeprint();

  ori = 0;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b01001001 );
  floodfill();
  mazeprint();

  ori = 7;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b01001001 );
  floodfill();
  mazeprint();

  ori = 6;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b00101010 );
  floodfill();
  mazeprint();

  ori = 4;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b10001010 );
  floodfill();
  mazeprint();

  ori = 2;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b00101000 );
  mazeprint();

  ori = 4;
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0b11111111 );
  floodfill();
  mazeprint();
  clearline();
  floodfill();
  mazeprint();
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
void set() //initialisation of variables//
{
  endX = 5;
  endY = 2;
  startX = 3;
  startY = 6;
  ori = 0;
  for (int i = 0; i < n; i++)
  { for (int j = 0; j < m; j++)
  { //point[i][j][8] = 255; // (abs(i - endX) + abs(j - endY)) - min(abs(j - endY), abs(i - endX));
    for (int k = 0; k < 8; k++)
    {
    if (((i + negh[k][0]) > -1) && ((j + negh[k][1]) > -1) && ((i + negh[k][0]) < n) && ((j + negh[k][1]) < m)) {
      point[i][j][k] = 10;
    }
    else point[i][j][k] = 0;

    }
  }
  }

}
int sori(int ch) //calculate orientation due to turns//
{
  return (( ori + ch) % 8);
}
int globalori = 0;
void setline( boolean line , byte lines) //set lines are present or not at left front and right//
{

  xn = pr[0] + negh[ori][0];
  ym = pr[1] + negh[ori][1];
  for (int sli = 0; sli < 8; sli++)
  {
  globalori = (ori + sli) % 8;

  point[xn][ym][globalori] = bitRead(lines, (7 - sli));
  }
}
void mazeprint() //print the maze in serial monitor
{

  for (int u = 0; u < (3 * n) ; u++)
  {
  for (int t = 0; t < m ; t++)
  {
    switch ((u % 3))
    {
    case (0):
      {
      if (point[u / 3][t][7]) Serial.print(F("\\ "));
      else Serial.print(F("  "));
      if (point[u / 3][t][0]) Serial.print(F("|"));
      else Serial.print(F(" "));
      if (point[u / 3][t][1]) Serial.print(F(" /"));
      else Serial.print(F("  "));

      break;
      }
    case (1):
      {
      if (point[u / 3][t][6]) Serial.print(F("-"));
      else Serial.print(F(" "));
      if (point[u / 3][t][8] < 10)
      {
        if (((pr[0]  ) == (u / 3) ) && (pr[1] == (t))) Serial.print(ori);
        else Serial.print(F("9"));
      }
      if (point[u / 3][t ][8] < 100) Serial.print(F("0"));
      Serial.print(point[u / 3][t ][8]);
      if (point[u / 3][t][2]) Serial.print(F("-"));
      else Serial.print(F(" "));
      break;
      }
    case (2):
      {
      if (point[u / 3][t][5]) Serial.print(F("/ "));
      else Serial.print(F("  "));
      if (point[u / 3][t][4]) Serial.print(F("|"));
      else Serial.print(F(" "));
      if (point[u / 3][t][3]) Serial.print(F(" \\"));
      else Serial.print(F("  "));
      break;
      }

    };
  }
  Serial.println();

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
void setf()//
{
  Kp = consKpf;
  Ki = consKif;
  Kd = consKdf;
  finals = true;

}
void sett()//
{
  Kp = consKpt;
  Ki = consKit;
  Kd = consKdt;
  finals = false;
}
boolean lineconnect(int x, int y, int lori)//
{
  if (point[x][y][lori] && point[x + negh[lori][0]][y + negh[lori][1]][(lori + 4) % 8]) return true;
  else return false;
}
int dir() //chooses best possible dirction//
{
  byte co[8] = {0, 0, 0, 0, 0, 0, 0, 0}, pd[8] = {0, 0, 0, 0, 0, 0, 0, 0}, mini = 255, maxi = 0, tr[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  int ux[8], uy[8], vx, vy;
  vx = pr[0] + negh[ori][0];
  vy = pr[1] + negh[ori][1];
  for (int u = 0; u < 8; u++)
  {
  ux[u] = vx + negh[u][0];
  uy[u] = vy + negh[u][1];
  }
  for (int u = 0; u < 8; u++)
  {
  if ((lineconnect(vx, vy, u)) && (point[ux[u]][uy[u]][8] <= mini))
  { tr[u] = 1;
    mini = point[ux[u]][uy[u]][8];
  }
  }
  for (int u = 0; u < 8; u++)
  {
  if (point[ux[u]][uy[u]][8] == mini)
    tr[u] += 2;
  }//
  for (int u = 0; u < 8; u++)
  {
  if (tr[u] == 3)
  {
    for (int v = 0; v < 8; v++)
    {
    if (point[ux[u]][uy[u]][v] == 10) co[u]++;
    }

    if (co[u] >= maxi) {
    maxi = co[u];

    }
  }
  }
  for (int u = 0; u < 8; u++)
  {
  if (co[u] == maxi)
    tr[u] += 4;
  }
  Serial.print(F("t"));
  for (int u = 0; u < 8; u++)
  {
  Serial.println(tr[u]);
  }

  if (tr[ori] == 7) return ori;
  else if (tr[sori(7)] == 7) return sori(7);
  else if (tr[sori(1)] == 7) return sori(1);
  else if (tr[sori(6)] == 7) return sori(6);
  else if (tr[sori(2)] == 7) return sori(2);
  else if (tr[sori(5)] == 7) return sori(5);
  else if (tr[sori(3)] == 7) return sori(3);
  else if (tr[sori(4)] == 7) return sori(4);
}
byte save[30][2];
void complete()//
{ delay(5);
  if (true)
  { digitalWrite(13, 1);

  //floodfill();
  address = 500;
  ee = (10 * endX) + endY;
  EEPROM.update(address, ee);

  address = 501;
  ee = (10 * startX) + startY;
  EEPROM.update(address, ee);
  address = 250;
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < m; j++)
    {
    for (int k = 0; k < 8; k++)
    {
      bitWrite(save[i][j], (point[i][j][k] & 0b00000001), k);
    }//if value of line is 10 then line status is unknown
    ee = save[i][j];
    Serial.println(ee);
    EEPROM.update(address, ee);
    address += 1;
    }
  }

  }
  ee = 255;
  Serial.println(ee);
  EEPROM.update(address, ee);
}
void getlines()
{
  int lx, ly;
  ee = EEPROM.read(500);
  endX = int(ee / 10);
  endY = (ee % 10);
  Serial.println(ee);
  ee = EEPROM.read(501);
  startX = int(ee / 10);
  startY = (ee % 10);
  Serial.println(ee);
  address = 250;
  for (int i = 0; i < n; i++)
  {
  for (int j = 0; j <= m; j++)
  {
    //  lue of line is 10 then line status is unknown
    ee = EEPROM.read(address);
    address += 1;
    for (int k = 0; k < 8; k++)
    {
    point[i][j][k] = bitRead(ee, k);
    }//if va
  }
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
    { stepmotor(kds - 1, lspdt, (-rspdt));
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
    { stepmotor(kds - 1, (-lspdt), rspdt );
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
  case (6):
    { stepmotor(2 * kds, (-lspdt), rspdt );
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
void clearline()
{
  for (int i = 0; i < n; i++)
  {
  for (int j = 0; j < m; j++)
  {
    for (int io = 0; io < 8; io++)
    {
    if (point[i][j][io] == 10) point[i][j][io] = 0; //if value of line is 10 then line status is unknown
    }
  }
  }
}
void setnode()
{ if ((dl + dr > 288) && ((ori % 2) == 0)) {
  int extranode = int(((dl + dr) - 100) / 196);
  while (extranode)
  {
    if (trial) {
    setline(true, 0b10001000);
    }
    pr[0] += negh[ori][0];
    pr[1] += negh[ori][1];
    extranode--;
    Serial.println(F("setnode"));
  }
  }
}

void resetconditions()
{
  for (int ci = 0; ci < 4; ci++)
  {
  mean[ci][0] = 0;
  mean[ci][1] = 0;
  }
  conset = 0;
  endr = 0;
  endc = 0;
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
void finalrun() {
  Serial.print("getting lines");
  set();
  getlines();
  clearline();
  floodfill();
  mazeprint();
  //  nextnode();
  trial = false;
  finalr = true;
  ori = 0;
  pr[0] = startX - negh[ori][0];
  pr[1] = startY - negh[ori][1];
  ori = 0;
  setline(true, 0b10000000);
  mazeprint();
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
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
  /*else {
    if (conset) {

    stepmotor(1, lspd, rspd);
    //stopp();
    cpm--;
    }
    }*/


  gen();
  } while (!endr);
  if (endr) {
  digitalWrite(13, 1);
  stopp();
  while (1)
  {}
  }
}
void trialrun() {
  set();
  trial = true;
  finalr = false;
  pr[0] = startX - negh[ori][0];
  pr[1] = startY - negh[ori][1];
  ori = 0;
  setline(true, 0b10000000);
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
  /*else {
    if (conset) {

    stepmotor(1, lspd, rspd);
    //stopp();
    cpm--;
    }
    }*/


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

void setup()
{
  attachInterrupt(0, spl, HIGH);
  attachInterrupt(1, spr, HIGH);

  Serial.begin(57600);

  //pidcheck();
  //generate();
  //floodfill();
  //mazeprint();
  //trialmaze();

  //floodfill();
  //mazeprint();
  // trialmaze();

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
  //stepmotor(1, lspd, rspd);
  trialrun();
  //motor(lspd, rspd);
  //delay(2000);
  //pidcheck();

}
int sti = 0;

void loop() {
  //stepmotor(1, lspd, rspd);
  //Serial.println(sti++);
  /*sense();
  conditions();
  if (conditions()) delay(5000);
  gen();

     getParam();
     sense();
     conditions();
     gen();
     //mazeprint();*/
}

