#include<EEPROM.h>
#define lcolor 1
#define spt 125
#define mKp 1
#define mKd 1
#define mKi 1
#define maxs 1000
#define kdst 7
#define kdsf 7
#define kds2 15
#define threshold 550
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
#define lspdt 168
#define rspdt 180
#define lspdf 238
#define rspdf 255
#define lspdn 165
#define rspdn 170
int n, m, pr[2], node[9][9], negh[4][2] = {{0, -1}, {1, 0}, {0, 1}, { -1, 0}}, neghl[4][2] = {{0, 0}, {1, 0}, {0, 1}, {0, 0}}, lspd = 148, rspd = 165;
int nx, ny, ul, ur, t[6], sm, endX, endY, startX , startY , address, cpm = 10;;
byte verline[9][10], horline[10][9], ori, c;
unsigned long int rr, ll, dr, dl, prevdist;
float consKp, consKi, consKd, P, I, D, input, pinput, output, l, r, rad, krad, sen, turnsen, dis = 3,  dino = 0, turndino = 0;
boolean trial, finalr, endr = false, resetconditionsflag = 0, zeroresetconditionsflag = 0, tur = 0, bc, finals = false, speedfg = true, speedtg = true;
byte con, prevcon, pcon, conset, ee, kds = kdst;
void set() //initialisation of variables
{ n = 9;
  m = 9;
  endX = 1;
  endY = 1;
  startX = 7;
  startY = 7;
  ori = 0;
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < m; j++)
    {
      node[i][j] = 99;
    }
  }
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j <= m; j++)
    {
      verline[i][j] = 10; //if value of line is 10 then line status is unknown
    }
  }

  for (int i = 0; i <= n; i++)
  {
    for (int j = 0; j < m; j++)
    { horline[i][j] = 10; //if value of line is 10 then line status is unknown
    }
  }
}
int sori(int ch) //calculate orientation due to turns
{
  return (( ori + ch) % 4);
}
void setline( boolean line , byte left, byte front, byte right) //set lines are present or not at left front and right
{
  int lk, lx, ly;
  if (left)
  {
    lk = sori(3);
    lx = pr[0] + negh[ori][0] + neghl[lk][0];
    ly = pr[1] + negh[ori][1] + neghl[lk][1];
    if (lk == 0 || lk == 2)
    {
      verline[lx][ly] = line;
    }
    else
    {
      horline[lx][ly] = line;
    }
  }
  if (front)
  {
    lk = sori(0);
    lx = pr[0] + negh[ori][0] + neghl[lk][0];
    ly = pr[1] + negh[ori][1] + neghl[lk][1];
    if (lk == 0 || lk == 2)
    {
      verline[lx][ly] = line;
    }
    else
    {
      horline[lx][ly] = line;
    }
  }
  if (right)
  {
    lk = sori(1);
    lx = pr[0] + negh[ori][0] + neghl[lk][0];
    ly = pr[1] + negh[ori][1] + neghl[lk][1];
    if (lk == 0 || lk == 2)
    {
      verline[lx][ly] = line;
    }
    else
    {
      horline[lx][ly] = line;
    }
  }
}
void solve() //solves the maze and assign node values
{
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < m; j++)
    {
      node[i][j] = 99;
    }
  }
  solven();
  solven();
}
void solven()
{
  node[endX][endY] = 0;
  for (int q = 0; q < n; q++) {
    for (int w = 0; w < m; w++) {
      for (int k = 0; k < 4; k++)
      {
        nx = q + negh[k][0];
        ny = w + negh[k][1];
        if (nx >= 0 && nx < n && ny >= 0 && ny < m) {
          int nlx = q + neghl[k][0];
          int nly = w + neghl[k][1];

          if ((k == 0 || k == 2) && verline[nlx][nly])
          {

            if (node[q][w] + 1 < node[nx][ny])
              node[nx][ny] = node[q][w] + 1;

          }
          if ((k == 1 || k == 3) && horline[nlx][nly])
          {

            if (node[q][w] + 1 < node[nx][ny])
              node[nx][ny] = node[q][w] + 1;
          }
        }
      }
    }
  }
}
/*
  recsolve(ix, iy)
  { if ((ix == startX) && (iy == startY))
  {
    break;
  }
  for
  }
  void solven()
  { node[endX][endY] = 0;
  recsolve
  }*/
void mazeprint() //print the maze in serial monitor
{ Serial.println("inside mazeprint");
  for (int t = 0; t < (m + n + 1); t++)
  {
    int l = (int)(t / 2);

    if (t % 2 == 0) {
      for (int i = 0; i < n; i++)
      {
        Serial.print("    ");
        if (verline[i][l])
          Serial.print("|");
        else
          Serial.print(" ");
      }
    }
    else
    { for (int e = 0; e < (n + m + 1); e++)
      {
        int g = (int)(e / 2);
        if (e % 2 == 0) {
          if (horline[g][l])
            Serial.print("---");
          else
            Serial.print("   ");

        }
        else {
          if ((g == pr[0] + negh[ori][0]) && (l == pr[1] + negh[ori][1]))
          {
            switch (ori)
            {
              case (0): {
                  Serial.print("^");
                  break;
                }
              case (1): {
                  Serial.print(">");
                  break;
                }
              case (2): {
                  Serial.print("√");
                  break;
                }
              case (3): {
                  Serial.print("<");
                  break;
                }
            }
          }
          else
          { if (node[g][l] < 10)
            { Serial.print("0");
              Serial.print(node[g][l]);
            }
            else
              Serial.print(node[g][l]);
          }
        }
      }
    }
    Serial.println();
  }
}
void spl()
{
  dl++;
}
void spr() //interupt function right rotary encoder
{
  dr++;
}
void motor(int left, int right) //drive motor
{
  lm(left);
  rm(right);
}
/*
  }
  lspeed[1] = lspeed[2];
  lspeed[2] = lspeed[3];
  lspeed[3] = lspeed[4];
  if (left == 0)
   lspeed[4] = micros() - plr[0];
  else
   lspeed[4] = maxs / (plr[1] - plr[0]);
  lspeed[0] = (lspeed[1] + lspeed[2] + lspeed[3] + lspeed[4]) / 4;
  rspeed[1] = rspeed[2];
  rspeed[2] = rspeed[3];
  rspeed[3] = rspeed[4];
  if (right == 0)
   rspeed[4] = micros() - prr[0];
  else
   rspeed[4] = maxs / (prr[1] - prr[0]);
  rspeed[0] = (rspeed[1] + rspeed[2] + rspeed[3] + rspeed[4]) / 4;
  if (left != 10000)
   pleft = left;
  if (right != 10000)
   pright = right;
  sleft = left;
  sright = right;
  leftm.Compute();
  rightm.Compute();


  if (l >= 0) {
  digitalWrite(l1, 1);
  digitalWrite(l2, 0);
  analogWrite(sl, l);
  ml = 1;
  }
  else {
  digitalWrite(l1, 0);
  digitalWrite(l2, 1);
  analogWrite(sl, -l);
  ml = -1;
  }
  if (r >= 0) {
  digitalWrite(r1, 1);
  digitalWrite(r2, 0);
  analogWrite(sr, r);
  mr = 1;
  }
  else {
  digitalWrite(r1, 0);
  digitalWrite(r2, 1);
  analogWrite(sr, -r);
  mr = -1;
  }
  }*/
void lm(int lsp)
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
void stopp()
{
  PORTD &= 0b01001111;
  PORTB &= 0b01111110;
}
void rm(int rsp)
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
void setf()
{
  consKp = consKpf;
  consKi = consKif;
  consKd = consKdf;
  lspd = lspdf;
  rspd = rspdf;
  finals = true;

}
void sett()
{
  consKp = consKpt;
  consKi = consKit;
  consKd = consKdt;
  lspd = lspdn;
  rspd = rspdn;
  finals = false;
}
void gen()
{ if (tur)
  {
    if (turndino != 0) {

      input = (-1) * ((turnsen / turndino) - 3);

    }
  }
  else
  {
    if (dino != 0) {
      input = (-1) * ((sen / dino) - 3);
    }
  }
  P = consKp * input;
  I += consKi * input;
  D = consKd * (input - pinput);
  pinput = input;
  output = P + I + D;
  rad = abs(output);
  krad = (dis - (rad)) / (dis + (rad));
  if (output >= 0)
  {

    sm = rspd * krad;
    if (finals)
    {
      sm = map(sm, 0, rspd, 90, rspd);
    }
    rm(sm);
    lm(lspd);
    Serial.print("\t left motor=");
    Serial.print(lspd);
    Serial.print("\t right motor=");
    Serial.print(sm);
    Serial.println();

  }
  else
  {

    sm = lspd * krad;
    if (finals)
    {
      sm = map(sm, 0, lspd, 78, lspd);
    }
    rm(rspd);
    lm(sm);
    Serial.print("\t left motor=");
    Serial.print(sm);
    Serial.print("\t right motor=");
    Serial.print(rspd);
    Serial.println();

  }
}
byte sense() //get status of all sensor
{ c = 0;
  dino = 0;
  turnsen = 0;
  sen = 0;
  if (analogRead(A0) < threshold)
  { c = c + 0b00000001;
    turnsen++;
    turndino++;
  }
  if (analogRead(A5) < threshold)
  { turnsen += 5;
    c = c + 0b00010000;
    turndino++;
  }
  if (analogRead(A2) < threshold)
  {
    turnsen += 2.0;
    sen += 2.0;
    c = c + 0b00000010;
    dino++;
    turndino++;
  }
  if (analogRead(A3) < threshold)
  {
    turnsen += 3.0;
    sen += 3.0;
    c = c + 0b00000100;
    dino++;
    turndino;
  }
  if (analogRead(A4) < threshold)
  {
    turnsen += 4.0;
    sen += 4.0;
    c = c + 0b00001000;
    dino++;
    turndino++;
  }
  if ((analogRead(A1)) < 60)
    endr = true;
  return c;
}
void floodfill() {
}
int dir() //chooses best possible dirction
{
  byte co[4] = {0, 0, 0, 0}, pd[4] = {0, 0, 0, 0}, mini = 255, max = 0, tr[4] = {0, 0, 0, 0};
  int ux[4], uy[4], vx, vy;
  vx = pr[0] + negh[ori][0];
  vy = pr[1] + negh[ori][1];
  for (int u = 0; u < 4; u++)
  {
    ux[u] = vx + negh[u][0];
    uy[u] = vy + negh[u][1];
  }

  for (int u = 0; u < 4; u++)
  {
    if (u % 2 == 0)
    { if (((verline[vx + neghl[u][0]][vy + neghl[u][1]]) == 1) && (node[ux[u]][uy[u]] <= mini))
      { tr[u] = 1;
        mini = node[ux[u]][uy[u]];
      }
    }
    else
    { if (((horline[vx + neghl[u][0]][vy + neghl[u][1]]) == 1) && (node[ux[u]][uy[u]] <= mini))
      { tr[u] = 1;
        mini = node[ux[u]][uy[u]];
      }
    }
  }

  for (int u = 0; u < 4; u++)
  {
    if (node[ux[u]][uy[u]] == mini)
      tr[u] += 2;
  }

  for (int u = 0; u < 4; u++)
  {
    if (tr[u] == 3)
    {
      for (int v = 0; v < 4; v++)
      {

        if (v % 2 == 0)
        { if (verline[ux[u] + neghl[v][0]][uy[u] + neghl[v][1]] == 10) co[u]++;
        }
        else
        { if (horline[ux[u] + neghl[v][0]][uy[u] + neghl[v][1]] == 10) co[u]++;
        }
      }

      if (co[u] >= max) {
        max = co[u];

      }
    }
  }

  for (int u = 0; u < 4; u++)
  {
    if (co[u] == max)
      tr[u] += 4;
  }
  Serial.print("t");
  Serial.print(tr[0]);
  Serial.print(tr[1]);
  Serial.print(tr[2]);
  Serial.print(tr[3]);
  if (tr[ori] == 7) return ori;
  else if (tr[sori(3)] == 7) return sori(3);
  else if (tr[sori(1)] == 7) return sori(1);
  else if (tr[sori(2)] == 7) return sori(2);
}

void complete()
{ delay(5);
  if (true)
  { digitalWrite(13, 1);

    for (int i = 0; i < n; i++)
    {
      for (int j = 0; j <= m; j++)
      {
        if (verline[i][j] == 10) verline[i][j] = 0; //if value of line is 10 then line status is unknown
      }
    }

    for (int i = 0; i <= n; i++)
    {
      for (int j = 0; j < m; j++)
      { if (horline[i][j] == 10) horline[i][j] = 0; //if value of line is 10 then line status is unknown
      }
    }

    solve();
    address = 500;
    ee = (10 * endX) + endY;
    EEPROM.update(address, ee);
    address = 250;
  }
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j <= m; j++)
    {
      if (verline[i][j] == 1)
      {
        Serial.print("write into eeeprom ");
        ee = ((10 * i) + j);
        Serial.println(ee);
        EEPROM.update(address, ee);
        address += 1;
      }
    }
  }
  Serial.print("write into eeeprom");
  ee = 255;
  Serial.println(ee);
  EEPROM.update(address, ee);

  address = 750;
  for (int i = 0; i <= n; i++)
  {
    for (int j = 0; j < m; j++)
    {
      if (horline[i][j] == 1)
      {
        Serial.print("write into eeeprom");
        ee = ((10 * i) + j);
        Serial.println(ee);
        EEPROM.update(address, ee);
        address += 1;
      }
    }
  }

  Serial.print("write into eeeprom ");
  ee = 255;
  Serial.println(ee);
  EEPROM.update(address, ee);
}
void setnode()
{ if (dl + dr > 144) {
    int extranode = int(((dl + dr) - 140) / 96);
    while (extranode)
    {
      if (trial) {
        setline(1, 0, 1, 0);
        setline(0, 1, 0, 1);
      }
      pr[0] += negh[ori][0];
      pr[1] += negh[ori][1];
      extranode--;

    }
  }
}
void resetconditions()
{
  con = 4;
  prevcon = 4;
  conset = 0;
  resetconditionsflag = 0;
  prevdist = dl + dr;
  bc = false;
}
void zeroresetconditions()
{
  conset = 0;
  zeroresetconditionsflag = 0;
  prevdist = dl + dr;
  prevcon = 4;
  bc = false;
}
void concheck(byte f)
{

  if ((bitRead(f, 1) || bitRead(f, 2) || bitRead(f, 3)) && (!bitRead(f, 0) && (!bitRead(f, 4)))) con = 4 ;
  else if (((0b00011111) & (f)) == 0b00000000) con = 0;
  else if (((0b00010001) & (f)) == 0b00010000) con = 2;
  else if (((0b00010001) & (f)) == 0b00010001) con = 1;
  else if (((0b00010001) & (f)) == 0b00000001) con = 3;
  else ;
  if (((con == 1) || (con == 2) || (con == 3)) && (conset != 1))
  { conset = 1;
    if (finalr)
      cpm = 1;
    else
      cpm = 3;
    prevdist = dl + dr;
    motor(-lspd, -rspd);
    delay(5);
    stopp();
  }
}
boolean conditions(byte ert)
{
  boolean flag = false;
  concheck(ert);
  Serial.print("\t pcon=");
  Serial.print(pcon);
  Serial.print("\t con=");
  Serial.print(con);
  Serial.print("\t prevcon=");
  Serial.print(prevcon);
  switch (pcon)
  { case (4):
      { switch (con)
        { /*case (0):
                {
                  prevcon = 4 * (1 - conset);
                  break;
            }*/
          case (1):
            {
              prevcon = 4 - (conset * 3);
              break;
            }
          case (2):
            {
              prevcon = 2 + ((prevcon - 4) * conset);

              break;
            }
          case (3):
            {
              prevcon = 3 + ((prevcon - 4) * conset);
              break;
            }
        }
        break;
      }
    case (1):
      { switch (con)
        { case (0):
            {
              prevcon = conset;
              break;
            }
          case (4):
            {
              prevcon = conset;
              break;
            }
          case (2):
            {
              prevcon = conset;
              break;
            }
          case (3):
            {
              prevcon = conset;
              break;
            }
        }
        break;
      }
    case (2):
      { switch (con)
        { /* case (4):
             {
               prevcon = prevcon(1-conset);
               break;
             }*/
          case (1):
            {
              prevcon = 1;
              break;
            }
        }
        break;
      }

    case (3):
      { switch (con)
        { /*case (4):
            {
              prevcon = 3;
              break;
            }*/
          case (1):
            {
              prevcon = 1;
              break;
            }
        }
        break;
      }
  }

  Serial.print("\t prevcon=");
  Serial.println(prevcon);

  if (con == 0)
  {
    if (prevcon == 4)
    {
      setnode();
      if (trial) {
        setline(1, 0, 0, 0);
        setline(0, 1, 1, 1);
      }
      flag = true; //b
      zeroresetconditionsflag = 1;

    }
    if ((cpm < 3) && (conset)) { //|| finalr) {
      switch (prevcon)
      {
        case (3):
          { setnode();
            if (trial) {
              setline(1, 0, 0, 1);
              setline(0, 1, 1, 0);
            }
            flag = true; //r
            zeroresetconditionsflag = 1;
            break;
          }
        case (2):
          { setnode();
            if (trial) {
              setline(1, 1, 0, 0);
              setline(0, 0, 1, 1);
            }
            flag = true; //l
            zeroresetconditionsflag = 1;
            break;
          }
        case (1):
          { setnode();
            if (trial) {
              setline(1, 1, 0, 1);
              setline(0, 0, 1, 0);
            }
            flag = true; //rl
            zeroresetconditionsflag = 1;
            break;
          }
      }
    }
  }
  if (con == 4 && (bc))
  {
    if (cpm < 1) { //|| finalr) {
      switch (prevcon) //check for previous sensor status
      {

        case (2):
          {
            setnode();
            if (trial) {
              setline(1, 1, 1, 0);
              setline(0, 0, 0, 1);
            }
            flag = true; //fl
            resetconditionsflag = 1;
            break;
          }
        case (3):
          {
            setnode();
            if (trial) {
              setline(1, 0, 1, 1);
              setline(0, 1, 0, 0);
            }
            flag = true;     //fr
            resetconditionsflag = 1;
            break;
          }
        case (1):
          {
            setnode();
            if (trial) {
              setline(1, 1, 1, 1);
              setline(0, 0, 0, 0);
            }
            flag = true; //frl
            resetconditionsflag = 1;
            break;
          }
      }
    }
  }
  if (con == 4) bc = true;
  pcon = con;
  if (zeroresetconditionsflag) zeroresetconditions();
  if (resetconditionsflag) resetconditions();
  return flag;
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
      { stepmotor(kds, lspdt, (-rspdt));
        sense();
        while (!(bitRead(c, 2)))
        { stepmotor(1, lspdt, (-rspdt));
          sense();
        }
        break;
      }
    case (3):
      { stepmotor(kds, (-lspdt), rspdt );
        sense();
        while (!(bitRead(c, 2)))
        { stepmotor(1, (-lspdt), rspdt );
          sense();
        }
        break;
      }
    case (2):
      {
        stepmotor(kds2, lspdt , -rspdt);
        sense();
        while (!(bitRead(c, 2)))
        { stepmotor(1, lspdt, (-rspdt));
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
      Serial.print("s");
      do
      {
        sense();
        gen();
        Serial.print("\t");
        Serial.print(dl);
        Serial.print("\t");
        Serial.print(dr);
      }
      while (((dl + dr - prevdist) < 3) || (!(bitRead(c, 2))) ); //|| (bitRead(c, 1)) || (bitRead(c, 2)) || (bitRead(c, 3))
    }*/
  Serial.print("s");
  motor(0, 0);
  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  ori = sori(tu);
  tur = 0;
  dis = 3;
}
byte tyu;
void trialrun() {
  set();
  sett();
  trial = true;
  finalr = false;
  pr[0] = startX;
  pr[1] = startY;
  ori = 0;
  verline[7][7] = 1;
  delay(1000);
  do
  { sense();
    if ((dl + dr) > 48) {
      if (conditions(c))
      { digitalWrite(13, 1);
        motor(-lspd, 0);
        delay(5);
        motor(-lspd, -rspd);
        delay(10);
        stopp();
        solve();
        mazeprint();
        tyu = ((dir() - ori) + 4) % 4;
        turn(tyu);
        dl = 0;
        dr = 0;
        digitalWrite(13, 0);
      }
      else {
        if (conset == 1) {

          stepmotor(1, lspd, rspd);
          //stopp();
          cpm--;
        }
      }
      if (con == 0) {
        motor(-lspd, -rspd);
        stopp();
      }
    }
    if (conset == 0)
      gen();
  } while (!endr);
  if (endr) {
    digitalWrite(13, 1);
    stopp();
    pr[0] += negh[ori][0];
    pr[1] += negh[ori][1];
    endX = pr[0];
    endY = pr[1];
    clearline();
    solve();
    mazeprint();
    complete();
    while (1)
    {}
  }
}
void clearline()
{
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j <= m; j++)
    {
      if (verline[i][j] == 10) verline[i][j] = 0; //if value of line is 10 then line status is unknown
    }
  }

  for (int i = 0; i <= n; i++)
  {
    for (int j = 0; j < m; j++)
    { if (horline[i][j] == 10) horline[i][j] = 0; //if value of line is 10 then line status is unknown
    }
  }
}
boolean orichange[9][9];
void nextnode()
{ pr[0] = startX;
  pr[1] = startY;
  Serial.println("inside nextnode");
  do {
    tyu = ((dir() - ori) + 4) % 4;
    if (tyu == 0)
      orichange[pr[0]][pr[1]] = 0;
    else {
      orichange[pr[0]][pr[1]] = 1;
    }
    Serial.println(orichange[pr[0]][pr[1]]);
    pr[0] += negh[ori][0];
    pr[1] += negh[ori][1];
    ori = sori(tyu);
  }
  while ((pr[0] != endX) && (pr[1] != endY));

}
unsigned long int ndl = 48, ndr = 48;

void finalrun()
{
  Serial.print("getting lines");
  set();
  getlines();
  clearline();
  solve();
  mazeprint();
  nextnode();
  trial = false;
  finalr = true;
  pr[0] = startX;
  pr[1] = startY;
  ori = 0;
  sett();
  speedfg = true;
  speedtg = true;
  kds = kdst;
  dl = 0;
  dr = 0;
  do
  { sense();
    if ((dl + dr) > 48) {
      if (conditions(c))
      { digitalWrite(13, 1);
        motor(-lspd, 0);
        delay(5);
        motor(-lspd, -rspd);
        delay(10);
        stopp();
        tyu = ((dir() - ori) + 4) % 4;
        turn(tyu);
        dl = 0;
        dr = 0;
        digitalWrite(13, 0);
      }
      else {
        if (conset == 1) {

          stepmotor(1, lspd, rspd);
          //stopp();
          cpm--;
        }
      }
      if (con == 0) {
        motor(-lspd, -rspd);
        stopp();
      }
    }
    if (conset == 0)
      gen();
  } while (!endr);
  if (endr) {
    digitalWrite(13, 1);
    stopp();
    while (1)
    {}
  }
}
/*
  do
  { sense();
  if ((dl == 0) && (dr == 0))
    speedfg = true;
  Serial.println(speedfg);
  Serial.println(speedtg);
  if (((dl + dr) > 18) && (speedfg) && (input == 0))
  {
    Serial.print("sp");
    setf();
    speedfg = false;
  }
  Serial.println(dl);
  Serial.println(dr);
  if (orichange[pr[0]][pr[1]]) {
    if (((dl + dr) > 60 ) && speedtg)  {
      motor(-255, -255);
      delay(7);
      stopp();
      sett();
      speedtg = false;
    }
    if ((conditions(c)) && (!speedtg))
    { digitalWrite(13, 1);
      Serial.println(dl);
      Serial.println(dr);
      motor(-255, -255);
      if ((lspd + rspd) > 350)
        delay(5);
      else
        delay(2);
      stopp();
      //solve();
      mazeprint();
      tyu = ((dir() - ori) + 4) % 4;
      turn(tyu);
      dl = 0;
      dr = 0;
      speedtg = true;
      speedfg == true;
      conset = 0;
      digitalWrite(13, 0);
    }
    else if (conset == 1) {
      stepmotor(1, lspd, rspd);
      cpm--;
    }
    if (con == 0) {
      motor(-255, -255);
      delay(2);
      sett();
      stopp();
      Serial.print("ze");
    }
  }
  else
  {
    if ((conset != 1) && ((dl + dr) > 48))
    { concheck(c);
      if (conset == 1)
      {
        stepmotor(2, lspd, rspd);
        conset = 0;
        Serial.println("ncdnswrt");
      }
    }
    if ((dl + dr) > 144)
    { Serial.println("ncdns");
      pr[0] += negh[ori][0];
      pr[1] += negh[ori][1];
      dl -= ndl;
      dr -= ndr;
      ndl = 48;
      ndr = 48;
    }
  }
  if (conset == 0)
    gen();
  } */


void getlines()
{
  int lx, ly;
  ee = EEPROM.read(500);
  endX = int(ee / 10);
  endY = (ee % 10);
  Serial.println(ee);
  address = 250;
  while (1) {
    ee = EEPROM.read(address);
    if (ee == 255) break;
    lx = int(ee / 10);
    ly = (ee % 10);
    Serial.println(ee);

    verline[lx][ly] = 1;
    address++;
  }
  address = 750;
  while (1) {
    ee = EEPROM.read(address);
    if (ee == 255) break;
    lx = int(ee / 10);
    ly = (ee % 10);
    Serial.println(ee);
    horline[lx][ly] = 1;
    address++;
  }
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
void trialmaze()
{ ori = 0;

  verline[6][6] = true;
  pr[0] = startX;
  pr[1] = startY;
  setline(true, 1, 1, 0);
  setline(false, 0, 0, 1);

  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 1, 1, 0);
  setline(false, 0, 0, 1);

  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0, 1, 0);
  setline(false, 1, 0, 1);

  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0, 1, 0);
  setline(false, 1, 0, 1);

  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 1, 1, 0);
  setline(false, 0, 0, 1);

  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 1, 0, 0);
  setline(false, 0, 1, 1);

  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  ori = 3;
  setline(true, 1, 0, 0);
  setline(false, 0, 1, 1);

  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  ori = 2;
  setline(true, 1, 1, 1);
  setline(false, 0, 0, 0);

  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  ori = 3;
  setline(true, 0, 1, 1);
  setline(false, 1, 0, 0);

  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0, 1, 1);
  setline(false, 1, 0, 0);

  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 0, 1, 1);
  setline(false, 1, 0, 0);

  pr[0] += negh[ori][0];
  pr[1] += negh[ori][1];
  setline(true, 1, 0, 1);
  setline(false, 0, 1, 0);

}
int p = 0;
void setup()
{
  attachInterrupt(0, spl, HIGH);
  attachInterrupt(1, spr, HIGH);
  pinMode(12, INPUT_PULLUP);
  pinMode(endp, INPUT_PULLUP);
  Serial.begin(19200);
  pinMode(13, 1);
  pinMode(l1, 1);
  pinMode(l2, 1);
  pinMode(sl, 1);
  pinMode(r1, 1);
  pinMode(r2, 1);
  pinMode(sr, 1);
  delay(2500); setmode();
  digitalWrite(13, psy);
  delay(3000);
  while (digitalRead(12) == 1)
  {}
  resetconditions();
  digitalWrite(13, 0);
  digitalWrite(13, 1);
  digitalWrite(13, 0);
  delay(500);

}
void loop() {


  /* sense();
    Serial.print(input);
    Serial.print("\t");
    Serial.print(output);
    gen();
    Serial.print("\t");
    Serial.print(dl);
    Serial.print("\t");
    Serial.print(dr);
    Serial.println();

  */

  if (psy)
  {
    finalrun();
  }
  else
  {
    trialrun();
  }
  /*
    if (Serial.available())
    {
    byte plus = Serial.read();
    if (plus)
      nio++;

    }
    Serial.println(nio);
    motor(0,nio);
  */
}

/*void loop()
  {
  delay(8000);
  Serial.println("printing ");
  set();
  mazeprint();
  trialmaze();
  mazeprint();
  complete();
  mazeprint();
  delay(10000);
  }*/

