#define stepspermmx 160.0
#define stepspermmy 160.0
#define stepspermmz 5.0
#define max_speed 200
#define min_speed 200

#include <Stepper.h>
#include "DRV8834.h"
#include "A4988.h"
#include "DRV8825.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

// All the wires needed for full functionality
#define DIRX 3
#define STEPX 4
#define ENBLX 5

#define DIRY 6
#define STEPY 7
#define ENBLY 8

#define MODE0 2
#define MODE1 2
#define MODE2 2

DRV8825 stepperX(MOTOR_STEPS, DIRX, STEPX, MODE0, MODE1, MODE2);
DRV8825 stepperY(MOTOR_STEPS, DIRY, STEPY, MODE0, MODE1, MODE2);

Stepper stepperZ(200, 10, 11, 12, 13);

String data;
int x_destination, y_destination, z_destination, x_source, y_source, z_source, dy, dx, sx, sy, pk;
String g, m, x, y, z, r, ig, k;
int G, M, ydir, xdir;
double  X, Y, Z, R, I, K;
boolean increment_mode = false;

void gotop(float xx, float yy, float zz)
{
  sx = sy = 0;
  x_destination = stepspermmx * xx;
  y_destination = stepspermmy * yy;
  z_destination = stepspermmz * zz;
  dy = y_destination - y_source;
  dx = x_destination - x_source;
  if (dx < 0) xdir = -1;
  else xdir = 1;
  if (dy < 0) ydir = -1;
  else ydir = 1;
  dx *= xdir;
  dy *= ydir;
  printstep(0, 0, (z_destination - z_source));
  if (dx > dy)
  {
    pk = 2 * dy - dx;

    while (sx != dx)
    {
      if (pk < 0)
      {
        pk = pk + 2 * dy;
        sx++;
        printstep(xdir, 0, 0);
      }
      else
      {
        pk = pk + 2 * dy - 2 * dx;
        sx++;
        sy++;
        printstep(xdir, ydir, 0);
      }
    }
  }
  else
  {
    pk = 2 * dx - dy;

    while (sy != dy)
    {
      if (pk < 0)
      {
        pk = pk + 2 * dx;
        sy++;
        printstep(0, ydir, 0);
      }
      else
      {
        pk = pk + 2 * dx - 2 * dy;
        sy++;
        sx++;
        printstep(xdir, ydir, 0);
      }
    }
  }

  x_source = x_destination;
  y_source = y_destination;
  z_source = z_destination;
}
/*void circle(xcenter, ycenter, xx, yy)
  {

  x_destination = stepspermmx * xx;
  y_destination = stepspermmy * yy;

  sx = sy = 0;
  radius = pow((pow((xcenter - x_source), 2)) + (pow((ycenter - y_source), 2)), 0.5);
  pk = 1.25 - radius;
  x = 0;
  y = radius;

  while (y >= x)
  {
    if (pk < 0)
    {
      pk = pk + 2 * x + 3;
      x++;

    }
    else
    {
      pk = pk + 2 * x + 3 + (((y - 1) * (y - 1)) - (y * y)) - ((y - 1) - y);
      //pk=pk+2*x+3+(-2)*y+2;        //parenthesis matters a lot XD
      x++;
      y--;
      printstep(1, -1);
    }
  }
  }
  void circle(xcenter, ycenter, xx, yy)
  {
  // Centre coordinates are always relative
  float angleA, angleB, angle, radius, length, aX, aY, bX, bY;
  radius = pow((pow((xcenter - x_source), 2)) + (pow((ycenter - y_source), 2)), 0.5);
  x_destination = stepspermmx * xx;
  y_destination = stepspermmy * yy;

  aX = (current_units.x - xcenter);
  aY = (current_units.y - ycenter);
  bX = (x_destination - xcenter);
  bY = (y_destination - ycenter);

  if (code == 2) { // Clockwise
    angleA = atan2(bY, bX);
    angleB = atan2(aY, aX);
  } else { // Counterclockwise
    angleA = atan2(aY, aX);
    angleB = atan2(bY, bX);
  }
  // Make sure angleB is always greater than angleA
  // and if not add 2PI so that it is (this also takes
  // care of the special case of angleA == angleB,
  // ie we want a complete circle)
  if (angleB <= angleA) angleB += 2 * M_PI;
  angle = angleB - angleA;

  radius = sqrt(aX * aX + aY * aY);
  length = radius * angle;
  int steps, s, step;
  steps = (int) ceil();

  FloatPoint newPoint;
  for (s = 1; s <= steps; s++) {
    step = (G == 3) ? s : steps - s; // Work backwards for CW
    sx = xcenter + radius * cos(angleA + angle * ((float) step / steps));
    sy = ycenter + radius * sin(angleA + angle * ((float) step / steps));
    goto(sx, sy, z_source);

  }*/
void setup_action()
{
  switch (G)
  {
    case (90): { //abs cordinate
        increment_mode = 0;
        break;
      }
    case (91): { //
        increment_mode = 1;
        break;
      }/*
  case ():{//
  break;
  }*/
  }
}
void cordinates()
{
  if (increment_mode)
  {
    X += x_source;
    Y += y_source;
    Z += z_source;
  }
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
            g = "";
            while (instruction[i] != ' ')
            {
              g += instruction[i];
              i++;
            }
            G = g.toInt();
            setup_action();
            break;
          }
        case ('X'):
          {
            x = "";
            i++;
            while (instruction[i] != ' ')
            {
              x += instruction[i];
              i++;
            }
            X = x.toFloat();
            break;
          }
        case ('Y'):
          {
            y = "";
            i++;
            while (instruction[i] != ' ')
            {
              y += instruction[i];
              i++;
            }
            Y = y.toFloat();
            break;
          }
        case ('Z'):
          {
            z = "";
            i++;
            while (instruction[i] != ' ')
            {
              z += instruction[i];
              i++;
            }
            Z = z.toFloat();
            break;
          }
        case ('R'):
          {
            r = "";
            i++;
            while (instruction[i] != ' ')
            {
              r += instruction[i];
              i++;
            }
            R = r.toFloat();
            break;
          }
        case ('M'):
          {
            m = "";
            i++;
            while (instruction[i] != ' ')
            {
              m += instruction[i];
              i++;
            }
            M = m.toInt();
            break;
          }
        case ('I'):
          {
            i++;
            ig = "";
            while (instruction[i] != ' ')
            {
              ig += instruction[i];
              i++;
            }
            I = ig.toFloat();
            break;
          }
        case ('K'):
          {
            i++;
            k = "";
            while (instruction[i] != ' ')
            {
              k += instruction[i];
              i++;
            }
            K = k.toFloat();
            break;
          }
      }
      i++;
    }
    cordinates();
    switch (G)
    {
      case (0): {
          stepperX.setRPM(max_speed);
          stepperY.setRPM(max_speed);
          //micro_step = false;
          gotop(X, Y, Z);
          break;
        }//rapid traverse
      case (1): { //slow traverse
          stepperX.setRPM(min_speed);
          stepperY.setRPM(min_speed);
          //micro_step = true;
          gotop(X, Y, Z);
          break;
        }/*
      case (2): { //clk circle
          break;
        }
      case (3): { //anticlk circle
          break;
        }
      case (): { //
          break;
        }*/
      case (100): { //
          x_source = y_source = z_source = 0;
          break;
        }
    }
  }
}
void stopp()
{
  digitalWrite(ENBLX, 1);
  digitalWrite(ENBLY, 1);
  digitalWrite(10, 0);
  digitalWrite(11, 0);
  digitalWrite(12, 0);
  digitalWrite(13, 0);
}
void printstep(int xxx, int yyy, int zzz)
{
  digitalWrite(ENBLX, 0);
  digitalWrite(ENBLY, 0);
  stepperX.move(xxx);
  stepperY.move(yyy);
  stepperZ.step(zzz);
}
void setup()
{
  Serial.begin(115200);
  stepperX.setRPM(min_speed);
  stepperY.setRPM(min_speed);
  stepperZ.setSpeed(100);
  stepperX.setMicrostep(32);
  stepperY.setMicrostep(32);
  digitalWrite(ENBLX, 1);
  digitalWrite(ENBLY, 1);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  stopp();
}
void loop()
{
  if (Serial.available())
  {
    data = Serial.readString();
    data += ' ';
    process_string(data);
    stopp();
    Serial.println("OK");
  }
}
/*
  G00 X2.0 Y2.0
*/
