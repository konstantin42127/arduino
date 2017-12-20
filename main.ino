#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <Arduino.h>
#include <Wire.h>

#define DIR 5
#define PWM 6

#define BUTTON0 8 // против часовой
#define BUTTON1 9 // север
#define BUTTON2 10 // по часовой

void(* resetFunc) (void) = 0;

int pinA = 3;  // номер вывода, подключенный к CLK енкодера
int pinB = 2;  // номер вывода контроллера, подключенный к DT енкодера
int encoderPosCount = 0;
int pinALast;
int aVal;
boolean bCW;

static const int RXPin = 12, TXPin = 11;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

double pi = 3.14159265359;

double tpi = 2 * pi;
double degs = 180.0 / pi;
double rads = pi / 180.0;

double L, RA, g, daylen, delta, x, y, z;
double SunDia = 0.53;

double AirRefr = 34.0 / 60.0;

double FNday (double y, double m, double d, double h) {
  double luku = - 7 * (y + (m + 9) / 12) / 4 + 275 * m / 9 + d;
  luku += (double)y * 367;
  return (double)luku - 730530.0 + h / 24.0;
};

double FNrange (double x) {
  double b = x / tpi;

  double a = tpi * (b - (long)(b));

  if (a < 0) a = tpi + a;

  return a;
};

double f0(double lat, double declin) {
  double fo, dfo;
  dfo = rads * (0.5 * SunDia + AirRefr);
  if (lat < 0.0) dfo = -dfo;
  fo = tan(declin + dfo) * tan(lat * rads);
  if (fo > 0.99999) fo = 1.0;
  fo = asin(fo) + pi / 2.0;

  return fo;
};

double FNsun (double d) {
  double w, M, v, r;

  w = 282.9404 + 4.70935E-5 * d;
  M = 356.047 + 0.9856002585 * d;

  L = FNrange(w * rads + M * rads);

  g = FNrange(M * rads);

  double ecc = 0.016709 - 1.151E-9 * d;

  double obliq = 23.4393 * rads - 3.563E-7 * rads * d;
  double E = M + degs * ecc * sin(g) * (1.0 + ecc * cos(g));
  E = degs * FNrange(E * rads);
  x = cos(E * rads) - ecc;
  y = sin(E * rads) * sqrt(1.0 - ecc * ecc);
  r = sqrt(x * x + y * y);
  v = atan2(y, x) * degs;

  double lonsun = v + w;
  lonsun -= 360.0 * (lonsun > 360.0);

  x = r * cos(lonsun * rads);
  y = r * sin(lonsun * rads);
  double yequat = y * cos(obliq);
  double zequat = y * sin(obliq);
  RA = atan2(yequat, x);
  delta = atan2(zequat, sqrt(x * x + yequat * yequat));
  RA *= degs;


  return FNrange(L + 1.915 * rads * sin(g) + .02 * rads * sin(2 * g));
};

double altitude = 0;
double azimuth = 0;

void calculationSunPosition(double year, double m, double day, double h, double latit, double doubleit, double timezone ) {
  double UT = h - timezone;
  double jd = FNday(year, m, day, UT);

  double lambda = FNsun(jd);

  double obliq = 23.4393 * rads - 3.563E-7 * rads * jd;

  double GMST0 = L * degs / 15.0 + 12.0;
  double SIDTIME = GMST0 + UT + doubleit / 15.0;

  double ha = 15.0 * SIDTIME - RA;
  ha = FNrange(rads * ha);
  x = cos(ha) * cos(delta);
  y = sin(ha) * cos(delta);
  z = sin(delta);
  double xhor = x * sin(latit * rads) - z * cos(latit * rads);
  double yhor = y;
  double zhor = x * cos(latit * rads) + z * sin(latit * rads);
  double azim = atan2(yhor, xhor) + pi;
  azim = FNrange(azim);
  double altit = asin(zhor) * degs;

  double alpha = atan2(cos(obliq) * sin(lambda), cos(lambda));

  double equation = 1440 - (L - alpha) * degs * 4;

  ha = f0(latit, delta);

  daylen = degs * ha / 7.5;
  if (daylen < 0.0001) {
    daylen = 0.0;
  }

  double riset = 12.0 - 12.0 * ha / pi - doubleit / 15.0 + equation / 60.0;
  double settm = 12.0 + 12.0 * ha / pi - doubleit / 15.0 + equation / 60.0;
  double noont = riset + 12.0 * ha / pi;
  double altmax = 90.0 + delta * degs - latit;
  if (altmax > 90.0) altmax = 180.0 - altmax;

  noont -= 24 * (noont > 24);

  if (riset > 24.0) riset -= 24.0;
  if (settm > 24.0) settm -= 24.0;

  altitude = altit;
  azimuth = azim * degs;
}

void setup() {

  Wire.begin();
  ss.begin(GPSBaud);

  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);

  pinMode(BUTTON0, INPUT);
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);

  pinMode (pinA, INPUT);
  pinMode (pinB, INPUT);
  /* Считываем значение на выводе A и запоминаем его как последнее значение на этом выводе */
  pinALast = digitalRead(pinA);
  Serial.begin (115200);


}

double R = encoderPosCount; //3690 * D / 360;
double D = R / 3740 * 360;
float heading = D;

bool chekButton = false;
int lasthour = -1;
void loop()
{

  if (digitalRead(BUTTON1) == LOW) {
    Serial.println("Set north");
    resetFunc();
  }
  while (digitalRead(BUTTON0) == LOW) {
    Serial.println("Rotate left");
    digitalWrite(DIR, LOW);
    digitalWrite(PWM, HIGH);
    chekButton = true;
  }
  if (chekButton) {
    digitalWrite(DIR, HIGH);
    digitalWrite(PWM, HIGH);
    chekButton = false;
  }

  while (digitalRead(BUTTON2) == LOW) {
    Serial.println("Rotate right");
    digitalWrite(DIR, HIGH);
    digitalWrite(PWM, LOW);
    chekButton = true;
  }
  if (chekButton) {
    digitalWrite(DIR, HIGH);
    digitalWrite(PWM, HIGH);
    chekButton = false;
  }
  aVal = digitalRead(pinA);

  if (lasthour != gps.time.hour()) {
    calculationSunPosition(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.location.lat(), gps.location.lng(), 0);
    lasthour = gps.time.hour();
  }


  if (aVal != pinALast) { // проверка на изменение значения на выводе А по сравнению с предыдущим запомненным, что означает, что вал повернулся
    // а чтобы определить направление вращения, нам понадобится вывод В.
    if (digitalRead(pinB) != aVal) {  // Если вывод A изменился первым - вращение по часовой стрелке
      encoderPosCount ++;
      bCW = true;
    } else {// иначе B изменил свое состояние первым - вращение против часовой стрелки
      bCW = false;
      encoderPosCount--;
    }
    R = encoderPosCount;
    D = R / 3740 * 360;
    if (encoderPosCount < 0) {
      heading  = D * -1;
    }
    else {
      heading  = D ;
    }

  }
  pinALast = aVal;





  /* Serial.println("Year: " + String(int(gps.date.year())) + "; month: " + String(int(gps.date.month())) + "; day: " + String(int(gps.date.day())) + "; hour: " + String(int(gps.time.hour())));
    Serial.println("Your coordinates: " + String( gps.location.lat()) + " " + String( gps.location.lng()));
    Serial.println("Azimuth (degrees): " + String(azimuth));
    Serial.println("Altitude (degrees):" + String(altitude));(*/
  Serial.println("Azimuth (degrees): " + String(azimuth));
  Serial.print("Heading: \t");
  Serial.println(String(encoderPosCount) + " | " + String(heading));

if (heading > azimuth - 3 && heading < azimuth + 3) {
      Serial.println("Found");
      digitalWrite(DIR, HIGH);
      digitalWrite(PWM, HIGH);
    }
    else {
      if (RotateRightStepCount(azimuth, heading) > RotateLeftStepCount(azimuth, heading)) {
        Serial.println("Rotate left");
        digitalWrite(DIR, LOW);
        digitalWrite(PWM, HIGH);
      }
      else {
        Serial.println("Rotate right");
        digitalWrite(DIR, HIGH);
        digitalWrite(PWM, LOW);
      }
    }

  /* if (-6 < altitude && altitude <= 0) {
     Serial.println("***Civil twilight***");
    }
    if (-12 < altitude && altitude <= -6) {
     Serial.println("***Navigational twilight***");
    }
    if (-18 < altitude && altitude <= -12) {
     Serial.println("***Astronomical twilight***");
    }
    if (altitude <= -18) {
     Serial.println("***Night***");
    }*/

  Serial.println("");

  smartDelay();

}

static void smartDelay()//unsigned long ms)
{
  // unsigned long start = millis();

  while (ss.available())
  {
    gps.encode(ss.read());
  }

}

int RotateRightStepCount( int heading, int azimuth ) {
  int summ = 0;
  int azimuthstart = azimuth;
  while (true) {
    azimuth++;
    if (azimuth >= 360) {
      summ = azimuth;
      azimuth = 0;
    }
    if (azimuth == heading) {
      summ = summ + azimuth - azimuthstart;
      return summ;
      break;
    }
  }
}

int RotateLeftStepCount( int heading, int azimuth ) {
  int summ = 0;
  azimuth--;
  bool swichmod = false;
  while (true) {
    summ++;
    azimuth--;
    if (azimuth <= 0) {
      swichmod = true;
      azimuth = 360;
    }
    if (azimuth == heading) {
      return summ;
      break;
    }
  }
}

