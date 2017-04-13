#include <math.h>

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

void setup() {
Serial.begin(9600);
double year, m, day, h, latit, doubleit;

year = 2017;

m = 1;
day = 1;
h = 1;
latit = 1;
doubleit = 1;

double UT = h;
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

Serial.println(String(azim*degs) + " azim");
Serial.println(String(altit) + " altit");

}

void loop() {


}
