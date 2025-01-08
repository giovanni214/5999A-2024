#include "pros/optical.hpp"
#include <cmath>

// Helper function to convert hue to RGB
double hueToRgb(double p, double q, double t) {
    if (t < 0) t += 1;
    if (t > 1) t -= 1;
    if (t < 1.0 / 6.0) return p + (q - p) * 6 * t;
    if (t < 1.0 / 2.0) return q;
    if (t < 2.0 / 3.0) return p + (q - p) * (2.0 / 3.0 - t) * 6;
    return p;
}

// Function to convert HSL to RGB
void hslToRgb(double h, double s, double l, int &r, int &g, int &b) {
    double red, green, blue;

    if (s == 0) {
        // achromatic case (greyscale)
        red = green = blue = l;
    } else {
        double q = (l < 0.5) ? (l * (1 + s)) : (l + s - l * s);
        double p = 2 * l - q;
        red = hueToRgb(p, q, h + 1.0 / 3.0);
        green = hueToRgb(p, q, h);
        blue = hueToRgb(p, q, h - 1.0 / 3.0);
    }

    // Convert to RGB range [0, 255]
    r = round(red * 255);
    g = round(green * 255);
    b = round(blue * 255);
}

int getRingColor(pros::Optical& optical_sensor) {
  double hue = optical_sensor.get_hue() / 360;
  double saturation = optical_sensor.get_saturation();

  int r, g, b;
  hslToRgb(hue, saturation, 0.5, r, g, b);

  if(r > 200) return 0; //red
  else if(b > 180) return 1; //blue
  else return -1;
}

float centiToDegrees(int centidegrees) {
  return (float) centidegrees / 100;
}

/*
optical_sensor.set_led_pwm(100);
  
  lift.move(127);
  //lift.move(127);
  while(true) {
    int color = getRingColor();
    pros::lcd::print(5, "Color: %d", color);

    if(color == 1) {
      pros:pros::c::delay(70);
      lift.brake();
      pros::delay(1000);

      lift.move(127);
    }
    
    pros::delay(20);
	}
*/