#include <webots/robot.h>
#include <webots/motor.h>
#include <math.h>

#define TIME_STEP 32

int main() {
  wb_robot_init();

  WbDeviceTag motor = wb_robot_get_device("my_motor");

  const double F = 2.0;   // frequency 2 Hz
  double t = 0.0;         // elapsed simulation time

  while (wb_robot_step(TIME_STEP) != -1) {
    const double position = sin(t * 2.0 * M_PI * F);
    wb_motor_set_position(motor, position);
    t += (double)TIME_STEP / 1000.0;
  }

  wb_robot_cleanup();
  return 0;
}
