#include <limits>
#include "PID.h"

using namespace std;

/*
 * ****************** Reflection ******************
 * Proportional term: 0.12
 *      This term controls how hard the car steers
 *      towards the goal line. A value of 0.12 is
 *      a good sweet-spot because it does not turn
 *      too quickly which causes an erratic driving
 *      pattern, but the car also does not end up
 *      too far off of the goal line.
 *
 *      I am also able to have a lower value in this
 *      case because the car starts in the center of
 *      the lane so the CTE is very small. If the car
 *      started too far away from the center, a small
 *      P term might cause the car to take a long time
 *      to correct itself and it could drive off the
 *      road in that time.
 *
 * Integral term: 0.0
 *      This term controls how quickly the car corrects
 *      for systematic bias. Because the car is simulated
 *      and seems to be programmed to not have any
 *      systematic bias (it drives perfectly straight when
 *      the driving angle is 0 degrees), this term can
 *      be set to 0.0.
 *
 * Differential term: 2.5
 *      This term controls the amount the car counter-steers
 *      as it gets closer to the goal line. A value of 2.5
 *      reduces oscillations in the motion of the car as it
 *      tries to correct itself, but doesn't cause the car to
 *      steer too slowly towards the line.
 *
 * How coefficients were chosen:
 *      All three values were chosen by taking values given
 *      in the lessons and manually tweaking them until they
 *      produced good results. The original values already
 *      caused the controller to perform decently, so this
 *      didn't take too long.
 * ************************************************
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;

    p_error = std::numeric_limits<double>::max();
}

void PID::UpdateError(double cte) {
    if (p_error == std::numeric_limits<double>::max()) {
        p_error = cte;
    }

    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
    return Kp * p_error + Ki * i_error + Kd * d_error;
}

