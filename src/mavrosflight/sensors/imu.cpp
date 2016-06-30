/**
 * \file imu.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <mavrosflight/sensors/imu.h>

namespace mavrosflight
{
namespace sensors
{

bool Imu::correct(mavlink_small_imu_t msg,
                  double *xacc, double *yacc, double *zacc, double *xgyro, double *ygyro, double *zgyro)
{
  *xacc = msg.xacc * ACCEL_SCALE;
  *yacc = msg.yacc * ACCEL_SCALE;
  *zacc = msg.zacc * ACCEL_SCALE;
  *xgyro = msg.xgyro * GYRO_SCALE;
  *ygyro = msg.ygyro * GYRO_SCALE;
  *zgyro = msg.zgyro * GYRO_SCALE;

  return true;
}



} // namespace sensors
} // namespace mavrosflight
