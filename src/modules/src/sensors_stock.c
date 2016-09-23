/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * sensors_stock.c - Crazyflie stock sensor acquisition function
 */
#include "sensors.h"

#include "imu.h"
#ifdef PLATFORM_CF1
  #include "ms5611.h"
#else
  #include "lps25h.h"
#endif

#include "param.h"

static point_t position;
static point_t pos_worker;
static velocity_t vel_worker;
static uint32_t last_worker_update_n = 0;

#define IMU_RATE RATE_500_HZ
#define BARO_RATE RATE_100_HZ

void sensorsInit(void)
{
 imu6Init();
}

bool sensorsTest(void)
{
 bool pass = true;

 pass &= imu6Test();

 return pass;
}

void sensorsAcquire(sensorData_t *sensors, const uint32_t tick)
{
  if (RATE_DO_EXECUTE(IMU_RATE, tick)) {
    imu9Read(&sensors->gyro, &sensors->acc, &sensors->mag);
  }

 if (RATE_DO_EXECUTE(BARO_RATE, tick) && imuHasBarometer()) {
#ifdef PLATFORM_CF1
    ms5611GetData(&sensors->baro.pressure,
                 &sensors->baro.temperature,
                 &sensors->baro.asl);
#else
    lps25hGetData(&sensors->baro.pressure,
                 &sensors->baro.temperature,
                 &sensors->baro.asl);
#endif
    // Experimental: receive the position from parameters
    if (position.timestamp) {   // LPS - Part of original firmware
      sensors->position = position;
    }
    if (vel_worker.timestamp != last_worker_update_n) { // Spotter sending update to worker
      sensors->pos_worker = pos_worker;
      sensors->vel_worker = vel_worker;
      last_worker_update_n = vel_worker.timestamp;
    }
  }
}

bool sensorsAreCalibrated()
{
  Axis3f dummyData;
  imu6Read(&dummyData, &dummyData);
  return imu6IsCalibrated();
}

PARAM_GROUP_START(lps)
PARAM_ADD(PARAM_UINT32, t, &position.timestamp)
PARAM_ADD(PARAM_FLOAT, x, &position.x)
PARAM_ADD(PARAM_FLOAT, y, &position.y)
PARAM_ADD(PARAM_FLOAT, z, &position.z)
PARAM_GROUP_STOP(lps)

PARAM_GROUP_START(worker)
PARAM_ADD(PARAM_UINT32, t, &pos_worker.timestamp)
PARAM_ADD(PARAM_UINT32, n, &vel_worker.timestamp)
PARAM_ADD(PARAM_FLOAT, pX, &pos_worker.x)
PARAM_ADD(PARAM_FLOAT, pY, &pos_worker.y)
PARAM_ADD(PARAM_FLOAT, pZ, &pos_worker.z)
PARAM_ADD(PARAM_FLOAT, vX, &vel_worker.x)
PARAM_ADD(PARAM_FLOAT, vY, &vel_worker.y)
PARAM_ADD(PARAM_FLOAT, vZ, &vel_worker.z)
PARAM_GROUP_STOP(worker)
