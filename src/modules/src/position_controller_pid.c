/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
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
 * position_estimator_pid.c: PID-based implementation of the position controller
 */

#include <math.h>
#include "num.h"

#include "commander.h"
#include "log.h"
#include "param.h"
#include "pid.h"
#include "num.h"
#include "position_controller.h"

struct pidInit_s {
  float kp;
  float ki;
  float kd;
};

struct pidAxis_s {
  PidObject pid;

  struct pidInit_s init;
  mode_t previousMode;
  float setpoint;

  float output;
};

struct this_s {
  struct pidAxis_s pidPosX;
  struct pidAxis_s pidPosY;
  struct pidAxis_s pidPosZ;
  struct pidAxis_s pidVelX;
  struct pidAxis_s pidVelY;
  struct pidAxis_s pidVelZ;

  uint16_t thrustBase; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
};

// Maximum roll/pitch angle permited
static float rpLimit = 20;

#define DT 0.01

#ifndef UNIT_TEST
static struct this_s this = {

  .pidPosX = {
    .init = {
      .kp = 0.5,
      .ki = 0,
      .kd = 0
    },
    .pid.dt = DT,
  },

  .pidPosY = {
    .init = {
      .kp = 0.5,
      .ki = 0,
      .kd = 0
    },
    .pid.dt = DT,
  },

  .pidPosZ = {
    .init = {
      .kp = 0.5,
      .ki = 0,
      .kd = 0
    },
    .pid.dt = DT,
  },

  .pidVelX = {
    .init = {
      .kp = 5,
      .ki = 0.1,
      .kd = 0
    },
    .pid.dt = DT,
  },

  .pidVelY = {
    .init = {
      .kp = 5,
      .ki = 0.1,
      .kd = 0
    },
    .pid.dt = DT,
  },

  .pidVelZ = {
    .init = {
      .kp = 5000.0,
      .ki = 1000.0,
      .kd = 0
    },
    .pid.dt = DT,
  },

  .thrustBase = 44000,
};
#endif

static float runPid(float input, struct pidAxis_s *axis, mode_t mode,
                    float setpointPos, float setpointVel, float dt) {
  if (axis->previousMode == modeDisable && mode != modeDisable) {
    if (mode == modeVelocity) {
      axis->setpoint = input;
    } else {
      axis->setpoint = setpointPos;
    }
    pidInit(&axis->pid, axis->setpoint, axis->init.kp, axis->init.ki, axis->init.kd, dt);
  }
  axis->previousMode = mode;

  // This is a position controller so if the setpoint is in velocity we
  // integrate it to get a position setpoint
  if (mode == modeAbs) {
    axis->setpoint = setpointPos;
  } else if (mode == modeVelocity) {
    axis->setpoint += setpointVel * dt;
  }

  pidSetDesired(&axis->pid, axis->setpoint);
  return pidUpdate(&axis->pid, input, true);
}

void positionController(float* thrust, attitude_t *attitude, const state_t *state,
                               const setpoint_t *setpoint)
{
  // X, Y
  float x = runPid(state->position.x, &this.pidPosX, setpoint->mode.x, setpoint->position.x, setpoint->velocity.x, DT);
  float y = runPid(state->position.y, &this.pidPosY, setpoint->mode.y, setpoint->position.y, setpoint->velocity.y, DT);

  if (setpoint->mode.x == modeAbs) {
    x = runPid(state->velocity.x, &this.pidVelX, modeAbs, x, 0, DT);
  }
  if (setpoint->mode.y == modeAbs) {
    y = runPid(state->velocity.y, &this.pidVelY, modeAbs, y, 0, DT);
  }

  float yawRad = state->attitude.yaw * (float)M_PI / 180;
  attitude->pitch = - (x * cosf(yawRad)) - (y * sinf(yawRad));
  attitude->roll =  - (y * cosf(yawRad)) + (x * sinf(yawRad));

  attitude->roll = max(min(attitude->roll, rpLimit), -rpLimit);
  attitude->pitch = max(min(attitude->pitch, rpLimit), -rpLimit);

  // Z
  float newThrust = runPid(state->position.z, &this.pidPosZ, setpoint->mode.z, setpoint->position.z, setpoint->velocity.z, DT);
  if (setpoint->mode.z == modeAbs) {
    newThrust = runPid(state->velocity.z, &this.pidVelZ, modeAbs, newThrust, 0, DT);
  }
  *thrust = newThrust + this.thrustBase;
<<<<<<< HEAD
  if (*thrust > 50000) {
    *thrust = 50000;
=======
  if (*thrust > 55000) {
    *thrust = 55000;
>>>>>>> bitcraze/master
  }
}


LOG_GROUP_START(posCtlAlt)
LOG_ADD(LOG_FLOAT, targetX, &this.pidPosX.setpoint)
LOG_ADD(LOG_FLOAT, targetY, &this.pidPosY.setpoint)
LOG_ADD(LOG_FLOAT, targetZ, &this.pidPosZ.setpoint)
LOG_ADD(LOG_FLOAT, pxP, &this.pidPosX.pid.outP)
LOG_ADD(LOG_FLOAT, pxI, &this.pidPosX.pid.outI)
LOG_ADD(LOG_FLOAT, pxD, &this.pidPosX.pid.outD)
LOG_ADD(LOG_FLOAT, pyP, &this.pidPosY.pid.outP)
LOG_ADD(LOG_FLOAT, pyI, &this.pidPosY.pid.outI)
LOG_ADD(LOG_FLOAT, pyD, &this.pidPosY.pid.outD)
LOG_ADD(LOG_FLOAT, pzP, &this.pidPosZ.pid.outP)
LOG_ADD(LOG_FLOAT, pzI, &this.pidPosZ.pid.outI)
LOG_ADD(LOG_FLOAT, pzD, &this.pidPosZ.pid.outD)
LOG_ADD(LOG_FLOAT, vxP, &this.pidVelX.pid.outP)
LOG_ADD(LOG_FLOAT, vxI, &this.pidVelX.pid.outI)
LOG_ADD(LOG_FLOAT, vxD, &this.pidVelX.pid.outD)
LOG_ADD(LOG_FLOAT, vyP, &this.pidVelY.pid.outP)
LOG_ADD(LOG_FLOAT, vyI, &this.pidVelY.pid.outI)
LOG_ADD(LOG_FLOAT, vyD, &this.pidVelY.pid.outD)
LOG_ADD(LOG_FLOAT, vzP, &this.pidVelZ.pid.outP)
LOG_ADD(LOG_FLOAT, vzI, &this.pidVelZ.pid.outI)
LOG_ADD(LOG_FLOAT, vzD, &this.pidVelZ.pid.outD)
LOG_GROUP_STOP(posCtlAlt)

PARAM_GROUP_START(posCtlPid)
PARAM_ADD(PARAM_FLOAT, pxKp, &this.pidPosX.init.kp)
PARAM_ADD(PARAM_FLOAT, pxKi, &this.pidPosX.init.ki)
PARAM_ADD(PARAM_FLOAT, pxKd, &this.pidPosX.init.kd)
PARAM_ADD(PARAM_FLOAT, vxKp, &this.pidVelX.init.kp)
PARAM_ADD(PARAM_FLOAT, vxKi, &this.pidVelX.init.ki)
PARAM_ADD(PARAM_FLOAT, vxKd, &this.pidVelX.init.kd)

PARAM_ADD(PARAM_FLOAT, pyKp, &this.pidPosY.init.kp)
PARAM_ADD(PARAM_FLOAT, pyKi, &this.pidPosY.init.ki)
PARAM_ADD(PARAM_FLOAT, pyKd, &this.pidPosY.init.kd)
PARAM_ADD(PARAM_FLOAT, vyKp, &this.pidVelY.init.kp)
PARAM_ADD(PARAM_FLOAT, vyKi, &this.pidVelY.init.ki)
PARAM_ADD(PARAM_FLOAT, vyKd, &this.pidVelY.init.kd)

PARAM_ADD(PARAM_FLOAT, pzKp, &this.pidPosZ.init.kp)
PARAM_ADD(PARAM_FLOAT, pzKi, &this.pidPosZ.init.ki)
PARAM_ADD(PARAM_FLOAT, pzKd, &this.pidPosZ.init.kd)
PARAM_ADD(PARAM_FLOAT, vzKp, &this.pidVelZ.init.kp)
PARAM_ADD(PARAM_FLOAT, vzKi, &this.pidVelZ.init.ki)
PARAM_ADD(PARAM_FLOAT, vzKd, &this.pidVelZ.init.kd)

PARAM_ADD(PARAM_UINT16, thrustBase, &this.thrustBase)

PARAM_ADD(PARAM_FLOAT, rpLimit, &rpLimit)
PARAM_GROUP_STOP(posCtlPid)
