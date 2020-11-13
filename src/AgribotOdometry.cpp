/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_localization/AgribotOdometry.h>
#include <ros/ros.h>

AgribotOdometry::AgribotOdometry()
  : _fWheelRadius(DEFAULT_WHEEL_RADIUS)
  , _fChassisLength(DEFAULT_CHASSIS_LENGTH)
  , _bTheta0Received(true)  // On default theta0 is internal state
  , _bTheta1Received(false)
  , _bTheta2Received(false)
  , _bDtSet(false)
  , _fX(0.0)
  , _fY(0.0)
  , _fTheta0(0.0)
  , _fTheta1(0.0)
  , _fTheta2(0.0)
{
}

int AgribotOdometry::configure()
{
  return 0;
}

bool AgribotOdometry::startup()
{
  return false;
}

bool AgribotOdometry::update()
{
  if (_bTheta0Received && _bTheta1Received && _bTheta2Received)
  {
    vUpdateVelocities();

    if (_bDtSet)
    {
      vUpdatePositions();
    }

    return true;
  }
  return false;
}

bool AgribotOdometry::stop()
{
  return false;
}

bool AgribotOdometry::cleanup()
{
  return false;
}

void AgribotOdometry::vUpdateVelocities()
{
  _agribot_velocities = computeVelocities(_fEta1, _fTheta0, _fTheta1, _fTheta2);
}

agribotVelocities AgribotOdometry::computeVelocities(float eta1, float theta0, float theta1, float theta2)
{
  agribotVelocities agribot_velocities;

  float s0 = sin(theta0);
  float c0 = cos(theta0);
  float s2 = sin(theta2);
  float c2 = cos(theta2);
  float s01 = sin(theta0 + theta1);
  float c01 = cos(theta0 + theta1);
  float s1_2 = sin(theta1 - theta2);

  // Based on new formulas (complying to ROS coordinates)
  agribot_velocities.fLinearVelocityX = eta1 * (c01 + (s0 * s1_2 / 2.0 / c2));
  agribot_velocities.fLinearVelocityY = eta1 * (s01 - (c0 * s1_2 / 2.0 / c2));
  agribot_velocities.fAngularVelocityZ = eta1 * s1_2 / c2 / _fChassisLength;

  return agribot_velocities;
}

void AgribotOdometry::vUpdatePositions()
{
  _fX += _agribot_velocities.fLinearVelocityX * _dDt;
  _fY += _agribot_velocities.fLinearVelocityY * _dDt;
  _fTheta0 += _agribot_velocities.fAngularVelocityZ * _dDt;

  if (_fTheta0 < -M_PI)
  {
    _fTheta0 += (2 * M_PI);
  }
  else if (_fTheta0 > M_PI)
  {
    _fTheta0 -= (2 * M_PI);
  }
}

void AgribotOdometry::vUpdatePositionsWheels(double front_right_radians_diff, double front_left_radians_diff)
{
  double distance_front_right = (_fWheelRadius * front_right_radians_diff);
  double distance_front_left = (_fWheelRadius * front_left_radians_diff);

  double distance_front_center = (distance_front_right + distance_front_left) / 2;

  agribotVelocities agribot_velocities = computeVelocities(distance_front_center, _fThetaWheel, _fTheta1, _fTheta2);

  _fXWheel += agribot_velocities.fLinearVelocityX;
  _fYWheel += agribot_velocities.fLinearVelocityY;
  _fThetaWheel += agribot_velocities.fAngularVelocityZ;

  if (_fThetaWheel < -M_PI)
  {
    _fThetaWheel += (2 * M_PI);
  }
  else if (_fThetaWheel > M_PI)
  {
    _fThetaWheel -= (2 * M_PI);
  }
}

void AgribotOdometry::vSetTheta0(float value)
{
  _fTheta0 = value + THETA0_OFFSET;
  _bTheta0Received = true;
}

void AgribotOdometry::vSetTheta1(float value)
{
  _fTheta1 = value + THETA1_OFFSET;
  _bTheta1Received = true;
}

void AgribotOdometry::vSetTheta2(float value)
{
  _fTheta2 = value + THETA2_OFFSET;
  _bTheta2Received = true;
}

void AgribotOdometry::vSetEta1(float value)
{
  _fEta1 = value;
}

void AgribotOdometry::vSetWheelRadius(float value)
{
  _fWheelRadius = value;
  _wheelDiameter = 2 * value;
}

void AgribotOdometry::vSetWheelbase(double value)
{
  _wheelbase = value;
}

void AgribotOdometry::vSetChassisLength(float value)
{
  _fChassisLength = value;
}

void AgribotOdometry::vSetDt(float value)
{
  _dDt = value;
  _bDtSet = true;
}

float AgribotOdometry::fGetTheta0()
{
  return _fTheta0;
}

float AgribotOdometry::fGetTheta1()
{
  return _fTheta1;
}

float AgribotOdometry::fGetTheta2()
{
  return _fTheta2;
}

float AgribotOdometry::fGetEta1()
{
  return _fEta1;
}

float AgribotOdometry::fGetWheelRadius()
{
  return _fWheelRadius;
}

float AgribotOdometry::fGetChassisLength()
{
  return _fChassisLength;
}

float AgribotOdometry::fGetX()
{
  return _fX;
}

float AgribotOdometry::fGetY()
{
  return _fY;
}

double AgribotOdometry::fGetXWheel()
{
  return _fXWheel;
}
double AgribotOdometry::fGetYWheel()
{
  return _fYWheel;
}
double AgribotOdometry::fGetThetaWheel()
{
  return _fThetaWheel;
}

float AgribotOdometry::fGetLinearVelocityX()
{
  return _agribot_velocities.fLinearVelocityY;
}

float AgribotOdometry::fGetLinearVelocityY()
{
  return _agribot_velocities.fLinearVelocityY;
}

float AgribotOdometry::fGetAngularVelocityZ()
{
  return _agribot_velocities.fAngularVelocityZ;
}
