/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#include <agribot_localization/AgribotOdometry.h>
#include <ros/ros.h>

AgribotOdometry::AgribotOdometry():
  _fWheelRadius(DEFAULT_WHEEL_RADIUS),
  _fChassisLength(DEFAULT_CHASSIS_LENGTH),
  _bTheta0Received(true),         // On default theta0 is internal state
  _bTheta1Received(false),
  _bTheta2Received(false),
  _bDtSet(false),
  _fX(0.0),
  _fY(0.0),
  _fTheta0(0.0),
  _fTheta1(0.0),
  _fTheta2(0.0)
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
  float s0 = sin(_fTheta0);
  float c0 = cos(_fTheta0);
  float s2 = sin(_fTheta2);
  float c2 = cos(_fTheta2);
  float s01 = sin(_fTheta0 + _fTheta1);
  float c01 = cos(_fTheta0 + _fTheta1);
  float s1_2 = sin(_fTheta1 - _fTheta2);

  // Based on new formulas (complying to ROS coordinates)
  _fLinearVelocityX = _fEta1 * (c01 + (s0 * s1_2 / 2.0 / c2));
  _fLinearVelocityY = _fEta1 * (s01 - (s0 * s1_2 / 2.0 / c2));
  _fAngularVelocityZ = _fEta1 * s1_2 / c2 / _fChassisLength;
}

void AgribotOdometry::vUpdatePositions()
{
  _fX   += _fLinearVelocityX  * _dDt;
  _fY   += _fLinearVelocityY  * _dDt;
  _fTheta0 += _fAngularVelocityZ * _dDt;

  if (_fTheta0 < -M_PI)
  {
    _fTheta0 += (2*M_PI);
  }
  else if (_fTheta0 > M_PI)
  {
    _fTheta0 -= (2*M_PI);
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

float AgribotOdometry::fGetLinearVelocityX()
{
  return _fLinearVelocityX;
}

float AgribotOdometry::fGetLinearVelocityY()
{
  return _fLinearVelocityY;
}

float AgribotOdometry::fGetAngularVelocityZ()
{
  return _fAngularVelocityZ;
}
