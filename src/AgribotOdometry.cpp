#include <AgribotOdometry.h>

AgribotOdometry::AgribotOdometry():
  _fWheelRadius(DEFAULT_WHEEL_RADIUS),
  _fChassisLength(DEFAULT_CHASSIS_LENGTH),
  _bTheta0Received(false),
  _bTheta1Received(false),
  _bTheta2Received(false)
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
    _fLinearVelocityX = -sin(_fTheta0 + _fTheta1) * _fWheelRadius * _fEta1;
    _fLinearVelocityY = cos(_fTheta0 + _fTheta1) * _fWheelRadius * _fEta1;
    _fAngularVelocityZ = sin(_fTheta1 - _fTheta2) / cos(_fTheta2) * _fWheelRadius
    / _fChassisLength * _fEta1;

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


void AgribotOdometry::vSetTheta0(float value)
{
  _fTheta0 = value;
  _bTheta0Received = true;
}

void AgribotOdometry::vSetTheta1(float value)
{
  _fTheta1 = value;
  _bTheta1Received = true;
}

void AgribotOdometry::vSetTheta2(float value)
{
  _fTheta2 = value;
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
