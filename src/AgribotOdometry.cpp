#include <AgribotOdometry.h>
#include <ros/ros.h>

AgribotOdometry::AgribotOdometry():
  _fWheelRadius(DEFAULT_WHEEL_RADIUS),
  _fChassisLength(DEFAULT_CHASSIS_LENGTH),
  _bTheta0Received(false),
  _bTheta1Received(false),
  _bTheta2Received(false),
  _bDtSet(false),
  _fX(0.0),
  _fY(0.0),
  _fYaw(0.0),
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

    if(_bDtSet)
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
  ROS_WARN("Theta0: %f\tTheta1: %f\tTheta2: %f\t", _fTheta0, _fTheta1, _fTheta2);

  // Formulas adjusted to ROS coordinate frame standard
  // x=y and y=-x compared to mathematical model derived by Robert Muszyński
  _fLinearVelocityX = cos(_fTheta0 + _fTheta1) * _fEta1;
  _fLinearVelocityY = sin(_fTheta0 + _fTheta1) * _fEta1;
  _fAngularVelocityZ = sin(_fTheta1 - _fTheta2) / cos(_fTheta2)
                        / _fChassisLength * _fEta1;
}

void AgribotOdometry::vUpdatePositions()
{
  // TODO - better integration (?)
  ROS_WARN("%f", _dDt);

  _fX   += _fLinearVelocityX  * _dDt;
  _fY   += _fLinearVelocityY  * _dDt;
  _fYaw += _fAngularVelocityZ * _dDt;
  _fTheta0 = _fYaw;

  if(_fYaw < -M_PI)
  {
    _fYaw += (2*M_PI);
  }
  else if(_fYaw > M_PI)
  {
    _fYaw -= (2*M_PI);
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

float AgribotOdometry::fGetYaw()
{
  return _fYaw;
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
