/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_common/AgribotComponent.h>
#include <cmath>

static const float DEFAULT_WHEEL_RADIUS = 0.16;
static const float DEFAULT_CHASSIS_LENGTH = 2.1;

// Offsets to adjust theta values received from HAL to mathematical model
static const float THETA0_OFFSET = 0.0;
static const float THETA1_OFFSET = 0.0;
static const float THETA2_OFFSET = M_PI;

class AgribotOdometry : public AgribotComponent
{
private:
  float _fTheta0;
  float _fTheta1;
  float _fTheta2;
  float _fEta1;
  float _fWheelRadius;
  float _fChassisLength;
  float _fX;
  float _fY;
  float _fLinearVelocityX;
  float _fLinearVelocityY;
  float _fAngularVelocityZ;
  double _dDt;
  bool _bTheta0Received;
  bool _bTheta1Received;
  bool _bTheta2Received;
  bool _bDtSet;

public:
  AgribotOdometry();
  virtual int configure();
  virtual bool startup();
  virtual bool update();
  virtual bool stop();
  virtual bool cleanup();

  void vUpdateVelocities();
  void vUpdatePositions();

  void vSetTheta0(float value);
  void vSetTheta1(float value);
  void vSetTheta2(float value);
  void vSetEta1(float value);
  void vSetWheelRadius(float value);
  void vSetChassisLength(float value);
  void vSetDt(float value);

  float fGetTheta0();
  float fGetTheta1();
  float fGetTheta2();
  float fGetEta1();
  float fGetWheelRadius();
  float fGetChassisLength();

  float fGetX();
  float fGetY();

  float fGetLinearVelocityX();
  float fGetLinearVelocityY();
  float fGetAngularVelocityZ();
};
