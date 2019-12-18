#include <agribot_common/AgribotComponent.h>
#include <cmath>

static const float DEFAULT_WHEEL_RADIUS   = 0.04;
static const float DEFAULT_CHASSIS_LENGTH = 1.0;

class AgribotOdometry: public AgribotComponent
{
private:
  float _fTheta0;
  float _fTheta1;
  float _fTheta2;
  float _fEta1;
  float _fWheelRadius;
  float _fChassisLength;
  float _fLinearVelocityX;
  float _fLinearVelocityY;
  float _fAngularVelocityZ;
  bool _bTheta0Received;
  bool _bTheta1Received;
  bool _bTheta2Received;

public:
  AgribotOdometry();
  virtual int configure();
  virtual bool startup();
  virtual bool update();
  virtual bool stop();
  virtual bool cleanup();

  void vSetTheta0(float value);
  void vSetTheta1(float value);
  void vSetTheta2(float value);
  void vSetEta1(float value);
  void vSetWheelRadius(float value);
  void vSetChassisLength(float value);

  float fGetTheta0();
  float fGetTheta1();
  float fGetTheta2();
  float fGetEta1();
  float fGetWheelRadius();
  float fGetChassisLength();

  float fGetLinearVelocityX();
  float fGetLinearVelocityY();
  float fGetAngularVelocityZ();
};
