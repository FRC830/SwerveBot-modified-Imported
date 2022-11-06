// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/filter/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
// #include <frc/Timer.h>

#include "Drivetrain.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override{
    ConfigurePIDF(belt, .03, 6E-05, 0, 0);
	  belt.SetInverted(false);
	  belt.SetNeutralMode(motorcontrol::NeutralMode::Brake);
    /*=============
    Flywheel
    =============*/
    ConfigurePIDF(flywheelMotor, .04, 6E-05, 0, 0);
    flywheelMotorFollow.Follow(flywheelMotor);
    flywheelMotorFollow.SetInverted(false);
    flywheelMotor.ConfigClosedloopRamp(2);
    flywheelMotor.SetInverted(true);
    flywheelMotor.SetNeutralMode(motorcontrol::NeutralMode::Coast);
    flywheelMotor.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration{true,20,30,0.5}); // if above 30A for .5s, limit to 20A
      
    /*=============
    Elevator
    =============*/
    elevatorMotor.ConfigFactoryDefault();
    elevatorMotor.SetInverted(true);
    elevatorMotor.SetSelectedSensorPosition(0);
    elevatorMotor.SetNeutralMode(motorcontrol::NeutralMode::Brake);
  }
  void AutonomousInit() override {
    timer.Reset();
    timer.Start();
  }

  void AutonomousPeriodic() override {
    frc::SmartDashboard::PutNumber("timer", (double)timer.Get());
    if ((int) timer.Get() < 2){
      m_swerve.m_frontLeft.m_driveMotor.Set(0.4);
      m_swerve.m_frontRight.m_driveMotor.Set(0.4);
      m_swerve.m_backLeft.m_driveMotor.Set(0.4);
      m_swerve.m_backRight.m_driveMotor.Set(0.4);
      // m_swerve.Drive(units::velocity::meters_per_second_t(0), units::velocity::meters_per_second_t(0.5), units::angular_velocity::radians_per_second_t(0), false);
    }
    else{
      m_swerve.m_frontLeft.m_driveMotor.Set(0);
      m_swerve.m_frontRight.m_driveMotor.Set(0);
      m_swerve.m_backLeft.m_driveMotor.Set(0);
      m_swerve.m_backRight.m_driveMotor.Set(0);

      // m_swerve.Drive(units::velocity::meters_per_second_t(0), units::velocity::meters_per_second_t(0), units::angular_velocity::radians_per_second_t(0), false);
    }
    // DriveWithJoystick(false);
    // m_swerve.UpdateOdometry();\[]

  }
  void TeleopInit() {
    // reset elevator
    inputScale = 1.0;
    elevatorMotor.SetSelectedSensorPosition(0);
  }
  void TeleopPeriodic() override {
    DriveWithJoystick(false);
    
	  HandleElevator();
  }

 private:
  frc::Timer timer;
  // static const frc::GenericHID::JoystickHand LEFT = frc::GenericHID::kLeftHand;
  // static const frc::GenericHID::JoystickHand RIGHT = frc::GenericHID::kRightHand;

  frc::XboxController m_controller{0};
  frc::XboxController m_controllerCopilot{1};
  
  Drivetrain m_swerve;
  TalonSRX belt{5}; // The old id was 7
  TalonFX flywheelMotor{10};
  TalonFX flywheelMotorFollow{11};

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};
  
  //Elevator variables 
  int elevatorBreaksPoint = 600000;
  int maxElevatorUp = 300000;
  int minElevatorDown = 250000;
  double elevatorSpeedUp = 0.75;
  double elevatorSpeedDown = 0.6;
  TalonFX elevatorMotor{18};
  double inputScale = 1.0;

  void ConfigurePIDF(TalonFX &motor, double p, double i, double d, double f, bool reset=true) {
    if (reset) {
      motor.ConfigFactoryDefault();
    }
    motor.Config_kP(0, p);
    motor.Config_kI(0, i);
    motor.Config_kD(0, d);
    motor.Config_kF(0, f);
  }

  void ConfigurePIDF(TalonSRX &motor, double p, double i, double d, double f, bool reset=true) {
    if (reset) {
      motor.ConfigFactoryDefault();
    }
    motor.Config_kP(0, p);
    motor.Config_kI(0, i);
    motor.Config_kD(0, d);
    motor.Config_kF(0, f);
  }

  double ApplyDeadzone(double val, double deadzone) {
	  return fabs(val) < deadzone ? 0 : val;
  }

  void DriveWithJoystick(bool fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const units::velocity::meters_per_second_t xSpeed = -m_xspeedLimiter.Calculate(
                            ApplyDeadzone((-m_controller.GetLeftY()), 0.1)) *
                            // m_controller.GetTriggerAxis(frc::GenericHID::kLeftHand)) *
                        Drivetrain::kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = m_yspeedLimiter.Calculate(
                            ApplyDeadzone((m_controller.GetRightX()), 0.1)) *
                        Drivetrain::kMaxSpeed;


    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = -m_rotLimiter.Calculate(
                         m_controller.GetRightX()) *
                     Drivetrain::kMaxAngularSpeed;
    if (m_controllerCopilot.GetRightTriggerAxis() > 0.1)
    {
          flywheelMotor.Set(ControlMode::Velocity, (int) (4000 / (10.0 / 2048.0 * 60.0)));
    }
    else 
    {
          flywheelMotor.Set(ControlMode::Velocity, (int) (0));
    }
    
    // This expression should make it so that the wheels
    // will not turn back towards the front when the pilot'
    // controls aren't receiving any input


    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, (xSpeed == 0_mps && ySpeed == 0_mps && rot == 0_rad_per_s));

    double manualBeltPower = ApplyDeadzone(-m_controllerCopilot.GetYButton(), .35);
    // if (manualBeltPower != 0) {
      belt.Set(ControlMode::PercentOutput, manualBeltPower);
    // }

  }

  void HandleElevator() {
    double encoder = elevatorMotor.GetSelectedSensorPosition();
    //debugTab->PutNumber("elevator encoder", encoder);
    bool down = ApplyDeadzone(m_controller.GetLeftTriggerAxis(), 0.1) > 0;
    bool up = ApplyDeadzone(m_controller.GetRightTriggerAxis(), 0.1) > 0;
    bool scaryReverse = m_controller.GetBackButton();
    if (encoder > minElevatorDown) {
      inputScale = 0.5; // slow down driving when completely extended
    }
    if (up && (encoder < maxElevatorUp) && (encoder < elevatorBreaksPoint)) {
      elevatorMotor.Set(ControlMode::PercentOutput, elevatorSpeedUp);
    } else if (down && (encoder > minElevatorDown) && (encoder < elevatorBreaksPoint)) {
      elevatorMotor.Set(ControlMode::PercentOutput, elevatorSpeedDown);	
    } else if (scaryReverse) {
      elevatorMotor.Set(ControlMode::PercentOutput, -elevatorSpeedDown);
    } else {
      elevatorMotor.Set(ControlMode::PercentOutput, 0);
    }


  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif