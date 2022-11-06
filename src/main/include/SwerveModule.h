// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>

#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>
#include <ctre/phoenix/sensors/CANCoder.h>

class SwerveModule {
 private:
  static constexpr double kWheelRadius = 0.0508;
  static constexpr double kDriveGearRatio = 6.6;
  static constexpr double kTurningGearRatio = 12.8;

  static constexpr auto kModuleMaxAngularVelocity =
      wpi::numbers::pi * 3_rad_per_s;  // radians per second
  static constexpr auto kModuleMaxAngularAcceleration =
      wpi::numbers::pi * 7_rad_per_s / 1_s;  // radians per second^2
   std::string m_driveMotorName;
   std::string m_turningMotorName;

 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel, std::string driveMotorName, std::string turningMotorName, int turningCANCoderChannel);
  frc::SwerveModuleState GetState();
  void SetDesiredState(const frc::SwerveModuleState& state);
  void stop();


  // Motor Values
  rev::CANSparkMax m_driveMotor;
  rev::SparkMaxRelativeEncoder m_driveMotorEncoder = m_driveMotor.GetEncoder();
  rev::CANSparkMax m_turningMotor;
  rev::SparkMaxRelativeEncoder m_turningMotorEncoder = m_turningMotor.GetEncoder();
  ctre::phoenix::sensors::CANCoder m_turningMotorCANCoder; //arbitrary id

  frc2::PIDController m_drivePIDController{1.0, 0, 0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      4.0,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

  frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V,
                                                                3_V / 1_mps};
  frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
      1_V, 0.5_V / 1_rad_per_s};

  void OutputValues(){ //Outputs all the SmartDashboard widgets when called
      frc::SmartDashboard::PutNumber(m_driveMotorName + " Velocity: ", m_driveMotorEncoder.GetVelocity());
      frc::SmartDashboard::PutNumber(m_driveMotorName + " Rotations: ", m_driveMotorEncoder.GetPosition());
      frc::SmartDashboard::PutNumber(m_turningMotorName + " Velocity: ", m_turningMotorEncoder.GetVelocity());
      frc::SmartDashboard::PutNumber(m_turningMotorName + " Rotations: ", m_turningMotorEncoder.GetPosition());
      frc::SmartDashboard::PutNumber(m_turningMotorName + " Absolute Position: ", m_turningMotorCANCoder.GetAbsolutePosition());
  }
};
