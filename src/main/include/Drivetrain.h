// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/numbers>

#include <AHRS.h>

#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() {
      m_gyro = new AHRS(frc::SPI::Port::kMXP);
  }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, bool isZero);
  void UpdateOdometry();

  static constexpr units::meters_per_second_t kMaxSpeed =
      5.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      wpi::numbers::pi * 2};  // 1/2 rotation per second

 public:
  frc::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
  frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
  frc::Translation2d m_backLeftLocation{-0.381_m, +0.381_m};
  frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

  SwerveModule m_frontLeft{13, 20, "Front Left Drive Motor", "Front Left Turn Motor", 20};
  SwerveModule m_frontRight{12, 19, "Front Right Drive Motor", "Front Right Turn Motor", 22};
  SwerveModule m_backLeft{15, 22, "Back Left Drive Motor", "Back Left Turn Motor", 23};
  SwerveModule m_backRight{14, 21, "Back Right Drive Motor", "Back Right Turn Motor", 21};

  AHRS *m_gyro;

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, {}};
};
