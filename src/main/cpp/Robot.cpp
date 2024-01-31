// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// hi
// merry christmas
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix6/Orchestra.hpp>

class Robot : public frc::TimedRobot {
 public:
  Robot() {
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_left);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_right);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_right.SetInverted(true);
    m_robotDrive.SetExpiration(100_ms);
    m_timer.Start();
  }

  void AutonomousInit() override { m_timer.Restart(); }

  void AutonomousPeriodic() override {
    // Drive for 2 seconds
    if (m_timer.Get() < 0.5_s) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.ArcadeDrive(0.05, 0.0, false);
    } else {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0, false);
    }
  }

  void TeleopInit() override {}

  void TeleopPeriodic() override {
    // Drive with arcade style (use right stick to steer)
    m_robotDrive.ArcadeDrive(-m_stick.GetY(), m_stick.GetX());
    // Code for shooter
    
    // Code for intake

    // Code for climber

  }

  void TestInit() override {}

  void TestPeriodic() override {}

 private:
  // Robot drive system
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_left{1};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_right{2};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};
  // Setup motors for intake
  rev::CANSparkMax i_left{? rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax i_right{? rev::CANSparkMax::MotorType::kBrushless};
  // Setup motors for shooter
  rev::CANSparkMax s_left{? rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax s_right{? rev::CANSparkMax::MotorType::kBrushless};
  // Setup motors for climber
  

  frc::Joystick m_stick{0};
  frc::Timer m_timer;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
