// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Filesystem.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <ctre/phoenix6/Orchestra.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

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
    s_follow.SetInverted(true);
    s_follow.Follow(s_lead)
    Orchestra m_orchestra;
    m_orchestra.addInstrument(m_sus1);
    m_orchestra.addInstrument(m_sus2);
    string deploy_path = frc::filesystem::GetDeployDirectory();
    m_orchestra.loadMusic(deploy_path + "/sus.chirp");
    
  }

  void AutonomousInit() override { 
    // Reset Timer for use with auton
    m_timer.Restart(); 
    // Sets the time for sus to false
    bool sus_time = false; 
    }

  void AutonomousPeriodic() override {
    // Drive for 2 seconds
    if (m_timer.Get() < 0.5_s) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.ArcadeDrive(0.05, 0.0,m_orchestra.Play(); false);
    } else {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0, false);
    }

    if (m_timer.Get() > 0.5_s && sus_time == false) {
      m_orchestra.Play();
      sus_time = true;
    }
  }

  void TeleopInit() override { m_orchestra.Play() }

  void TeleopPeriodic() override {
    // Drive with arcade style (use right stick to steer)
    m_robotDrive.ArcadeDrive(-m_stick.GetY(), m_stick.GetTwist());
    // Code for shooter
    if (m_stick.GetRawButton(1)) {
      s_lead.Set(0.5);
    }
    else{
      s_lead.StopMotor();
    }
    
    // Code for intake

    // Code for climber

  }

  void TestInit() override {}

  void TestPeriodic() override {}

  void DisabledInit() override {}

 private:
  // Robot drive system
  ctre::phoenix6::hardware::TalonFX m_left{1};
  ctre::phoenix6::hardware::TalonFX m_right{2};
  ctre::phoenix6::hardware::TalonFX m_sus1{3};
  ctre::phoenix6::hardware::TalonFX m_sus2{4};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};
  // Setup motors for intake
  rev::CANSparkMax i_left{0, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax i_right{1, rev::CANSparkMax::MotorType::kBrushless};
  // Setup motors for shooter
  rev::CANSparkMax s_lead{7, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax s_follow{8, rev::CANSparkMax::MotorType::kBrushless};
  // Setup motors for climber
  

  frc::Joystick m_stick{0};
  frc::Timer m_timer;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
