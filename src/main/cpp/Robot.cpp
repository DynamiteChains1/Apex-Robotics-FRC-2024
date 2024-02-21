// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Includes for VisionProcessing
#include <cstdio>
#include <thread>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>
#include <opencv2/opencv.hpp>

//Main includes neccessary for robot functionality
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>

//Includes for using shooter/intake motors
#include <rev/CANSparkMax.h>

//Includes for playing music on Falcon500s
#include <ctre/phoenix6/Orchestra.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

//Robot class for all of our functions and controlling the robot
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

    //Shooter inversion/follow setup
    s_follow.SetInverted(true);
    c_follow.SetInverted(true);
    s_follow.Follow(s_lead);
    c_follow.Follow(c_lead);

    //Load music and setup orchestra
    m_orchestra.LoadMusic("sus.chrp");
    m_orchestra.AddInstrument(m_sus1);
    m_orchestra.AddInstrument(m_sus2);

    //Shooter PID setup using Shooter PID variables
    sPID.SetP(kShootP);
    sPID.SetI(kShootI);
    sPID.SetD(kShootD);
    sPID.SetFF(kShootFF);
    //This is to make sure its not trying to output more than possible
    sPID.SetOutputRange(-1.0, 1.0);
  }

  void AutonomousInit() override { 
    m_orchestra.LoadMusic("sus.chrp");
    // Reset Timer for use with auton
    m_timer.Restart(); 
    // Sets the time for sus to false
    sus_time = false; 
    }

  void AutonomousPeriodic() override {
    // Drive for 2 seconds
    if (m_timer.Get() < 0.5_s) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.ArcadeDrive(0.05, 0.0, false);
    } else {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0, false);
    }

    if (m_timer.Get() > 0.5_s && sus_time == false) {
      m_orchestra.Play();
      sus_time = true;
    }
  }

  void TeleopInit() {m_orchestra.LoadMusic("sus.chrp");}

  void TeleopPeriodic() override {
    // Drive with arcade style
    m_robotDrive.ArcadeDrive(-m_stick.GetY(), m_stick.GetTwist());

    // Code for shooter
    if (m_stick.GetRawButton(1)) {
      s_setpoint = 300;
    }
    else {
      s_setpoint = 0;
    }
    sPID.SetReference(s_setpoint, rev::CANSparkMax::ControlType::kVelocity);

    //Code for playing/pausing among us music
    if (m_stick.GetRawButton(6)) {
      m_orchestra.Play();
    }
    if (m_stick.GetRawButton(5)) {
      m_orchestra.Pause();
    }

    //This function tests for controller input to increase/decrease the speed of the climber
    ClimberSpeed();

    //This tests for controller input to climb up/down
    if (m_stick.GetRawButton(4)) {
      c_lead.Set(c_speed);
      c_follow.Set(c_speed);
    }
    else if (m_stick.GetRawButton(3)) {
      c_lead.Set(-1 * c_speed);
      c_follow.Set(-1 * c_speed);
    }
    else {
      c_lead.Set(0);
      c_follow.Set(0);
    }
    // Code for intake
    

  }

  void TestInit() override { m_orchestra.LoadMusic("sus.chrp"); m_orchestra.Play(); }

  void TestPeriodic() override {}

  void DisabledInit() override {}

  void RobotInit() {
    
  }

 private:
  // Robot drive system
  ctre::phoenix6::hardware::TalonFX m_left {1};
  ctre::phoenix6::hardware::TalonFX m_right {2};
  ctre::phoenix6::hardware::TalonFX m_sus1{3};
  ctre::phoenix6::hardware::TalonFX m_sus2{4};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};

  // Setup motors for intake
  rev::CANSparkMax i_turn{5, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax i_take{6, rev::CANSparkMax::MotorType::kBrushless};

  // Setup motors for shooter
  rev::CANSparkMax s_lead{7, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax s_follow{8, rev::CANSparkMax::MotorType::kBrushless}; 

  // Setup motors for climber
  rev::CANSparkMax c_lead{10, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax c_follow{9, rev::CANSparkMax::MotorType::kBrushless};

  // Setup Joystick and Timer
  frc::Joystick m_stick{0};
  frc::Timer m_timer;
  
  // Setup Orchestra
  ctre::phoenix6::Orchestra m_orchestra;
  
  bool sus_time = false;

  // PID setup for shooter
  double s_setpoint = 0;
  double kShootFF = 0.000015;
  double kShootP = 0.00006;
  double kShootI = 0.000001;
  double kShootD = 0.0;
  rev::SparkPIDController sPID = s_lead.GetPIDController();

  // PID setup for intake rotation
  double i_setpos = 0;
  double kI_turnFF = 0;
  double kI_turnP = 0;
  double kI_turnI = 0;
  double kI_turnD = 0;
  rev::SparkPIDController i_turnPID = i_turn.GetPIDController();

  // Variable setup for climber
  double c_speed = 1;

  //Climber speed change function
  void ClimberSpeed() {
    if (m_stick.GetRawButtonPressed(10)){
      if (c_speed < 1) {
        c_speed = c_speed + 0.1;
      }
    }   
    else if (m_stick.GetRawButtonPressed(12)) {
      if (0 < c_speed) {
        c_speed = c_speed - 0.1;
      }
    }
  }
  //April Tag stuff + camera
  static void VisionThread() {
    //Get usb input from cameraserver
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
    camera.SetResolution(640, 480);

    //Make a CV sink to hold Mats (images/frames but for OpenCV)
    cs::CvSink cvSink = frc::CameraServer::GetVideo();

    //Direct the output back to the dashboard with CVSource
    cs::CvSource camOutStream = frc::CameraServer::PutVideo("CameraOutput", 640, 480);

    //Never explained why, but we reuse this mat bc mats
    //are memory extensive. Don't know how it is being reused
    //or when it started existing in the first place
    cv::Mat mat;

    //Image to mat conversion loop
    while (true) {

      //Tells the CvSink to grab a frame and put it into the mat
      //and notify of errors
      if (cvSink.GrabFrame(mat) == 0) {
        //Sends Camera Output Stream the error
        camOutStream.NotifyError(cvSink.GetError());
        //skip rest of loop because of error
        continue;
      }
      //Give camera stream new image to display
      camOutStream.PutFrame(mat);
    }
  }

};


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
