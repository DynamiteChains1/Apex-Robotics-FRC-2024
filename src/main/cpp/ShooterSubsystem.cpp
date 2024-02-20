#include <ShooterSubsystem.hpp>
#include <constants.hpp>

ShooterSubsystem::ShooterSubsystem() :
    s_lead{7, rev::CANSparkMax::MotorType::kBrushless},
    s_follow{8, rev::CANSparkMax::MotorType::kBrushless} {
        s_follow.SetInverted(true);
        s_follow.Follow(s_lead);
        s_setpoint = 300;
        rev::SparkPIDController sPID = s_lead.GetPIDController();
}

void ShooterSubsystem::GetInput() {
            //die die die die
    if (m_stick.GetRawButton(1)) {
        s_setpoint = 300;

    };
    else if not (m_stick.GetRawButton(1)) {
        s_setpoint = 0;
    };
}
        
    
