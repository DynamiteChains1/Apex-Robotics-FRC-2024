#include <ShooterSubsystem.hpp>

ShooterSubsystem::ShooterSubsystem() :
    s_lead{7, rev::CANSparkMax::MotorType::kBrushless},
    s_follow{8, rev::CANSparkMax::MotorType::kBrushless} {
        s_follow.SetInverted(true);
        s_follow.Follow(s_lead);
    }
        
    
