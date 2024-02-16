#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>

class ShooterSubsystem {
    public:
        ShooterSubsystem();
        void GetInput();

    private:
        rev::CANSparkMax s_lead;
        rev::CANSparkMax s_follow;
};