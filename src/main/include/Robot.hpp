#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/Joystick.h>

class Robot : public frc::TimedRobot() {
    public:
    void Robot();
    private:
    frc::Joystick m_stick{0};
    frc::Timer m_timer;
};