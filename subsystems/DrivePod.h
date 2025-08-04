#include "ctre/phoenix6/TalonFXS.hpp"
#include "ctre/phoenix6/CANcoder.hpp"

#include "utils/math/pid.h"
#include "utils/telemetry/Teleplot.h"
#include <cmath>
#include "utils/swerve/PodState.h"
#include "utils/translation2d.h"
#include "Constants.h"

using namespace ctre::phoenix6;
using namespace std;

class DrivePod {
private: 
    void NormalizePodState(PodState &state);

    // modifies the target angle and output to enable shortspin
    void OptimizePodState(PodState &targetState, PodState currentState);
    // Limits the target angle to within the operating zone
    void LimitPodState(PodState &targetState);

    void stopWhileTurning(PodState &targetState, PodState currentState);

    double getLeftSpeed() const;

    double getRightSpeed() const;

    double getAngle() const;
    
    

public:
    translation2d GetPosition();
    void Initialize() ;
    void Periodic();

    void SetPodState(PodState &targetState);

    PodState GetPodState() const;
};
