#include "utilities/ICMechanism2d.h"

void MechanismCircle2d::SetAngle(units::degree_t angle) {
    _indicatorLigament->SetAngle(angle);
}

void MechanismCircle2d::SetIndicatorColor(const frc::Color8Bit& color) {
    _indicatorLigament->SetColor(color);
}

void MechanismCircle2d::SetCircleColor(const frc::Color8Bit& color) {
    for (frc::MechanismLigament2d* spoke : _backgroundSpokeLigaments) {
        spoke->SetColor(color);
    }
}

SharedMechanism2d::SharedMechanism2d() {}

frc::Mechanism2d& SharedMechanism2d::Get() {
    return _mechanism;
}