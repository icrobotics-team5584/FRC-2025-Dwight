#include "utilities/MechanismCircle2d.h"

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