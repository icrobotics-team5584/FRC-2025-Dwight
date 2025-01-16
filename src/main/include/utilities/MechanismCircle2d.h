#pragma once

#include <string_view>
#include <units/angle.h>
#include <frc/util/Color8Bit.h>

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

class MechanismCircle2d {
    public:
        template<typename T>
        requires std::same_as<T*, frc::MechanismRoot2d*> || std::convertible_to<T*, frc::MechanismObject2d*>
        MechanismCircle2d(T* location, std::string name, double radius,
            units::degree_t angle,
            int backgroundSpokes=36,
            double spokeWidth=6.0,
            const frc::Color8Bit& indicatorColor={255, 255, 255},
            const frc::Color8Bit& color={235, 137, 52}) {
            //Create circle background spokes
            for (int i = 0; i < backgroundSpokes; i++) {
                frc::MechanismLigament2d* spoke = location->template Append<frc::MechanismLigament2d>(
                        name+"_spoke"+std::to_string(i),
                        radius,
                        angle+(360_deg/backgroundSpokes)*i,
                        spokeWidth,
                        color); //append spoke ligament to chosen location
                _backgroundSpokeLigaments.push_back(spoke); //add to list of spokes
            }

            //Create indicator ligament
            _indicatorLigament = location->template Append<frc::MechanismLigament2d>(name+"~indicator", radius, angle, spokeWidth, indicatorColor);
            /* tilde is at the end of the ASCII character set. by putting it in the name
            the indicator ligament goes to the bottom of networktables, ensuring it is
            not hidden by the background spokes. */
        }
        
        void SetAngle(units::degree_t angle);
        void SetIndicatorColor(const frc::Color8Bit& color);
        void SetCircleColor(const frc::Color8Bit& color);

    private:
        std::vector<frc::MechanismLigament2d*> _backgroundSpokeLigaments;
        frc::MechanismLigament2d* _indicatorLigament;
};