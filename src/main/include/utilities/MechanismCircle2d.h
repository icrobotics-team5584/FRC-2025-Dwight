#pragma once

#include <string_view>
#include <units/angle.h>
#include <frc/util/Color8Bit.h>

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

/*
Parameters:
- std::string_view name (will correlate with iterative ligament names, probably)
- double radius (will correlate with ligament length)
- units::degree_t angle
- const frc::Color8Bit& indicatorColor = {r, g, b} (for the one ligament that is a different colour to show rotation)
- const frc::Color8Bit& color = {235, 137, 52} (background colour of circle)
- int spokes = 36 (how many ligaments will be used to make up the circle; consequently, how many spokes it will have)
- double spokeWidth = 6 (will correlate with ligament lineWidth)

Needed functions:
- setters and getters for all parameters EXCEPT spokes
- spin
*/

class MechanismCircle2d {
    public:
        template <typename T>
        MechanismCircle2d(T* location,
                            std::string name,
                            double radius,
                            units::degree_t angle,
                            int backgroundSpokes=36,
                            double spokeWidth=6,
                            const frc::Color8Bit& indicatorColor={255, 255, 255},
                            const frc::Color8Bit& color={235, 137, 52})
        {
            //Create circle background spokes
            for (int i = 0; i < backgroundSpokes; i++) {
                frc::MechanismLigament2d* spoke =
                    location->Append<frc::MechanismLigament2d>(name+"_spoke"+std::to_string(i),
                                                                radius,
                                                                angle+(360_deg/backgroundSpokes)*i,
                                                                spokeWidth,
                                                                color); //append spoke ligament to chosen location
                _backgroundSpokeLigaments.push_back(spoke); //add to list of spokes
            }

            //Create indicator ligament
            _indicatorLigament = location->Append<frc::MechanismLigament2d>(name+"~indicator", radius, angle, spokeWidth, indicatorColor);
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