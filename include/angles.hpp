#pragma once
#include <Arduino.h>

namespace angle
{

    enum class domain : uint8_t
    {
        continuous, // 0 360
        mirror,     // -180 180
    };

    enum class AngleUnit : uint8_t
    {
        radians,
        degrees,
    };

    template <const domain D, const AngleUnit U>
    struct angle
    {
        // TODO: these static members might be pointless...
        static const domain _domain = D;
        static const AngleUnit _unit = U;
        float value;
    };

    /**
     *  assumes source domain is oposite of target domain
     */

    template <const domain target_d, const domain src_d, const AngleUnit src_u>
    angle<target_d, src_u> convert_domain(const angle<src_d, src_u> &a)
    {
        // CHORE: remove old impl that was used as reference
        // if (target_domain == AngleDomain::continuous)
        // {
        //     return angle >= 0 ? angle : 360 + angle;
        // }
        // else
        // {
        //     return angle <= 180 ? angle : -360 + angle;
        // }

        if (target_d == src_d)
            return a;
        if (target_d ==  
        return {0};
    }

    float travel_deg(float from, float to)
    {
        auto zeroed = to - from;
        if (zeroed > 180)
        {
            return zeroed - 360;
        }
        else if (zeroed < -180)
        {
            return zeroed + 360;
        }
        else
        {
            return zeroed;
        }
    }

    const angle<domain::continuous, AngleUnit::radians> my_angle_ex{PI};
    auto my_angle_ex_converted_domain = convert_domain<domain::continuous>(my_angle_ex);
};
