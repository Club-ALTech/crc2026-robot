#pragma once
#include <Arduino.h>

namespace angle
{
    static constexpr float HALF_RAD = PI, HALF_DEG = 180;

    enum class domain : uint8_t
    {
        continuous, // ex: 0 to 360
        mirror,     // ex: -180 to 180
    };

    enum class unit : uint8_t
    {
        radians,
        degrees,
    };

    /**
     * identity of a pair. useful for comparisons.
     */
    template <const domain D, const unit U>
    constexpr uint8_t identity()
    {
        return (static_cast<uint8_t>(D) << 1) & static_cast<uint8_t>(U);
    }

    template <const unit U>
    constexpr float half()
    {
        switch (U)
        {
        case unit::radians:
            return HALF_RAD;
        case unit::degrees:
            return HALF_DEG;
        }
    }

    template <const domain D, const unit U>
    constexpr float min_a()
    {
        switch (D)
        {
        case domain::continuous:
            return 0;
        case domain::mirror:
            return -half<U>();
        }
    }

    template <const domain D, const unit U>
    constexpr float max_a()
    {
        switch (D)
        {
        case domain::continuous:
            return 2 * half<U>();
        case domain::mirror:
            return half<U>();
        }
    }

    template <const domain D, const unit U>
    struct angle
    {
        float value;
    };

    /**
     * convert an angle from one domain to the other
     */
    template <const domain target_d, const domain src_d, const unit U>
    angle<target_d, U> convert(const angle<src_d, U> &a)
    {
        if (target_d == src_d)
            return a;
        switch (target_d)
        {
        case domain::mirror:
            return {a.value > half<U>()
                        ? -max_a<domain::continuous, U>() + a.value
                        : a.value};
        case domain::continuous:
            return {a.value < 0
                        ? max_a<domain::continuous, U>() + a.value
                        : a.value};
        }
    }

    /**
     * translate an angle from one unit to the other
     */
    template <const domain D, const unit src_u, const unit target_u>
    angle<D, target_u> translate(const angle<D, src_u> &a)
    {
        if (target_u == src_u)
            return a;
        switch (target_u)
        {
        case unit::radians:
            return {a.value * max_a<D, unit::radians>() / max_a<D, unit::degrees>()};
        case unit::degrees:
            return {a.value * max_a<D, unit::degrees>() / max_a<D, unit::radians>()};
        }
    }

    template <const domain D, const unit U>
    angle<domain::mirror, U> travel(angle<D, U> from, angle<D, U> to)
    {
        auto zeroed = convert<domain::continuous>(to).value - convert<domain::continuous>(from).value;
        if (zeroed > max_a<domain::mirror, U>())
        {
            return {zeroed - max_a<domain::continuous, U>()};
        }
        else if (zeroed < min_a<domain::mirror, U>())
        {
            return {zeroed + max_a<domain::continuous, U>()};
        }
        else
        {
            return {zeroed};
        }
    }

    template <const domain D, const unit U>
    angle<D, U> wrap(angle<D, U> a)
    {
        if (D == domain::continuous)
        {
            auto value = fmod(a.value, max_a<domain::continuous, U>());
            return {value};
        }
        return convert<domain::mirror>(wrap(convert<domain::continuous>(a)));
    }

    /**
     * using [EMA](https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average)
     */
    class AngleMovingAvg
    {
        float _alpha, _running_avg_x, _running_avg_y;
        bool _init;

    public:
        static const domain DOMAIN = domain::mirror;
        static const unit UNIT = unit::radians;

        AngleMovingAvg(float alpha)
            : _alpha(alpha), _running_avg_x(0), _running_avg_y(0), _init(false) {}

        void add(float a)
        {
            if (!this->_init)
            {
                this->_running_avg_x = cos(a);
                this->_running_avg_y = sin(a);
                this->_init = true;
            }
            else
            {
                this->_running_avg_x = this->_alpha * cos(a) + (1.0 - this->_alpha) * this->_running_avg_x;
                this->_running_avg_y = this->_alpha * sin(a) + (1.0 - this->_alpha) * this->_running_avg_y;
            }
        }

        float calc()
        {
            if (!this->_init)
                return NAN;
            return atan2(this->_running_avg_y, this->_running_avg_x);
        }

        bool is_init()
        {
            return this->_init;
        }
    };

    const angle<domain::continuous, unit::radians> my_angle_ex{PI};
    auto my_angle_ex_converted_domain = convert<domain::continuous>(my_angle_ex);
};
