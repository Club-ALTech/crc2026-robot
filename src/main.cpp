#include <Arduino.h>
#include <CrcLib.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <Smoothed.h>
#include "AHRSProtocol.h"
#include <QuickPID.h>

/**
 * =============
 * TOOLS & UTILS
 * =============
 */


using pin_t = uint8_t;

enum class AngleDomain : uint8_t
{
    continuous, // 0 360
    mirror,     // -180 180
};

/** assumes source domain is oposite of target domain
 *
 */
float convert_domain(float angle, AngleDomain target_domain)
{
    if (target_domain == AngleDomain::continuous)
    {
        return angle >= 0 ? angle : 360 + angle;
    }
    else
    {
        return angle <= 180 ? angle : -360 + angle;
    }
}

class ReadPWM
{
    uint32_t _last, _timeout;
    uint8_t _pin, _mode;

public:
    ReadPWM(uint8_t pin, uint8_t mode = HIGH, uint32_t timeout = 1050)
        : _last(0), _timeout(timeout), _pin(pin), _mode(mode)
    {
        pinMode(pin, INPUT);
    }

    bool read(uint32_t &into)
    {
        into = pulseIn(this->_pin, this->_mode, this->_timeout);
        if (into == 0)
        {
            into = this->_last;
            return false;
        }
        else
        {
            this->_last = into;
            return true;
        }
    }
};

class PwmToAngleConverter
{
public:
    float _offset_deg;
    bool _reverse;
    uint32_t _min_pulse, _max_pulse;

public:
    PwmToAngleConverter(bool reverse = false, float offset_deg = 0, uint32_t min_pulse = 1, uint32_t max_pulse = 1024)
        : _offset_deg(offset_deg), _reverse(reverse), _min_pulse(min_pulse), _max_pulse(max_pulse) {}

    float convert(uint32_t pwm)
    {
        auto deg = (pwm - this->_min_pulse) * 360 / (this->_max_pulse - this->_min_pulse);
        deg = this->_reverse ? 360 - deg : deg;
        return deg + this->_offset_deg;
    }

    void set_offset(float offset_deg)
    {
        this->_offset_deg = offset_deg;
    }
};

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

/**
 * expects {AngleDomain::mirror}
 * expects radians
 */
template <const int window_size>
class AngleMovingAvg
{
    size_t _ptr;
    float _values[window_size];
    float _running_sum_x, _running_sum_y;

public:
    static const AngleDomain DOMAIN = AngleDomain::mirror;

    AngleMovingAvg() : _ptr(0), _values{0}, _running_sum_x(0), _running_sum_y(0) {}

    void add(float a)
    {
        this->_running_sum_x -= cos(this->_values[this->_ptr]);
        this->_running_sum_y -= sin(this->_values[this->_ptr]);

        this->_values[this->_ptr] = a;
        
        this->_running_sum_x += cos(this->_values[this->_ptr]);
        this->_running_sum_y += sin(this->_values[this->_ptr]);

        this->_ptr++;
        this->_ptr %= window_size;
    }

    float calc()
    {
        return atan2(this->_running_sum_y, this->_running_sum_x);
    }
};

int8_t clean_joystick_input(int8_t input)
{
    if (abs(input) < 10)
        return 0;
    auto constrained = constrain(input, -127, 127); // Fix max value to be symmetric
    return constrained;
}

class NavX
{

    static const int ITERATION_DELAY_MS = 10;
    static const int NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT = 0x32;
    static const int NUM_BYTES_TO_READ = 8;

    static const int register_address = NAVX_REG_YAW_L;

    byte _data[512];

public:
    struct Heading
    {
        float yaw;
        float pitch;
        float roll;
        float heading;
    };

    struct FieldCentricInput
    {
        int8_t forward;
        int8_t strafe;
        double rotation;
    };

    NavX() : _data{0} {}

    Heading read()
    {
        /* Transmit I2C data request */
        Wire.beginTransmission(NavX::NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT); // Begin transmitting to navX-Sensor
        Wire.write(NavX::register_address);                                // Sends starting register address
        Wire.write(NavX::NUM_BYTES_TO_READ);                               // Send number of bytes to read
        Wire.endTransmission();                                            // Stop transmitting

        /* Receive the echoed value back */
        Wire.beginTransmission(NavX::NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT);                    // Begin transmitting to navX-Sensor
        Wire.requestFrom(NavX::NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT, NavX::NUM_BYTES_TO_READ); // Send number of bytes to read
        for (size_t i = 0; Wire.available(); i++)
        { // Read data (slave may send less than requested)
            this->_data[i++] = Wire.read();
        }
        Wire.endTransmission(); // Stop transmitting

        /* Decode received data to floating-point orientation values */
        float pitch = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&this->_data[0]) / 2.55;     // The cast is needed on arduino
        float yaw = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&this->_data[2]) / 2.55;       // The cast is needed on arduino
        float roll = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&this->_data[4]) / 2.55;      // The cast is needed on arduino
        float heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat((char *)&this->_data[6]) / 2.55; // The cast is needed on arduino

        return (Heading){.yaw = yaw * 360,
                         .pitch = pitch * 360,
                         .roll = roll * 360,
                         .heading = heading * 360};
    }

    static FieldCentricInput convertToRobotCentric(double forward, double strafe, double rotation, double gyroAngle)
    {
        // Convert gyro angle to radians
        double angleRad = (gyroAngle * PI) / 180.0;

        // Perform field-centric to robot-centric conversion
        auto field_forward = constrain(forward * cos(angleRad) + strafe * sin(angleRad), -127, 128);
        auto field_strafe = constrain(-forward * sin(angleRad) + strafe * cos(angleRad), -127, 128);

        return (FieldCentricInput){.forward = field_forward,
                                   .strafe = field_strafe,
                                   .rotation = rotation};
    }
};

struct Joystick
{
    float x, y;
};

struct JoystickPair
{
    Joystick left, right;
};

/**
 * =============
 * CONFIGURATION
 * =============
 */

const pin_t WHEEL_FL_M_p = CRC_PWM_11;
const pin_t WHEEL_FR_M_p = CRC_PWM_2;
const pin_t WHEEL_BL_M_p = CRC_PWM_12;
const pin_t WHEEL_BR_M_p = CRC_PWM_4;

const pin_t LIFT_L_M_p = CRC_PWM_10;
const pin_t LIFT_R_M_p = CRC_PWM_3;
const pin_t LIFT_E_p = CRC_DIG_3; // lift height

const pin_t MANIP_PITCH_E_p = CRC_DIG_4;
const pin_t MANIP_PITCH_M_p = CRC_PWM_8;

const pin_t MANIP_ROLL_E_p = CRC_DIG_2;
const pin_t MANIP_ROLL_M_p = CRC_PWM_1; // NOTE: limit to 20% is a good default speed

const pin_t MANIP_BELT_A_p = CRC_PWM_9;
const pin_t MANIP_BELT_B_p = CRC_PWM_5;

const pin_t BEAM_p = CRC_DIG_1;

const int PRINT_TIMER_DELAY = 1000 / 20; // 20Hz

const double FIELD_CENTRIC_P = 7;
const double FIELD_CENTRIC_I = 0;
const double FIELD_CENTRIC_D = 0.0011;

const double FIELD_CENTRIC_OUTPUT_LIM = 60;
const double FIELD_CENTRIC_SAMPLE_FREQ_HZ = 50;

/**
 * ========================
 * WORKERS (OBJECTS & VARS)
 * ========================
 */

Servo manip_belt_a, manip_belt_b;

NavX navx;

CrcLib::Timer print_timer, battery_low_timeout;

ReadPWM lift_PWM(LIFT_E_p), pitch_PWM(MANIP_PITCH_E_p), roll_PWM(MANIP_ROLL_E_p);
PwmToAngleConverter lift_converter, pitch_converter, roll_converter;
AngleMovingAvg<200> lift_averager, pitch_averager, roll_averager; // TODO: 20 might be alot

float input, output, setpoint = 0;
QuickPID pid(&input, &output, &setpoint,
             FIELD_CENTRIC_P, FIELD_CENTRIC_I, FIELD_CENTRIC_D, QuickPID::Action::reverse);

/**
 * ============
 * SETUP & LOOP
 * ============
 */

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    CrcLib::Initialize();

    CrcLib::InitializePwmOutput(WHEEL_FL_M_p, false);
    CrcLib::InitializePwmOutput(WHEEL_FR_M_p, true); // Is normally true
    CrcLib::InitializePwmOutput(WHEEL_BL_M_p, false);
    CrcLib::InitializePwmOutput(WHEEL_BR_M_p, true); // Is normally true

    CrcLib::InitializePwmOutput(LIFT_L_M_p, true);
    CrcLib::InitializePwmOutput(LIFT_R_M_p, true);

    CrcLib::InitializePwmOutput(MANIP_PITCH_M_p, false);
    CrcLib::InitializePwmOutput(MANIP_ROLL_M_p, false);

    pinMode(BEAM_p, INPUT);

    // manip_belt_a.attach(MANIP_BELT_A_p);
    // manip_belt_b.attach(MANIP_BELT_B_p);

    print_timer.Start(PRINT_TIMER_DELAY);

    pid.SetMode(QuickPID::Control::automatic);
    pid.SetSampleTimeUs(1000 / FIELD_CENTRIC_SAMPLE_FREQ_HZ);
    pid.SetOutputLimits(-FIELD_CENTRIC_OUTPUT_LIM, FIELD_CENTRIC_OUTPUT_LIM);
}

void soft_kill()
{
    CrcLib::SetPwmOutput(WHEEL_BL_M_p, 0);
    CrcLib::SetPwmOutput(WHEEL_BR_M_p, 0);
    CrcLib::SetPwmOutput(WHEEL_FL_M_p, 0);
    CrcLib::SetPwmOutput(WHEEL_FR_M_p, 0);
    CrcLib::SetPwmOutput(LIFT_L_M_p, 0);
    CrcLib::SetPwmOutput(LIFT_R_M_p, 0);
    CrcLib::SetPwmOutput(MANIP_PITCH_M_p, 0);
    CrcLib::SetPwmOutput(MANIP_ROLL_M_p, 0);
    // manip_belt_a.write(0);
    // manip_belt_b.write(0);
    pid.SetOutputSum(0);
}

void loop()
{
    CrcLib::Update();

    /**
     * ------
     * GUARDS
     * ------
     */

    static float battery_voltage_limit = 11.0;
    if (CrcLib::GetBatteryVoltage() < battery_voltage_limit)
    {
        // TODO: figure out something more graceful.
        battery_voltage_limit = 15.0f;
        Serial.println("Battery LOW: " + String(CrcLib::GetBatteryVoltage()));
        soft_kill();
        return;
    }

    if (!CrcLib::IsCommValid())
    {
        // block everything if controller is not connected
        soft_kill();
        Serial.println("no com");
        return;
    }

    /**
     * ----------------
     * CONTROLLER INPUT
     * ----------------
     */

    auto joysticks_raw = (JoystickPair){
        .left = {
            .x = (float)CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_X),
            .y = (float)CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_Y),
        },
        .right = {
            .x = (float)CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK2_X),
            .y = (float)CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK2_Y),
        }};

    int8_t trig_L = CrcLib::ReadAnalogChannel(ANALOG::GACHETTE_L);
    int8_t trig_R = CrcLib::ReadAnalogChannel(ANALOG::GACHETTE_R);

    /**
     * ------------------
     * SENSOR AQUISITIONS
     * ------------------
     */

    uint32_t manip_pitch_signal;
    pitch_PWM.read(manip_pitch_signal);
    uint32_t manip_roll_signal;
    roll_PWM.read(manip_roll_signal);
    uint32_t lift_height_signal;
    lift_PWM.read(lift_height_signal);

    bool beam_obstructed = CrcLib::GetDigitalInput(BEAM_p);

    NavX::Heading h = navx.read();

    /**
     * ---------------
     * DATA PROCESSING
     * --------------
     */

    auto joysticks_clean = (JoystickPair){
        .left = {
            .x = (float)clean_joystick_input(joysticks_raw.left.x) / 5,
            .y = (float)clean_joystick_input(joysticks_raw.left.y) / 5,
        },
        .right = {
            .x = (float)clean_joystick_input(joysticks_raw.right.x) / 5,
            .y = (float)clean_joystick_input(joysticks_raw.right.y) / 5,
        }};

    lift_averager.add(lift_converter.convert(lift_height_signal));
    pitch_averager.add(pitch_converter.convert(manip_pitch_signal));
    roll_averager.add(roll_converter.convert(manip_roll_signal));
    auto lift_deg = lift_averager.calc();
    auto roll_deg = roll_averager.calc();
    auto pitch_deg = pitch_averager.calc();

    float current_rotation = h.yaw; // TODO figure out
    // TODO: fix navx instead, will make a better resolution
    if (current_rotation < 180)
    {
        current_rotation = map(current_rotation, 0.0, 100.0, 0.0, 180.0);
    }
    else
    {
        current_rotation = map(current_rotation, 260.0, 360.0, 180.0, 360.0);
    }

    /**
     * ---------------
     * PID SHENANIGANS
     * ---------------
     */

    static float target_rotation = NAN;
    if (isnan(target_rotation))
    {
        // NOTE: intialize target to the current rotation to avoid the robot trying to move right after turning on
        target_rotation = current_rotation;
    }

    if (abs(joysticks_clean.right.x) > 5 || abs(joysticks_clean.right.y) > 5)
    {
        target_rotation = 180 - (atan2(joysticks_clean.right.x, joysticks_clean.right.y) * 180 / PI);
    }

    input = travel_deg(current_rotation, target_rotation);

    /**
     * -----------------------
     * MOTOR OUTPUTS, CONTROLS
     * -----------------------
     */

    if (true && pid.Compute())
    {
        // Convert joystick inputs to field-centric
        NavX::FieldCentricInput robotCentric = NavX::convertToRobotCentric(
            joysticks_clean.left.y, // Forward
            joysticks_clean.left.x, // Strafe
            output,                 // Rotation
            -convert_domain(current_rotation, AngleDomain::mirror));

        // Apply converted values to motors
        CrcLib::MoveHolonomic(
            robotCentric.forward,
            robotCentric.rotation,
            robotCentric.strafe,
            WHEEL_FL_M_p, WHEEL_BL_M_p, WHEEL_FR_M_p, WHEEL_BR_M_p);
    }

    if (false && CrcLib::ReadDigitalChannel(BUTTON::COLORS_UP))
    {
        /* LIFT */
        CrcLib::SetPwmOutput(LIFT_L_M_p, trig_L);
        CrcLib::SetPwmOutput(LIFT_R_M_p, trig_R);
    }

    if (false && CrcLib::ReadDigitalChannel(BUTTON::COLORS_LEFT))
    {
        /* MANIPULATOR PITCH/ROLL */
        CrcLib::SetPwmOutput(MANIP_PITCH_M_p, trig_L);
        CrcLib::SetPwmOutput(MANIP_ROLL_M_p, trig_R);
    }

    if (false && CrcLib::ReadDigitalChannel(BUTTON::COLORS_RIGHT))
    {
        /* BELTS */
        // TODO: UNIMPLEMENTED
    }

    if (CrcLib::ReadDigitalChannel(BUTTON::COLORS_DOWN))
    {
        Serial.println("softkilling");
        soft_kill();
    }

    /**
     * ----------------
     * SERIAL REPORTING
     * ----------------
     */
    if (print_timer.IsFinished())
    {
        print_timer.Start(PRINT_TIMER_DELAY);

        // Serial.println("Battery voltage: " + String(CrcLib::GetBatteryVoltage()));

        /* beam state */
        // Serial.println("beam: " + String(beam_obstructed));

        /* reading encoders */
        Serial.print("l: " + String(lift_converter.convert(lift_height_signal)) +
                     "\tp: " + String(pitch_converter.convert(manip_pitch_signal)) +
                     "\tr: " + String(roll_converter.convert(manip_roll_signal)));

        // FIXME: averager does NOT work
        Serial.println("\t\tl: " + String(lift_averager.calc()) +
                       "\tp: " + String(pitch_averager.calc()) +
                       "\tr: " + String(roll_averager.calc()));

        /* controller trigger states */
        // Serial.println("triggers:\tL:" + String(trig_L) + "\tR: " + String(trig_R));

        /* cmp expected rotation with curent rotation*/
        // Serial.println("expected: " + String(target_rotation) + "\tactual: " + String(current_rotation));

        /* field centric PID info */
        // Serial.println("input: " + String(input) + "\toutput: " + String(output));
    }
}