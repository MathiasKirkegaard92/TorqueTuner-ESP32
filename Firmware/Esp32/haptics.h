#ifndef __HAPTICS_H__
#define  __HAPTICS_H__

#include <stdint.h>
#include <functional>
#include <cmath>
#include "tf_magnet.h"
#include "tf_click.h"
#include "tf_wall.h"
#include "tf_sin.h"
#include "tf_click_2.h"
#include "arduino.h"

const int TABLE_RESOLUTION = 65535;
const int MAX_TORQUE = 233;
const int WALL_TORQUE = 150;
const float MAX_VELOCITY = 350;

class Mode
{
public:
    Mode( float torque_ = MAX_TORQUE / 2, int angle_ = 1, float min_ = 0, float max_ = 3600, float damping_ = 0)
        : min(min_), max(max_), scale_default(torque_), stretch_default(angle_), damping(damping_) {};
    virtual int16_t calc(void* ptr) = 0;
    int16_t calc_index(void* ptr);
    void reset(int16_t angle_);

    float scale_default, stretch_default, min, max, damping;
    int offset = 0;

    bool wrap_output = false;
    bool wrap_haptics = false;
    char pid_mode = 't';
    int idx = 0;
};

class Wall: public Mode
{
public:
    Wall() : Mode(MAX_TORQUE / 2) {}
    int16_t calc(void* ptr);
};

class Click: public Mode
{
public:
    Click() : Mode(MAX_TORQUE / 3.0, 10, 0, 3600) {
        offset = 1799;
        wrap_output = true;
        wrap_haptics = true;
    }
    int16_t calc(void* ptr);
};

class Magnet: public Mode
{
public:
    Magnet() : Mode() {}
    int16_t calc(void* ptr);
};

class Inertia: public Mode
{
public:
    Inertia() : Mode(MAX_TORQUE) {}
    int16_t calc(void* ptr);
};

class ExpSpring: public Mode
{
public:
    ExpSpring() : Mode(150, 1, 0, 3600, 0.1) {
        stretch_default  = 8;
        wrap_output = false;
        wrap_haptics = false;
    }
    int16_t calc(void* ptr);
};

class LinSpring: public Mode
{
public:
    LinSpring() : Mode(MAX_TORQUE / 2, 1, 0, 3600) {
        wrap_output = true;
        wrap_haptics = false;
    }
    int16_t calc(void* ptr);
};

class Free_torque: public Mode
{
public:
    Free_torque() : Mode(0, 1) {}
    int16_t calc(void* ptr);
};

class Spin: public Mode
{
public:
    Spin() : Mode(0, 1) {
        pid_mode = 'v';
        wrap_output = true;
    }
    int16_t calc(void* ptr);
};


class HapticKnob
{
public:
    enum MODE {
        CLICK = 0,
        MAGNET = 1,
        WALL = 2,
        INERTIA = 3,
        LINSPRING = 4,
        EXPSPRING = 5,
        FREE_TORQUE = 6,
        SPIN = 7,
    };
    const int trigger_interval = 50000;  // 10 ms

    HapticKnob();

    void update();
    void update_angle();
    void update_trig();
    float calc_acceleration(float velocity_);
    float filter(float x);
    float gate(float val, float threshold, float floor);
    int32_t getTime();

    void set_mode(MODE mode_);
    void set_mode(int mode_idx);
    void print_mode(MODE mode_);
    void set_defaults(Mode * mode);
    void set_stretch(float stretch_);
    void reset(Mode * mode_);

    MODE mode = WALL;
    int16_t angle = 0; // unwrap_outputped angle representing encoder reading
    int16_t angle_last = 0;
    int32_t angle_out = 0;
    int32_t angle_out_last = 0;
    int16_t wrap_count = 0;
    int16_t angle_delta = 0;
    int16_t angle_unclipped = 0;
    int16_t angle_discrete = 0; // angle output value syncronized via libmapper
    int16_t angle_discrete_last = 0;
    int16_t torque = 0;
    int trigger = -1;
    int num_modes = 0;

    float velocity = 0;
    float velocity_out = 0;
    float target_velocity = 350; // [-500;500]
    float acceleration = 0; // [-100;100]
    float scale = 75.0;
    float stretch = 1; // Corresponds to detents in click and magnet mode.

    Click click;
    Magnet magnet;
    Wall wall;
    LinSpring lin_spring;
    ExpSpring exp_spring;
    Free_torque free_torque;
    Inertia inertia;
    Spin spin;
    std::vector<Mode * > mode_list = {&click, &magnet, &wall, &inertia, &lin_spring, &exp_spring,  &free_torque, &spin};
    Mode * active_mode;

private:
    // Filter variables
    float a[3] = {1.0000,   -1.3329,    0.5053};
    float b[3] = { 0.0431,    0.0862,    0.0431};
};

int zero_crossing(int in);


inline int mod(int in, int hi) {
    const int lo = 0;
    if (in >= hi) {
        in -= hi;
        if (in < hi) {
            return in;
        }
    } else if (in < lo) {
        in += hi;
        if (in >= lo) {
            return in;
        }
    } else {
        return in;
    }
    if (hi == lo) {
        return lo;
    }
    int c;
    c = in % hi;
    if (c < 0) {
        c += hi;
    }
    return c;
}

inline int fold(int in, int lo, int hi) {
    int b = hi - lo;
    int b2 = b + b;
    int c = mod(in - lo, b2);
    if (c > b)
        c = b2 - c;
    return c + lo;
}

inline int clip(int in, int lo, int hi) {
    if (in > hi) {
        return hi;
    } else if (in < lo) {
        return lo;
    } else {
        return in;
    }
}



#endif
