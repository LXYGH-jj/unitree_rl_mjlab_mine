#pragma once

#include <iostream>
#include <unitree/dds_wrapper/common/unitree_joystick.hpp>
#include "joystick/joystick.h"
#include <memory>
#include <atomic>
#include <GLFW/glfw3.h>


class XBoxJoystick : public unitree::common::UnitreeJoystick
{
public:
    XBoxJoystick(std::string device, int bits = 15)
	: unitree::common::UnitreeJoystick()
	{
		js_ = std::make_unique<Joystick>(device);
		if(!js_->isFound()) {
			std::cout << "Error: Joystick open failed." << std::endl;
			exit(1);
		}
        max_value_ = 1 << (bits - 1);
	}

    void update() override
    {
        js_->getState();
        back(js_->button_[6]);
        start(js_->button_[7]);
        LB(js_->button_[4]);
        RB(js_->button_[5]);
        A(js_->button_[0]);
        B(js_->button_[1]); 
        X(js_->button_[2]);
        Y(js_->button_[3]);
        up(js_->axis_[7] < 0);
        down(js_->axis_[7] > 0);
        left(js_->axis_[6] < 0);
        right(js_->axis_[6] > 0);
        LT(js_->axis_[2] > 0);
        RT(js_->axis_[5] > 0);
        lx(double(js_->axis_[0]) / max_value_);
        ly(-double(js_->axis_[1]) / max_value_);
        rx(double(js_->axis_[3]) / max_value_);
        ry(-double(js_->axis_[4]) / max_value_);
    }
private:
	std::unique_ptr<Joystick> js_;
	int max_value_;
};


class SwitchJoystick : public unitree::common::UnitreeJoystick
{
public:
    SwitchJoystick(std::string device, int bits = 15)
	: unitree::common::UnitreeJoystick()
	{
		js_ = std::make_unique<Joystick>(device);
		if(!js_->isFound()) {
			std::cout << "Error: Joystick open failed." << std::endl;
			exit(1);
		}
        max_value_ = 1 << (bits - 1);
	}

    void update() override
    {
        js_->getState();
        back(js_->button_[10]);
        start(js_->button_[11]);
        LB(js_->button_[6]);
        RB(js_->button_[7]);
        A(js_->button_[0]);
        B(js_->button_[1]); 
        X(js_->button_[3]);
        Y(js_->button_[4]);
        up(js_->axis_[7] < 0);
        down(js_->axis_[7] > 0);
        left(js_->axis_[6] < 0);
        right(js_->axis_[6] > 0);
        LT(js_->axis_[5] > 0);
        RT(js_->axis_[4] > 0);
        lx(double(js_->axis_[0]) / max_value_);
        ly(-double(js_->axis_[1]) / max_value_);
        rx(double(js_->axis_[2]) / max_value_);
        ry(-double(js_->axis_[3]) / max_value_);
    }
private:
	std::unique_ptr<Joystick> js_;
	int max_value_;
};


struct KeyboardTeleopState
{
    std::atomic<bool> w{false}, a{false}, s{false}, d{false};
    std::atomic<bool> q{false}, e{false};
    std::atomic<bool> up{false}, down{false}, left{false}, right{false};
    std::atomic<bool> lt{false}, rt{false};
    std::atomic<bool> button_a{false}, button_b{false}, button_x{false}, button_y{false};
    std::atomic<bool> lb{false}, rb{false};
    std::atomic<bool> start{false}, back{false};
    std::atomic<float> lx{0.0f}, ly{0.0f}, rx{0.0f}, ry{0.0f};
};

inline KeyboardTeleopState g_keyboard_teleop_state;

inline void update_keyboard_teleop_axes()
{
    const float lx = (g_keyboard_teleop_state.d.load() ? 1.0f : 0.0f) - (g_keyboard_teleop_state.a.load() ? 1.0f : 0.0f);
    const float ly = (g_keyboard_teleop_state.w.load() ? 1.0f : 0.0f) - (g_keyboard_teleop_state.s.load() ? 1.0f : 0.0f);
    const float rx = (g_keyboard_teleop_state.e.load() ? 1.0f : 0.0f) - (g_keyboard_teleop_state.q.load() ? 1.0f : 0.0f);
    g_keyboard_teleop_state.lx.store(lx);
    g_keyboard_teleop_state.ly.store(ly);
    g_keyboard_teleop_state.rx.store(rx);
    g_keyboard_teleop_state.ry.store(0.0f);
}

inline void keyboard_teleop_on_key(int key, int action)
{
    const bool pressed = action != GLFW_RELEASE;
    switch (key)
    {
    case GLFW_KEY_W: g_keyboard_teleop_state.w.store(pressed); break;
    case GLFW_KEY_A: g_keyboard_teleop_state.a.store(pressed); break;
    case GLFW_KEY_S: g_keyboard_teleop_state.s.store(pressed); break;
    case GLFW_KEY_D: g_keyboard_teleop_state.d.store(pressed); break;
    case GLFW_KEY_Q: g_keyboard_teleop_state.q.store(pressed); break;
    case GLFW_KEY_E: g_keyboard_teleop_state.e.store(pressed); break;

    case GLFW_KEY_UP: g_keyboard_teleop_state.up.store(pressed); break;
    case GLFW_KEY_DOWN: g_keyboard_teleop_state.down.store(pressed); break;
    case GLFW_KEY_LEFT: g_keyboard_teleop_state.left.store(pressed); break;
    case GLFW_KEY_RIGHT: g_keyboard_teleop_state.right.store(pressed); break;

    case GLFW_KEY_LEFT_SHIFT:
    case GLFW_KEY_RIGHT_SHIFT:
      g_keyboard_teleop_state.lt.store(pressed);
      break;
    case GLFW_KEY_LEFT_CONTROL:
    case GLFW_KEY_RIGHT_CONTROL:
      g_keyboard_teleop_state.rt.store(pressed);
      break;

    case GLFW_KEY_J: g_keyboard_teleop_state.button_a.store(pressed); break;
    case GLFW_KEY_K: g_keyboard_teleop_state.button_b.store(pressed); break;
    case GLFW_KEY_U: g_keyboard_teleop_state.button_x.store(pressed); break;
    case GLFW_KEY_I: g_keyboard_teleop_state.button_y.store(pressed); break;
    case GLFW_KEY_Z: g_keyboard_teleop_state.lb.store(pressed); break;
    case GLFW_KEY_C: g_keyboard_teleop_state.rb.store(pressed); break;
    case GLFW_KEY_ENTER: g_keyboard_teleop_state.start.store(pressed); break;
    case GLFW_KEY_BACKSPACE: g_keyboard_teleop_state.back.store(pressed); break;
    default:
      break;
    }
    update_keyboard_teleop_axes();
}

class KeyboardJoystick : public unitree::common::UnitreeJoystick
{
public:
    KeyboardJoystick()
    : unitree::common::UnitreeJoystick()
    {}

    void update() override
    {
        back(g_keyboard_teleop_state.back.load());
        start(g_keyboard_teleop_state.start.load());
        LB(g_keyboard_teleop_state.lb.load());
        RB(g_keyboard_teleop_state.rb.load());
        A(g_keyboard_teleop_state.button_a.load());
        B(g_keyboard_teleop_state.button_b.load());
        X(g_keyboard_teleop_state.button_x.load());
        Y(g_keyboard_teleop_state.button_y.load());
        up(g_keyboard_teleop_state.up.load());
        down(g_keyboard_teleop_state.down.load());
        left(g_keyboard_teleop_state.left.load());
        right(g_keyboard_teleop_state.right.load());
        LT(g_keyboard_teleop_state.lt.load());
        RT(g_keyboard_teleop_state.rt.load());
        lx(g_keyboard_teleop_state.lx.load());
        ly(g_keyboard_teleop_state.ly.load());
        rx(g_keyboard_teleop_state.rx.load());
        ry(g_keyboard_teleop_state.ry.load());
    }
};
