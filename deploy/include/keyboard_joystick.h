#pragma once

#include <algorithm>
#include <string>

#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree/dds_wrapper/common/unitree_joystick.hpp"

class KeyboardJoystick
{
public:
    explicit KeyboardJoystick(std::shared_ptr<Keyboard> keyboard)
    : keyboard_(std::move(keyboard))
    {
    }

    void update(unitree::common::UnitreeJoystick& joystick)
    {
        if (!keyboard_) return;
        update_joystick(joystick, keyboard_->key());
    }

    static void update_joystick(unitree::common::UnitreeJoystick& joystick, const std::string& key)
    {
        if (key.empty()) return;

        // Overlay keyboard shortcuts onto the latest real joystick state.
        if (key == "f" || key == "p") joystick.LT(1.0f);
        if (key == "v") joystick.RT(1.0f);
        if (key == "f") joystick.up(1);
        if (key == "v") joystick.A(1);
        if (key == "p") joystick.B(1);
        if (key == "o") joystick.Y(1);
        if (key == "g") joystick.X(1);  // M2.0a SIM-only FSM select (X.on_pressed)
        if (key == "l") {
            joystick.LT(1.0f);
            joystick.down(1);
        }

        if (key == "w" || key == "s") joystick.ly(axis(key == "w", key == "s"));
        if (key == "d" || key == "a") joystick.lx(axis(key == "d", key == "a"));
        if (key == "e" || key == "q") joystick.rx(axis(key == "e", key == "q"));
    }

private:
    static float axis(bool positive, bool negative)
    {
        return static_cast<float>(positive) - static_cast<float>(negative);
    }

    std::shared_ptr<Keyboard> keyboard_;
};
