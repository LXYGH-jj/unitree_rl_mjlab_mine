// Standalone SIM-only keyboard state test; requires no DDS or MuJoCo.
#include "../interactive_keyboard_sim.h"
#include <cassert>
#include <cmath>
int main() {
    InteractiveKeyboardState s;
    assert(s.feed('w', 2.6, 20) == KeyboardEvent::TooEarly);
    assert(s.velocity(3., 20) == 0.f);
    assert(s.feed('w', 4.51, 20) == KeyboardEvent::Forward);
    assert(std::abs(s.velocity(5., 20) - .12f) < 1.e-7f);
    assert(s.feed('w', 5.2, 20) == KeyboardEvent::None);
    assert(s.forwardTimedOut(6.52));
    assert(s.velocity(6.52, 20) == 0.f);
    assert(s.feed('W', 6.6, 20) == KeyboardEvent::None); // requires SPACE rearm
    assert(s.feed(' ', 6.7, 20) == KeyboardEvent::Stop);
    assert(s.feed('W', 7., 20) == KeyboardEvent::Forward);
    assert(s.velocity(18., 20) == 0.f); // final 2s recovery
    assert(s.feed('\x1b', 19.1, 20) == KeyboardEvent::Exit);
    assert(s.velocity(19.2, 20) == 0.f);
    assert(!s.readyToExit(21.05));
    assert(s.readyToExit(21.11));
}
