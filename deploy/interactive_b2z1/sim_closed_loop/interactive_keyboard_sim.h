// M1.6a: SIM-ONLY terminal keyboard command adapter, no DDS publisher.
#pragma once

#include <cerrno>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace {

enum class KeyboardEvent { None, Forward, Stop, Exit, TooEarly };

class InteractiveKeyboardState {
public:
    KeyboardEvent feed(char ch, double sim_t, int sim_seconds) {
        if(ch=='\x1b') {
            if(!exit_requested_) {
                exit_requested_=true;
                exit_t_=sim_t;
                moving_=false;
                return KeyboardEvent::Exit;
            }
            return KeyboardEvent::None;
        }
        if(ch==' ') {
            if(moving_) {
                moving_=false;
                return KeyboardEvent::Stop;
            }
            return KeyboardEvent::None;
        }
        if(ch=='w' || ch=='W') {
            if(exit_requested_) return KeyboardEvent::None;
            if(sim_t < 4.5) return KeyboardEvent::TooEarly;
            if(sim_t >= static_cast<double>(sim_seconds)-2.0) return KeyboardEvent::None;
            if(!moving_) {
                moving_=true;
                ever_moved_=true;
                motion_started_t_=sim_t;
                return KeyboardEvent::Forward;
            }
        }
        return KeyboardEvent::None;
    }
    float velocity(double sim_t, int sim_seconds) const {
        if(exit_requested_ || sim_t < 4.5 ||
           sim_t >= static_cast<double>(sim_seconds)-2.0) return 0.f;
        return (moving_ && sim_t < motion_started_t_ + 2.0) ? 0.12f : 0.f;
    }
    bool everMoved() const { return ever_moved_; }
    bool forwardTimedOut(double sim_t) const {
        return moving_ && sim_t >= motion_started_t_ + 2.0;
    }
    bool readyToExit(double sim_t) const {
        return exit_requested_ && sim_t >= exit_t_ + 2.0;
    }
private:
    bool moving_{false};
    bool ever_moved_{false};
    bool exit_requested_{false};
    double exit_t_{0.0};
    double motion_started_t_{0.0};
};

// Only for a foreground controlling terminal. No threads or blocking reads.
// Keep ISIG enabled so Ctrl+C still reaches the existing safety handler.
class InteractiveTerminalInput {
public:
    explicit InteractiveTerminalInput(bool enabled) : enabled_(enabled) {
        if(!enabled_) return;
        if(!isatty(STDIN_FILENO))
            throw std::runtime_error("--interactive-keys requires a foreground TTY on stdin");
        if(tcgetattr(STDIN_FILENO,&old_term_)!=0)
            throw std::runtime_error("tcgetattr failed for interactive keyboard");
        old_flags_=fcntl(STDIN_FILENO,F_GETFL);
        if(old_flags_<0) throw std::runtime_error("fcntl F_GETFL failed");
        termios raw=old_term_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON|ECHO));
        raw.c_iflag &= static_cast<tcflag_t>(~(IXON|ICRNL));
        raw.c_cc[VMIN]=0;
        raw.c_cc[VTIME]=0;
        if(tcsetattr(STDIN_FILENO,TCSANOW,&raw)!=0)
            throw std::runtime_error("tcsetattr failed");
        terminal_changed_=true;
        if(fcntl(STDIN_FILENO,F_SETFL,old_flags_|O_NONBLOCK)!=0) {
            restore();
            throw std::runtime_error("fcntl O_NONBLOCK failed");
        }
    }
    InteractiveTerminalInput(const InteractiveTerminalInput&)=delete;
    InteractiveTerminalInput& operator=(const InteractiveTerminalInput&)=delete;
    ~InteractiveTerminalInput() { restore(); }

    template<typename Callback>
    void poll(Callback&& cb) {
        if(!enabled_) return;
        for(int i=0;i<32;++i) {
            char ch=0;
            const auto count=read(STDIN_FILENO,&ch,1);
            if(count==1) { cb(ch); continue; }
            if(count==0) return;
            if(errno==EINTR) continue;
            if(errno==EAGAIN || errno==EWOULDBLOCK) return;
            throw std::runtime_error("read stdin failed in interactive mode");
        }
    }
private:
    void restore() noexcept {
        if(!enabled_) return;
        if(old_flags_>=0) (void)fcntl(STDIN_FILENO,F_SETFL,old_flags_);
        if(terminal_changed_) (void)tcsetattr(STDIN_FILENO,TCSANOW,&old_term_);
        enabled_=false;
    }
    bool enabled_{false};
    bool terminal_changed_{false};
    int old_flags_{-1};
    termios old_term_{};
};
} // namespace
