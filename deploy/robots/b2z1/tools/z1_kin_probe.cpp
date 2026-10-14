// M1.9a - Offline ONLY, no Unitree robot comms, no DDS, no motor commands.
// Link to the same vendor Z1 SDK used by the native B2Z1 controller.
#include <unitree_arm_sdk/model/ArmModel.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    try {
        std::string output = "deploy/robots/b2z1/build/m19_z1_sdk_fk.csv";
        for (int i=1; i<argc; ++i) {
            if (std::string(argv[i]) == "--output" && i+1<argc) output=argv[++i];
            else throw std::runtime_error("Usage: b2z1_z1_kin_probe [--output <csv>]");
        }
        const std::array<double,6> home{{0., .5, -1., 0., 0., 0.}};
        UNITREE_ARM::Z1Model arm;
        std::ofstream out(output);
        if (!out) throw std::runtime_error("Cannot open output file: " + output);
        out << std::setprecision(12);
        out << "sample,q1,q2,q3,q4,q5,q6";
        for (int r=0;r<4;++r) for (int c=0;c<4;++c) out << ",T" << r << c;
        out << "\n";
        int n=0;
        const auto dump=[&](const std::array<double,6>& qvals) {
            Vec6 q; for(int j=0;j<6;++j) q(j)=qvals[j];
            const HomoMat T=arm.forwardKinematics(q);
            if (!T.allFinite()) throw std::runtime_error("Non-finite SDK FK");
            out << n++;
            for (int j=0;j<6;++j) out << ',' << q(j);
            for (int r=0;r<4;++r) for (int c=0;c<4;++c) out << ',' << T(r,c);
            out << '\n';
        };
        dump(home);
        // One-joint perturbations isolate sign/order/axis inconsistencies.
        for (int j=0;j<6;++j) {
            for (double d : {-.08, .08}) {
                auto q=home;
                q[j]+=d;
                // For joint 2 and 3, their home is safely within limits.
                dump(q);
            }
        }
        // Couple joints to verify kinematics beyond isolated axes.
        for (int k=0;k<4;++k) {
            auto q=home;
            q[0]+=(k%2 ? -.10 : .10);
            q[1]+=(k%2 ? -.08 : .06);
            q[2]+=(k>=2 ? -.09 : .07);
            q[4]+=(k>=2 ? -.07 : .09);
            dump(q);
        }
        out.flush();
        std::cerr << "[M19] Wrote " << n << " Z1 SDK FK poses to: " << output << "\n";
        const Vec6 q0=(Vec6() << home[0],home[1],home[2],home[3],home[4],home[5]).finished();
        const HomoMat initial=arm.forwardKinematics(q0);
        for (int axis=0;axis<3;++axis) {
            HomoMat desired=initial;
            desired(axis,3)+=.01; // 10 mm in SDK model's base frame
            Vec6 solved=q0;
            const bool ok=arm.inverseKinematics(desired,q0,solved,false);
            const double position_error=(arm.forwardKinematics(solved).block<3,1>(0,3)-desired.block<3,1>(0,3)).norm();
            const double max_joint_change=(solved-q0).cwiseAbs().maxCoeff();
            std::cerr << "[M19IK] axis=" << axis << " success=" << ok
                      << " pos_error_m=" << position_error
                      << " max_joint_change_rad=" << max_joint_change
                      << " q=" << solved.transpose() << '\n';
        }
        std::cerr << "[M19] Probe is OFFLINE. No SDK2 / LowCmd / Z1 UDP control was started.\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr << "[M19 ERROR] " << e.what() << '\n';
        return 1;
    }
}
