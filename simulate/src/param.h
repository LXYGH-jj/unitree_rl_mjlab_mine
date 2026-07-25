#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace param
{

inline struct SimulationConfig
{
    std::filesystem::path config_file;
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    double joint_pos_noise_std = 0.0;
    double joint_vel_noise_std = 0.0;
    double imu_gyro_noise_std = 0.0;
    double imu_acc_noise_std = 0.0;
    double imu_rpy_noise_std = 0.0;
    double base_lin_vel_noise_std = 0.0;

    int action_delay_ms = 0;

    double ctrl_noise_std = 0.0;
    double ctrl_noise_rate = 0.0;

    int randomize_foot_friction = 0;
    double foot_friction_min = 0.4;
    double foot_friction_max = 0.4;

    int publish_foot_force_from_contacts = 0;
    double foot_contact_force_value = 100.0;
    std::vector<std::string> foot_contact_geom_names = {"FL", "FR", "RL", "RR"};

    int depth_preview_enabled = 0;
    std::string depth_preview_camera_name = "front_depth";
    int depth_preview_width = 174;
    int depth_preview_height = 116;
    int depth_preview_update_every = 2;
    int depth_preview_print_stats = 0;
    std::string depth_preview_source = "opengl";
    std::vector<int> depth_preview_raycast_geom_groups = {0};

    int depth_camera_publisher_enabled = 0;
    std::string depth_camera_publisher_mode = "viewer";
    double depth_camera_publisher_publish_dt = 0.02;
    std::string depth_camera_publisher_topic = "/go2_depth/sim/depth/image_raw";
    std::string depth_camera_publisher_node_name = "go2_mujoco_depth_camera_publisher";
    int depth_camera_publisher_publish_every = 2;
    int depth_camera_publisher_print_every = 30;

    int depth_camera_publisher_opengl_far_to_zero_enabled = 0;
    double depth_camera_publisher_opengl_far_to_zero_threshold = 20.0;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();

            if (cfg["publish_foot_force_from_contacts"]) {
                publish_foot_force_from_contacts = cfg["publish_foot_force_from_contacts"].as<int>();
            }
            if (cfg["foot_contact_force_value"]) {
                foot_contact_force_value = cfg["foot_contact_force_value"].as<double>();
            }
            if (cfg["foot_contact_geom_names"] && cfg["foot_contact_geom_names"].IsSequence()) {
                foot_contact_geom_names = cfg["foot_contact_geom_names"].as<std::vector<std::string>>();
            }
            if (cfg["depth_preview"])
            {
                const auto depth_preview = cfg["depth_preview"];
                depth_preview_enabled = depth_preview["enabled"].as<int>(depth_preview_enabled);
                depth_preview_camera_name =
                    depth_preview["camera_name"].as<std::string>(depth_preview_camera_name);
                depth_preview_width = depth_preview["width"].as<int>(depth_preview_width);
                depth_preview_height = depth_preview["height"].as<int>(depth_preview_height);
                depth_preview_update_every =
                    depth_preview["update_every"].as<int>(depth_preview_update_every);
                depth_preview_print_stats =
                    depth_preview["print_stats"].as<int>(depth_preview_print_stats);
                depth_preview_source =
                    depth_preview["source"].as<std::string>(depth_preview_source);
                if (depth_preview["raycast_geom_groups"] &&
                    depth_preview["raycast_geom_groups"].IsSequence()) {
                    depth_preview_raycast_geom_groups =
                        depth_preview["raycast_geom_groups"].as<std::vector<int>>();
                }
            }
            if (cfg["depth_camera_publisher"])
            {
                const auto depth_pub = cfg["depth_camera_publisher"];
                depth_camera_publisher_enabled =
                    depth_pub["enabled"].as<int>(depth_camera_publisher_enabled);
                depth_camera_publisher_mode =
                    depth_pub["mode"].as<std::string>(depth_camera_publisher_mode);
                depth_camera_publisher_publish_dt =
                    depth_pub["publish_dt"].as<double>(depth_camera_publisher_publish_dt);
                depth_camera_publisher_topic =
                    depth_pub["topic"].as<std::string>(depth_camera_publisher_topic);
                depth_camera_publisher_node_name =
                    depth_pub["node_name"].as<std::string>(depth_camera_publisher_node_name);
                depth_camera_publisher_publish_every =
                    depth_pub["publish_every"].as<int>(depth_camera_publisher_publish_every);
                depth_camera_publisher_print_every =
                    depth_pub["print_every"].as<int>(depth_camera_publisher_print_every);
                depth_camera_publisher_opengl_far_to_zero_enabled =
                    depth_pub["opengl_far_to_zero_enabled"].as<int>(
                        depth_camera_publisher_opengl_far_to_zero_enabled);
                depth_camera_publisher_opengl_far_to_zero_threshold =
                    depth_pub["opengl_far_to_zero_threshold"].as<double>(
                        depth_camera_publisher_opengl_far_to_zero_threshold);
            }

            if (cfg["lightweight_realism"])
            {
                const auto realism = cfg["lightweight_realism"];
                joint_pos_noise_std = realism["joint_pos_noise_std"].as<double>(joint_pos_noise_std);
                joint_vel_noise_std = realism["joint_vel_noise_std"].as<double>(joint_vel_noise_std);
                imu_gyro_noise_std = realism["imu_gyro_noise_std"].as<double>(imu_gyro_noise_std);
                imu_acc_noise_std = realism["imu_acc_noise_std"].as<double>(imu_acc_noise_std);
                imu_rpy_noise_std = realism["imu_rpy_noise_std"].as<double>(imu_rpy_noise_std);
                base_lin_vel_noise_std = realism["base_lin_vel_noise_std"].as<double>(base_lin_vel_noise_std);

                action_delay_ms = realism["action_delay_ms"].as<int>(action_delay_ms);

                ctrl_noise_std = realism["ctrl_noise_std"].as<double>(ctrl_noise_std);
                ctrl_noise_rate = realism["ctrl_noise_rate"].as<double>(ctrl_noise_rate);

                randomize_foot_friction = realism["randomize_foot_friction"].as<int>(randomize_foot_friction);
                if (realism["foot_friction_range"] && realism["foot_friction_range"].IsSequence() &&
                    realism["foot_friction_range"].size() >= 2)
                {
                    foot_friction_min = realism["foot_friction_range"][0].as<double>();
                    foot_friction_max = realism["foot_friction_range"][1].as<double>();
                }
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
        ("config,c", po::value<std::filesystem::path>(&config.config_file), "Simulation config file")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}
