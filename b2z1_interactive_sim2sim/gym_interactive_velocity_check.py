#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Check the SAME exported Interactive actor in the ORIGINAL Isaac Gym B2+Z1 environment.

The environment's own observation implementation (799D), action processing, 50 Hz
policy stepping, arm IK and contact simulation are preserved. We force a constant
linear command after the warmup and log measured world/body velocities and yaw.

No training, checkpoint writing, robot DDS or ROS operations are performed.
"""
import argparse
import csv
import math
import sys
from pathlib import Path


def get_options():
    parser = argparse.ArgumentParser(description="Interactive original Gym velocity tracking test")
    parser.add_argument("--interactive-root", required=True,
                        help="Path to Interactive-Navigation-for-legged-manipulator/low_level_WBC")
    parser.add_argument("--policy", required=True, help="Previously exported [1,799]->[1,18] jit .pt")
    parser.add_argument("--vx", type=float, default=0.30)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--warmup", type=float, default=1.0)
    parser.add_argument("--sample-every", type=float, default=0.5)
    parser.add_argument("--csv", default="gym_velocity_vx_030.csv")
    parser.add_argument("--viewer", action="store_true", help="Show Gym viewer (headless by default)")
    parser.add_argument("--device", default="cuda:0")
    op = parser.parse_args()
    if not (math.isfinite(op.vx) and abs(op.vx) <= 1.0):
        parser.error("--vx must be finite with absolute value <= 1.0 m/s")
    if not math.isfinite(op.seconds) or op.seconds <= 0 or op.seconds > 120:
        parser.error("--seconds must be in (0,120]")
    if not math.isfinite(op.warmup) or op.warmup < 0 or op.warmup >= op.seconds:
        parser.error("--warmup must be in [0,seconds)")
    if not math.isfinite(op.sample_every) or op.sample_every <= 0:
        parser.error("--sample-every must be positive")
    root = Path(op.interactive_root).expanduser().resolve()
    policy = Path(op.policy).expanduser().resolve()
    if not (root / "legged_gym/envs/manip_loco/manip_loco.py").is_file():
        parser.error("Invalid --interactive-root: missing legged_gym/envs/manip_loco/manip_loco.py")
    if not policy.is_file():
        parser.error("Policy file does not exist: " + str(policy))
    return op, root, policy


def snapshot(env, t, command):
    # GPU tensors are converted to CPU scalars only for logging; these are the
    # same true root velocities that Isaac Gym reports, not estimated velocities.
    import torch
    with torch.no_grad():
        r = env.root_states[0].detach().cpu().tolist()
        x, y, z = r[0:3]
        qx, qy, qz, qw = r[3:7]          # Isaac Gym quaternion convention: xyzw
        vx_w, vy_w, vz_w = r[7:10]       # linear velocity in world frame
        wz_w = r[12]                     # world-frame angular velocity about z
        yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
        c, s = math.cos(yaw), math.sin(yaw)
        vx_b = c*vx_w + s*vy_w
        vy_b = -s*vx_w + c*vy_w
        return {
            "t": t, "cmd_vx": command, "x": x, "y": y, "z": z,
            "yaw_deg": math.degrees(yaw),
            "world_vx": vx_w, "world_vy": vy_w, "world_vz": vz_w,
            "body_vx": vx_b, "body_vy": vy_b, "world_wz": wz_w,
        }


def print_record(row):
    print("t={t:4.2f}s cmd_vx={cmd_vx:+.2f}  x={x:+.3f} y={y:+.3f} z={z:.3f} "
          "yaw={yaw_deg:+.2f}deg  world_vx={world_vx:+.3f} world_vy={world_vy:+.3f} "
          "body_vx={body_vx:+.3f} body_vy={body_vy:+.3f}"
          .format(**row), flush=True)


def main():
    opt, root, policy_path = get_options()
    sys.path.insert(0, str(root))

    # Legged Gym's args parser consumes sys.argv. These are the validated
    # arguments also used for exporting the same model. No resume or training.
    sys.argv = ["gym_interactive_velocity_check", "--task", "b2z1",
                "--num_envs", "1", "--sim_device", opt.device,
                "--rl_device", opt.device, "--pipeline", "gpu",
                "--observe_gait_commands", "--exptid", "gym_velocity_check"]
    if not opt.viewer:
        sys.argv.append("--headless")

    from isaacgym import gymapi  # IMPORTANT: Isaac Gym must be imported before torch
    import torch
    import legged_gym.envs  # registers the b2z1 task
    from legged_gym.utils import get_args, task_registry

    args = get_args()
    args.resume = False
    args.resumeid = None
    cfg, _ = task_registry.get_cfgs("b2z1")
    cfg.env.num_envs = 1
    cfg.env.teleop_mode = True       # Critical: no periodic command re-sampling
    cfg.env.stand_by = False
    cfg.env.episode_length_s = opt.seconds + 10.0
    cfg.terrain.mesh_type = "plane"
    cfg.terrain.curriculum = False
    cfg.terrain.measure_heights = False
    cfg.terrain.height = [0.0, 0.0]
    cfg.noise.add_noise = False
    cfg.commands.curriculum = False
    cfg.domain_rand.push_robots = False
    cfg.domain_rand.randomize_base_mass = False
    cfg.domain_rand.randomize_base_com = False
    cfg.domain_rand.randomize_friction = False
    cfg.domain_rand.randomize_motor = False
    cfg.domain_rand.randomize_gripper_mass = False
    # Remove initial random yaw and velocity for an easier X/Y comparison.
    cfg.init_state.rand_yaw_range = 0.0
    cfg.init_state.origin_perturb_range = 0.0
    cfg.init_state.init_vel_perturb_range = 0.0
    # Do not change Gym's native arm IK, action scale, observation history,
    # contact state estimation, joint gains, or policy step rate.

    env = None
    try:
        env, _ = task_registry.make_env("b2z1", args=args, env_cfg=cfg)
        assert env.num_obs == 799, f"Expected 799 observations, got {env.num_obs}"
        assert env.num_actions == 18, f"Expected 18 actions, got {env.num_actions}"
        model = torch.jit.load(str(policy_path), map_location=env.device).eval()
        print("Loaded exported Interactive actor:", policy_path)
        print("Gym control_dt=", env.dt, "obs_dim=", env.num_obs,
              "action_dim=", env.num_actions)
        print("Command fixed by teleop_mode=True; randomness disabled; original arm IK enabled")
        print("Warmup:", opt.warmup, "seconds; command:", opt.vx, "m/s", flush=True)

        obs, _ = env.reset()
        assert tuple(obs.shape) == (1,799), tuple(obs.shape)
        env.commands[:, :] = 0.0
        rows = [snapshot(env, 0.0, 0.0)]
        print_record(rows[0])
        steps = max(1, int(round(opt.seconds/env.dt)))
        print_every = max(1, int(round(opt.sample_every/env.dt)))

        with torch.no_grad():
            for k in range(steps):
                t_before = k*env.dt
                vx = 0.0 if t_before < opt.warmup else opt.vx
                env.commands[:, 0] = vx
                env.commands[:, 1] = 0.0
                env.commands[:, 2] = 0.0
                # The observation returned from the previous step may contain
                # the previous command for ONE tick at the warmup boundary.
                # We do not call compute_observations() twice, as that would
                # otherwise advance history incorrectly.
                actions = model(obs)
                if tuple(actions.shape) != (1,18):
                    raise ValueError("Unexpected policy output: " + str(tuple(actions.shape)))
                if not bool(torch.isfinite(actions).all().item()):
                    raise RuntimeError("Non-finite actions at step " + str(k))
                result = env.step(actions.detach().clone())
                obs = result[0]
                done = result[4]
                t = (k+1)*env.dt
                row = snapshot(env, t, vx)
                rows.append(row)
                if k % print_every == 0 or k == steps-1:
                    print_record(row)
                if bool(done[0].item()):
                    print("WARNING: Isaac Gym episode terminated/reset at t={:.3f}s. "
                          "Stopping; results after reset would not be comparable.".format(t))
                    break

        file = Path(opt.csv).expanduser().resolve()
        file.parent.mkdir(parents=True, exist_ok=True)
        with file.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print("Saved CSV:", file)

        # Exclude warmup / acceleration transient for steady-state velocity.
        # If the user selects <3 seconds after warmup, use the second half.
        post = [r for r in rows if r["t"] >= opt.warmup + 1.5 and r["t"] <= opt.seconds - 0.1]
        if len(post) < 2:
            post = [r for r in rows if r["t"] >= opt.warmup + 0.5]
        if post:
            keys = ("body_vx", "body_vy", "world_vx", "world_vy", "z", "world_wz")
            avg = {k: sum(r[k] for r in post)/len(post) for k in keys}
            print("===== Steady-state averages (after acceleration) =====")
            print("target body_vx={:.3f} m/s, body_vy=0.000 m/s".format(opt.vx))
            print("avg_body_vx={:.3f} avg_body_vy={:.3f} avg_world_vx={:.3f} "
                  "avg_world_vy={:.3f} avg_root_z={:.3f} avg_world_wz={:.3f}"
                  .format(avg["body_vx"], avg["body_vy"], avg["world_vx"],
                          avg["world_vy"], avg["z"], avg["world_wz"]))
        first, last = rows[0], rows[-1]
        dx, dy = last["x"]-first["x"],last["y"]-first["y"]
        print("World displacement: dx={:+.3f} m, dy={:+.3f} m, path_angle={:+.2f} deg".format(
            dx,dy,math.degrees(math.atan2(dy,dx))))
        print("Final yaw={:+.2f} deg".format(last["yaw_deg"]))
    finally:
        if env is not None:
            if env.viewer is not None:
                env.gym.destroy_viewer(env.viewer)
            env.gym.destroy_sim(env.sim)


if __name__ == "__main__":
    main()
