"""Export model_57000.pt -> an ALORE-compatible [1,799]->[1,18] TorchScript actor.

Use ONLY from the existing Interactive training checkout / b2z1_wbc conda env;
this script requires Isaac Gym and the environment to construct Actor architecture.
"""
import argparse
import os
import sys
from pathlib import Path


def main():
    cli = argparse.ArgumentParser()
    cli.add_argument('--output', required=True)
    cli.add_argument('--checkpoint', default='')
    cli.add_argument('--force', action='store_true')
    opts = cli.parse_args()
    target = Path(opts.output).expanduser().resolve()
    if target.exists() and not opts.force:
        raise FileExistsError(f'{target} already exists (use --force only if intentional)')
    # Override Isaac Gym CLI with validated settings from the local preview.
    sys.argv = ['export_interactive_actor', '--task','b2z1', '--num_envs','1',
                '--sim_device','cuda:0','--rl_device','cuda:0', '--pipeline','gpu',
                '--headless','--observe_gait_commands','--exptid','export_jit']
    from isaacgym import gymapi  # must import before torch
    import torch
    import copy
    from legged_gym import LEGGED_GYM_ROOT_DIR
    import legged_gym.envs  # register b2z1 task without wildcard import
    from legged_gym.utils import get_args, task_registry

    args = get_args()
    args.resume = False
    args.resumeid = None
    cfg, train_cfg = task_registry.get_cfgs('b2z1')
    cfg.terrain.mesh_type = 'plane'
    cfg.terrain.measure_heights = False
    cfg.terrain.curriculum = False
    cfg.noise.add_noise = False
    train_cfg.runner.resume = False
    checkpoint = (Path(opts.checkpoint).expanduser().resolve() if opts.checkpoint else
                  Path(LEGGED_GYM_ROOT_DIR)/'logs/b2z1-low/kp_350_kd_5/model_57000.pt')
    if not checkpoint.is_file():
        raise FileNotFoundError(checkpoint)
    env = None
    try:
        env, _ = task_registry.make_env('b2z1', args=args, env_cfg=cfg)
        runner, _, _ = task_registry.make_alg_runner(env, name='b2z1', args=args,
                                                      train_cfg=train_cfg, log_root=None)
        weights = torch.load(str(checkpoint), map_location='cpu')
        runner.alg.actor_critic.load_state_dict(weights['model_state_dict'], strict=True)
        actor = copy.deepcopy(runner.alg.actor_critic.actor).cpu().eval()

        class HistoryActor(torch.nn.Module):
            def __init__(self, core):
                super().__init__()
                self.core = core
            def forward(self, obs):
                return self.core(obs, hist_encoding=True)

        model = HistoryActor(actor).eval()
        rng = torch.Generator().manual_seed(7)
        samples = [torch.randn(1,799, generator=rng) for _ in range(5)]
        with torch.no_grad():
            jit = torch.jit.trace(model, samples[0], check_trace=True).eval()
            for sample in samples:
                ref = model(sample)
                got = jit(sample)
                assert got.shape == (1,18), got.shape
                error = (ref-got).abs().max().item()
                print(f'Parity diff={error:.9g}')
                if error > 1e-5:
                    raise RuntimeError(f'TorchScript parity FAIL: {error}')
        target.parent.mkdir(parents=True,exist_ok=True)
        jit.save(str(target))
        loaded = torch.jit.load(str(target), map_location='cpu').eval()
        with torch.no_grad():
            assert torch.allclose(model(samples[1]), loaded(samples[1]), atol=1e-5, rtol=1e-5)
        print('EXPORTED:',target)
        print('OUTPUT SHAPE [1,18], INPUT SHAPE [1,799], hist_encoding=True')
    finally:
        if env is not None:
            if env.viewer is not None:
                env.gym.destroy_viewer(env.viewer)
            env.gym.destroy_sim(env.sim)


if __name__ == '__main__':
    main()
