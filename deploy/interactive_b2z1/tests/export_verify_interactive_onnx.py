#!/usr/bin/env python3
"""M1.2 (OFFLINE ONLY): export an existing Interactive TorchScript actor to ONNX.

No SDK2, Z1 SDK, DDS, ROS, MuJoCo or motor commands.

Defaults to static float32 [1,799] -> [1,18], opset 13. Performs an ONNX
structure check and TorchScript-versus-ONNX Runtime CPU numerical comparisons.
Only publishes the output path after every check succeeds.

Example from repository root:
  python deploy/interactive_b2z1/tests/export_verify_interactive_onnx.py \\
      --policy b2z1_interactive_sim2sim/b2z1_low_policy_jit.pt \\
      --output deploy/interactive_b2z1/build/b2z1_interactive_policy.onnx \\
      --fixture deploy/interactive_b2z1/build/interactive_obs_fixture
"""
import argparse
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile

import numpy as np

INPUT_SHAPE = (1, 799)
OUTPUT_SHAPE = (1, 18)


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def fixture_samples(path):
    text = subprocess.check_output([str(path.resolve())], text=True)
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    rows = np.asarray([np.fromstring(line, sep=',', dtype=np.float32)
                       for line in lines], dtype=np.float32)
    if rows.shape != (5, 800):
        raise ValueError('Expected 5 fixture rows of [frame_idx, 799 values], got {}'.format(rows.shape))
    np.testing.assert_array_equal(rows[:, 0], np.arange(5, dtype=np.float32))
    if not np.isfinite(rows).all():
        raise ValueError('C++ fixture produced NaN or Inf')
    return [row[1:].reshape(INPUT_SHAPE) for row in rows]


def input_cases(fixture):
    rng = np.random.RandomState(20261010)
    cases = [('zeros', np.zeros(INPUT_SHAPE, dtype=np.float32)),
             ('small_random', rng.normal(0, 0.05, INPUT_SHAPE).astype(np.float32)),
             ('larger_random', rng.normal(0, 0.8, INPUT_SHAPE).astype(np.float32))]
    if fixture is not None:
        cases.extend(('cpp_obs_frame_{}'.format(i), arr)
                     for i, arr in enumerate(fixture_samples(fixture)))
    return cases


def require_static_float32(tensor_value_info, name, shape):
    import onnx
    if tensor_value_info.name != name:
        raise ValueError('Expected ONNX tensor name {!r}, got {!r}'.format(name, tensor_value_info.name))
    tensor_type = tensor_value_info.type.tensor_type
    if tensor_type.elem_type != onnx.TensorProto.FLOAT:
        raise ValueError('{} must be float32'.format(name))
    got_shape = []
    for d in tensor_type.shape.dim:
        # dim_value=0 is not a valid static axis for our fixed-size contract.
        got_shape.append(d.dim_value if d.HasField('dim_value') else None)
    if tuple(got_shape) != tuple(shape):
        raise ValueError('{} expected static shape {}, got {}'.format(name, shape, got_shape))


def validate_onnx_structure(onnx_path):
    import onnx
    onnx.checker.check_model(str(onnx_path))
    model = onnx.load(str(onnx_path))
    if len(model.graph.input) != 1 or len(model.graph.output) != 1:
        raise ValueError('Expected exactly one ONNX input and one output')
    require_static_float32(model.graph.input[0], 'obs', INPUT_SHAPE)
    require_static_float32(model.graph.output[0], 'actions', OUTPUT_SHAPE)
    print('[ONNX] structure PASS: obs [1,799] float32 -> actions [1,18] float32')
    print('[ONNX] opsets:', [(x.domain or 'ai.onnx', x.version) for x in model.opset_import])


def verify_onnx_numerics(policy, onnx_path, fixture, atol, rtol):
    import onnxruntime as ort
    import torch
    sess = ort.InferenceSession(str(onnx_path), providers=['CPUExecutionProvider'])
    if len(sess.get_inputs()) != 1 or len(sess.get_outputs()) != 1:
        raise ValueError('ONNX Runtime I/O count mismatch')
    if (sess.get_inputs()[0].name != 'obs' or
            sess.get_outputs()[0].name != 'actions'):
        raise ValueError('ONNX Runtime input/output name mismatch')
    if sess.get_inputs()[0].type != 'tensor(float)' or sess.get_outputs()[0].type != 'tensor(float)':
        raise ValueError('ONNX Runtime input/output dtype mismatch')
    max_abs = 0.0
    max_rel = 0.0
    cases = input_cases(fixture)
    for name, obs in cases:
        with torch.no_grad():
            ref = policy(torch.from_numpy(obs.copy()))
        if isinstance(ref, (list, tuple)) or not torch.is_tensor(ref):
            raise ValueError('TorchScript must return one Tensor, got {}'.format(type(ref)))
        if tuple(ref.shape) != OUTPUT_SHAPE:
            raise ValueError('TorchScript returned {}, expected {}'.format(tuple(ref.shape), OUTPUT_SHAPE))
        ref = ref.detach().cpu().numpy().astype(np.float32, copy=False)
        pred = sess.run(['actions'], {'obs': obs})[0]
        if pred.shape != OUTPUT_SHAPE or pred.dtype != np.float32:
            raise ValueError('ONNX returned unexpected shape/dtype: {} {}'.format(pred.shape, pred.dtype))
        if not np.isfinite(ref).all() or not np.isfinite(pred).all():
            raise ValueError('Non-finite action for {}'.format(name))
        err = np.abs(ref.astype(np.float64) - pred.astype(np.float64))
        scale = np.maximum(np.abs(ref.astype(np.float64)), 1e-6)
        this_max_abs = float(err.max())
        this_max_rel = float((err / scale).max())
        max_abs = max(max_abs, this_max_abs)
        max_rel = max(max_rel, this_max_rel)
        print('[PARITY] {:20s} max_abs={:.9g} max_rel={:.9g}'.format(name, this_max_abs, this_max_rel))
        np.testing.assert_allclose(pred, ref, atol=atol, rtol=rtol,
                                   err_msg='ONNX and TorchScript differ on {}'.format(name))
    print('M1.2 ONNX PARITY: PASS ({} samples; max_abs={:.9g}, max_rel={:.9g})'.format(
        len(cases), max_abs, max_rel))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy', type=Path, required=True,
                        help='Existing b2z1_low_policy_jit.pt; never overwritten')
    parser.add_argument('--output', type=Path, required=True,
                        help='Write validated ONNX, default under ignored build/ dir')
    parser.add_argument('--fixture', type=Path, default=None,
                        help='Optional compiled C++ 5-frame observation fixture')
    parser.add_argument('--opset', type=int, default=13,
                        help='Default 13 supports older PyTorch exporters and current ONNX Runtime')
    parser.add_argument('--atol', type=float, default=1e-4)
    parser.add_argument('--rtol', type=float, default=1e-4)
    parser.add_argument('--overwrite', action='store_true',
                        help='Explicitly replace preexisting output AFTER all checks pass')
    parser.add_argument('--expected-sha256', type=str, default='',
                        help='Optional SHA256 guard for frozen TorchScript baseline')
    args = parser.parse_args()

    policy_path = args.policy.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    fixture_path = args.fixture.expanduser().resolve() if args.fixture else None
    if not policy_path.is_file():
        parser.error('TorchScript policy does not exist: {}'.format(policy_path))
    if fixture_path is not None and not fixture_path.is_file():
        parser.error('C++ fixture does not exist: {}'.format(fixture_path))
    if output_path == policy_path:
        parser.error('Output must not overwrite TorchScript source')
    if output_path.exists() and not args.overwrite:
        parser.error('Output already exists; use --overwrite only if deliberate: {}'.format(output_path))
    if args.opset < 11 or args.opset > 17:
        parser.error('Choose an opset between 11 and 17 (default 13)')
    if args.atol < 0 or args.rtol < 0:
        parser.error('Tolerance must be >= 0')

    print('[BASELINE] policy:', policy_path)
    digest = file_sha256(policy_path)
    print('[BASELINE] TorchScript SHA256:', digest)
    if args.expected_sha256 and digest.lower() != args.expected_sha256.lower():
        raise RuntimeError('Frozen TorchScript SHA256 mismatch; abort export')

    import torch
    # ONNX and ONNX Runtime are imported only here to give a clear error if
    # the legacy training environment does not contain them.
    import onnx  # noqa: F401
    import onnxruntime  # noqa: F401

    print('[VERSIONS] torch={} onnx={} onnxruntime={}'.format(
        torch.__version__, onnx.__version__, onnxruntime.__version__))
    torch.set_grad_enabled(False)
    policy = torch.jit.load(str(policy_path), map_location='cpu').eval()
    probe = torch.zeros(INPUT_SHAPE, dtype=torch.float32)
    y = policy(probe)
    if not torch.is_tensor(y) or tuple(y.shape) != OUTPUT_SHAPE or not torch.isfinite(y).all():
        raise RuntimeError('TorchScript must map float32 [1,799] to finite [1,18]')
    print('[JIT] valid [1,799] -> [1,18]')

    output_path.parent.mkdir(parents=True, exist_ok=True)
    # Export into a temporary path; only publish after validation succeeds.
    with tempfile.TemporaryDirectory(prefix='interactive_onnx_', dir=str(output_path.parent)) as work_dir:
        temporary_path = Path(work_dir) / 'candidate.onnx'
        print('[EXPORT] opset={} -> {}'.format(args.opset, temporary_path))
        torch.onnx.export(
            policy, probe, str(temporary_path),
            export_params=True,
            opset_version=args.opset,
            do_constant_folding=True,
            input_names=['obs'], output_names=['actions'],
            dynamic_axes=None,
            training=torch.onnx.TrainingMode.EVAL,
            dynamo=False,
        )
        validate_onnx_structure(temporary_path)
        verify_onnx_numerics(policy, temporary_path, fixture_path, args.atol, args.rtol)
        # tmp and output share filesystem; atomic file replacement on success.
        os.replace(str(temporary_path), str(output_path))

    print('[OUTPUT] verified ONNX:', output_path)
    print('[OUTPUT] SHA256:', file_sha256(output_path))
    print('[OUTPUT] bytes:', output_path.stat().st_size)
    print('NO hardware commands were sent.')


if __name__ == '__main__':
    main()
