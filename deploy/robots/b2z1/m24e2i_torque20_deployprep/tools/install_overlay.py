#!/usr/bin/env python3
"""Apply ONLY 50 -> 20 Nm gravity/extra FF guard in installed M2.4e2h header."""
import argparse,hashlib,shutil,subprocess
from pathlib import Path
BEFORE='6b88856d7748d5d8377d4735128ef006d963ff70ee7a101410b02fb3e8a4da01'
AFTER='26f3324bd6056cd57e76ce709df0aae3c74b0653cf311cbf1f7dd20129f7c872'
OLD=b'kModelTorqueSanityBoundNm=50.0;'
NEW=b'kModelTorqueSanityBoundNm=20.0;'
REL=Path('deploy/robots/b2z1/m24e2d_sdk_worker_bridge/include/m24e2h_gravity_guard.h')
DEST=Path('deploy/robots/b2z1/m24e2i_torque20_deployprep')

def main():
    p=argparse.ArgumentParser()
    p.add_argument('repo',type=Path)
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument('--check',action='store_true')
    group.add_argument('--apply',action='store_true')
    a=p.parse_args()
    repo=a.repo.resolve(strict=True)
    if not (repo/'.git').exists():raise SystemExit('Not a git worktree')
    target=repo/REL
    if not target.is_file():raise SystemExit('M2.4e2h header missing: '+str(target))
    current=target.read_bytes()
    digest=hashlib.sha256(current).hexdigest()
    if digest not in (BEFORE,AFTER):
        raise SystemExit('Unexpected/modified header sha256='+digest+'; REFUSE overwrite')
    if digest==AFTER:
        print('M2.4e2i ALREADY APPLIED: 20.0 Nm FF guard; no change')
        return
    if current.count(OLD)!=1:raise SystemExit('Expected unique 50.0 declaration missing')
    result=current.replace(OLD,NEW)
    if hashlib.sha256(result).hexdigest()!=AFTER:raise SystemExit('Patch verification failure')
    print('TARGET:',target)
    print('CHANGE: M2.4e2h model feedforward |tau_ff| >50.0 -> >20.0 Nm => FAULT; not clipping')
    print('NO change to motor total torque limit, Z1 SDK, gains, B2 FSM or main YAML')
    if a.check:
        print('DRY RUN; no changes');return
    src=Path(__file__).resolve().parents[1]
    dst=repo/DEST
    if dst.exists():raise SystemExit('E2i destination already exists unexpectedly; refusing overwrite')
    # Install entire testing/transfer kit first, then write guard atomically.
    shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__'))
    (dst/'backups').mkdir()
    shutil.copy2(target,dst/'backups'/'m24e2h_gravity_guard.h.before20')
    tmp=target.with_suffix('.h.e2i_temp')
    tmp.write_bytes(result)
    tmp.replace(target)
    print('M2.4e2i INSTALLED; backup under',dst/'backups')
    print('Verified patched sha256:',hashlib.sha256(target.read_bytes()).hexdigest())
if __name__=='__main__':main()
