#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Make a NON-DESTRUCTIVE B2 inertia A/B test from Unitree's official MuJoCo MJCF.

Only B2's 13 articulated body inertial tags are replaced with verbatim official
MuJoCo inertial attributes.  Arm, mounting, contacts, actuators, time step,
and joint definitions remain exactly as in the existing Stage-1 model.

This is an inertia-sensitivity test, NOT a fully official B2+Z1 conversion.
Requires a local clone of https://github.com/unitreerobotics/unitree_ros.git
"""
import argparse
from pathlib import Path
import re
import xml.etree.ElementTree as ET

NAMES = ['base_link'] + [f'{leg}_{part}' for leg in ('FL','FR','RL','RR') for part in ('hip','thigh','calf')]


def get_model_info(path):
    tree = ET.parse(str(path))
    d = {}
    for name in NAMES:
        body = tree.find(f".//body[@name='{name}']")
        if body is None: raise ValueError(f'{path} missing body {name}')
        inertia = body.find('inertial')
        if inertia is None: raise ValueError(f'{path} missing inertia for {name}')
        attrs = dict(inertia.attrib)
        for key in ('mass','pos','quat','diaginertia'):
            if key not in attrs: raise ValueError(f'{path} {name} missing inertial attribute {key}')
        d[name] = {'attrs':attrs, 'body_pos':body.get('pos','0 0 0'), 'body_quat':body.get('quat','1 0 0 0')}
    return d


def replace_inertial(xmltext, name, attrs):
    # In these two source XML files, inertial tag is immediately after body start.
    pattern = re.compile(r'(<body\b[^>]*\bname="' + re.escape(name)
                         + r'"[^>]*>\s*)<inertial\b[^>]*/>', re.M)
    def once(match):
        # Equivalent to a verbatim copy of official numerical values.
        content = ' '.join(f'{k}="{v}"' for k, v in attrs.items())
        return match.group(1) + '<inertial ' + content + '/>'
    new, count = pattern.subn(once, xmltext)
    if count != 1: raise ValueError(f'Expected exactly one {name} inertia match, got {count}')
    return new


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--official',type=Path,required=True,help='.../unitree_ros/robots/b2_description_mujoco/xml/b2.xml')
    ap.add_argument('--xml-dir',type=Path,required=True,help='.../unitree_b2z1/xmls')
    ap.add_argument('--report-only',action='store_true')
    a=ap.parse_args()
    official=a.official.expanduser().resolve()
    directory=a.xml_dir.expanduser().resolve()
    source=directory/'b2z1_interactive_stage1.xml'
    srcscene=directory/'scene_b2z1_interactive_stage1.xml'
    robot_out=directory/'b2z1_stage1_official_b2_inertia.xml'
    scene_out=directory/'scene_b2z1_stage1_official_b2_inertia.xml'
    for f in (official,source,srcscene):
        if not f.is_file(): ap.error('Missing file: '+str(f))
    off=get_model_info(official)
    custom=get_model_info(source)
    print(f"{'name':17s}  {'custom_kg':>10s} {'unitree_kg':>11s} {'delta_kg':>10s}")
    for name in NAMES:
        x=float(custom[name]['attrs']['mass']); y=float(off[name]['attrs']['mass'])
        print(f'{name:17s} {x:10.4f} {y:11.4f} {y-x:+10.4f}')
        for key in ('body_pos','body_quat'):
            if [round(float(k),6) for k in custom[name][key].split()] != [round(float(k),6) for k in off[name][key].split()]:
                print(f'  WARNING: {name} {key} differs; this script DOES NOT change body geometry')
    print('MuJoCo source = official Unitree B2 MJCF; Z1 is kept from existing custom model.')
    if a.report_only:
        print('Report only. XML files unchanged.')
        return
    if robot_out.exists() or scene_out.exists():
        raise FileExistsError('Destination already exists; will NOT overwrite. Remove only after manual review.')
    body=source.read_text(encoding='utf-8')
    for name in NAMES:
        body=replace_inertial(body,name,off[name]['attrs'])
    ref='<include file="b2z1_interactive_stage1.xml"/>'
    scenetext=srcscene.read_text(encoding='utf-8')
    if scenetext.count(ref)!=1:
        raise ValueError('Expected precisely one include: '+ref)
    scene=scenetext.replace(ref,'<include file="b2z1_stage1_official_b2_inertia.xml"/>')
    ET.fromstring(body); ET.fromstring(scene)
    # Validate generated body inertia values match official exactly.
    robot_out.write_text(body,encoding='utf-8')
    scene_out.write_text(scene,encoding='utf-8')
    check=get_model_info(robot_out)
    assert all(check[n]['attrs']==off[n]['attrs'] for n in NAMES)
    print('Generated robot: ',robot_out)
    print('Generated scene: ',scene_out)
    print('Original files not modified. This is ONLY inertia A/B; do not deploy physically.')

if __name__=='__main__':main()
