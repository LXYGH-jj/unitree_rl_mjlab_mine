#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Non-destructively prepare an Interactive training arm-mount MuJoCo scene.

Copies existing MJCF in place next to meshes/includes, preserving original XML.
Only changes the Z1 fixed mount: [0.03, 0, 0.08] -> [0.20, 0, 0.10].
Does NOT silently rewrite mass, inertia, friction, actuators, joints or assets.
"""
import argparse
import re
from pathlib import Path


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--xml-dir', required=True, type=Path)
    opts=ap.parse_args()
    d=opts.xml_dir.expanduser().resolve()
    robot=d/'b2z1.xml'
    scene=d/'scene_b2z1.xml'
    assert robot.is_file() and scene.is_file(), f'找不到 MuJoCo 原始XML: {d}'
    t=robot.read_text(encoding='utf-8')
    # XML formatting independently handled by limited regex, matching one link00 body.
    pat=r'(<body\s+name=[\"\']link00[\"\']\s+pos=[\"\'])([^\"\']+)([\"\'])'
    matches=list(re.finditer(pat,t))
    assert len(matches)==1, f'link00安装点不唯一或不存在: {len(matches)}'
    old=matches[0].group(2)
    assert [round(float(x),6) for x in old.split()] in ([0.03,0,0.08],[0.2,0,0.1]), f'未知安装坐标 {old}，已终止'
    new_robot=d/'b2z1_interactive_stage1.xml'
    new_scene=d/'scene_b2z1_interactive_stage1.xml'
    assert not new_robot.exists() and not new_scene.exists(), '对照文件已存在：不会覆盖。请先检查或重命名现有对照文件。'
    result=t[:matches[0].start(2)]+'0.2 0 0.1'+t[matches[0].end(2):]
    scene_t=scene.read_text(encoding='utf-8')
    old_ref='<include file="b2z1.xml"/>'
    assert scene_t.count(old_ref)==1, f'场景XML中找不到唯一引用：{old_ref}'
    scene_result=scene_t.replace(old_ref,'<include file="b2z1_interactive_stage1.xml"/>')
    new_robot.write_text(result,encoding='utf-8')
    new_scene.write_text(scene_result,encoding='utf-8')
    print('模型原位置：',old)
    print('训练安装位置：0.2 0 0.1')
    print('新增机器人：',new_robot)
    print('新增场景：',new_scene)
    print('原始 b2z1.xml 和 scene_b2z1.xml 未改变')

if __name__=='__main__':
    main()
