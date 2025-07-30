#!/usr/bin/env python3
# expand_world.py
with open('fsac_track.world', 'w') as f:
    f.write('<?xml version="1.0"?>\n<sdf version="1.6">\n  <world name="fsac_track">\n')
    f.write('    <include><uri>model://sun</uri></include>\n')
    f.write('    <include><uri>model://ground_plane</uri></include>\n')
    f.write('    <physics type="ode">\n')
    f.write('      <real_time_update_rate>1000</real_time_update_rate>\n')
    f.write('      <max_step_size>0.001</max_step_size>\n')
    f.write('    </physics>\n')

    # 0–75 m 红蓝各 16 根
    for x in range(0, 76, 5):
        f.write(f'    <include><name>red_{x}</name><uri>model://red_cone_1</uri><pose>{x} 1.5 0 0 0 0</pose></include>\n')
        f.write(f'    <include><name>blue_{x}</name><uri>model://blue_cone_1</uri><pose>{x}  -1.5 0 0 0 0</pose></include>\n')

    # 75–175 m 黄色左右各 21 根
    for x in range(80, 176, 5):
        f.write(f'    <include><name>yellow_R{x}</name><uri>model://yellow_cone_1</uri><pose>{x} -1.5 0 0 0 0</pose></include>\n')
        f.write(f'    <include><name>yellow_L{x}</name><uri>model://yellow_cone_1</uri><pose>{x}  1.5 0 0 0 0</pose></include>\n')

    f.write('  </world>\n</sdf>\n')