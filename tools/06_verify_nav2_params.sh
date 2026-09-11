#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"; source install/setup.bash

for lp in dwb rpp mppi; do
  ros2 run nav2_config compose_nav2_params.py --robot wheeled_quadruped \
    --local-planner $lp --out /tmp/c_$lp.yaml >/dev/null 2>&1
done

python3 - <<'PY'
import yaml
for lp in ('dwb', 'rpp', 'mppi'):
    d = yaml.safe_load(open(f'/tmp/c_{lp}.yaml'))
    cs = d['controller_server']['ros__parameters']
    key = cs['controller_plugins'][0]
    sec = cs[key]
    speed = {k: v for k, v in sec.items()
             if any(s in k for s in ('vel', 'speed', 'acc', 'vx', 'wz', 'angular'))}
    lc = d['local_costmap']['local_costmap']['ros__parameters']
    leftover = [k for k in cs if 'vel' in k or 'speed' in k]
    print(f"  {lp:5s} 插件 = {key}")
    print(f"        速度参数 : {speed}")
    print(f"        width/height : {type(lc['width']).__name__}/{type(lc['height']).__name__} = {lc['width']}/{lc['height']}")
    print(f"        controller_server 残留速度参数 : {leftover or '无'}")
PY
