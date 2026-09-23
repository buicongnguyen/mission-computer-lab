"""Capture runtime inputs when an experiment starts, not when it is published."""
import hashlib
import importlib.metadata
from pathlib import Path
import platform
import subprocess

ROOT=Path(__file__).resolve().parents[1]

def command(args):return subprocess.check_output(args,text=True).strip()

def capture(workspace):
    upstream={name:command(['git','-C',str(path),'rev-parse','HEAD']) for name,path in {
        'PX4 v1.16.0':workspace/'PX4-Autopilot',
        'px4_msgs release/1.16':workspace/'ros_ws/src/px4_msgs',
        'XRCE Agent v2.4.3':workspace/'Micro-XRCE-DDS-Agent'}.items()}
    inputs=[]
    for pattern in ('integration/*.py','tools/*.py','src/*','include/*','scripts/*.sh',
                    'ros2/mission_interfaces/msg/*','simulation/worlds/*'):
        inputs.extend(ROOT.glob(pattern))
    inputs += [ROOT/'build/mission_supervisor',ROOT/'CMakeLists.txt',ROOT/'requirements-sitl.lock.txt']
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(inputs) if p.is_file()}
    binaries={str(p.relative_to(workspace)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        [workspace/'PX4-Autopilot/build/px4_sitl_default/bin/px4',workspace/'agent-install/bin/MicroXRCEAgent']}
    return {'platform':platform.platform(),'python':platform.python_version(),
        'packages':{p:importlib.metadata.version(p) for p in ('numpy','onnx','onnxruntime','pymavlink','cryptography')},
        'apt':command(['dpkg-query','-W','-f=${Package} ${Version}\n','ros-humble-ros-base','gz-harmonic']),
        'source_commit':command(['git','-C',str(ROOT),'rev-parse','HEAD']),
        'upstream_revisions':upstream,'input_sha256':hashes,'upstream_binary_sha256':binaries}
