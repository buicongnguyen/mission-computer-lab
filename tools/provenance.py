"""Capture runtime inputs when an experiment starts, not when it is published."""
import ast
import hashlib
import importlib.metadata
from pathlib import Path
import platform
import subprocess

ROOT=Path(__file__).resolve().parents[1]

def command(args):return subprocess.check_output(args,text=True).strip()

def runtime_python():
    """The repository's Python a flight executes: every integration module except tests, and every repository
    module those import, directly or indirectly. Tests, publishers and documentation tools can change after a
    run without making its evidence stale; anything a flight runs cannot."""
    local={p.stem:p for p in [*ROOT.glob('integration/*.py'),*ROOT.glob('tools/*.py')]}
    todo=[p for p in ROOT.glob('integration/*.py') if not p.name.startswith('test_')];seen=set()
    while todo:
        path=todo.pop()
        if path in seen:continue
        seen.add(path)
        for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
            if isinstance(node,ast.Import):names=[alias.name.split('.')[0] for alias in node.names]
            elif isinstance(node,ast.ImportFrom) and node.module and node.level==0:names=[node.module.split('.')[0]]
            else:continue
            todo+=[local[name] for name in names if name in local]
    return sorted(seen)

def capture(workspace):
    upstream={name:command(['git','-C',str(path),'rev-parse','HEAD']) for name,path in {
        'PX4 v1.16.0':workspace/'PX4-Autopilot',
        'px4_msgs release/1.16':workspace/'ros_ws/src/px4_msgs',
        'XRCE Agent v2.4.3':workspace/'Micro-XRCE-DDS-Agent'}.items()}
    inputs=runtime_python()
    for pattern in ('src/*','include/*','scripts/*.sh','ros2/mission_interfaces/msg/*','simulation/worlds/*'):
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
