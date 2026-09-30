# Source from any directory: source /path/to/workspace/tools/activate_ros.sh
zekeep_workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
zekeep_venv_dir="${ZKEEP_ROS_VENV:-$zekeep_workspace_dir/.venv-ros}"
if [[ ! -f "$zekeep_venv_dir/bin/activate" ]]; then
    echo "Create the ROS venv first; see docs/INSTALL_PORTABLE.md" >&2
    return 1
fi
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source "$zekeep_venv_dir/bin/activate"
if [[ -f "$zekeep_workspace_dir/install/setup.bash" ]]; then
    source "$zekeep_workspace_dir/install/setup.bash"
fi
zekeep_python_site="$(python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
# ROS ships an older Pinocchio. The SDK's Python module and native ABI must win.
export PYTHONPATH="$zekeep_python_site/cmeel.prefix/lib/python3.10/site-packages:$zekeep_python_site:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$zekeep_python_site/cmeel.prefix/lib:${LD_LIBRARY_PATH:-}"
