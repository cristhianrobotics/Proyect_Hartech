#!/usr/bin/env python3
"""Regenerate only the URDF after editing Xacro/YAML; no CAD libraries needed."""
from pathlib import Path
import xacro

package = Path(__file__).resolve().parents[1]
if not (package / 'urdf/robot_6dof.urdf.xacro').is_file():
    # A regular CMake install places this executable in lib/robot_description.
    from ament_index_python.packages import get_package_share_directory
    package = Path(get_package_share_directory('robot_description'))
source = package / 'urdf/robot_6dof.urdf.xacro'
target = package / 'urdf/robot_6dof.urdf'
target.write_text(xacro.process_file(str(source)).toprettyxml(indent='  '), encoding='utf-8')
print(target)
