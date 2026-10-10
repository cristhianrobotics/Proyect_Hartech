"""Generate the editable Xacro and expand it with the actual xacro processor.

Run from a checkout containing the supplied GPT6 source folder. CAD dimensions,
limits and HOME are taken from its design scripts and saved reports, never from
the posed global mesh bounds. All ROS units are SI.
"""
import csv
import json
from pathlib import Path
import re
import shutil
import warnings
import numpy as np
from scipy.spatial.transform import Rotation
import xacro
import yaml

PKG=Path(__file__).resolve().parents[1]
SRC=PKG.parent/'GPT6'

def fmt(values):
    return ' '.join(f'{0.0 if abs(float(v))<1e-14 else float(v):.16g}' for v in np.atleast_1d(values))

def rpy(matrix):
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', UserWarning)
        return Rotation.from_matrix(matrix).as_euler('xyz')

def frames():
    # Columns are the ROS link axes expressed in the nominal CAD link frame.
    return [np.eye(3), np.eye(3), Rotation.from_euler('x',-np.pi/2).as_matrix(),
            Rotation.from_euler('x',-np.pi/2).as_matrix(), Rotation.from_euler('y',np.pi/2).as_matrix(),
            Rotation.from_euler('x',-np.pi/2).as_matrix(), Rotation.from_euler('y',np.pi/2).as_matrix()]

def source_parameters():
    source=(SRC/'08_Scripts/RobotBuild.cs').read_text(encoding='utf-8-sig')
    def array(key):
        return [float(v) for v in re.search(r'\b'+key+r'=\{([^}]+)\}',source)[1].split(',')]
    offsets=np.array(list(zip(array('offsetsX'),np.zeros(6),array('offsetsZ'))))*.001
    minimum,maximum=array('mins'),array('maxs')
    poses=list(csv.DictReader((SRC/'00_Documentacion/POSES_VERIFICADAS.csv').open(encoding='utf-8-sig')))
    home=next(row for row in poses if row['pose']=='HOME')
    home=[np.deg2rad(float(home[f'q{i}'])) for i in range(1,7)]
    return offsets, minimum, maximum, home

def main():
    for folder in ['urdf','config','docs/sources','meshes','launch','rviz']:
        (PKG/folder).mkdir(parents=True,exist_ok=True)
    for path in ['01_Calculos/PROPIEDADES_INERCIALES.csv','01_Calculos/INERCIAS_API_AUDITORIA_FINAL.csv',
                 '01_Calculos/GEOMETRIA_FINAL_JOINTS.csv','00_Documentacion/POSES_VERIFICADAS.csv',
                 '00_Documentacion/LIMITES_ARTICULARES.md','00_Documentacion/DECISIONES_DISENO.md',
                 '00_Documentacion/CORRECCION_PROPIEDADES_FISICAS_V2.md',
                 '00_Documentacion/INVENTARIO_OCURRENCIAS_ACTIVAS.csv',
                 '00_Documentacion/INVENTARIO_ARCHIVOS_ACTIVOS.csv',
                 '08_Scripts/RobotBuild.cs','08_Scripts/RobotPhysicalExportV2.cs']:
        shutil.copyfile(SRC/path,PKG/'docs/sources'/Path(path).name)
    masses=list(csv.DictReader((SRC/'01_Calculos/PROPIEDADES_INERCIALES.csv').open(encoding='utf-8-sig')))
    assert [r['link'] for r in masses]==[f'L{i}' for i in range(7)]
    assert len({r['export_id'] for r in masses})==1
    aa=frames()
    offsets,minimum,maximum,home=source_parameters()
    data={'status':'visualization_and_kinematics_only', 'units':'m, rad, kg, kg*m^2, N*m, rad/s',
          'effort_velocity_status':'Zero requested by user; physical values not specified',
          'angular_limits_status':'Provisional CAD limits; see docs/sources/LIMITES_ARTICULARES.md',
          'inertia_export_id':masses[0]['export_id'], 'links':{},'joints':{}}
    calls=[]
    names=['base_body']+[f'link_{i}' for i in range(1,7)]
    colors=['0.32 0.36 0.40 1','0.18 0.46 0.59 1','0.21 0.53 0.65 1','0.22 0.59 0.68 1',
            '0.85 0.57 0.22 1','0.88 0.65 0.30 1','0.60 0.65 0.70 1']
    for i,row in enumerate(masses):
        assert row['reference_frame']==f'LOCAL_NOMINAL_L{i}'
        assert row['inertia_convention']=='negative_integral_tensor'
        v=lambda key:float(row[key+'_kgm2'])
        tensor=np.array([[v('Ixx'),v('Ixy'),v('Ixz')],[v('Ixy'),v('Iyy'),v('Iyz')],[v('Ixz'),v('Iyz'),v('Izz')]])
        transformed=aa[i].T@tensor@aa[i]
        com=aa[i].T@np.array([float(row['com_'+c+'_m']) for c in 'xyz'])
        data['links'][names[i]]={'mass':float(row['mass_kg']),'com':fmt(com),'color':colors[i],
            'inertia':{key:float(transformed[a,b]) for key,a,b in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]}}
        calls.append(f'  <xacro:cad_link name="{names[i]}" data="${{cfg[\'links\'][\'{names[i]}\']}}"/>')
    for i in range(6):
        name=f'joint_{i+1}'
        data['joints'][name]={'parent':names[i],'child':names[i+1],
            'xyz':fmt(aa[i].T@offsets[i]),'rpy':fmt(rpy(aa[i].T@aa[i+1])),
            'lower':float(np.deg2rad(minimum[i])),'upper':float(np.deg2rad(maximum[i])),
            'effort':0.0,'velocity':0.0,'home':float(home[i])}
        calls.append(f'  <xacro:cad_joint name="{name}" data="${{cfg[\'joints\'][\'{name}\']}}"/>')
    (PKG/'config/robot_parameters.yaml').write_text('# SI units. CAD angular limits are provisional.\n'+yaml.safe_dump(data,sort_keys=False),encoding='utf-8')
    # Independent source-referenced metadata used to verify mesh alignment.
    (PKG/'docs/nominal_frames.json').write_text(json.dumps({'nominal_offsets_m':offsets.tolist(),
        'ros_axes_in_cad': [a.tolist() for a in aa], 'home_rad':home},indent=2),encoding='utf-8')
    main_xml='''<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_6dof">
  <!-- Source: supplied Inventor export, verified manifest. All joint axes are local +Z.
       q=0 is the horizontal nominal CAD pose, NOT the saved HOME pose.
       Effort and velocity are zero by user request; not a hardware control configuration. -->
  <xacro:arg name="parameters_file" default="../config/robot_parameters.yaml"/>
  <xacro:arg name="mesh_prefix" default="package://robot_description/meshes"/>
  <xacro:property name="cfg" value="${xacro.load_yaml('$(arg parameters_file)')}"/>
  <xacro:include filename="cad_macros.xacro"/>
  <!-- A massless root avoids the KDL root-inertia limitation. -->
  <link name="base_link"/>
  <joint name="base_mount" type="fixed">
    <parent link="base_link"/><child link="base_body"/><origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
'''+ '\n'.join(calls)+'''
  <!-- Flange mounting face: CAD nominal X=+9 mm relative to J6.
       This is a flange frame, not a calibrated tool TCP. -->
  <link name="tool0"/>
  <joint name="flange_to_tool0" type="fixed">
    <parent link="link_6"/><child link="tool0"/>
    <origin xyz="0 0 0.009" rpy="0 0 0"/>
  </joint>
</robot>
'''
    (PKG/'urdf/robot_6dof.urdf.xacro').write_text(main_xml,encoding='utf-8')
    expanded=xacro.process_file(str(PKG/'urdf/robot_6dof.urdf.xacro'))
    (PKG/'urdf/robot_6dof.urdf').write_text(expanded.toprettyxml(indent='  '),encoding='utf-8')
    print('Xacro expanded. Mass:',sum(float(r['mass_kg']) for r in masses),'kg')

if __name__=='__main__':
    main()
