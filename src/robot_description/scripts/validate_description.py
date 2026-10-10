"""Validate URDF against independently exported STEP poses and CAD properties."""
import csv
import hashlib
import json
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import xacro
from urdf_parser_py.urdf import URDF

PKG=Path(__file__).resolve().parents[1]

def transform(origin):
    result=np.eye(4)
    if origin is not None:
        result[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        result[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return result

def forward(root,angles):
    poses={'base_link':np.eye(4)}
    remaining=list(root.findall('joint'))
    while remaining:
        progress=False
        for joint in remaining[:]:
            parent=joint.find('parent').get('link')
            child=joint.find('child').get('link')
            if parent not in poses:
                continue
            assert child not in poses,'Cycle or duplicate parent'
            rotation=np.eye(4)
            if joint.get('type')=='revolute':
                axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
                rotation[:3,:3]=Rotation.from_rotvec(axis*angles[joint.get('name')]).as_matrix()
            poses[child]=poses[parent]@transform(joint.find('origin'))@rotation
            remaining.remove(joint)
            progress=True
        assert progress,'Disconnected tree'
    return poses

def main():
    root=ET.parse(PKG/'urdf/robot_6dof.urdf').getroot()
    regenerated=ET.fromstring(xacro.process_file(str(PKG/'urdf/robot_6dof.urdf.xacro')).toxml())
    def canonical(node):
        return (node.tag,sorted(node.attrib.items()),[canonical(c) for c in node])
    assert canonical(root)==canonical(regenerated),'Xacro and URDF differ'
    parser=URDF.from_xml_file(str(PKG/'urdf/robot_6dof.urdf'))
    assert parser.get_root()=='base_link'
    assert len(parser.links)==9 and len(parser.joints)==8
    joints=[j for j in root.findall('joint') if j.get('type')=='revolute']
    assert len(joints)==6
    for j in joints:
        assert j.find('axis').get('xyz')=='0 0 1'
        assert float(j.find('limit').get('effort'))==0
        assert float(j.find('limit').get('velocity'))==0
    data=json.loads((PKG/'docs/nominal_frames.json').read_text())
    angles={f'joint_{i+1}':q for i,q in enumerate(data['home_rad'])}
    home=forward(root,angles)
    aa=[np.array(a) for a in data['ros_axes_in_cad']]
    step=json.loads((PKG/'docs/step_structure.json').read_text())
    matrix_errors=[]
    for i,row in enumerate(step):
        expected=np.array(row['matrix'])
        expected[:3,3]*=.001
        expected[:3,:3]=expected[:3,:3]@aa[i]
        lname='base_body' if i==0 else f'link_{i}'
        matrix_errors.append(float(np.max(np.abs(home[lname]-expected))))
    assert max(matrix_errors)<1e-10, matrix_errors
    geometry=list(csv.DictReader((PKG/'docs/sources/GEOMETRIA_FINAL_JOINTS.csv').open(encoding='utf-8-sig')))
    joint_errors=[]
    for i,row in enumerate(geometry):
        pose=home[f'link_{i+1}']
        pos=np.array([float(row['origin_'+c+'_mm']) for c in 'xyz'])*.001
        axis=np.array([float(row['axis_'+c]) for c in 'xyz'])
        error=max(np.max(abs(pos-pose[:3,3])),np.max(abs(axis-pose[:3,2])))
        joint_errors.append(float(error))
    assert max(joint_errors)<1e-10,joint_errors
    audit=list(csv.DictReader((PKG/'docs/sources/INERCIAS_API_AUDITORIA_FINAL.csv').open(encoding='utf-8-sig')))
    mass_rows=list(csv.DictReader((PKG/'docs/sources/PROPIEDADES_INERCIALES.csv').open(encoding='utf-8-sig')))
    total=0.0
    for i,lname in enumerate(['base_body']+[f'link_{k}' for k in range(1,7)]):
        inertia=root.find(f"link[@name='{lname}']/inertial")
        mass=float(inertia.find('mass').get('value'))
        total+=mass
        assert mass==float(mass_rows[i]['mass_kg'])
        vals=inertia.find('inertia').attrib
        tensor=np.array([[float(vals['i'+a+b if a<=b else 'i'+b+a]) for b in 'xyz'] for a in 'xyz'])
        eig=np.linalg.eigvalsh(tensor)
        expected=sorted(float(audit[i][f'principal_{k}_kgm2']) for k in (1,2,3))
        assert min(eig)>0 and max(eig)<=sum(eig)-max(eig)+1e-10
        assert np.allclose(eig,expected,rtol=1e-5,atol=1e-12)
        com=np.fromstring(inertia.find('origin').get('xyz'),sep=' ')
        original=np.array([float(mass_rows[i]['com_'+c+'_m']) for c in 'xyz'])
        assert np.allclose(aa[i]@com,original,atol=1e-12)
    mesh_report=[]
    for mesh in root.findall('.//mesh'):
        uri=mesh.get('filename')
        assert uri.startswith('package://robot_description/')
        path=PKG/uri.removeprefix('package://robot_description/')
        assert path.is_file()
    for path in sorted((PKG/'meshes').glob('*.stl')):
        mesh=trimesh.load_mesh(path,process=True)
        assert len(mesh.faces)>0 and np.isfinite(mesh.vertices).all()
        # A rigid link is a collection of touching CAD solids, not a boolean
        # union. Record open/nonmanifold edges rather than claiming a closed
        # simulation solid. Consistent winding and finite nonempty data required.
        assert mesh.is_winding_consistent,path.name
        counts=np.bincount(mesh.edges_unique_inverse)
        mesh_report.append({'file':path.name,'triangles':len(mesh.faces),'watertight':bool(mesh.is_watertight),
                            'open_edges':int(np.sum(counts==1)),
                            'nonmanifold_edges':int(np.sum(counts>2)),
                            'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    assert len(mesh_report)==7
    zero=forward(root,{f'joint_{i}':0 for i in range(1,7)})
    assert np.allclose(zero['tool0'][:3,3],[.448,0,.273],atol=1e-12)
    # Check arbitrary poses against the original CAD-axis chain, not the URDF origins.
    offsets=np.array(data['nominal_offsets_m'])
    axes=np.array([[0,0,1],[0,1,0],[0,1,0],[1,0,0],[0,1,0],[1,0,0]])
    rng=np.random.default_rng(6)
    random_error=0.0
    for _ in range(100):
        qs=[rng.uniform(float(j.find('limit').get('lower')),float(j.find('limit').get('upper'))) for j in joints]
        observed=forward(root,{f'joint_{i+1}':q for i,q in enumerate(qs)})
        nominal=np.eye(4)
        for i,q in enumerate(qs):
            t=np.eye(4)
            t[:3,3]=offsets[i]
            t[:3,:3]=Rotation.from_rotvec(axes[i]*q).as_matrix()
            nominal=nominal@t
            expected=nominal.copy()
            expected[:3,:3]=nominal[:3,:3]@aa[i+1]
            random_error=max(random_error,float(np.max(abs(observed[f'link_{i+1}']-expected))))
    assert random_error<1e-12
    report={'result':'PASS_VISUAL_KINEMATIC_WITH_MESH_LIMITATIONS','xacro_matches_urdf':True,'links':9,'physical_links':7,'revolute_joints':6,
            'fixed_joints':2,'axis_all_local_z':True,'mass_kg':total,
            'max_home_step_matrix_error':max(matrix_errors),'max_native_joint_geometry_error':max(joint_errors),
            'random_fk_cases':100,'max_random_fk_error':random_error,
            'inertias_match_native_principal_moments':True,'meshes':mesh_report,
            'mesh_limitations':'Multipart CAD surfaces are not watertight manifold solids. Motor surfaces in base_body/link_1/link_2 contain open tessellation edges. Not certified for dynamics or collision safety.',
            'ros2_rviz_runtime_test':'NOT RUN: ROS 2 is not installed in this workspace',
            'hardware_or_dynamic_validation':'NOT PERFORMED',
            'limits':'CAD positional limits provisional; effort=velocity=0 by user request'}
    (PKG/'docs/validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))

if __name__=='__main__':
    main()
