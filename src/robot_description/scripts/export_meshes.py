"""Tessellate each physical STEP occurrence in its rigid-link frame, in metres.

Uses the native occurrence manifest to exclude reference geometry. Does not
flatten the robot in HOME. No Inventor source document is edited.
"""
import csv
import json
from pathlib import Path
import numpy as np
import trimesh
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.BRepBuilderAPI import BRepBuilderAPI_NurbsConvert
from inspect_step import read, children, name, matrix
from build_description import frames, PKG, SRC

def triangulate(shape):
    converted=0
    mesher=BRepMesh_IncrementalMesh(shape,0.05,False,0.20,True)
    mesher.Perform()
    if not mesher.IsDone():
        raise RuntimeError('CAD mesher failed')
    vertices,faces=[],[]
    explorer=TopExp_Explorer(shape,TopAbs_FACE)
    while explorer.More():
        cast_face=getattr(TopoDS,'Face_s',None) or TopoDS.Face
        face=cast_face(explorer.Current())
        loc=TopLoc_Location()
        tri=BRep_Tool.Triangulation_s(face,loc)
        if tri is None:
            # Exact rational representation of the affected face only. Converting
            # the entire imported motor also converts unrelated tiny spline edges.
            face=cast_face(BRepBuilderAPI_NurbsConvert(face,True).Shape())
            face_mesher=BRepMesh_IncrementalMesh(face,0.05,False,0.20,False)
            loc=TopLoc_Location()
            tri=BRep_Tool.Triangulation_s(face,loc)
            if tri is None:
                raise RuntimeError('A CAD face could not be triangulated after NURBS conversion')
            converted+=1
        base=len(vertices)
        for n in range(1,tri.NbNodes()+1):
            p=tri.Node(n).Transformed(loc.Transformation())
            vertices.append([p.X(),p.Y(),p.Z()])
        for n in range(1,tri.NbTriangles()+1):
            ids=list(tri.Triangle(n).Get())
            if face.Orientation()==TopAbs_REVERSED:
                ids[1],ids[2]=ids[2],ids[1]
            faces.append([base+i-1 for i in ids])
        explorer.Next()
    return np.array(vertices,dtype=float),np.array(faces,dtype=int),converted

def main():
    doc,st,roots=read()
    assert roots.Length()==1
    links=children(st,roots.Value(1))
    assert len(links)==7
    rows=list(csv.DictReader((SRC/'00_Documentacion/INVENTARIO_OCURRENCIAS_ACTIVAS.csv').open(encoding='utf-8-sig')))
    manifest={row['ruta_ocurrencia']:row for row in rows}
    aa=frames()
    report=[]
    (PKG/'meshes').mkdir(exist_ok=True)
    for i,link in enumerate(links):
        assert name(link).startswith(f'L{i}_')
        parts=children(st,link)
        actual={f'ROBOT_6DOF/{name(link)}/{name(p)}' for p in parts}
        expected={p for p in manifest if p.startswith(f'ROBOT_6DOF/{name(link)}/')}
        assert actual==expected,(actual-expected,expected-actual)
        meshes=[]
        detail=[]
        for part in parts:
            path=f'ROBOT_6DOF/{name(link)}/{name(part)}'
            row=manifest[path]
            if row['es_referencia']=='True':
                if name(part)=='REFERENCIAS':
                    assert np.max(np.abs(np.array(matrix(st.GetLocation_s(part)))-np.eye(4)))<1e-10
                detail.append({'name':name(part),'excluded':'reference_BOM'})
                continue
            shape=st.GetShape_s(part)
            if shape.IsNull():
                raise RuntimeError('Missing physical shape: '+path)
            try:
                verts,faces,converted=triangulate(shape)
            except Exception as exc:
                raise RuntimeError(path) from exc
            if len(faces)==0:
                raise RuntimeError('Empty physical shape: '+path)
            # Local CAD mm -> reoriented ROS link metres, without root placement.
            verts=verts@aa[i]*.001
            mesh=trimesh.Trimesh(verts,faces,process=False)
            props=GProp_GProps()
            BRepGProp.VolumeProperties_s(shape,props)
            cad_volume=props.Mass()*1e-9
            error=abs(mesh.volume-cad_volume)/max(abs(cad_volume),1e-18)
            if mesh.volume<=0 or error>0.035:
                raise RuntimeError(f'Invalid tessellation {path}: volume error {error}')
            meshes.append(mesh)
            detail.append({'name':name(part),'triangles':len(faces),'nurbs_faces':converted,'cad_volume_m3':cad_volume,
                           'mesh_volume_m3':float(mesh.volume),'relative_volume_error':float(error)})
        combined=trimesh.util.concatenate(meshes)
        lname='base_body' if i==0 else f'link_{i}'
        combined.export(PKG/'meshes'/f'{lname}.stl')
        # Volume is checked per occurrence so errors cannot cancel between parts.
        item={'cad_link':name(link),'ros_link':lname,'step_pose_mm':matrix(st.GetLocation_s(link)),
              'physical_occurrences':len(meshes),'excluded_references':len(parts)-len(meshes),
              'triangles':len(combined.faces),'bounds_m':combined.bounds.tolist(),'parts':detail}
        report.append(item)
        print(lname,'parts',len(meshes),'triangles',len(combined.faces),flush=True)
    (PKG/'docs/mesh_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')

if __name__=='__main__':
    main()
