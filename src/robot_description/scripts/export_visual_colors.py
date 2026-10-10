"""Add STEP colors to the existing STL triangles without changing any geometry."""
import hashlib
import json
from pathlib import Path
import numpy as np
import collada
from collada.common import tag
from OCP.XCAFPrs import XCAFPrs
from OCP.collections import IndexedDataMap_TopoDS_Shape_XCAFPrs_Style_TopTools_ShapeMapHasher
from OCP.TopLoc import TopLoc_Location
from OCP.TDF import TDF_Label
from OCP.TopoDS import TopoDS,TopoDS_Iterator
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.Quantity import Quantity_TOC_sRGB
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from inspect_step import read,children,name,PKG


class ExactFloatSource(collada.source.FloatSource):
    def save(self):
        super().save()
        self.xmlnode.find(tag('float_array')).text=' '.join('%.9g'%v for v in self.data.ravel())


def face_colors(label,shape):
    styles=IndexedDataMap_TopoDS_Shape_XCAFPrs_Style_TopTools_ShapeMapHasher()
    XCAFPrs.CollectStyleSettings_s(label,TopLoc_Location(),styles)
    result={}
    def walk(current,inherited=None):
        c=inherited
        if styles.Contains(current):
            style=styles.FindFromKey(current)
            if style.IsSetColorSurf():
                rgba=style.GetColorSurfRGBA()
                c=tuple(round(float(v),9) for v in rgba.GetRGB().Values(Quantity_TOC_sRGB))+(float(rgba.Alpha()),)
        if current.ShapeType()==TopAbs_FACE:
            if c is None:
                raise RuntimeError('STEP face has no source color: '+name(label))
            result[hash(current)]=c
            return
        it=TopoDS_Iterator(current)
        while it.More():
            walk(it.Value(),c)
            it.Next()
    walk(shape)
    ordered=[]
    exp=TopExp_Explorer(shape,TopAbs_FACE)
    while exp.More():
        ordered.append(result[hash(exp.Current())])
        exp.Next()
    return ordered


def read_stl(path):
    dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
    data=np.fromfile(path,dtype=dtype,offset=84)
    count=int.from_bytes(path.read_bytes()[80:84],'little')
    assert count==len(data)
    return data


def write_dae(path,triangles,normals,ids,palette):
    doc=collada.Collada()
    doc.assetInfo.unitmeter=1.0
    doc.assetInfo.unitname='meter'
    doc.assetInfo.upaxis=collada.asset.UP_AXIS.Z_UP
    vertices,vi=np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
    normal_values,ni=np.unique(normals,axis=0,return_inverse=True)
    sources=[ExactFloatSource('positions',vertices,('X','Y','Z')),
             ExactFloatSource('normals',normal_values,('X','Y','Z'))]
    geom=collada.geometry.Geometry(doc,'geometry',path.stem,sources)
    inputs=collada.source.InputList()
    inputs.addInput(0,'VERTEX','#positions')
    inputs.addInput(1,'NORMAL','#normals')
    vertex_indices=vi.reshape(-1,3)
    materials=[]
    used=sorted(set(ids.tolist()))
    for cid in used:
        color=palette[cid]
        effect=collada.material.Effect(f'effect_{cid}',[],'phong',diffuse=color,
                                      ambient=(*color[:3],1.0),specular=(0.15,0.15,0.15,1),shininess=20.0)
        mat=collada.material.Material(f'material_{cid}',f'CAD_color_{cid}',effect)
        doc.effects.append(effect);doc.materials.append(mat)
        selected=ids==cid
        indices=np.column_stack((vertex_indices[selected].ravel(),np.repeat(ni[selected],3))).ravel()
        primitive=geom.createTriangleSet(indices,inputs,f'color_{cid}')
        geom.primitives.append(primitive)
        materials.append(collada.scene.MaterialNode(f'color_{cid}',mat,inputs=[]))
    doc.geometries.append(geom)
    node=collada.scene.Node('link',children=[collada.scene.GeometryNode(geom,materials)])
    scene=collada.scene.Scene('scene',[node]);doc.scenes.append(scene);doc.scene=scene
    doc.write(str(path))
    # Read the actual file back, checking every triangle at float32 precision.
    saved=collada.Collada(str(path))
    assert saved.assetInfo.unitmeter==1 and saved.assetInfo.upaxis=='Z_UP'
    for cid,primitive in zip(used,saved.geometries[0].primitives):
        actual=primitive.vertex[primitive.vertex_index]
        assert np.array_equal(actual.astype(np.float32),triangles[ids==cid]),path.name
        assert np.allclose(saved.materials[f'material_{cid}'].effect.diffuse,palette[cid],atol=1e-9)


def main():
    doc,st,roots=read()
    audit=json.loads((PKG/'docs/mesh_audit.json').read_text())
    original=json.loads((PKG/'docs/validation.json').read_text())
    original_hashes={r['file']:r['sha256'] for r in original['meshes']}
    palette=[];palette_lookup={};cache={};report=[]
    def index(c):
        if c not in palette_lookup:
            palette_lookup[c]=len(palette);palette.append(c)
        return palette_lookup[c]
    for link,lrow in zip(children(st,roots.Value(1)),audit):
        assert name(link)==lrow['cad_link']
        path=PKG/'meshes'/(lrow['ros_link']+'.stl')
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        assert digest==original_hashes[path.name]
        source=read_stl(path)
        ids=np.empty(len(source),dtype=np.int32)
        offset=0;part_report=[]
        for part,prow in zip(children(st,link),lrow['parts']):
            assert name(part)==prow['name']
            if 'excluded' in prow:continue
            ref=TDF_Label();assert st.GetReferredShape_s(part,ref)
            definition=st.GetShape_s(ref)
            key=hash(definition)
            if key not in cache:
                colors=face_colors(ref,definition)
                if len(set(colors))==1:
                    ranges=[(prow['triangles'],colors[0])]
                else:
                    placed=st.GetShape_s(part)
                    mesher=BRepMesh_IncrementalMesh(placed,.05,False,.20,True)
                    mesher.Perform()
                    exp=TopExp_Explorer(placed,TopAbs_FACE);ranges=[];n=0
                    while exp.More():
                        face=TopoDS.Face(exp.Current())
                        tri=BRep_Tool.Triangulation_s(face,TopLoc_Location())
                        assert tri is not None,'Multicolor face has no triangulation'
                        ranges.append((tri.NbTriangles(),colors[n]))
                        n+=1;exp.Next()
                    assert n==len(colors)
                cache[key]=ranges
            ranges=cache[key]
            assert sum(n for n,c in ranges)==prow['triangles'],name(part)
            for n,c in ranges:
                ids[offset:offset+n]=index(c);offset+=n
            part_report.append({'occurrence':name(part),'triangles':prow['triangles'],
                                'colors_srgb':sorted(set(c for n,c in ranges))})
        assert offset==len(source)
        target=path.with_suffix('.dae')
        write_dae(target,source['vertices'],source['normal'],ids,palette)
        # Used by the preview to read the same material assignment as the DAE.
        np.savez_compressed(PKG/'docs'/(lrow['ros_link']+'_colors.npz'),ids=ids,palette=np.array(palette))
        report.append({'link':lrow['ros_link'],'stl_sha256_unchanged':digest,'triangles':len(source),
                       'dae_sha256':hashlib.sha256(target.read_bytes()).hexdigest(),
                       'triangle_geometry_exact_float32':True,'parts':part_report})
        print(lrow['ros_link'],'colors',len(set(ids.tolist())),'triangles',len(source),flush=True)
    result={'source_step_sha256':hashlib.sha256((PKG.parent/'GPT6/06_STEP/ROBOT_6DOF.step').read_bytes()).hexdigest(),
            'color_space':'sRGB recovered from STEP/XCAF styles','unique_colors':len(palette),
            'geometry_changed':False,'collision_stls_changed':False,'links':report}
    (PKG/'docs/color_audit.json').write_text(json.dumps(result,indent=2),encoding='utf-8')

if __name__=='__main__':main()
