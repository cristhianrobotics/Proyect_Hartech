"""Read STEP assembly names and placements without altering the source."""
import json
from pathlib import Path
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
try:
    from OCP.TDF import TDF_LabelSequence
except ImportError:
    from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.XCAFDoc import XCAFDoc_DocumentTool

PKG = Path(__file__).resolve().parents[1]

def name(label):
    attr = TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else ''

def matrix(loc):
    tr = loc.Transformation()
    return [[tr.Value(i,j) for j in range(1,5)] for i in range(1,4)] + [[0,0,0,1]]

def read():
    doc = TDocStd_Document(TCollection_ExtendedString('robot'))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    status = reader.ReadFile(str(PKG.parent/'GPT6/06_STEP/ROBOT_6DOF.step'))
    print('Read status:', status, flush=True)
    if not reader.Transfer(doc):
        raise RuntimeError('STEP transfer failed')
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    st.GetFreeShapes(roots)
    return doc, st, roots

def children(st,label):
    ref=TDF_Label()
    if st.GetReferredShape_s(label,ref):
        label=ref
    seq=TDF_LabelSequence()
    st.GetComponents_s(label,seq)
    return [seq.Value(i) for i in range(1,seq.Length()+1)]

if __name__=='__main__':
    doc,st,roots=read()
    rows=[]
    for i in range(1,roots.Length()+1):
        root=roots.Value(i)
        print('ROOT',name(root))
        for child in children(st,root):
            parts=children(st,child)
            row=dict(name=name(child),matrix=matrix(st.GetLocation_s(child)),parts=len(parts),first_parts=[name(p) for p in parts[:4]])
            rows.append(row)
            print(row,flush=True)
    (PKG/'docs/step_structure.json').write_text(json.dumps(rows,indent=2))
