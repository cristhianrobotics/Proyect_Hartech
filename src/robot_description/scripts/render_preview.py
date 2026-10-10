"""Render the unchanged triangles with the material colors verified in the DAE."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk,numpy_to_vtkIdTypeArray
from validate_description import forward

PKG=Path(__file__).resolve().parents[1]
root=ET.parse(PKG/'urdf/robot_6dof.urdf').getroot()
meta=json.loads((PKG/'docs/nominal_frames.json').read_text())
window=vtk.vtkRenderWindow()
window.SetOffScreenRendering(1)
window.SetSize(1800,1000)
window.SetMultiSamples(4)

for column,(label,q) in enumerate([('HOME | 0, -50, 80, 0, -30, 0 deg',meta['home_rad']),('CERO ARTICULAR | 0, 0, 0, 0, 0, 0 deg',[0]*6)]):
    renderer=vtk.vtkRenderer()
    renderer.SetViewport(column*.5,0,(column+1)*.5,1)
    renderer.SetBackground(.94,.96,.98)
    window.AddRenderer(renderer)
    poses=forward(root,{f'joint_{i+1}':v for i,v in enumerate(q)})
    for link in root.findall('link'):
        visual=link.find('visual')
        if visual is None:
            continue
        path=PKG/visual.find('geometry/mesh').get('filename').split('robot_description/')[1]
        dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
        source=np.fromfile(path.with_suffix('.stl'),dtype=dtype,offset=84)
        poly=vtk.vtkPolyData()
        points=vtk.vtkPoints()
        points.SetData(numpy_to_vtk(source['vertices'].reshape(-1,3).copy(),deep=True))
        poly.SetPoints(points)
        cells=vtk.vtkCellArray()
        cells.SetData(numpy_to_vtkIdTypeArray(np.arange(len(source)+1,dtype=np.int64)*3,deep=True),
                      numpy_to_vtkIdTypeArray(np.arange(len(source)*3,dtype=np.int64),deep=True))
        poly.SetPolys(cells)
        poly.GetCellData().SetNormals(numpy_to_vtk(source['normal'].copy(),deep=True))
        colors=np.load(PKG/'docs'/(link.get('name')+'_colors.npz'))
        cell_colors=np.rint(colors['palette'][colors['ids']]*255).astype(np.uint8)
        assert len(cell_colors)==poly.GetNumberOfCells()
        scalars=numpy_to_vtk(cell_colors,deep=True,array_type=vtk.VTK_UNSIGNED_CHAR)
        scalars.SetName('CAD_colors')
        poly.GetCellData().SetScalars(scalars)
        mapper=vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly)
        mapper.SetScalarModeToUseCellData()
        mapper.SetColorModeToDirectScalars()
        actor=vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetSpecular(.25)
        actor.GetProperty().SetSpecularPower(30)
        mat=vtk.vtkMatrix4x4()
        for a in range(4):
            for b in range(4):
                mat.SetElement(a,b,poses[link.get('name')][a,b])
        actor.SetUserMatrix(mat)
        renderer.AddActor(actor)
    text=vtk.vtkTextActor()
    text.SetInput(label)
    text.SetPosition(35,930)
    text.GetTextProperty().SetFontSize(23)
    text.GetTextProperty().SetColor(.12,.18,.25)
    text.GetTextProperty().BoldOn()
    renderer.AddActor2D(text)
    note=vtk.vtkTextActor()
    note.SetInput('Colores originales del STEP | Geometria conservada\nVista estatica; no es una ejecucion de RViz')
    note.SetPosition(35,35)
    note.GetTextProperty().SetFontSize(18)
    note.GetTextProperty().SetColor(.25,.30,.35)
    renderer.AddActor2D(note)
    camera=renderer.GetActiveCamera()
    camera.SetPosition(.90,-1.20,.72)
    camera.SetFocalPoint(.13,0,.22)
    camera.SetViewUp(0,0,1)
    camera.ParallelProjectionOn()
    camera.SetParallelScale(.42)
    renderer.ResetCameraClippingRange()

window.Render()
capture=vtk.vtkWindowToImageFilter()
capture.SetInput(window)
capture.ReadFrontBufferOff()
capture.Update()
writer=vtk.vtkPNGWriter()
writer.SetFileName(str(PKG/'docs/robot_preview.png'))
writer.SetInputConnection(capture.GetOutputPort())
writer.Write()
window.Finalize()
print(PKG/'docs/robot_preview.png')
