# -*- coding: utf-8 -*-
"""
crater_mesh_view — 지상국 크레이터 정보 창 안에 넣는 3D mesh 보기 (PyQt5 + VTK). 작성 261009.

글·버튼 없이 mesh 만. 마우스: 왼쪽 드래그 = 3축 회전, 휠 = 확대/축소, 가운데 드래그 = 이동 (VTK trackball camera).
pin_gallery 가 관측 종료 뒤 워커에서 만든 completed_mesh.npz 를 불러와 그리기만 한다 (여기서 계산하지 않음).
좌표 = 지면 기준(top_local_m, z = 지면 법선 위), 높이 = 측정값 그대로(시험장 Z 증폭 지형 그대로, 추가 배율 없음).
음영·색은 전달본 html 뷰어와 같게: 회색 245, shade = 0.40 + 0.55·max(0, n·l).
"""
import numpy as np
from PyQt5 import QtWidgets

import vtkmodules.vtkInteractionStyle  # noqa: F401  (상호작용 스타일 등록)
import vtkmodules.vtkRenderingOpenGL2  # noqa: F401  (OpenGL 백엔드 등록)
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtkmodules.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray
from vtkmodules.vtkCommonCore import vtkPoints
from vtkmodules.vtkCommonDataModel import vtkCellArray, vtkPolyData
from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera
from vtkmodules.vtkRenderingCore import vtkActor, vtkPolyDataMapper, vtkRenderer


class CraterMeshView(QtWidgets.QWidget):
    def __init__(self, parent=None, height=300):
        super().__init__(parent)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.vtkw = QVTKRenderWindowInteractor(self)
        self.vtkw.setMinimumHeight(height)
        lay.addWidget(self.vtkw)
        self.ren = vtkRenderer()
        self.ren.SetBackground(0x24 / 255, 0x24 / 255, 0x24 / 255)
        self.vtkw.GetRenderWindow().AddRenderer(self.ren)
        self.vtkw.GetRenderWindow().GetInteractor().SetInteractorStyle(vtkInteractorStyleTrackballCamera())
        self.actor = None
        self.path = None
        self._ready = False

    def showEvent(self, e):
        super().showEvent(e)
        if not self._ready:                                    # 처음 보일 때 한 번 (숨은 창에서 GL 초기화하면 X 오류)
            self.vtkw.Initialize()
            self._ready = True

    def load(self, path):
        """completed_mesh.npz 를 그린다. 같은 파일이면 다시 안 읽음(시점 유지). None = 비움."""
        if path == self.path:
            return
        self.path = path
        if self.actor is not None:
            self.ren.RemoveActor(self.actor)
            self.actor = None
        if path:
            with np.load(path) as d:
                v, f = d['top_local_m'].astype(np.float64), d['top_faces'].astype(np.int64)
            pts = vtkPoints()
            pts.SetData(numpy_to_vtk(v, deep=True))
            cells = vtkCellArray()
            cells.SetCells(len(f), numpy_to_vtkIdTypeArray(np.c_[np.full(len(f), 3), f].ravel(), deep=True))
            pd = vtkPolyData()
            pd.SetPoints(pts)
            pd.SetPolys(cells)
            m = vtkPolyDataMapper()
            m.SetInputData(pd)
            self.actor = vtkActor()
            self.actor.SetMapper(m)
            pr = self.actor.GetProperty()
            pr.SetColor(245 / 255, 245 / 255, 245 / 255)
            pr.SetInterpolationToFlat()
            pr.SetAmbient(0.40)
            pr.SetDiffuse(0.55)
            pr.SetSpecular(0.0)
            self.ren.AddActor(self.actor)
            self.reset_view()
        self.vtkw.GetRenderWindow().Render()

    def reset_view(self):
        """전달본 미리보기와 같은 첫 시점: 고도 38°, 방위 −57°, z 위."""
        cam = self.ren.GetActiveCamera()
        el, az = np.deg2rad(38), np.deg2rad(-57)
        cam.SetFocalPoint(0, 0, 0)
        cam.SetPosition(np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el))
        cam.SetViewUp(0, 0, 1)
        self.ren.ResetCamera()
        cam.Zoom(1.2)
