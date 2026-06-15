# -*- coding: utf-8 -*-

import os
import numpy as np
from PyQt5.QtWidgets import QMessageBox
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLViewWidget, GLMeshItem, MeshData, GLGridItem
import pyqtgraph as pg

try:
    import trimesh
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False


class IMU3DWidget(GLViewWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setBackgroundColor('k')
        self.setCameraPosition(distance=3)
        grid = GLGridItem()
        grid.scale(1, 1, 1)
        self.addItem(grid)
        self.current_mesh_item = None
        self.set_default_cube()

    def set_default_cube(self):
        if self.current_mesh_item:
            self.removeItem(self.current_mesh_item)
        vertices = np.array([
            [ 0.5,  0.5,  0.5], [ 0.5,  0.5, -0.5], [ 0.5, -0.5,  0.5], [ 0.5, -0.5, -0.5],
            [-0.5,  0.5,  0.5], [-0.5,  0.5, -0.5], [-0.5, -0.5,  0.5], [-0.5, -0.5, -0.5]
        ])
        faces = np.array([
            [0,1,3], [0,3,2], [4,6,7], [4,7,5],
            [0,4,5], [0,5,1], [2,3,7], [2,7,6],
            [0,2,6], [0,6,4], [1,5,7], [1,7,3]
        ])
        colors = np.array([
            [1,0,0,1], [1,0,0,1], [0,1,0,1], [0,1,0,1],
            [0,0,1,1], [0,0,1,1], [1,1,0,1], [1,1,0,1],
            [1,0,1,1], [1,0,1,1], [0,1,1,1], [0,1,1,1]
        ])
        meshdata = MeshData(vertexes=vertices, faces=faces, faceColors=colors)
        self.current_mesh_item = GLMeshItem(meshdata=meshdata, smooth=False, drawEdges=True, edgeColor=(1,1,1,1))
        self.addItem(self.current_mesh_item)

    def load_model_from_file(self, filepath):
        if not TRIMESH_AVAILABLE:
            QMessageBox.critical(None, "Error", "trimesh library not installed.")
            return False
        if not os.path.exists(filepath):
            return False
        try:
            mesh = trimesh.load(filepath, force='mesh')
            if mesh is None or len(mesh.vertices) == 0:
                raise ValueError("Empty mesh")
            if not isinstance(mesh.faces, np.ndarray) or len(mesh.faces) == 0:
                mesh = trimesh.Trimesh(vertices=mesh.vertices, faces=mesh.faces)
            vertices = mesh.vertices
            faces = mesh.faces
            if hasattr(mesh.visual, 'vertex_colors') and mesh.visual.vertex_colors is not None:
                vc = mesh.visual.vertex_colors
                if vc.shape[1] == 3:
                    vc = np.hstack((vc, np.ones((vc.shape[0], 1))))
                face_colors = vc[faces].mean(axis=1)
            else:
                face_colors = np.ones((len(faces), 4)) * [0.7, 0.7, 0.7, 1.0]
            meshdata = MeshData(vertexes=vertices, faces=faces, faceColors=face_colors)
            if self.current_mesh_item:
                self.removeItem(self.current_mesh_item)
            self.current_mesh_item = GLMeshItem(meshdata=meshdata, smooth=True, drawEdges=False)
            self.addItem(self.current_mesh_item)
            bounds = mesh.bounds
            center = (bounds[0] + bounds[1]) / 2.0
            center_vec = pg.Vector(center[0], center[1], center[2])
            size = bounds[1] - bounds[0]
            distance = max(size) * 2.0
            self.setCameraPosition(distance=distance, pos=center_vec)
            return True
        except Exception:
            return False

    def set_orientation(self, roll_deg, pitch_deg, yaw_deg):
        if self.current_mesh_item:
            self.current_mesh_item.resetTransform()
            self.current_mesh_item.rotate(yaw_deg, 0, 0, 1)
            self.current_mesh_item.rotate(pitch_deg, 0, 1, 0)
            self.current_mesh_item.rotate(roll_deg, 1, 0, 0)