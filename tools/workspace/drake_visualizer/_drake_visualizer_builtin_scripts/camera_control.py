from director import applogic
from PythonQt import QtCore, QtGui

import numpy as np

from _drake_visualizer_builtin_scripts import scoped_singleton_func

class _ConfigDialog(QtGui.QDialog):
    """Dialog for configuring the camera."""

    def __init__(self, parent=None):
        QtGui.QDialog.__init__(self, parent)
        self._view = None
        self.setModal(False)
        self.setWindowTitle("Configure the camera")
        layout = QtGui.QGridLayout()
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)

        row = 0

        layout.addWidget(QtGui.QLabel("Set Orthographic"), row, 0)
        set_ortho = QtGui.QCheckBox()
        set_ortho.setChecked(False)
        set_ortho.setToolTip("When checked, the camera use use orthographic "
                             "projection")
        layout.addWidget(set_ortho, row, 1)
        set_ortho.toggled.connect(self.toggle_ortho)
        row += 1

        layout.addWidget(QtGui.QLabel("Orient to axis"), row, 0)
        self.axis_orient = QtGui.QComboBox()
        self.axis_orient.addItems(
            ("Unspecified", "+X", "-X", "+Y", "-Y", "+Z", "-Z"))
        self.axis_orient.setToolTip("Orients the camera to look in the "
                                    "indicated direction")
        self.axis_orient.setCurrentIndex(0)
        self.axis_orient.connect("currentIndexChanged(int)",
                                 self.orient_on_axis)
        layout.addWidget(self.axis_orient, row, 1)
        row += 1

        camera_group = QtGui.QGroupBox("Camera Position")
        camera_layout = QtGui.QGridLayout()
        camera_group.setLayout(camera_layout)
        camera_layout.setColumnStretch(0, 0)
        camera_layout.setColumnStretch(1, 1)
        local_row = 0
        layout.addWidget(camera_group, row, 0, 1, 2)
        row += 1

        def add_float(label, layout):
            nonlocal local_row
            layout.addWidget(QtGui.QLabel(label), local_row, 0)
            value = QtGui.QLineEdit()
            QtGui.QDoubleValidator(value)
            layout.addWidget(value, local_row, 1)
            local_row += 1
            return value

        self.set_p_WC_x = add_float("x", camera_layout)
        self.set_p_WC_y = add_float("y", camera_layout)
        self.set_p_WC_z = add_float("z", camera_layout)
        self.ortho_scale = add_float("Ortho. scale", camera_layout)
        self.ortho_scale.setEnabled(set_ortho.checked)
        set_pos = QtGui.QPushButton("Set Camera Position")
        camera_layout.addWidget(set_pos, local_row, 0, 1, 2)
        set_pos.clicked.connect(self.set_position)
        local_row += 1
        get_pos = QtGui.QPushButton("Read Camera Position")
        camera_layout.addWidget(get_pos, local_row, 0, 1, 2)
        get_pos.clicked.connect(self.read_position)
        local_row += 1

        focal_group = QtGui.QGroupBox("Focal point")
        focal_layout = QtGui.QGridLayout()
        focal_group.setLayout(focal_layout)
        focal_layout.setColumnStretch(0, 0)
        focal_layout.setColumnStretch(1, 1)
        local_row = 0
        layout.addWidget(focal_group, row, 0, 1, 2)
        row += 1

        local_row = 0
        self.set_p_WF_x = add_float("x", focal_layout)
        self.set_p_WF_y = add_float("y", focal_layout)
        self.set_p_WF_z = add_float("z", focal_layout)
        set_pos = QtGui.QPushButton("Set Focal Point")
        focal_layout.addWidget(set_pos, local_row, 0, 1, 2)
        set_pos.clicked.connect(self.set_focal_position)
        local_row += 1
        get_pos = QtGui.QPushButton("Read Focal Point")
        focal_layout.addWidget(get_pos, local_row, 0, 1, 2)
        get_pos.clicked.connect(self.read_focal_position)
        local_row += 1

        self.setLayout(layout)

    def set_view(self, view):
        self._view = view

    def toggle_ortho(self, state: bool):
        """
        Toggles the camera between orthographic (True) and perspective (False).
        """
        self.ortho_scale.setEnabled(state)
        camera = self._view.camera()
        camera.SetParallelProjection(state)
        self._view.render()

    def orient_on_axis(self, selection: int):
        if selection == 0: return
        if selection == 1:  # +X
            p_CF_W = np.array((1, 0, 0))
        elif selection == 2:  # -X
            p_CF_W = np.array((-1, 0, 0))
        elif selection == 3:  # +Y
            p_CF_W = np.array((0, 1, 0))
        elif selection == 4:  # -Y
            p_CF_W = np.array((0, -1, 0))
        elif selection == 5:  # +Z
            p_CF_W = np.array((0, 0, 1))
        elif selection == 6:  # -Z
            p_CF_W = np.array((0, 0, -1))
        else:
            raise ValueError("Combo box has invalid value")
        camera = self._view.camera()
        p_WC = camera.GetPosition()
        p_WF = p_WC + p_CF_W
        camera.SetFocalPoint(p_WF)
        self._view.render()

    def set_position(self):
        x = float(self.set_p_WC_x.text)
        y = float(self.set_p_WC_y.text)
        z = float(self.set_p_WC_z.text)
        camera = self._view.camera()
        p_WC_current = np.array(camera.GetPosition())
        p_WC_new = np.array((x, y, z))
        p_CN = p_WC_new - p_WC_current
        p_WF_current = np.array(camera.GetFocalPoint())
        p_WF_new = p_WF_current + p_CN
        camera.SetPosition(p_WC_new)
        camera.SetFocalPoint(p_WF_new)
        if self.ortho_scale.enabled:
            camera.SetParallelScale(float(self.ortho_scale.text))
        self._view.render()

    def read_position(self):
        camera = self._view.camera()
        p_WC = camera.GetPosition()
        
        self.set_p_WC_x.text = f'{p_WC[0]:.5f}'
        self.set_p_WC_y.text = f'{p_WC[1]:.5f}'
        self.set_p_WC_z.text = f'{p_WC[2]:.5f}'

        if self.ortho_scale.enabled:
            self.ortho_scale.text = f'{camera.GetParallelScale():.5f}'

    
    def set_focal_position(self):
        x = float(self.set_p_WF_x.text)
        y = float(self.set_p_WF_y.text)
        z = float(self.set_p_WF_z.text)
        camera = self._view.camera()
        camera.SetFocalPoint((x, y, z))
        self._view.render()

    def read_focal_position(self):
        camera = self._view.camera()
        p_WF = camera.GetFocalPoint()
        
        self.set_p_WF_x.text = f'{p_WF[0]:.5f}'
        self.set_p_WF_y.text = f'{p_WF[1]:.5f}'
        self.set_p_WF_z.text = f'{p_WF[2]:.5f}'


# TODO(SeanCurtis): This would be better extracted out of *this* plugin
def get_sub_menu_or_make(menu, menu_name):
    for a in menu.actions():
        if a.text == menu_name:
            return a.menu()
    return menu.addMenu(menu_name)


class CameraController:
    def __init__(self):
        self._enabled = False
        self.set_enabled(True)
        self._name = "Camera Controller"

        menu_bar = applogic.getMainWindow().menuBar()
        plugin_menu = get_sub_menu_or_make(menu_bar, '&Plugins')
        contact_menu = get_sub_menu_or_make(plugin_menu, '&Camera')
        self.configure_action = contact_menu.addAction(
            '&Configure Camera')
        self.configure_action.connect('triggered()', self.show_dialog)
        self.dlg = _ConfigDialog(applogic.getMainWindow())
        self.show_dialog()

    def show_dialog(self):
        self.dlg.set_view(applogic.getCurrentRenderView())
        self.dlg.show()

    def is_enabled(self):
        return self._enabled

    def set_enabled(self, enable):
        self._enabled = enable

@scoped_singleton_func
def init_plugin():
    # Create a visualizer instance.
    controller = CameraController()
    # Adds to the 'Tools' menu.
    applogic.MenuActionToggleHelper(
        'Tools', controller._name,
        controller.is_enabled, controller.set_enabled)
    return controller

if __name__ == '__main__':
    cam_plugin = init_plugin()
