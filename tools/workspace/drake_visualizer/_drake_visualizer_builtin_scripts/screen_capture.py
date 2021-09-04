import os

from director import applogic
from director import ioUtils as io
import director.vtkAll as vtk
from PythonQt import QtGui

def saveScreenshot(view, filename, should_render=False, should_write=True):
    """
    Grab the window contents and write it out to a file. This code is
    taken directly from director, screengrabberpanel.py.

    Director's License

    Copyright (c) 2012, Robot Locomotion Group @ CSAIL
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    """
    if should_render:
        view.forceRender()

    grabber = vtk.vtkWindowToImageFilter()
    grabber.SetInput(view.renderWindow())
    grabber.SetInputBufferTypeToRGB()
    grabber.ReadFrontBufferOff()
    grabber.SetShouldRerender(False)
    grabber.Update()

    if should_write:
        io.writeImage(grabber.GetOutput(), filename)
    return grabber.GetOutput()

class ScreenCaptureWidget(QtGui.QGroupBox):
    def __init__(self, prefix: str, parent=None):
        QtGui.QGroupBox.__init__(self, "Screen Capture", parent)
        layout = QtGui.QGridLayout()
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)

        row = 0

        layout.addWidget(QtGui.QLabel("Enable"))
        self.grab_enabled = QtGui.QCheckBox()
        layout.addWidget(self.grab_enabled, row, 1)
        row += 1

        layout.addWidget(QtGui.QLabel("Output Directory"), row, 0)
        self.browse_btn = QtGui.QPushButton("Browse...")
        layout.addWidget(self.browse_btn, row, 1)
        self.browse_btn.clicked.connect(self.select_directory)
        row += 1

        layout.addWidget(QtGui.QLabel("File prefix"), row, 0)
        self.prefix_edt = QtGui.QLineEdit("hydro")
        layout.addWidget(self.prefix_edt, row, 1)
        row += 1

        self.output_dir_edit = QtGui.QLineEdit()
        layout.addWidget(self.output_dir_edit, row, 0, 1, 2)
        self.output_dir_edit.text = "~/Pictures/drake"
        row += 1
        
        layout.addWidget(QtGui.QLabel("Next image index:"), row, 0)
        self.index_label = QtGui.QLabel("0")
        layout.addWidget(self.index_label, row, 1)
        row += 1

        self.reset_btn = QtGui.QPushButton("Reset index counter")
        layout.addWidget(self.reset_btn, row, 0, 1, 2)
        self.reset_btn.clicked.connect(self.reset_counter)

        row += 1

        self.setLayout(layout)

    def select_directory(self):
        newDir = QtGui.QFileDialog.getExistingDirectory(
            applogic.getMainWindow(), "Choose directory...",
            self.output_directory())
        if newDir:
            self.output_dir_edit.text = newDir

    def output_directory(self):
        return os.path.expanduser(self.output_dir_edit.text)

    def reset_counter(self):
        self.index_label.text = "0"

    def grab_screen(self, view):
        if self.grab_enabled.checked:
            view.setFixedSize(640, 640)
            index = int(self.index_label.text)
            next = index + 1
            self.index_label.text = f"{next}"
            file_path = os.path.join(self.output_directory(),
                                    f'{self.prefix_edt.text}_{index:04}.png')
            saveScreenshot(view, file_path, should_render=True)
            display_time_ms = 2000
            applogic.getMainWindow().statusBar().showMessage(
                f'Saved: {file_path}', display_time_ms)


def AddScreenCapture():
    """Creates a widget containing the functionality necessary for writing"""
    group = QtGUi.QGroupBox("Screen Capture")
    layout = QtGUi.QGridLayout()

    

    group.setLayout(layout)
    return group

print("Screen capture loaded")