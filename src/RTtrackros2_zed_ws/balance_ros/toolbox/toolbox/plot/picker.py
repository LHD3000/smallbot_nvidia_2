from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import os

class Picker(QWidget):
    onFileSelected = pyqtSignal(str)

    def __init__(self, parent=None, initFilePath: str = ""):
        super(Picker, self).__init__(parent)
        self.initPath = initFilePath
        self.initUI()

    def initUI(self):
        self.lable = QLabel("Select data file:")
        self.filePath = QLineEdit()
        self.picker = QPushButton("browse")
        self.picker.clicked.connect(self.browseFile)

        # Layout
        self.hb = QHBoxLayout()
        self.hb.addWidget(self.filePath)
        self.hb.addWidget(self.picker)
        self.vb = QVBoxLayout()
        self.vb.addWidget(self.lable)
        self.vb.addLayout(self.hb)
        self.setLayout(self.vb)

    def browseFile(self):
        options = QFileDialog.Options()
        fileName, _ = QFileDialog.getOpenFileName(
            self, "Open File", self.initPath, "Data files (*.csv)", options=options
        )
        if fileName:
            self.filePath.setText(fileName)
            if os.path.exists(fileName):
                self.onFileSelected.emit(fileName)

    def getFilePath(self) -> str:
        return os.path.exists(self.filePath.text()) and self.filePath.text() or None


if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)
    picker = Picker()
    picker.show()
    sys.exit(app.exec_())
