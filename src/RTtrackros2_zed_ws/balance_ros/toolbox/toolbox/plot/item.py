from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import QDoubleValidator


class Item(QWidget):
    index:int
    tag:str
    type:str
    multiply:float
    offset:float
    def __init__(self, index: int, tag: str = "unknown",type:str="",parent=None):
        super(Item, self).__init__(parent)
        self.index = index
        self.tag = tag
        self.type = type
        self.multiply = 1.0
        self.offset = 0.0
        self.initUI()

    def initUI(self):
        self.hb = QHBoxLayout()
        self.lable = QLabel(self.tag)
        self.lable.setFixedWidth(100)

        self.multiplyLine = QLineEdit()
        self.multiplyLine.setValidator(QDoubleValidator())
        self.multiplyLine.setFixedWidth(50)
        self.multiplyLine.textChanged.connect(self.updateMultiply)
        self.multiplyLine.setText(str(self.multiply))

        self.offsetLine = QLineEdit()
        self.offsetLine.setValidator(QDoubleValidator())
        self.offsetLine.setFixedWidth(50)
        self.offsetLine.textChanged.connect(self.updateOffset)
        self.offsetLine.setText(str(self.offset))

        self.hb.addWidget(self.lable)
        self.hb.addWidget(self.multiplyLine)
        self.hb.addWidget(self.offsetLine)
        self.setLayout(self.hb)

    def updateMultiply(self, text):
        try:
            self.multiply = float(text)
        except:
            pass

    def updateOffset(self, text):
        try:
            self.offset = float(text)
        except:
            pass


if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)
    picker = Item(1)
    picker.show()
    sys.exit(app.exec_())
