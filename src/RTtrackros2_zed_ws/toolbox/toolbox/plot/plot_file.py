import sys
from typing import List
import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import pandas as pd
from .picker import Picker
from .item import Item


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.data = None

    def initUI(self):
        self.setWindowTitle("Plot_File")

        self.filePicker = Picker(self)
        self.filePicker.onFileSelected.connect(self.onFileChanged)

        self.list = QListWidget()
        self.list.setDragDropMode(QListWidget.InternalMove)
        self.list.setDragEnabled(True)
        self.list.setAcceptDrops(True)
        self.list.setDropIndicatorShown(True)
        self.list.setSelectionMode(QListWidget.ExtendedSelection)

        self.button = QPushButton("Plot")
        self.button.clicked.connect(self.plot)
        self.button.setEnabled(False)

        self.fig = plt.figure()
        self.cav = FigureCanvas(self.fig)
        self.toolbar = NavigationToolbar(self.cav, self)

        # Layout
        self.mainLayout = QSplitter(Qt.Horizontal)
        # self.mainLayout = QHBoxLayout()
        self.leftPanel = QVBoxLayout()
        self.leftPanel.addWidget(self.filePicker)
        self.leftPanel.addWidget(self.list)
        self.leftPanel.addWidget(self.button)
        self.lw = QWidget()
        self.lw.setLayout(self.leftPanel)

        self.rightPanel = QVBoxLayout()
        self.rightPanel.addWidget(self.toolbar)
        self.rightPanel.addWidget(self.cav)
        self.rw = QWidget()
        self.rw.setLayout(self.rightPanel)

        self.mainLayout.addWidget(self.lw)
        self.mainLayout.addWidget(self.rw)
        self.setCentralWidget(self.mainLayout)

    def onFileChanged(self, fileName):
        # print(fileName)
        self.data = pd.read_csv(fileName)
        if self.data is None:
            self.button.setEnabled(False)
            self.list.clear()
            return
        self.list.clear()
        for i in range(len(self.data.columns.values)):
            item = Item(i, self.data.columns.values[i])
            list_item = QListWidgetItem()
            list_item.setSizeHint(item.sizeHint())
            self.list.addItem(list_item)
            self.list.setItemWidget(list_item, item)
            # print(data.columns.values[i])
        self.button.setEnabled(True)

    def plot(self):
        if self.data is None:
            return

        items: List[Item] = []
        for witem in self.list.selectedItems():
            items.append(self.list.itemWidget(witem))

        self.fig.clear()
        ax = self.fig.add_subplot()
        time = self.data.iloc[1:, 0].to_numpy()  # 默认第一行作为时间轴，但实际可能不存在时间轴
        for item in items:
            dataLines = self.data.iloc[1:, item.index].copy().to_numpy()
            dataLines += item.offset
            dataLines *= item.multiply
            ax.plot(time, dataLines, label=item.tag)
        ax.legend()
        ax.set_xlabel("Time")
        ax.set_ylabel("Value")
        ax.grid()
        self.cav.draw()


def main():
    app = QApplication(sys.argv)
    mainWindow = MainWindow()
    mainWindow.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
