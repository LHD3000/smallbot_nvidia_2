import sys
import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import *
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

import rclpy
from rclpy.subscription import Subscription
from rclpy.node import Node
from std_msgs.msg import Float64

from .item import Item
from .topic_filter import TopicFilter


class MainWindow(QMainWindow):
    recording: bool
    ploting: bool
    subs: dict[str, Subscription]
    datas: dict[str, list[float]]
    node: Node

    def __init__(self):
        super().__init__()
        self.recording = False
        self.ploting = False
        self.datas = {}
        self.subs = {}

        if not rclpy.ok():
            rclpy.init()
        self.node = Node("plot_runtime")
        self.node.get_logger().info("ploting runtime start")
        self.node.declare_parameter("topics", "")
        args = self.node.get_parameter("topics").value.split(",")
        self.node.declare_parameter("auto_start", False)
        start = self.node.get_parameter("auto_start").value

        self.initUI()
        if args is not None:
            for topic in args:
                self.onTopicAdd(topic, "std_msgs/msg/Float64")

        self.timer = QTimer()
        self.timer.timeout.connect(self.plot)
        self.timer.start(100)

        self.ros2Timer = QTimer()
        self.ros2Timer.timeout.connect(self.ros2TimerCallback)
        self.ros2Timer.start(5)

        if start:
            self.list.selectAll()
            self.recordOrPause()
            self.plotOrPause()

    def initUI(self):
        self.setWindowTitle("Plot_Runtime")

        self.topicFilter = TopicFilter(
            lambda type: type in ["std_msgs/msg/Float64"], self.node, self
        )
        self.topicFilter.onTopicSelect.connect(self.onTopicAdd)

        self.list = QListWidget()
        self.list.setDragDropMode(QListWidget.InternalMove)
        self.list.setDragEnabled(True)
        self.list.setAcceptDrops(True)
        self.list.setDropIndicatorShown(True)
        self.list.setSelectionMode(QListWidget.ExtendedSelection)

        self.recordButton = QPushButton("Record")
        self.recordButton.clicked.connect(self.recordOrPause)
        self.plotButton = QPushButton("Play")
        self.plotButton.clicked.connect(self.plotOrPause)
        self.resetButton = QPushButton("Reset")
        self.resetButton.clicked.connect(self.reset)

        self.fig = plt.figure()
        self.cav = FigureCanvas(self.fig)
        self.toolbar = NavigationToolbar(self.cav, self)

        # Layout
        self.mainLayout = QSplitter(Qt.Horizontal)
        # self.mainLayout = QHBoxLayout()
        self.leftPanel = QVBoxLayout()
        self.leftPanel.addWidget(self.topicFilter)
        self.leftPanel.addWidget(self.list)
        self.endPanel = QHBoxLayout()
        self.endPanel.addWidget(self.recordButton)
        self.endPanel.addWidget(self.plotButton)
        self.endPanel.addWidget(self.resetButton)
        self.leftPanel.addLayout(self.endPanel)
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

    def onTopicAdd(self, topic: str, type: str):
        item = Item(self.list.count(), topic, type)
        list_item = QListWidgetItem()
        list_item.setSizeHint(item.sizeHint())
        self.list.addItem(list_item)
        self.list.setItemWidget(list_item, item)

    def recordOrPause(self):
        if self.recording == False:
            # step1 clear
            for topic, sub in self.subs.items():
                self.node.destroy_subscription(sub)
            self.subs.clear()
            # do not wnat to clear data,reset do this
            # self.datas.clear()
            # step2 create subscriber
            items: list[Item] = []
            for i in range(self.list.count()):
                witem = self.list.item(i)
                items.append(self.list.itemWidget(witem))
            for item in items:
                print("creat subscriber for " + item.tag)
                topicSub = self.node.create_subscription(
                    # msg_type=self.MapType(item.type),
                    msg_type=Float64,
                    topic=item.tag,
                    callback=lambda data, tag=item.tag: self.datas[tag].append(
                        data.data
                    ),
                    qos_profile=10,
                )
                self.datas[item.tag] = []
                self.subs[item.tag] = topicSub
            # step3 plot
            self.recording = True
            self.recordButton.setText("Pause Record")
        else:
            self.recording = False
            self.recordButton.setText("Start Record")
            # clear
            for topic, sub in self.subs.items():
                self.node.destroy_subscription(sub)
            self.subs.clear()

    def plotOrPause(self):
        if self.ploting == False:
            self.ploting = True
            self.plotButton.setText("Pause")
        else:
            self.ploting = False
            self.plotButton.setText("Play")

    def reset(self):
        for k, v in self.datas.items():
            v.clear()

    def ros2TimerCallback(self):
        #print("ros2TimerCallback")
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.001)
        else:
            sys.exit(0)

    def plot(self):
        if self.ploting == False:
            return
        items: list[Item] = []
        for witem in self.list.selectedItems():
            items.append(self.list.itemWidget(witem))
        self.fig.clear()
        ax = self.fig.add_subplot()
        for item in items:
            dataLines = self.datas[item.tag].copy()
            for i in range(len(dataLines)):
                dataLines[i] = (dataLines[i] + item.offset) * item.multiply
            ax.plot(dataLines, label=item.tag)
        ax.legend()
        ax.set_xlabel("Time")
        ax.set_ylabel("Value")
        ax.grid()
        self.cav.draw()

    # TODO
    def MapType(self, type: str):
        if type == "std_msgs/msg/float64":
            return Float64
        else:
            return float

    def closeEvent(self, a0):
        for topic, sub in self.subs.items():
            self.node.destroy_subscription(sub)
        rclpy.shutdown()


def main():
    app = QApplication(sys.argv)
    mainWindow = MainWindow()
    mainWindow.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
