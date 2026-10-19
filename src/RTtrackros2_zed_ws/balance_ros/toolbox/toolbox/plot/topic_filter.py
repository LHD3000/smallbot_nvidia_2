from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import rclpy
from rclpy.node import Node


class TopicFilter(QWidget):
    onTopicSelect = pyqtSignal(str, str)

    def __init__(self, filter, node: Node, parent=None):
        super(TopicFilter, self).__init__(parent)
        self.parent = parent
        self.initUI()
        if not rclpy.ok():
            rclpy.init()
        self.filter = filter
        self.node = node

    def initUI(self):
        self.button = QPushButton("Add Topic")
        self.vb = QVBoxLayout()
        self.vb.addWidget(self.button)
        self.setLayout(self.vb)
        self.button.clicked.connect(self.addTopic)

    def addTopic(self):
        items = []
        topics = self.node.get_topic_names_and_types()
        for topic in topics:
            if self.filter(topic[1][0]):
                items.append(topic[0])
        if len(items) == 0:
            QMessageBox.information(self, "Error", "No topics found")
            return
        val, ok = QInputDialog.getItem(self, "Topics", "select:", items, 1, True)
        if ok:
            for topic in topics:
                if topic[0] == val:
                    self.onTopicSelect.emit(val, topic[1][0])

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)
    filter = lambda type: type in ["std_msgs/msg/String"]
    tf = TopicFilter(filter)
    tf.show()
    tf.onTopicSelect.connect(lambda topic, type: print(topic, type))
    sys.exit(app.exec_())
