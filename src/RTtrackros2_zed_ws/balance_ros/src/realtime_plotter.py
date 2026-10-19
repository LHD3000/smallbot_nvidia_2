#!/usr/bin/env python3
import sys
import os
import numpy as np
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import *
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.font_manager import FontProperties

# 添加中文字体支持
try:
    # 尝试使用系统中文字体
    import matplotlib.font_manager as fm
    fonts = fm.findSystemFonts()
    chinese_fonts = [f for f in fonts if any(keyword in f.lower() for keyword in ['msyh', 'simhei', 'simsun', 'pingfang', 'noto'])]
    if chinese_fonts:
        plt.rcParams['font.sans-serif'] = [os.path.basename(chinese_fonts[0]).split('.')[0]]
    else:
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans']  # 回退到英文
    plt.rcParams['axes.unicode_minus'] = False
except:
    pass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import threading
import time
import queue


class ROS2ContextManager:
    """ROS2上下文管理器，确保只初始化一次"""
    _initialized = False
    
    @classmethod
    def ensure_initialized(cls):
        if not cls._initialized and not rclpy.ok():
            rclpy.init()
            cls._initialized = True


class DataSubscriber(Node):
    """ROS2数据订阅器 - 简化版本"""
    def __init__(self, topic_name, topic_type, data_queue):
        super().__init__(f'data_sub_{topic_name.replace("/", "_")[:20]}')
        self.topic_name = topic_name
        self.data_queue = data_queue
        
        # 获取话题类型
        self.topic_type = self.import_topic_type(topic_type)
        
        if self.topic_type:
            # 创建订阅器
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
            
            self.subscription = self.create_subscription(
                self.topic_type,
                topic_name,
                self.callback,
                qos
            )
            self.get_logger().info(f'Subscribed: {topic_name}')
        else:
            self.get_logger().error(f'Unsupported: {topic_type}')
    
    def import_topic_type(self, type_str):
        """动态导入话题类型"""
        try:
            # 常见消息类型的映射
            type_map = {
                'std_msgs/msg/Float64': 'std_msgs.msg.Float64',
                'std_msgs/msg/Float32': 'std_msgs.msg.Float32',
                'std_msgs/msg/Int32': 'std_msgs.msg.Int32',
                'std_msgs/msg/Bool': 'std_msgs.msg.Bool',
            }
            
            if type_str in type_map:
                import importlib
                module_name, class_name = type_map[type_str].rsplit('.', 1)
                module = importlib.import_module(module_name)
                return getattr(module, class_name)
        except Exception as e:
            self.get_logger().error(f'Import error: {type_str} - {e}')
        return None
    
    def callback(self, msg):
        """接收数据回调"""
        try:
            # 提取数据值
            if hasattr(msg, 'data'):
                value = msg.data
            else:
                # 尝试获取第一个数值属性
                value = 0
                for attr in dir(msg):
                    if not attr.startswith('_') and not callable(getattr(msg, attr)):
                        attr_val = getattr(msg, attr)
                        if isinstance(attr_val, (int, float)):
                            value = attr_val
                            break
            
            timestamp = time.time()
            
            # 将数据放入队列
            try:
                self.data_queue.put_nowait((self.topic_name, value, timestamp))
            except queue.Full:
                pass
                
        except Exception as e:
            self.get_logger().error(f'Process error: {e}')


class ROS2SpinThread(threading.Thread):
    """ROS2旋转线程 - 单线程处理所有节点"""
    def __init__(self):
        super().__init__(daemon=True)
        self.nodes = []
        self.running = True
        self.executor = None
        
    def add_node(self, node):
        """添加节点"""
        self.nodes.append(node)
        
    def remove_node(self, node):
        """移除节点"""
        if node in self.nodes:
            self.nodes.remove(node)
            node.destroy_node()
    
    def run(self):
        """运行线程"""
        from rclpy.executors import SingleThreadedExecutor
        
        self.executor = SingleThreadedExecutor()
        
        # 添加所有节点到执行器
        for node in self.nodes:
            self.executor.add_node(node)
        
        # 主循环
        while self.running and rclpy.ok():
            try:
                # 处理回调
                self.executor.spin_once(timeout_sec=0.01)
            except Exception as e:
                print(f"Executor error: {e}")
                break
        
        # 清理
        for node in self.nodes:
            self.executor.remove_node(node)
        
    def stop(self):
        """停止线程"""
        self.running = False
        if self.executor:
            self.executor.shutdown()


class TopicManager:
    """话题管理器"""
    def __init__(self):
        self.data_history = {}  # {topic_name: [(timestamp, value), ...]}
        self.max_history = 2000
        self.lock = threading.Lock()
        
        # 数据队列
        self.data_queue = queue.Queue(maxsize=1000)
        
        # ROS2旋转线程
        self.spin_thread = ROS2SpinThread()
        self.spin_thread.start()
        
        # 确保ROS2上下文已初始化
        ROS2ContextManager.ensure_initialized()
        
        # 启动数据处理线程
        self.data_thread = threading.Thread(target=self.process_data, daemon=True)
        self.data_thread.start()
    
    def subscribe(self, topic_name, topic_type):
        """订阅话题"""
        if self.get_subscriber_node(topic_name):
            return False
        
        try:
            node = DataSubscriber(topic_name, topic_type, self.data_queue)
            self.spin_thread.add_node(node)
            
            with self.lock:
                if topic_name not in self.data_history:
                    self.data_history[topic_name] = []
            
            print(f"Subscribed: {topic_name}")
            return True
        except Exception as e:
            print(f'Subscribe failed {topic_name}: {e}')
            return False
    
    def get_subscriber_node(self, topic_name):
        """获取订阅节点"""
        for node in self.spin_thread.nodes:
            if hasattr(node, 'topic_name') and node.topic_name == topic_name:
                return node
        return None
    
    def process_data(self):
        """处理数据队列"""
        while True:
            try:
                topic_name, value, timestamp = self.data_queue.get(timeout=0.1)
                with self.lock:
                    if topic_name not in self.data_history:
                        self.data_history[topic_name] = []
                    
                    # 添加新数据
                    self.data_history[topic_name].append((timestamp, value))
                    
                    # 保持数据长度
                    if len(self.data_history[topic_name]) > self.max_history:
                        self.data_history[topic_name] = self.data_history[topic_name][-self.max_history:]
            
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Process data error: {e}")
    
    def unsubscribe(self, topic_name):
        """取消订阅"""
        node = self.get_subscriber_node(topic_name)
        if node:
            self.spin_thread.remove_node(node)
            
            with self.lock:
                if topic_name in self.data_history:
                    del self.data_history[topic_name]
    
    def get_data(self, topic_name, max_points=None):
        """获取话题数据"""
        with self.lock:
            if topic_name not in self.data_history:
                return [], []
            
            data = self.data_history[topic_name]
            if not data:
                return [], []
            
            # 获取时间序列和值序列
            timestamps, values = zip(*data)
            
            if max_points and len(timestamps) > max_points:
                timestamps = timestamps[-max_points:]
                values = values[-max_points:]
            
            return list(timestamps), list(values)
    
    def get_all_topics(self):
        """获取所有已订阅的话题"""
        return list(self.data_history.keys())
    
    def clear_data(self, topic_name=None):
        """清除数据"""
        with self.lock:
            if topic_name:
                if topic_name in self.data_history:
                    self.data_history[topic_name] = []
            else:
                for topic in self.data_history:
                    self.data_history[topic] = []
    
    def shutdown(self):
        """关闭管理器"""
        self.spin_thread.stop()
        self.spin_thread.join(timeout=1.0)


class TopicDiscoveryNode(Node):
    """用于发现话题的ROS2节点"""
    def __init__(self):
        super().__init__("topic_discovery_node")
    
    def get_topics(self):
        """获取所有话题"""
        topics_and_types = []
        try:
            topics_and_types = self.get_topic_names_and_types()
        except Exception as e:
            self.get_logger().error(f'Get topics failed: {e}')
        return topics_and_types


class TopicDiscoveryWidget(QWidget):
    """话题发现组件"""
    topic_selected = pyqtSignal(str, str)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.discovery_node = None
        self.init_ui()
        self.init_ros2_node()
        
    def init_ui(self):
        layout = QVBoxLayout()
        
        # 标题
        title = QLabel("ROS2 Topics Discovery")
        title.setStyleSheet("font-weight: bold; font-size: 14px; margin: 5px;")
        layout.addWidget(title)
        
        # 搜索框
        search_layout = QHBoxLayout()
        self.search_edit = QLineEdit()
        self.search_edit.setPlaceholderText("Search topics...")
        self.search_edit.textChanged.connect(self.filter_topics)
        search_layout.addWidget(self.search_edit)
        
        self.refresh_btn = QPushButton("🔄 Refresh")
        self.refresh_btn.clicked.connect(self.refresh_topics)
        search_layout.addWidget(self.refresh_btn)
        
        layout.addLayout(search_layout)
        
        # 话题列表
        self.topic_list = QListWidget()
        self.topic_list.itemDoubleClicked.connect(self.on_item_double_clicked)
        layout.addWidget(self.topic_list)
        
        # 类型过滤
        type_filter_layout = QHBoxLayout()
        type_filter_layout.addWidget(QLabel("Filter:"))
        
        self.type_filter_combo = QComboBox()
        self.type_filter_combo.addItem("All types")
        self.type_filter_combo.addItem("std_msgs/msg/Float64")
        self.type_filter_combo.addItem("std_msgs/msg/Float32")
        self.type_filter_combo.addItem("std_msgs/msg/Int32")
        self.type_filter_combo.currentTextChanged.connect(self.filter_topics)
        type_filter_layout.addWidget(self.type_filter_combo)
        
        layout.addLayout(type_filter_layout)
        
        self.setLayout(layout)
        self.setMinimumHeight(300)
    
    def init_ros2_node(self):
        """初始化ROS2节点"""
        try:
            ROS2ContextManager.ensure_initialized()
            self.discovery_node = TopicDiscoveryNode()
        except Exception as e:
            print(f"Init ROS2 node failed: {e}")
    
    def refresh_topics(self):
        """刷新话题列表"""
        if not self.discovery_node:
            self.init_ros2_node()
            if not self.discovery_node:
                print("Cannot init ROS2 node")
                return
        
        self.topic_list.clear()
        try:
            topics_and_types = self.discovery_node.get_topics()
            for topic_name, topic_types in topics_and_types:
                for topic_type in topic_types:
                    # 应用类型过滤
                    filter_text = self.type_filter_combo.currentText()
                    if filter_text != "All types" and filter_text != topic_type:
                        continue
                    
                    item_text = f"{topic_name} [{topic_type}]"
                    item = QListWidgetItem(item_text)
                    item.setData(Qt.UserRole, (topic_name, topic_type))
                    self.topic_list.addItem(item)
            
            # 处理回调
            rclpy.spin_once(self.discovery_node, timeout_sec=0.1)
            
        except Exception as e:
            print(f"Get topics failed: {e}")
    
    def filter_topics(self, text):
        filter_text = self.search_edit.text().lower()
        for i in range(self.topic_list.count()):
            item = self.topic_list.item(i)
            item.setHidden(filter_text not in item.text().lower())
    
    def on_item_double_clicked(self, item):
        topic_name, topic_type = item.data(Qt.UserRole)
        self.topic_selected.emit(topic_name, topic_type)
    
    def close_node(self):
        """关闭节点"""
        if self.discovery_node:
            self.discovery_node.destroy_node()


class TopicListItem(QWidget):
    """话题列表项"""
    def __init__(self, index, topic_name, topic_type):
        super().__init__()
        self.index = index
        self.topic_name = topic_name
        self.topic_type = topic_type
        
        self.multiply = 1.0
        self.offset = 0.0
        self.color = None
        self.visible = True
        
        self.init_ui()
        
    def init_ui(self):
        layout = QHBoxLayout()
        
        # 可见性复选框
        self.visible_checkbox = QCheckBox()
        self.visible_checkbox.setChecked(True)
        self.visible_checkbox.stateChanged.connect(self.on_visibility_changed)
        layout.addWidget(self.visible_checkbox)
        
        # 话题标签
        label_text = f"[{self.index}] {self.topic_name.split('/')[-1]}"
        if len(label_text) > 20:
            label_text = label_text[:20] + "..."
        self.label = QLabel(label_text)
        self.label.setToolTip(self.topic_name)
        self.label.setMinimumWidth(150)
        layout.addWidget(self.label)
        
        # 颜色选择按钮
        self.color_btn = QPushButton("■")
        self.color_btn.setFixedWidth(30)
        self.color_btn.clicked.connect(self.choose_color)
        layout.addWidget(self.color_btn)
        
        # 缩放
        layout.addWidget(QLabel("×"))
        self.multiply_edit = QLineEdit("1.0")
        self.multiply_edit.setFixedWidth(50)
        self.multiply_edit.textChanged.connect(self.on_multiply_changed)
        layout.addWidget(self.multiply_edit)
        
        # 偏移
        layout.addWidget(QLabel("+"))
        self.offset_edit = QLineEdit("0.0")
        self.offset_edit.setFixedWidth(50)
        self.offset_edit.textChanged.connect(self.on_offset_changed)
        layout.addWidget(self.offset_edit)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def choose_color(self):
        color = QColorDialog.getColor()
        if color.isValid():
            self.color = color
            self.color_btn.setStyleSheet(f"background-color: {color.name()}; color: {color.name()}")
    
    def on_multiply_changed(self, text):
        try:
            self.multiply = float(text)
        except ValueError:
            self.multiply = 1.0
    
    def on_offset_changed(self, text):
        try:
            self.offset = float(text)
        except ValueError:
            self.offset = 0.0
    
    def on_visibility_changed(self, state):
        self.visible = (state == Qt.Checked)


class RealTimePlotter(QMainWindow):
    """实时绘图主窗口"""
    def __init__(self):
        super().__init__()
        self.topic_manager = TopicManager()
        self.is_plotting = False
        self.plot_start_time = None
        
        self.init_ui()
        self.setup_timers()
        
    def init_ui(self):
        self.setWindowTitle("ROS2 Real-time Plotter")
        self.resize(1400, 800)
        
        # 设置应用程序样式
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f5f5f5;
            }
            QLabel {
                color: #333;
            }
            QPushButton {
                padding: 8px 12px;
                border-radius: 4px;
                font-weight: bold;
                min-height: 30px;
            }
        """)
        
        # 中央分割器
        splitter = QSplitter(Qt.Horizontal)
        
        # 左侧面板
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        # 话题发现
        self.topic_discovery = TopicDiscoveryWidget()
        self.topic_discovery.topic_selected.connect(self.add_topic)
        left_layout.addWidget(self.topic_discovery)
        
        # 已订阅话题列表
        subscribed_label = QLabel("Subscribed Topics (check to show/hide)")
        subscribed_label.setStyleSheet("font-weight: bold; margin: 5px; color: #2c3e50;")
        left_layout.addWidget(subscribed_label)
        
        self.topic_list = QListWidget()
        self.topic_list.setDragDropMode(QListWidget.InternalMove)
        self.topic_list.setSelectionMode(QListWidget.ExtendedSelection)
        self.topic_list.setStyleSheet("""
            QListWidget {
                background-color: white;
                border: 1px solid #ddd;
                border-radius: 4px;
            }
            QListWidget::item {
                padding: 2px;
                border-bottom: 1px solid #eee;
            }
            QListWidget::item:selected {
                background-color: #e3f2fd;
            }
        """)
        left_layout.addWidget(self.topic_list)
        
        # 控制按钮
        button_layout = QGridLayout()
        button_layout.setSpacing(8)
        
        self.plot_btn = QPushButton("▶ Start Plotting")
        self.plot_btn.clicked.connect(self.toggle_plotting)
        self.plot_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        button_layout.addWidget(self.plot_btn, 0, 0)
        
        self.clear_btn = QPushButton("🗑 Clear Data")
        self.clear_btn.clicked.connect(self.clear_data)
        self.clear_btn.setStyleSheet("""
            QPushButton {
                background-color: #9E9E9E;
                color: white;
            }
            QPushButton:hover {
                background-color: #757575;
            }
        """)
        button_layout.addWidget(self.clear_btn, 0, 1)
        
        self.remove_btn = QPushButton("✕ Remove Selected")
        self.remove_btn.clicked.connect(self.remove_selected)
        self.remove_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
            }
            QPushButton:hover {
                background-color: #f57c00;
            }
        """)
        button_layout.addWidget(self.remove_btn, 1, 0)
        
        self.select_all_btn = QPushButton("✓ Select All")
        self.select_all_btn.clicked.connect(self.select_all_topics)
        self.select_all_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
            }
            QPushButton:hover {
                background-color: #0b7dda;
            }
        """)
        button_layout.addWidget(self.select_all_btn, 1, 1)
        
        left_layout.addLayout(button_layout)
        
        # 统计信息
        stats_label = QLabel("Statistics")
        stats_label.setStyleSheet("font-weight: bold; margin: 5px; color: #2c3e50;")
        left_layout.addWidget(stats_label)
        
        self.stats_text = QTextEdit()
        self.stats_text.setReadOnly(True)
        self.stats_text.setMaximumHeight(150)
        self.stats_text.setStyleSheet("""
            QTextEdit {
                background-color: white;
                border: 1px solid #ddd;
                border-radius: 4px;
                font-family: 'Courier New', monospace;
                font-size: 10pt;
            }
        """)
        left_layout.addWidget(self.stats_text)
        
        # 右侧绘图面板
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        # Matplotlib图形
        plt.style.use('seaborn-v0_8-darkgrid')
        self.fig, self.ax = plt.subplots(figsize=(10, 7))
        self.canvas = FigureCanvas(self.fig)
        self.toolbar = NavigationToolbar(self.canvas, self)
        
        right_layout.addWidget(self.toolbar)
        right_layout.addWidget(self.canvas)
        
        # 添加左右面板到分割器
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setSizes([500, 900])
        
        self.setCentralWidget(splitter)
    
    def setup_timers(self):
        # 绘图定时器
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(100)  # 10 Hz更新频率
        
        # 统计信息定时器
        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self.update_stats)
        self.stats_timer.start(1000)  # 1 Hz更新频率
    
    def add_topic(self, topic_name, topic_type):
        """添加话题到监视列表"""
        # 检查是否已存在
        for i in range(self.topic_list.count()):
            item = self.topic_list.item(i)
            widget = self.topic_list.itemWidget(item)
            if widget and widget.topic_name == topic_name:
                return
        
        # 创建列表项
        index = self.topic_list.count()
        item_widget = TopicListItem(index, topic_name, topic_type)
        list_item = QListWidgetItem()
        list_item.setSizeHint(item_widget.sizeHint())
        
        self.topic_list.addItem(list_item)
        self.topic_list.setItemWidget(list_item, item_widget)
        
        # 订阅话题
        if self.topic_manager.subscribe(topic_name, topic_type):
            print(f"Subscribed: {topic_name}")
    
    def remove_selected(self):
        """移除选中的话题"""
        selected_items = self.topic_list.selectedItems()
        for item in selected_items:
            widget = self.topic_list.itemWidget(item)
            if widget:
                self.topic_manager.unsubscribe(widget.topic_name)
            self.topic_list.takeItem(self.topic_list.row(item))
        
        # 重新编号
        self.renumber_topics()
    
    def select_all_topics(self):
        """选择所有话题"""
        for i in range(self.topic_list.count()):
            item = self.topic_list.item(i)
            widget = self.topic_list.itemWidget(item)
            if widget:
                widget.visible_checkbox.setChecked(True)
    
    def renumber_topics(self):
        """重新编号所有话题"""
        for i in range(self.topic_list.count()):
            item = self.topic_list.item(i)
            widget = self.topic_list.itemWidget(item)
            if widget:
                widget.index = i
                widget.label.setText(f"[{i}] {widget.topic_name.split('/')[-1][:20]}")
    
    def clear_data(self):
        """清除所有数据"""
        self.topic_manager.clear_data()
        self.plot_start_time = None
    
    def toggle_plotting(self):
        """切换绘图状态"""
        self.is_plotting = not self.is_plotting
        if self.is_plotting:
            self.plot_btn.setText("⏸ Stop Plotting")
            self.plot_btn.setStyleSheet("""
                QPushButton {
                    background-color: #2196F3;
                    color: white;
                }
                QPushButton:hover {
                    background-color: #0b7dda;
                }
            """)
            self.plot_start_time = time.time()
        else:
            self.plot_btn.setText("▶ Start Plotting")
            self.plot_btn.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                }
                QPushButton:hover {
                    background-color: #45a049;
                }
            """)
    
    def update_plot(self):
        """更新绘图 - 在同一个图上显示所有可见的话题"""
        if not self.is_plotting:
            return
        
        # 清除图形
        self.ax.clear()
        
        if not self.plot_start_time:
            self.plot_start_time = time.time()
        
        # 绘制所有可见的话题
        has_visible_topics = False
        
        for i in range(self.topic_list.count()):
            item = self.topic_list.item(i)
            widget = self.topic_list.itemWidget(item)
            if widget and widget.visible:
                has_visible_topics = True
                
                # 获取数据
                timestamps, values = self.topic_manager.get_data(widget.topic_name, max_points=1000)
                if not timestamps or len(timestamps) < 2:
                    continue
                
                # 应用缩放和偏移
                if values:
                    values = [(v + widget.offset) * widget.multiply for v in values]
                
                # 转换为相对时间
                if self.plot_start_time:
                    relative_times = [t - self.plot_start_time for t in timestamps]
                else:
                    relative_times = [t - timestamps[0] for t in timestamps]
                
                # 选择颜色
                if widget.color:
                    color = widget.color.name()
                else:
                    # 使用默认颜色循环
                    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
                    color_index = widget.index % len(colors)
                    color = colors[color_index]
                
                # 截断话题名作为标签
                label = widget.topic_name.split('/')[-1]
                if len(label) > 20:
                    label = label[:20] + "..."
                
                # 绘制曲线
                self.ax.plot(relative_times, values, 
                           label=f"{label}", 
                           color=color, 
                           linewidth=2,
                           alpha=0.8)
        
        if not has_visible_topics:
            # 如果没有可见的话题，显示提示
            self.ax.text(0.5, 0.5, 'No visible topic data\nCheck topics in left panel', 
                        horizontalalignment='center', verticalalignment='center',
                        transform=self.ax.transAxes, fontsize=14, color='gray')
        else:
            # 设置图形属性
            self.ax.set_xlabel("Time (seconds)", fontsize=12)
            self.ax.set_ylabel("Value", fontsize=12)
            self.ax.set_title("ROS2 Real-time Multi-Curve Plot", fontsize=14, fontweight='bold', pad=20)
            
            # 添加图例
            self.ax.legend(loc='upper right', fontsize=10, framealpha=0.9)
        
        # 添加网格
        self.ax.grid(True, alpha=0.3, linestyle='--')
        
        # 自动调整布局
        self.fig.tight_layout()
        
        # 重绘
        self.canvas.draw()
    
    def update_stats(self):
        """更新统计信息"""
        stats_lines = []
        
        for i in range(self.topic_list.count()):
            item = self.topic_list.item(i)
            widget = self.topic_list.itemWidget(item)
            if widget:
                timestamps, values = self.topic_manager.get_data(widget.topic_name)
                if values and len(values) > 0:
                    # 计算统计信息
                    count = len(values)
                    latest = values[-1]
                    mean_val = np.mean(values) if count > 0 else 0
                    std_val = np.std(values) if count > 1 else 0
                    
                    # 截断话题名
                    topic_name = widget.topic_name.split('/')[-1]
                    if len(topic_name) > 15:
                        topic_name = topic_name[:15] + "..."
                    
                    visible_mark = "✓" if widget.visible else "✗"
                    
                    stats_lines.append(
                        f"{visible_mark} {topic_name:<18} "
                        f"Latest: {latest:8.4f} Mean: {mean_val:8.4f} "
                        f"±{std_val:8.4f} Count: {count}"
                    )
        
        if not stats_lines:
            stats_lines.append("No data available")
        
        self.stats_text.setText("\n".join(stats_lines))
    
    def closeEvent(self, event):
        """关闭窗口时的清理"""
        self.topic_discovery.close_node()
        self.topic_manager.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)
    
    # 设置应用程序样式
    app.setStyle('Fusion')
    
    # 设置全局字体
    font = app.font()
    font.setPointSize(9)
    app.setFont(font)
    
    # 创建主窗口
    window = RealTimePlotter()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
