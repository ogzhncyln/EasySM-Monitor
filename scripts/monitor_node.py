#!/usr/bin/env python3
import rospy
import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QObject, pyqtSignal
from easysm_monitor import StateMachineEditor
from std_msgs.msg import String

class GuiUpdater(QObject):
    clear_signal = pyqtSignal()
    highlight_signal = pyqtSignal(str)

class MonitorNode:
    def __init__(self, editor, monitor_topic, gui_updater):
        self.editor = editor
        self.gui_updater = gui_updater
        self.sub = rospy.Subscriber(monitor_topic, String, self.callback, queue_size=10)
        self.gui_updater.clear_signal.connect(self.clear_highlights)
        self.gui_updater.highlight_signal.connect(self.exec_highlight)

    def callback(self, msg):
        text = msg.data.strip()
        if text == "clear":
            self.gui_updater.clear_signal.emit()
        elif text.startswith("exec//"):
            _, name = text.split("//", 1)
            self.gui_updater.highlight_signal.emit(name)
        else:
            print(f"Unknown message: {text}")

    def clear_highlights(self):
        for node in self.editor.nodes:
            node.setHighlight(False)
        for tr in self.editor.transitions:
            tr.setHighlight(False)

    def exec_highlight(self, name):
        found = False
        for node in self.editor.nodes:
            if node.title == name:
                node.setHighlight(True)
                found = True
                return
        for transition in self.editor.transitions:
            if transition.name == name:
                transition.setHighlight(True)
                found = True
                return
        if not found:
            print(f"'{name}' not found.")

def main():
    rospy.init_node('easysm_monitor_node', anonymous=True)
    app = QApplication(sys.argv)
    editor = StateMachineEditor()
    file_path = rospy.get_param('~tree_file', '')
    monitor_topic = rospy.get_param('~monitor_topic', '/monitor_cmd')
    if file_path:
        editor.loadFromFile(file_path)
    editor.show()
    gui_updater = GuiUpdater()
    monitor = MonitorNode(editor, monitor_topic, gui_updater)
    timer = rospy.Timer(rospy.Duration(0.1), lambda event: None)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()