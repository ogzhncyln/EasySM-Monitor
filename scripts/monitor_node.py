#!/usr/bin/env python3
import rospy
import sys
import os
from PyQt5.QtWidgets import QApplication, QSplashScreen, QLabel
from PyQt5.QtCore import QObject, pyqtSignal, QTimer, Qt
from PyQt5.QtGui import QPixmap, QMovie
from easysm_monitor import StateMachineEditor
from std_msgs.msg import String

class GuiUpdater(QObject):
    clear_signal = pyqtSignal()
    highlight_signal = pyqtSignal(str)
    log_signal = pyqtSignal(str, str, str, str) 

class MonitorNode:
    def __init__(self, editor, monitor_topic, gui_updater):
        self.editor = editor
        self.gui_updater = gui_updater
        self.sub = rospy.Subscriber(monitor_topic, String, self.callback, queue_size=10)
        self.gui_updater.clear_signal.connect(self.clear_highlights)
        self.gui_updater.highlight_signal.connect(self.exec_highlight)
        self.gui_updater.log_signal.connect(self.editor.addLog)
        self.current_exec_item = None  
        self.previous_exec_item = None  
        self.current_state_name = "Unknown" 

    def callback(self, msg):
        text = msg.data.strip()
        if text == "clear":
            self.gui_updater.clear_signal.emit()
        elif text.startswith("exec//"):
            _, name = text.split("//", 1)
            self.current_state_name = name  
            self.gui_updater.highlight_signal.emit(name)
        elif text.startswith("log//"):
            parts = text.split("//", 3)
            if len(parts) >= 4:
                log_type = parts[1].lower() 
                message = parts[2]
                log_state_name = parts[3] 
                self.gui_updater.log_signal.emit(log_type, self.current_state_name, message, log_state_name)
            elif len(parts) >= 3:
                log_type = parts[1].lower()
                message = parts[2]
                self.gui_updater.log_signal.emit(log_type, self.current_state_name, message, "")
            else:
                print(f"Invalid log format: {text}")
        else:
            print(f"Unknown message: {text}")

    def clear_highlights(self):
        for node in self.editor.nodes:
            node.setHighlight(False)
        for tr in self.editor.transitions:
            tr.setHighlight(False)
        self.current_exec_item = None
        self.previous_exec_item = None

    def exec_highlight(self, name):
        found_item = None
        
        for node in self.editor.nodes:
            if node.title == name:
                found_item = node
                break
        
        if not found_item:
            for transition in self.editor.transitions:
                if transition.name == name:
                    found_item = transition
                    break
        
        if found_item:
            if self.current_exec_item:
                self.previous_exec_item = self.current_exec_item
                if hasattr(self.previous_exec_item, 'setHighlight'):
                    self.previous_exec_item.setHighlight(True, color="green")
            
            self.current_exec_item = found_item
            if hasattr(self.current_exec_item, 'setHighlight'):
                self.current_exec_item.setHighlight(True, color="yellow")
            
            self.current_state_name = name
        else:
            print(f"'{name}' not found.")

    def shutdown(self):
        """Node'u temizle"""
        try:
            if hasattr(self, 'sub') and self.sub:
                self.sub.unregister()
        except Exception as e:
            print(f"Subscriber cleanup error: {e}")
        
        try:
            if hasattr(self, 'gui_updater'):
                self.gui_updater.clear_signal.disconnect()
                self.gui_updater.highlight_signal.disconnect()
                self.gui_updater.log_signal.disconnect()
        except Exception as e:
            print(f"Signal cleanup error: {e}")

class SplashScreen(QSplashScreen):
    def __init__(self):
        super().__init__()
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        gif_path = os.path.join(script_dir, '..', 'data', 'easysm.gif')
        
        if os.path.exists(gif_path):
            self.movie = QMovie(gif_path)
            
            self.label = QLabel()
            self.label.setMovie(self.movie)
            self.label.setAlignment(Qt.AlignCenter)
            
            self.movie.frameChanged.connect(self.updateFrame)
            self.movie.start()
            
            QTimer.singleShot(100, self.setupSize)
        else:
            pixmap = QPixmap(400, 300)
            pixmap.fill(Qt.black)
            self.setPixmap(pixmap)
    
    def setupSize(self):
        if hasattr(self, 'movie'):
            size = self.movie.currentPixmap().size()
            self.setFixedSize(size)
            
            screen = QApplication.primaryScreen().geometry()
            x = (screen.width() - size.width()) // 2
            y = (screen.height() - size.height()) // 2
            self.move(x, y)
    
    def updateFrame(self):
        if hasattr(self, 'movie'):
            pixmap = self.movie.currentPixmap()
            self.setPixmap(pixmap)
    
    def closeEvent(self, event):
        if hasattr(self, 'movie'):
            self.movie.stop()
        super().closeEvent(event)

def main():
    rospy.init_node('easysm_monitor_node', anonymous=True)
    
    app = QApplication(sys.argv)
    
    splash = SplashScreen()
    splash.show()
    
    editor = StateMachineEditor()
    file_path = rospy.get_param('~tree_file', '')
    monitor_topic = rospy.get_param('~monitor_topic', '/monitor_cmd')
    
    if file_path:
        try:
            editor.loadFromFile(file_path)
        except Exception as e:
            print(f"File loading error: {e}")
    
    gui_updater = GuiUpdater()
    monitor = MonitorNode(editor, monitor_topic, gui_updater)
    
    editor.monitor_node = monitor
    
    from PyQt5.QtCore import QTimer
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: None)  
    ros_timer.start(100) 
    
    def show_main_window():
        splash.close()
        editor.show()
    
    QTimer.singleShot(3000, show_main_window)
    
    def cleanup():
        try:
            ros_timer.stop()
            monitor.shutdown()
            rospy.signal_shutdown("Application closing")
        except:
            pass
    
    app.aboutToQuit.connect(cleanup)
    
    try:
        exit_code = app.exec_()
        cleanup()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        cleanup()
        sys.exit(0)
    except Exception as e:
        print(f"Application error: {e}")
        cleanup()
        sys.exit(1)

if __name__ == '__main__':
    main()