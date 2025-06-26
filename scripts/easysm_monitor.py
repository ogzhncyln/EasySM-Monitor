import sys
import rospy
import os
from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication, QGraphicsView, QGraphicsScene, QGraphicsItem, QGraphicsEllipseItem,
    QGraphicsRectItem, QGraphicsTextItem, QGraphicsPathItem, QFileDialog, QMessageBox,
    QWidget, QVBoxLayout, QSplitter, QTextEdit, QToolTip, QSplashScreen, QLabel
)
from PyQt5.QtGui import QPen, QColor, QPainterPath, QPainter, QTextCursor, QFont, QBrush, QMovie, QPixmap
from PyQt5.QtCore import Qt, QPointF, QTimer, pyqtSignal, QRectF, QLineF

class Port(QGraphicsEllipseItem):
    def __init__(self, parent, x_offset, y_offset):
        super().__init__(-5, -5, 10, 10, parent)
        self.setBrush(QColor("lightgray"))
        self.setPen(QPen(Qt.black))
        self.setPos(x_offset, y_offset)
        self.connected_transitions = []

    def notifyConnections(self):
        for t in self.connected_transitions:
            t.updatePath()

class StateNode(QGraphicsRectItem):
    def __init__(self, title="State", width=120, height=80):
        super().__init__(0, 0, width, height)
        self.setFlags(
            QGraphicsItem.ItemIsMovable |
            QGraphicsItem.ItemIsSelectable |
            QGraphicsItem.ItemSendsGeometryChanges
        )
        self.title = title
        self.text = QGraphicsTextItem(title, self)
        self.text.setDefaultTextColor(Qt.white)
        self.text.setPos(10, 10)
        
        self.corner_radius = 15
        self.base_color = QColor(80, 80, 80, 180)
        self.setBrush(QBrush(self.base_color))
        self.setPen(QPen(Qt.white, 2))
        
        self.input_port = Port(self, self.rect().width() / 2, 0)
        self.output_port = Port(self, self.rect().width() / 2, self.rect().height())
        
        self.setAcceptHoverEvents(True)
        self.current_pen_color = QColor(255, 255, 255)
        
        self.tooltip_delay = 300
        self.last_event = None
        self.tooltip_active = False

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionChange:
            self.input_port.notifyConnections()
            self.output_port.notifyConnections()
        return super().itemChange(change, value)

    def setHighlight(self, highlight=True, color="green"):
        if highlight:
            if color == "yellow":
                self.setBrush(QBrush(QColor(255, 255, 0, 100)))
                self.setPen(QPen(QColor(255, 255, 0), 3))
                self.current_pen_color = QColor(255, 255, 0)
            elif color == "green":
                self.setBrush(QBrush(QColor(0, 255, 0, 100)))
                self.setPen(QPen(QColor(0, 255, 0), 2))
                self.current_pen_color = QColor(0, 255, 0)
            else:
                self.setBrush(QBrush(QColor(0, 255, 0, 100)))
                self.setPen(QPen(QColor(0, 255, 0), 2))
                self.current_pen_color = QColor(0, 255, 0)
        else:
            self.setBrush(QBrush(self.base_color))
            self.setPen(QPen(Qt.white, 2))
            self.current_pen_color = QColor(255, 255, 255)
        
        self.update()

    def hoverEnterEvent(self, event):
        self.tooltip_active = True
        self.last_event = event
        self.showStateTooltip(event)
        super().hoverEnterEvent(event)

    def hoverMoveEvent(self, event):
        if self.tooltip_active:
            self.last_event = event
            self.showStateTooltip(event)
        super().hoverMoveEvent(event)

    def hoverLeaveEvent(self, event):
        self.tooltip_active = False
        QToolTip.hideText()
        super().hoverLeaveEvent(event)

    def showDelayedStateTooltip(self):
        pass

    def showStateTooltip(self, event):
        if self.title and self.tooltip_active:
            try:
                color = self.current_pen_color
                color_name = f"rgb({color.red()}, {color.green()}, {color.blue()})"
                
                tooltip_html = f'<span style="color: {color_name}; font-weight: bold;">{self.title}</span>'
                
                scene_pos = event.scenePos()
                view = self.scene().views()[0] if self.scene().views() else None
                if view:
                    view_pos = view.mapFromScene(scene_pos)
                    global_pos = view.mapToGlobal(view_pos)
                    global_pos.setX(global_pos.x() + 15)
                    global_pos.setY(global_pos.y() + 10)
                    
                    QToolTip.showText(global_pos, tooltip_html)
            except Exception as e:
                print(f"State tooltip error: {e}")

    def paint(self, painter, option, widget):
        painter.setRenderHint(QPainter.Antialiasing, True)
        
        rect = self.rect()
        
        path = QPainterPath()
        path.addRoundedRect(rect, self.corner_radius, self.corner_radius)
        
        brush = self.brush()
        if brush.style() != Qt.NoBrush:
            painter.fillPath(path, brush)
        
        pen = self.pen()
        if pen.style() != Qt.NoPen:
            painter.setPen(pen)
            painter.drawPath(path)

class TransitionLine(QGraphicsPathItem):
    def __init__(self, start_state, end_state, name=None):
        super().__init__()
        self.start_port = start_state.output_port
        self.end_port = end_state.input_port
        self.name = name if name else "Transition"
        pen = QPen(QColor(255, 255, 255, 150), 2)
        self.setPen(pen)
        self.setZValue(1)
        self.setOpacity(0.7)
        self.start_port.connected_transitions.append(self)
        self.end_port.connected_transitions.append(self)
        self.updatePath()
        
        self.setAcceptHoverEvents(True)
        self.current_pen_color = QColor(255, 255, 255, 150)
        
        self.tooltip_delay = 300
        self.last_event = None
        self.tooltip_active = False

    def updatePath(self):
        start = self.start_port.scenePos()
        end = self.end_port.scenePos()
        
        path = self.calculateSmartPath(start, end)
        self.setPath(path)
    
    def calculateSmartPath(self, start, end):
        path = QPainterPath(start)
        
        blocking_obstacles = self.getBlockingObstacles(start, end)
        
        if not blocking_obstacles:
            dx = (end.x() - start.x()) * 0.5
            ctrl1 = QPointF(start.x() + dx, start.y())
            ctrl2 = QPointF(end.x() - dx, end.y())
            path.cubicTo(ctrl1, ctrl2, end)
        else:
            self.createAvoidancePath(path, start, end, blocking_obstacles)
        
        return path
    
    def getBlockingObstacles(self, start, end):
        blocking = []
        if not self.scene():
            return blocking
        
        start_state = self.start_port.parentItem()
        end_state = self.end_port.parentItem()
        
        for item in self.scene().items():
            if isinstance(item, StateNode):
                if item == start_state or item == end_state:
                    continue
                    
                rect = item.sceneBoundingRect()
                if self.lineIntersectsRect(start, end, rect):
                    blocking.append(item)
        
        return blocking
    
    def createAvoidancePath(self, path, start, end, obstacles):
        if not obstacles:
            return
            
        main_obstacle = self.findMainObstacle(start, end, obstacles)
        if not main_obstacle:
            return
            
        obstacle_rect = main_obstacle.sceneBoundingRect()
        
        dx = abs(end.x() - start.x())
        dy = abs(end.y() - start.y())
        
        if dx > dy:
            self.createVerticalAvoidance(path, start, end, obstacle_rect)
        else:
            self.createHorizontalAvoidance(path, start, end, obstacle_rect)
    
    def findMainObstacle(self, start, end, obstacles):
        """En problemli engeli bulur"""
        if not obstacles:
            return None
            
        mid_x = (start.x() + end.x()) / 2
        mid_y = (start.y() + end.y()) / 2
        
        closest = None
        min_dist = float('inf')
        
        for obstacle in obstacles:
            center = obstacle.sceneBoundingRect().center()
            dist = ((center.x() - mid_x) ** 2 + (center.y() - mid_y) ** 2) ** 0.5
            if dist < min_dist:
                min_dist = dist
                closest = obstacle
                
        return closest
    
    def createVerticalAvoidance(self, path, start, end, obstacle_rect):
        """Yukarı/aşağı kaçınma"""
        top_route = obstacle_rect.top() - 30
        bottom_route = obstacle_rect.bottom() + 30
        
        start_y = start.y()
        end_y = end.y()
        avg_y = (start_y + end_y) / 2
        
        if abs(avg_y - top_route) < abs(avg_y - bottom_route):
            route_y = top_route
        else:
            route_y = bottom_route
            
        mid_x = (start.x() + end.x()) / 2
        waypoint = QPointF(mid_x, route_y)
        
        dx1 = (waypoint.x() - start.x()) * 0.5
        ctrl1 = QPointF(start.x() + dx1, start.y())
        ctrl2 = QPointF(waypoint.x() - dx1, waypoint.y())
        path.cubicTo(ctrl1, ctrl2, waypoint)
        
        dx2 = (end.x() - waypoint.x()) * 0.5
        ctrl3 = QPointF(waypoint.x() + dx2, waypoint.y())
        ctrl4 = QPointF(end.x() - dx2, end.y())
        path.cubicTo(ctrl3, ctrl4, end)
    
    def createHorizontalAvoidance(self, path, start, end, obstacle_rect):
        left_route = obstacle_rect.left() - 30
        right_route = obstacle_rect.right() + 30
        
        start_x = start.x()
        end_x = end.x()
        avg_x = (start_x + end_x) / 2
        
        if abs(avg_x - left_route) < abs(avg_x - right_route):
            route_x = left_route
        else:
            route_x = right_route
            
        mid_y = (start.y() + end.y()) / 2
        waypoint = QPointF(route_x, mid_y)
        
        dy1 = (waypoint.y() - start.y()) * 0.5
        ctrl1 = QPointF(start.x(), start.y() + dy1)
        ctrl2 = QPointF(waypoint.x(), waypoint.y() - dy1)
        path.cubicTo(ctrl1, ctrl2, waypoint)
        
        dy2 = (end.y() - waypoint.y()) * 0.5
        ctrl3 = QPointF(waypoint.x(), waypoint.y() + dy2)
        ctrl4 = QPointF(end.x(), end.y() - dy2)
        path.cubicTo(ctrl3, ctrl4, end)
    
    def lineIntersectsRect(self, start, end, rect):
        start_state = self.start_port.parentItem()
        end_state = self.end_port.parentItem()
        
        for item in self.scene().items():
            if isinstance(item, StateNode) and item.sceneBoundingRect() == rect:
                if item == start_state or item == end_state:
                    return self.checkRealIntersection(start, end, rect, item)
        
        from PyQt5.QtCore import QLineF
        line = QLineF(start, end)
        
        edges = [
            QLineF(rect.topLeft(), rect.topRight()),       
            QLineF(rect.topRight(), rect.bottomRight()),    
            QLineF(rect.bottomRight(), rect.bottomLeft()), 
            QLineF(rect.bottomLeft(), rect.topLeft())      
        ]
        
        for edge in edges:
            intersection_point = QPointF()
            if line.intersect(edge, intersection_point) == QLineF.BoundedIntersection:
                return True
                
        return False
    
    def checkRealIntersection(self, start, end, rect, state_item):
        """Bağlantılı state için gerçek intersection kontrolü"""
        from PyQt5.QtCore import QLineF
        line = QLineF(start, end)
        
        start_dist_to_rect = min(
            abs(start.x() - rect.left()),
            abs(start.x() - rect.right()),
            abs(start.y() - rect.top()),
            abs(start.y() - rect.bottom())
        )
        
        end_dist_to_rect = min(
            abs(end.x() - rect.left()),
            abs(end.x() - rect.right()),
            abs(end.y() - rect.top()),
            abs(end.y() - rect.bottom())
        )
        
        if start_dist_to_rect < 10 or end_dist_to_rect < 10:
            return False
            
        edges = [
            QLineF(rect.topLeft(), rect.topRight()),
            QLineF(rect.topRight(), rect.bottomRight()),
            QLineF(rect.bottomRight(), rect.bottomLeft()),
            QLineF(rect.bottomLeft(), rect.topLeft())
        ]
        
        for edge in edges:
            intersection_point = QPointF()
            if line.intersect(edge, intersection_point) == QLineF.BoundedIntersection:
                return True
                
        return False

    def setHighlight(self, highlight=True, color="green"):
        if highlight:
            if color == "yellow":
                pen = QPen(QColor(255, 255, 0, 255), 3)  
                self.current_pen_color = QColor(255, 255, 0, 255)
                self.setPen(pen)
                self.setOpacity(1.0)
            elif color == "green":
                pen = QPen(QColor(0, 255, 0, 200), 2)   
                self.current_pen_color = QColor(0, 255, 0, 200)
                self.setPen(pen)
                self.setOpacity(1.0)
            else:
                pen = QPen(QColor(0, 255, 0, 200), 2)    
                self.current_pen_color = QColor(0, 255, 0, 200)
                self.setPen(pen)
                self.setOpacity(1.0)
        else:
            pen = QPen(QColor(255, 255, 255, 150), 2)
            self.current_pen_color = QColor(255, 255, 255, 150)
            self.setPen(pen)
            self.setOpacity(0.7)

    def hoverEnterEvent(self, event):
        self.tooltip_active = True
        self.last_event = event
        self.showTooltip(event)
        super().hoverEnterEvent(event)

    def hoverMoveEvent(self, event):
        if self.tooltip_active:
            self.last_event = event
            self.showTooltip(event)
        super().hoverMoveEvent(event)

    def hoverLeaveEvent(self, event):
        self.tooltip_active = False
        QToolTip.hideText()
        super().hoverLeaveEvent(event)

    def showDelayedTooltip(self):
        pass

    def showTooltip(self, event):
        if self.name and self.tooltip_active:
            try:
                color = self.current_pen_color
                color_name = self.getColorName(color)
                
                tooltip_html = f'<span style="color: {color_name}; font-weight: bold;">{self.name}</span>'
                
                scene_pos = event.scenePos()
                view = self.scene().views()[0] if self.scene().views() else None
                if view:
                    view_pos = view.mapFromScene(scene_pos)
                    global_pos = view.mapToGlobal(view_pos)
                    global_pos.setX(global_pos.x() + 15)
                    global_pos.setY(global_pos.y() + 10)
                    
                    QToolTip.showText(global_pos, tooltip_html)
            except Exception as e:
                print(f"Tooltip error: {e}")

    def getColorName(self, color):
        return f"rgb({color.red()}, {color.green()}, {color.blue()})"

class StateMachineEditor(QWidget):
    def __init__(self):
        super().__init__()
        self.setupUI()
        
    def setupUI(self):
        layout = QVBoxLayout(self)
        
        splitter = QSplitter(Qt.Vertical)
        
        self.graphics_view = QGraphicsView()
        self.graphics_view.setRenderHint(QPainter.Antialiasing)
        self.scene = QGraphicsScene()
        self.graphics_view.setScene(self.scene)
        self.graphics_view.setSceneRect(0, 0, 2000, 2000)
        self.graphics_view.setBackgroundBrush(QColor(30, 30, 30))
        self.graphics_view.setDragMode(QGraphicsView.RubberBandDrag)
        
        self.log_widget = QTextEdit()
        self.log_widget.setReadOnly(True)
        self.log_widget.setMaximumHeight(200)  
        self.log_widget.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: white;
                border: 1px solid #3c3c3c;
                font-family: 'Courier New', monospace;
                font-size: 10px;
            }
        """)
        
        splitter.addWidget(self.graphics_view)
        splitter.addWidget(self.log_widget)
        
        splitter.setSizes([800, 200])
        
        layout.addWidget(splitter)
        
        QToolTip.setFont(QFont('Arial', 12, QFont.Bold))
        app = QApplication.instance()
        if app:
            app.setStyleSheet("""
                QToolTip {
                    background-color: rgba(50, 50, 50, 220);
                    color: white;
                    border: 2px solid #666;
                    border-radius: 8px;
                    padding: 5px;
                    font-size: 12px;
                    font-weight: bold;
                }
            """)
            app.setAttribute(Qt.AA_DisableWindowContextHelpButton, True)
        
        
        self.nodes = []
        self.transitions = []
        self.timer = QTimer()
        self.timer.timeout.connect(self.updateTransitions)
        self.timer.start(30)
        
        self.graphics_view.wheelEvent = self.customWheelEvent
        self.graphics_view.mousePressEvent = self.customMousePressEvent
        self.graphics_view.mouseMoveEvent = self.customMouseMoveEvent
        self.graphics_view.mouseReleaseEvent = self.customMouseReleaseEvent
        self.graphics_view.drawBackground = self.drawBackground
        self.setFocusPolicy(Qt.StrongFocus)  

    def createExample(self):
        node1 = StateNode("Idle")
        node1.setPos(100, 100)
        self.scene.addItem(node1)
        node2 = StateNode("Move")
        node2.setPos(400, 300)
        self.scene.addItem(node2)
        node3 = StateNode("Node3")
        node3.setPos(600, 300)
        self.scene.addItem(node3)
        self.nodes += [node1, node2,node3]
        transition = TransitionLine(node1, node2)
        self.scene.addItem(transition)
        self.transitions.append(transition)
        transition1 = TransitionLine(node1, node3)
        self.scene.addItem(transition1)
        self.transitions.append(transition1)
        transition2 = TransitionLine(node3, node2)
        self.scene.addItem(transition2)
        self.transitions.append(transition2)

    def updateTransitions(self):
        for transition in self.transitions:
            transition.updatePath()

    def drawBackground(self, painter, rect):
        painter.fillRect(rect, QColor(40, 40, 40))
        grid_size = 20
        large_grid_size = grid_size * 10
        visible_rect = self.graphics_view.mapToScene(self.graphics_view.viewport().rect()).boundingRect()
        left = int(visible_rect.left()) - (int(visible_rect.left()) % grid_size) - grid_size
        right = int(visible_rect.right()) + grid_size
        top = int(visible_rect.top()) - (int(visible_rect.top()) % grid_size) - grid_size
        bottom = int(visible_rect.bottom()) + grid_size
        pen_light = QPen(QColor(60, 60, 60))
        pen_light.setCosmetic(True)
        painter.setPen(pen_light)
        x = left
        while x <= right:
            if x % large_grid_size != 0:
                painter.drawLine(x, top, x, bottom)
            x += grid_size
        y = top
        while y <= bottom:
            if y % large_grid_size != 0:
                painter.drawLine(left, y, right, y)
            y += grid_size
        pen_dark = QPen(QColor(0, 0, 0), 2)
        pen_dark.setCosmetic(True)
        painter.setPen(pen_dark)
        x = left
        while x <= right:
            if x % large_grid_size == 0:
                painter.drawLine(x, top, x, bottom)
            x += grid_size
        y = top
        while y <= bottom:
            if y % large_grid_size == 0:
                painter.drawLine(left, y, right, y)
            y += grid_size

    def customWheelEvent(self, event):
        zoom_factor = 1.15
        if event.angleDelta().y() > 0:
            self.graphics_view.scale(zoom_factor, zoom_factor)
        else:
            self.graphics_view.scale(1 / zoom_factor, 1 / zoom_factor)

    def customMousePressEvent(self, event):
        if event.button() == Qt.RightButton:
            self.graphics_view.setDragMode(QGraphicsView.ScrollHandDrag)
            self.graphics_view.viewport().setCursor(Qt.OpenHandCursor)
            self._drag_start_pos = event.pos()
        elif event.button() == Qt.LeftButton:
            self.graphics_view.setDragMode(QGraphicsView.RubberBandDrag)
        
        QGraphicsView.mousePressEvent(self.graphics_view, event)

    def customMouseMoveEvent(self, event):
        if event.buttons() == Qt.RightButton and hasattr(self, '_drag_start_pos'):
            delta = self._drag_start_pos - event.pos()
            self._drag_start_pos = event.pos()
            self.graphics_view.horizontalScrollBar().setValue(
                self.graphics_view.horizontalScrollBar().value() + delta.x())
            self.graphics_view.verticalScrollBar().setValue(
                self.graphics_view.verticalScrollBar().value() + delta.y())
        else:
            QGraphicsView.mouseMoveEvent(self.graphics_view, event)

    def customMouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            self.graphics_view.setDragMode(QGraphicsView.RubberBandDrag)
            self.graphics_view.viewport().setCursor(Qt.ArrowCursor)
        elif event.button() == Qt.LeftButton:
            self.graphics_view.setDragMode(QGraphicsView.RubberBandDrag)
        
        QGraphicsView.mouseReleaseEvent(self.graphics_view, event)

    def loadFromFile(self, filename):
        self.scene.clear()
        self.nodes = []
        self.transitions = []
        node_dict = {}
        with open(filename, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split("//")
                if parts[0] == "node":
                    name = parts[1]
                    x = float(parts[2])
                    y = float(parts[3])
                    node = StateNode(name)
                    node.setPos(x, y)
                    self.scene.addItem(node)
                    self.nodes.append(node)
                    node_dict[name] = node
                elif parts[0] == "transition":
                    tname = parts[1]
                    out_node = node_dict.get(parts[2])
                    in_node = node_dict.get(parts[3])
                    if out_node and in_node:
                        transition = TransitionLine(out_node, in_node, name=tname)
                        self.scene.addItem(transition)
                        self.transitions.append(transition)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_S and (event.modifiers() & Qt.ControlModifier):
            self.saveWithDialog()
        else:
            super().keyPressEvent(event)

    def saveWithDialog(self):
        reply = QMessageBox.question(self, "Save", "Save?", QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            path, _ = QFileDialog.getSaveFileName(self, "Save", "", "EasySM Tree (*.easysm_tree)")
            if path:
                self.saveToFile(path)

    def saveToFile(self, filename):
        with open(filename, "w", encoding="utf-8") as f:
            for node in self.nodes:
                x = node.pos().x()
                y = node.pos().y()
                f.write(f"node//{node.title}//{x}//{y}\n")
            for tr in self.transitions:
                out_name = tr.start_port.parentItem().title
                in_name = tr.end_port.parentItem().title
                f.write(f"transition//{tr.name}//{out_name}//{in_name}\n")

    def addLog(self, log_type, state_name, message, log_state_name=None):
        """Log mesajı ekle"""
        current_time = datetime.now().strftime("%H:%M:%S")
        
        display_state = log_state_name if log_state_name else state_name
        
        if log_type == "log_error":
            color = "#ff4444" 
        elif log_type == "log_warn":
            color = "#ffaa00"  
        else:  
            color = "#ffffff"  
        
        formatted_log = f'<span style="color: {color};">({current_time}) ({display_state}) {message}</span>'
        
        self.log_widget.append(formatted_log)
        
        cursor = self.log_widget.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.log_widget.setTextCursor(cursor)
    
    def clearLogs(self):
        """Tüm logları temizle"""
        self.log_widget.clear()

    def closeEvent(self, event):
        """Pencere kapatılırken çağrılır"""
        try:
            if hasattr(self, 'timer') and self.timer:
                self.timer.stop()
            
            QToolTip.hideText()
            
            if hasattr(self, 'monitor_node'):
                self.monitor_node.shutdown()
            
            if rospy and not rospy.is_shutdown():
                rospy.signal_shutdown("GUI closed")
        except Exception as e:
            print(f"Cleanup error: {e}")
        
        event.accept()

class SplashScreen(QSplashScreen):
    def __init__(self):
        temp_pixmap = QPixmap(400, 300)
        temp_pixmap.fill(QColor(30, 30, 30)) 
        super().__init__(temp_pixmap)
        
        self.setWindowFlags(Qt.SplashScreen | Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        
        self.gif_label = QLabel(self)
        self.gif_label.setAlignment(Qt.AlignCenter)
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        gif_path = os.path.join(script_dir, "../data/easysm.gif")
        
        if os.path.exists(gif_path):
            self.movie = QMovie(gif_path)
            self.gif_label.setMovie(self.movie)
            
            first_frame = self.movie.currentPixmap()
            if not first_frame.isNull():
                gif_size = first_frame.size()
                splash_pixmap = QPixmap(gif_size)
                splash_pixmap.fill(QColor(0, 0, 0, 0)) 
                self.setPixmap(splash_pixmap)
                
                self.gif_label.resize(gif_size)
                self.gif_label.move(0, 0)
            
            self.movie.start()
        else:
            self.gif_label.setText("EasySM Monitor\nLoading...")
            self.gif_label.setStyleSheet("""
                QLabel {
                    color: white;
                    font-size: 18px;
                    font-weight: bold;
                    padding: 20px;
                }
            """)
            self.gif_label.resize(400, 300)
        
        self.center_on_screen()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.close)
        self.timer.start(3000)  
    
    def center_on_screen(self):
        screen = QApplication.desktop().screenGeometry()
        splash_size = self.geometry()
        x = (screen.width() - splash_size.width()) // 2
        y = (screen.height() - splash_size.height()) // 2
        self.move(x, y)
    
    def closeEvent(self, event):
        if hasattr(self, 'movie'):
            self.movie.stop()
        super().closeEvent(event)

def main():
    import sys
    app = QApplication(sys.argv)
    
    splash = SplashScreen()
    splash.show()
    
    editor = StateMachineEditor()
    
    def show_main_window():
        splash.close()
        editor.show()
        editor.loadFromFile("../data/example.easysm_tree")
    
    QTimer.singleShot(3000, show_main_window)
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()