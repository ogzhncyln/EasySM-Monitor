import sys
from PyQt5.QtWidgets import (
    QApplication, QGraphicsView, QGraphicsScene, QGraphicsItem, QGraphicsEllipseItem,
    QGraphicsRectItem, QGraphicsTextItem, QGraphicsPathItem, QFileDialog, QMessageBox
)
from PyQt5.QtGui import QPen, QColor, QPainterPath, QPainter
from PyQt5.QtCore import Qt, QPointF, QTimer

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
        self.setBrush(QColor(80, 80, 80))
        self.setPen(QPen(Qt.white, 2))
        self.input_port = Port(self, self.rect().width() / 2, 0)
        self.output_port = Port(self, self.rect().width() / 2, self.rect().height())

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionChange:
            self.input_port.notifyConnections()
            self.output_port.notifyConnections()
        return super().itemChange(change, value)

    def setHighlight(self, highlight=True):
        if highlight:
            self.setPen(QPen(QColor(0, 255, 0), 2))
        else:
            self.setPen(QPen(Qt.white, 2))

class TransitionLine(QGraphicsPathItem):
    def __init__(self, start_state, end_state, name=None):
        super().__init__()
        self.start_port = start_state.output_port
        self.end_port = end_state.input_port
        self.name = name
        pen = QPen(QColor(255, 255, 255, 150), 2)
        self.setPen(pen)
        self.setZValue(1)
        self.setOpacity(0.7)
        self.start_port.connected_transitions.append(self)
        self.end_port.connected_transitions.append(self)
        self.updatePath()

    def updatePath(self):
        start = self.start_port.scenePos()
        end = self.end_port.scenePos()
        path = QPainterPath(start)
        dx = (end.x() - start.x()) * 0.5
        ctrl1 = QPointF(start.x() + dx, start.y())
        ctrl2 = QPointF(end.x() - dx, end.y())
        path.cubicTo(ctrl1, ctrl2, end)
        self.setPath(path)

    def setHighlight(self, highlight=True):
        if highlight:
            pen = QPen(QColor(0, 255, 0, 200), 2)
            self.setPen(pen)
            self.setOpacity(1.0)
        else:
            pen = QPen(QColor(255, 255, 255, 150), 2)
            self.setPen(pen)
            self.setOpacity(0.7)

class StateMachineEditor(QGraphicsView):
    def __init__(self):
        super().__init__()
        self.setRenderHint(QPainter.Antialiasing)
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        self.setSceneRect(0, 0, 2000, 2000)
        self.setBackgroundBrush(QColor(30, 30, 30))
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.nodes = []
        self.transitions = []
        self.timer = QTimer()
        self.timer.timeout.connect(self.updateTransitions)
        self.timer.start(30)

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
        visible_rect = self.mapToScene(self.viewport().rect()).boundingRect()
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

    def wheelEvent(self, event):
        zoom_factor = 1.15
        if event.angleDelta().y() > 0:
            self.scale(zoom_factor, zoom_factor)
        else:
            self.scale(1 / zoom_factor, 1 / zoom_factor)

    def mousePressEvent(self, event):
        if event.button() == Qt.RightButton:
            self.setDragMode(QGraphicsView.ScrollHandDrag)
            self.viewport().setCursor(Qt.OpenHandCursor)
            self._drag_start_pos = event.pos()
        elif event.button() == Qt.LeftButton:
            self.setDragMode(QGraphicsView.RubberBandDrag)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if event.buttons() == Qt.RightButton:
            delta = self._drag_start_pos - event.pos()
            self._drag_start_pos = event.pos()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() + delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() + delta.y())
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            self.setDragMode(QGraphicsView.NoDrag)
            self.viewport().setCursor(Qt.ArrowCursor)
        elif event.button() == Qt.LeftButton:
            self.setDragMode(QGraphicsView.NoDrag)
        super().mouseReleaseEvent(event)

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
                f.write(f"transition//T//{out_name}//{in_name}\n")

def main():
    import sys
    app = QApplication(sys.argv)
    editor = StateMachineEditor()
    editor.show()
    editor.loadFromFile("../data/example.easysm_tree")
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()