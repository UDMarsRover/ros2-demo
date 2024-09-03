import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QTextEdit, QLineEdit, QPushButton
import rclpy
from std_msgs.msg import String
from rclpy.qos import qos_profile_system_default
import uuid
import threading
class ChatWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        rclpy.init(args=None)
        self.node_name = "chat_node_" + str(uuid.uuid4()).replace("-", "_")
        
        self.node = rclpy.create_node(self.node_name)
        self.setWindowTitle("Chat Demo")
        self.setGeometry(100, 100, 400, 300)

        # Create a publisher
        self.publisher = self.node.create_publisher(String, 'chat_topic', qos_profile=qos_profile_system_default)

        # Subscribe to the chat_topic
        self.node.create_subscription(String, 'chat_topic', self.callback, qos_profile=qos_profile_system_default)

        # Create a thread to spin rospy
        spin_thread = threading.Thread(target=self.spin_ros)
        spin_thread.start()
        # Create a vertical layout
        layout = QVBoxLayout()

        # Create a text container
        self.text_container = QTextEdit()
        self.text_container.setReadOnly(True)
        layout.addWidget(self.text_container)

        # Create a text entry box
        self.text_entry = QLineEdit()
        layout.addWidget(self.text_entry)

        # Create a send button
        send_button = QPushButton("Send")
        send_button.clicked.connect(self.send_message)
        layout.addWidget(send_button)

        # Create a central widget and set the layout
        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def send_message(self):
        msg = String()
        msg.data = self.text_entry.text()
        self.publisher.publish(msg)
        # self.append_text(msg.data)
        self.text_entry.clear()
    
    def append_text(self, text):
        self.text_container.append(text)

    def callback(self, msg):
        self.append_text(msg.data)

    def spin_ros(self):
        rclpy.spin(self.node)

def main():
    app = QApplication(sys.argv)
    window = ChatWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()