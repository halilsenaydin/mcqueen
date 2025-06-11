import os, time, threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from custom_interfaces.srv import Screen, Buzzer
from mcqueen_teleop.voice_assistant.command_processor import CommandProcessor

from flask import Flask, render_template, request, jsonify
import paho.mqtt.publish as publish

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATE_PATH = os.path.join(CURRENT_DIR, "templates")
STATIC_PATH = os.path.join(CURRENT_DIR, "static")

app = Flask(__name__, template_folder=TEMPLATE_PATH, static_folder=STATIC_PATH)

# Global variables for ROS nodes
buzzer_service_client = None
cmd_vel_node = None
scan_node = None
screen_service_client = None

# Constants for ROS topics and services
BUZZER_SERVICE_NAME = "buzzer"
CONTROL_NODE_NAME = "/ackermann_cont/reference"
SCAN_TOPIC_NAME = "/scan"
SCREEN_SERVICE_NAME = "screen"

# Global variables for controlling the robot
linear_x = 0.0
angular_z = 0.0
gear = 0  # 0: D, 1: R

# Global variables for distance monitoring
distance_data = []

# MQTT configuration
MQTT_BROKER = "192.168.0.28"
MQTT_PORT = 1883
MQTT_TOPIC = "sensor/headlight"

command_processor = CommandProcessor()


def headlight_on_with_mqtt():
    publish.single(MQTT_TOPIC, payload="1", hostname=MQTT_BROKER, port=MQTT_PORT)

    return "Headlight turned ON"


def headlight_off_with_mqtt():
    publish.single(MQTT_TOPIC, payload="0", hostname=MQTT_BROKER, port=MQTT_PORT)

    return "Headlight turned OFF"


def mcqueen_stop():
    global linear_x, angular_z

    linear_x = 0.0
    angular_z = 0.0

    return "McQueen stop"


def mcqueen_move_forward():
    global linear_x, angular_z

    linear_x = 1.0
    angular_z = 0.0

    return "McQueen moving forward"


def mcqueen_move_backward():
    global linear_x, angular_z

    linear_x = -1.0
    angular_z = 0.0

    return "McQueen moving backward"


def mcqueen_draw_circle():
    global linear_x, angular_z

    linear_x = 1.0
    angular_z = 1.0

    return "McQueen drawing a circle"


COMMAND_FUNCTIONS = {
    "ışık_aç": headlight_on_with_mqtt,
    "ışık_kapat": headlight_off_with_mqtt,
    "mcqueen_dur": mcqueen_stop,
    "mcqueen_ileri": mcqueen_move_forward,
    "mcqueen_geri": mcqueen_move_backward,
    "mcqueen_yuvarlak": mcqueen_draw_circle,
}


# CmdVelWebNode is a ROS 2 node that publishes TwistStamped messages to control the robot
class CmdVelWebNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_web_node")

        self._timer = self.create_timer(1.0, self.timer_callback)
        self._publisher = self.create_publisher(TwistStamped, CONTROL_NODE_NAME, 10)

    def timer_callback(self):
        self.publish_cmd()

    def publish_cmd(self):
        global linear_x, angular_z

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z

        self._publisher.publish(msg)
        self.get_logger().info(f"Published: linear_x={linear_x}, angular_z={angular_z}")


# DistanceListener is a ROS 2 node that subscribes to the LaserScan topic
class DistanceListener(Node):
    def __init__(self):
        super().__init__("distance_listener")
        self.subscription = self.create_subscription(
            LaserScan, SCAN_TOPIC_NAME, self.scan_callback, 10
        )

    def scan_callback(self, msg):
        global distance_data

        now = time.strftime("%H:%M:%S")
        dist = msg.ranges[0] if msg.ranges else -1.0  # Default to -1 if no data
        description = "Distance: {:.2f} m".format(dist) if dist >= 0 else "No data"

        distance_data.append(
            {"time": now, "distance": dist, "description": description}
        )

        if len(distance_data) > 20:
            distance_data.pop(0)


# BuzzerServiceClient is a ROS 2 client that calls the Buzzer service
class BuzzerServiceClient(Node):
    def __init__(self):
        super().__init__("buzzer_service_client")

        self.cli = self.create_client(Buzzer, BUZZER_SERVICE_NAME)

    def call_service(self, state: bool):
        if not self.cli.service_is_ready():
            self.get_logger().error("Buzzer service is not available!")

            return None

        req = Buzzer.Request()
        req.state = state

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


# ScreenServiceClient is a ROS 2 client that calls the Screen service
class ScreenServiceClient(Node):
    def __init__(self):
        super().__init__("screen_service_client")

        self.cli = self.create_client(Screen, SCREEN_SERVICE_NAME)

    def call_service(self, pic_id: str):
        if not self.cli.service_is_ready():
            self.get_logger().error("Screen service is not available!")

            return None

        req = Screen.Request()
        req.payload = f"component=p0&property=pic&value={pic_id}"

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/control")
def control_panel():
    return render_template("mcqueen_teleop_control.html")


@app.route("/control/gear", methods=["POST"])
def control_gear():
    global gear

    data = request.get_json()
    gear = data.get("gear")

    return render_template("index.html")


@app.route("/control/accelerate", methods=["POST"])
def control_accelerate():
    global linear_x, gear

    if gear == 0:
        linear_x = 1.0
    else:
        linear_x = -1.0

    return render_template("index.html")


@app.route("/control/brake", methods=["POST"])
def control_brake():
    global linear_x

    linear_x = 0.0

    return render_template("index.html")


@app.route("/control/steer", methods=["POST"])
def control_steer():
    global angular_z

    data = request.get_json()
    angle_deg = data.get("angle", 0.0)

    if -10 <= angle_deg <= 10:
        angular_z = 0.0
    else:
        # Normalize angle: -90° -> -1.0, +90° -> +1.0
        max_deg = 90.0
        angular_z = 1.0 * (angle_deg / max_deg)

    return jsonify(success=True)


@app.route("/distance")
def distance_monitor():
    return render_template("mcqueen_teleop_distance.html")


@app.route("/distances")
def distance_datas():
    global distance_data

    return jsonify(distance_data)


@app.route("/buzzer")
def buzzer_control():
    return render_template("mcqueen_teleop_buzzer.html")


@app.route("/buzzer/on", methods=["POST"])
def buzzer_on():
    try:
        result = buzzer_service_client.call_service(True)
        if result.success:
            return jsonify(success=True)
        else:
            return jsonify(
                success=False, error="ROS servic unsuccessful: " + result.message
            )
    except Exception as e:
        return jsonify(success=False, error=str(e)), 500


@app.route("/buzzer/off", methods=["POST"])
def buzzer_off():
    try:
        result = buzzer_service_client.call_service(False)
        if result.success:
            return jsonify(success=True)
        else:
            return jsonify(
                success=False, error="ROS servic unsuccessful: " + result.message
            )
    except Exception as e:
        return jsonify(success=False, error=str(e)), 500


@app.route("/headlight")
def headlight_control():
    return render_template("mcqueen_teleop_headlight.html")


@app.route("/headlight/on", methods=["POST"])
def headlight_on():
    try:
        on = headlight_on_with_mqtt()
        return on, 200
    except Exception as e:
        return f"MQTT Error: {e}", 500


@app.route("/headlight/off", methods=["POST"])
def headlight_off():
    try:
        off = headlight_off_with_mqtt()
        return off, 200
    except Exception as e:
        return f"MQTT Error: {e}", 500


@app.route("/screen")
def screen_control():
    return render_template("mcqueen_teleop_screen.html")


@app.route("/screen/change-picture", methods=["POST"])
def screen_change_picture():
    data = request.get_json()
    pic_id = data.get("pic_id", None)

    if not pic_id:
        return jsonify(success=False, error="Invalid pic id")

    try:
        result = screen_service_client.call_service(pic_id)
        if result.success:
            return jsonify(success=True)
        else:
            return jsonify(
                success=False, error="ROS servic unsuccessful: " + result.message
            )
    except Exception as e:
        return jsonify(success=False, error=str(e)), 500


@app.route("/voice-assistant")
def voice_assistant():
    return render_template("mcqueen_teleop_voice_assistant.html")


@app.route("/voice-assistant/command", methods=["POST"])
def voice_assistant_command():
    data = request.get_json()
    command = data.get("command", None)

    if not command:
        return jsonify(success=False, error="Invalid command")

    result = command_processor.parse_command(command)

    if result in COMMAND_FUNCTIONS:
        response = COMMAND_FUNCTIONS[result]()
        return jsonify(success=True, message=response)

    if result.startswith("Şu an saat") or result.startswith("Bugün günlerden"):
        return jsonify(success=True, message=result)

    return jsonify(success=True, message="Command not recognized.")


def run_node(node):
    rclpy.spin(node)
    node.destroy_node()


def ros_thread():
    global buzzer_service_client, cmd_vel_node, scan_node, screen_service_client

    rclpy.init()

    buzzer_service_client = BuzzerServiceClient()
    cmd_vel_node = CmdVelWebNode()
    scan_node = DistanceListener()
    screen_service_client = ScreenServiceClient()

    # Running multiple nodes in the same thread with ROS Executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(buzzer_service_client)
    executor.add_node(cmd_vel_node)
    executor.add_node(scan_node)
    executor.add_node(screen_service_client)

    try:
        executor.spin()
    finally:
        buzzer_service_client.destroy_node()
        cmd_vel_node.destroy_node()
        scan_node.destroy_node()
        screen_service_client.destroy_node()
        rclpy.shutdown()


def main():
    threading.Thread(target=ros_thread, daemon=True).start()

    app.run(host="0.0.0.0", port=5050, ssl_context=("cert.pem", "key.pem"))


if __name__ == "__main__":
    main()
