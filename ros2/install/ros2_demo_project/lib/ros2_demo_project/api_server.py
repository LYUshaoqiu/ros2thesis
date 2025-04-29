from fastapi import FastAPI, File, UploadFile
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage
import io

app = FastAPI()

# ROS2 初始化
rclpy.init()
ros_node = rclpy.create_node('api_server_node')

# 发布者
image_publisher = ros_node.create_publisher(Image, '/kinect/depth_data', 10)

# 接收上传文件并发布为 ROS2 图像消息
@app.post("/api/upload")
async def upload_file(file: UploadFile = File(...)):
    try:
        # 读取文件内容
        content = await file.read()
        image = PILImage.open(io.BytesIO(content))

        # 转换为 ROS2 图像消息
        ros_image = Image()
        ros_image.header.stamp = ros_node.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera_frame"
        ros_image.height = image.height
        ros_image.width = image.width
        ros_image.encoding = "rgb8"  # 假设上传的是 RGB 图像
        ros_image.step = image.width * 3
        ros_image.data = list(image.tobytes())

        # 发布图像消息
        image_publisher.publish(ros_image)
        return {"status": "success", "message": "File received and published to ROS2"}
    except Exception as e:
        return {"status": "error", "message": str(e)}

# 启动 ROS2 spin
import threading
def spin_ros():
    rclpy.spin(ros_node)

threading.Thread(target=spin_ros, daemon=True).start()
