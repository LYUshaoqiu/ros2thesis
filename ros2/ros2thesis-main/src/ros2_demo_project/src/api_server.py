#!/usr/bin/env python3
from fastapi import FastAPI, File, UploadFile
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import asyncio
import time
from PIL import Image as PILImage
import io
import numpy as np
import os
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
# import open3d as o3d
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import RedirectResponse
from pydantic import BaseModel
from fastapi import Request
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped

app = FastAPI()

# ROS2 初始化
rclpy.init()
ros_node = rclpy.create_node('api_server_node')

# 发布者
image_publisher = ros_node.create_publisher(Image, '/kinect/depth_data', 10)
point_cloud_publisher = ros_node.create_publisher(PointCloud2, '/kinect/point_cloud', 10)

# CvBridge 用于 OpenCV 和 ROS 图像消息的转换
bridge = CvBridge()

# 视频/图片发布线程控制变量
playback_active = False

class StreamRequest(BaseModel):
    stream_url: str



# 允许前端访问 API
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 挂载 GUI 目录到 `/gui` 而不是 `/`
app.mount("/gui", StaticFiles(directory="gui", html=True), name="gui")



@app.post("/api/upload")
async def upload_file(file: UploadFile = File(...)):
    global playback_active

    # 获取文件扩展名
    file_extension = os.path.splitext(file.filename)[-1].lower()

    # 检查文件类型
    if file.content_type.startswith("image/") or file_extension in [".jpg", ".jpeg", ".png", ".bmp"]:
        return await handle_image(file)
    elif file.content_type.startswith("video/") or file_extension in [".mp4", ".avi", ".mkv", ".mov"]:
        return await handle_video(file)
    else:
        return {"status": "error", "message": f"Unsupported file type: {file.filename}"}


# @app.post("/api/start_stream")
# async def start_stream(request: StreamRequest):
#     """
#     接收视频流 URL 并启动实时处理
#     """
#     global playback_active

#     # 停止其他播放
#     playback_active = False
#     await asyncio.sleep(1)

#     stream_url = request.stream_url  # 获取 JSON 里的 stream_url
#     # 启动线程接收实时流
#     threading.Thread(target=process_stream, args=(stream_url,), daemon=True).start()
#     return {"status": "success", "message": f"Started processing stream from {stream_url}"}


@app.post("/api/start_stream")
async def start_stream(request: Request):
    """
    接收视频流 URL 并启动实时处理
    """
    global playback_active

    # 停止其他播放
    playback_active = False
    await asyncio.sleep(1)

    # ✅ 打印请求体内容，检查前端是否正确传递数据
    json_data = await request.json()  # 读取请求 JSON 数据
    print("Received request data:", json_data)  # ✅ 终端打印日志

    # 获取 stream_url
    stream_url = json_data.get("stream_url")
    if not stream_url:
        return {"status": "error", "message": "Missing stream_url"}

    # 启动线程接收实时流
    threading.Thread(target=process_stream, args=(stream_url,), daemon=True).start()
    return {"status": "success", "message": f"Started processing stream from {stream_url}"}

def process_stream(stream_url: str):
    """
    使用 OpenCV 接收视频流并发布为 ROS2 消息
    """
    global playback_active
    playback_active = True

    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        ros_node.get_logger().error(f"Failed to open video stream: {stream_url}")
        return

    # 处理实时视频流
    while playback_active:
        ret, frame = cap.read()
        if not ret:
            break

        # 转换帧为 ROS 图像消息
        ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        ros_image.header.stamp = ros_node.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera_frame"

        # 发布帧
        image_publisher.publish(ros_image)
        ros_node.get_logger().info("Published a video frame from stream.")

        # 避免 CPU 占用过高
        rclpy.spin_once(ros_node, timeout_sec=0.01)

    cap.release()
    playback_active = False
    ros_node.get_logger().info("Video stream processing finished.")


async def handle_image(file: UploadFile):
    global playback_active

    # 停止任何正在播放的内容
    playback_active = False
    await asyncio.sleep(1)

    # 读取图片
    try:
        content = await file.read()
        pil_image = PILImage.open(io.BytesIO(content))

        # 转换为 OpenCV 格式
        cv_image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)

        # 启动线程播放图片
        threading.Thread(target=publish_image_for_duration, args=(cv_image,), daemon=True).start()
        return {"status": "success", "message": "Image received and will be displayed for 5 seconds"}
    except Exception as e:
        return {"status": "error", "message": f"Failed to process image: {e}"}


async def handle_video(file: UploadFile):
    global playback_active

    # 停止任何正在播放的内容
    playback_active = False
    await asyncio.sleep(1)

    # 保存视频到本地
    video_path = f"/tmp/{file.filename}"
    with open(video_path, "wb") as f:
        f.write(await file.read())

    # 启动线程播放视频
    threading.Thread(target=publish_video_frames, args=(video_path,), daemon=True).start()
    return {"status": "success", "message": "Video received and is being published as ROS2 frames"}


def publish_image_for_duration(image, duration=5):
    """
    持续发布图片一定时间（默认5秒钟）。
    """
    global playback_active
    playback_active = True

    end_time = time.time() + duration
    while time.time() < end_time and playback_active:
        # 转换为 ROS 图像消息
        ros_image = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        ros_image.header.stamp = ros_node.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera_frame"

        # 发布消息
        image_publisher.publish(ros_image)
        ros_node.get_logger().info("Published image.")

        # 每秒钟发布一次
        time.sleep(1)

    playback_active = False
    ros_node.get_logger().info("Image playback finished.")


def publish_video_frames(video_path):
    """
    发布视频的每一帧。
    """
    global playback_active
    playback_active = True

    # 打开视频文件
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        ros_node.get_logger().error(f"Failed to open video: {video_path}")
        return

    # 获取视频帧率
    fps = cap.get(cv2.CAP_PROP_FPS)
    delay = 1.0 / fps

    while playback_active:
        ret, frame = cap.read()
        if not ret:
            break  # 视频播放结束

        # 转换帧为 ROS 图像消息
        ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        ros_image.header.stamp = ros_node.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera_frame"

        # 发布帧
        image_publisher.publish(ros_image)
        ros_node.get_logger().info("Published a video frame.")

        # 按帧率延迟
        rclpy.spin_once(ros_node, timeout_sec=0.01)
        time.sleep(delay)

    cap.release()
    playback_active = False
    ros_node.get_logger().info("Video playback finished.")



@app.post("/api/upload_depth")
async def upload_depth(file: UploadFile = File(...)):
    """
    接收深度图并发布到 ROS 2
    """
    content = await file.read()
    depth_image = cv2.imdecode(np.frombuffer(content, np.uint8), cv2.IMREAD_UNCHANGED)

    # 转换为点云并发布
    point_cloud = generate_point_cloud(depth_image)
    point_cloud_publisher.publish(point_cloud)

    return {"status": "success", "message": "Depth image received and published as PointCloud2"}


def generate_point_cloud(depth_image):
    """
    根据深度图生成点云
    """

    # 确保深度图是单通道
    if len(depth_image.shape) == 3:
        depth_image = depth_image[:, :, 0]  # 提取第一个通道（假设深度在R通道）

    # 使用真实相机参数（替换为你的设备参数）
    fx = 365.0
    fy = 365.0
    cx = 256.0
    cy = 212.0
    depth_scale = 0.005 # 深度单位转换（毫米转米）

    height, width = depth_image.shape
    points = []
    for v in range(height):
        for u in range(width):
            z = depth_image[v, u] * depth_scale
            if z <= 0.01:  # 忽略无效点（假设有效深度 > 1cm）
                continue
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            points.append([x, y, z])

    # 设置坐标系为 camera_frame
    header = Header()
    header.stamp = ros_node.get_clock().now().to_msg()
    header.frame_id = "base_link"  # 关键修改！


    point_cloud = PointCloud2(
        header=header,
        height=1,
        width=len(points),
        is_dense=True,
        is_bigendian=False,
        fields=[
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ],
        point_step=12,
        row_step=12 * len(points),
        data=np.array(points, dtype=np.float32).tobytes(),
    )

    return point_cloud


def publish_image_for_duration(image, duration=5):
    """
    持续发布图片一定时间（默认5秒钟）。
    """
    global playback_active
    playback_active = True

    end_time = time.time() + duration
    while time.time() < end_time and playback_active:
        # 转换为 ROS 图像消息
        ros_image = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        ros_image.header.stamp = ros_node.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera_frame"

        # 发布消息
        image_publisher.publish(ros_image)
        ros_node.get_logger().info("Published image.")

        # 每秒钟发布一次
        time.sleep(1)

    playback_active = False
    ros_node.get_logger().info("Image playback finished.")



@app.post("/api/upload_ply")
async def upload_ply(file: UploadFile = File(...)):
    """
    接收 PLY 文件，将其解析后转换为点云消息并发布到 ROS2
    """
    try:
        content = await file.read()
        # 使用 plyfile 库来解析 PLY 文件（无需 open3d）
        from plyfile import PlyData
        plydata = PlyData.read(io.BytesIO(content))
        # 获取顶点数据
        vertex = plydata['vertex']
        if vertex is None or len(vertex.data) == 0:
            return {"status": "error", "message": "读取到的点云为空"}
        # 提取 x, y, z 值
        points = np.vstack([vertex['x'], vertex['y'], vertex['z']]).T
        # 将点云数据转换为 PointCloud2 消息
        pc2_msg = convert_points_to_pointcloud2(points, frame_id="base_link")
        point_cloud_publisher.publish(pc2_msg)
        return {"status": "success", "message": "PLY 文件点云已发布到 ROS2"}
    except Exception as e:
        return {"status": "error", "message": f"Failed to process PLY file: {e}"}

def convert_points_to_pointcloud2(points, frame_id="base_link"):
    """
    将 numpy 数组（N x 3）转换为 sensor_msgs/PointCloud2 消息
    """
    header = Header()
    header.stamp = ros_node.get_clock().now().to_msg()
    header.frame_id = frame_id
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]
    point_step = 12  # 3 * 4字节
    row_step = point_step * points.shape[0]
    data = points.astype(np.float32).tobytes()
    pc2 = PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=point_step,
        row_step=row_step,
        data=data,
    )
    return pc2




# ---------------------
# 静态变换相关 (NEW)
# ---------------------
class StaticTransformPublisher:
    """
    用于在同一个 Node 中持续发布“静态”坐标变换，
    类似于 static_transform_publisher 的功能，但可随时动态修改参数。
    """
    def __init__(self, node: Node):
        self.node = node
        self.tf_broadcaster = TransformBroadcaster(node)
        # 当前要发布的变换信息（如果为 None 则不发布）
        self.current_transform = None
        # 每 0.1s 发布一次
        self.timer = node.create_timer(0.1, self._publish_transform)

    def set_transform(self, x, y, z, roll, pitch, yaw, parent_frame, child_frame):
        """
        设置要发布的静态变换：平移 + 欧拉角(roll, pitch, yaw)。
        """
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        
        # 平移
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        # 旋转：将欧拉角转成四元数
        q = quaternion_from_euler(roll, pitch, yaw)  # 默认弧度制
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.current_transform = t
        self.node.get_logger().info(
            f"Set static transform: parent={parent_frame}, child={child_frame}, "
            f"xyz=({x:.2f},{y:.2f},{z:.2f}), rpy=({roll:.2f},{pitch:.2f},{yaw:.2f})"
        )

    def _publish_transform(self):
        if self.current_transform is not None:
            # 更新时间戳
            self.current_transform.header.stamp = self.node.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.current_transform)

# 在同一个 Node 中创建发布器对象
static_tf_publisher = StaticTransformPublisher(ros_node)


class TransformData(BaseModel):
    """
    前端输入的静态变换参数
    """
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    parent_frame: str
    child_frame: str

@app.post("/api/set_transform")
async def set_transform(transform: TransformData):
    """
    接收前端传来的坐标变换参数，并设置到静态变换发布器中
    """
    static_tf_publisher.set_transform(
        x=transform.x,
        y=transform.y,
        z=transform.z,
        roll=transform.roll,
        pitch=transform.pitch,
        yaw=transform.yaw,
        parent_frame=transform.parent_frame,
        child_frame=transform.child_frame,
    )
    return {
        "status": "success",
        "message": (
            f"Static transform set: parent={transform.parent_frame}, "
            f"child={transform.child_frame}"
        )
    }



@app.get("/api/node_status")
def get_node_status():
    """
    返回当前 ROS2 网络中:
      - 所有在线节点列表 nodes
      - 正在发布话题（有数据传输）的节点列表 active_nodes
      - 整体状态 status ("active"/"offline")
      - 节点总数 node_count
    """
    # 1) 获取所有节点名与命名空间
    #    返回列表：[(node_name, namespace), ...]
    node_names_and_ns = ros_node.get_node_names_and_namespaces()

    # 2) 构造完整节点名称
    #    通常如果 namespace = "/" 则节点全称 = "/node_name"
    #    否则 = "/custom_ns/node_name"
    node_full_names = [
        (ns if ns == "/" else ns + "/") + name
        for (name, ns) in node_names_and_ns
    ]

    # 3) 判断哪些节点“有数据传输”
    #    这里以“该节点是否在发布任意话题”为判定标准
    #    如果你有更复杂需求(如订阅数/特定topic等)，可自行改写
    active_nodes = []
    for (name, ns) in node_names_and_ns:
        pubs = ros_node.get_publisher_names_and_types_by_node(name, ns)
        # 如果该节点至少发布了一个话题，则视为“活跃”
        if len(pubs) > 0:
            full_name = (ns if ns == "/" else ns + "/") + name
            active_nodes.append(full_name)

    # 4) 整体系统状态: 若有节点则 "active", 否则 "offline"
    node_count = len(node_full_names)
    if node_count > 0:
        status = "active"
    else:
        status = "offline"

    # 5) 返回 JSON
    return {
        "status": status,
        "node_count": node_count,
        "nodes": node_full_names,
        "active_nodes": active_nodes
    }









@app.get("/")
async def redirect_to_gui():
    return RedirectResponse(url="/gui/")

# 启动 ROS2 spin
def spin_ros():
    rclpy.spin(ros_node)

threading.Thread(target=spin_ros, daemon=True).start()

if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)


