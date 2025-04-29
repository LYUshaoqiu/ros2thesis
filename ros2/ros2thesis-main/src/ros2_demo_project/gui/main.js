// 获取页面元素
const fileTypeSelect = document.getElementById('fileType');
const fileInput = document.getElementById('fileInput');
const uploadBtn = document.getElementById('uploadBtn');
const uploadProgressBar = document.getElementById('uploadProgress');
const uploadStatus = document.getElementById('uploadStatus');

const streamUrlInput = document.getElementById('streamUrl');
const startStreamBtn = document.getElementById('startStreamBtn');
const stopStreamBtn = document.getElementById('stopStreamBtn');
const streamStatus = document.getElementById('streamStatus');

const pointCloudContainer = document.getElementById('pointCloudContainer');

// 上传文件
uploadBtn.addEventListener('click', () => {
  const files = fileInput.files;
  if (!files || files.length === 0) {
    alert('Please select a file to upload 请选择要上传的文件');
    return;
  }
  const fileType = fileTypeSelect.value;
  console.log("🧪 Selected file type is:", fileType);

  let uploadUrl = '/api/upload';
  if (fileType === 'ply') {
    uploadUrl = '/api/upload_ply';
  } else if (fileType === 'depth') {
    uploadUrl = '/api/upload_depth';
  }

  const formData = new FormData();
  formData.append('file', files[0]);

  const xhr = new XMLHttpRequest();
  xhr.open('POST', uploadUrl);

  xhr.upload.onprogress = (event) => {
    if (event.lengthComputable) {
      const percent = Math.round((event.loaded / event.total) * 100);
      uploadProgressBar.style.width = percent + '%';
      uploadProgressBar.textContent = percent + '%';
    }
  };

  xhr.onload = () => {
    if (xhr.status === 200) {
      uploadStatus.textContent = '✅ File uploaded successfully';
    } else {
      uploadStatus.textContent = `❌ Upload Failed: ${xhr.statusText}`;
    }
    setTimeout(() => {
      uploadProgressBar.style.width = '0%';
      uploadProgressBar.textContent = '';
    }, 2000);
  };

  xhr.onerror = () => {
    uploadStatus.textContent = '❌ Upload Failed';
  };

  uploadStatus.textContent = 'Uploading...';
  xhr.send(formData);
});

// 开始推流
startStreamBtn.addEventListener('click', () => {
  const streamUrl = streamUrlInput.value.trim();
  if (!streamUrl) {
    alert('Please enter a valid streaming URL');
    return;
  }
  streamStatus.textContent = 'Push streaming being initiated...';
  startStreamBtn.disabled = true;
  stopStreamBtn.disabled = true;

  fetch("/api/start_stream", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ stream_url: streamUrl })  
  })
  .then(response => response.json())
  .then((data) => {
    if (data.status === "success") {
      streamStatus.textContent = "✅ Push streaming start";
      stopStreamBtn.disabled = false;
    } else {
      streamStatus.textContent = "❌ " + data.message;
      startStreamBtn.disabled = false;
    }
  })
  .catch(() => {
    streamStatus.textContent = '❌ Push streaming failed';
    startStreamBtn.disabled = false;
  });
});

// 停止推流
stopStreamBtn.addEventListener('click', () => {
  fetch('/api/stop_stream', { method: 'POST' })
  .then(() => {
    streamStatus.textContent = '⏹️ Push streaming stopped';
  })
  .catch(() => {
    streamStatus.textContent = '❌ Stop streaming failed';
  })
  .finally(() => {
    startStreamBtn.disabled = false;
    stopStreamBtn.disabled = true;
  });
});

// 初始化 Three.js 场景
let scene, camera, renderer, pointCloud;
function initPointCloudScene() {
  const width = pointCloudContainer.clientWidth;
  const height = pointCloudContainer.clientHeight;
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x000000);
  camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
  camera.position.set(0, 0, 2);
  renderer = new THREE.WebGLRenderer();
  renderer.setSize(width, height);
  pointCloudContainer.appendChild(renderer.domElement);
  const geometry = new THREE.BufferGeometry();
  const material = new THREE.PointsMaterial({ color: 0x00ff00, size: 0.02 });
  pointCloud = new THREE.Points(geometry, material);
  scene.add(pointCloud);
  function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
  }
  animate();
}

// 连接 WebSocket 获取点云数据
function connectPointCloudStream() {
  const ws = new WebSocket('ws://' + window.location.host + '/kinect/point_cloud');
  ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.points) {
      const geometry = new THREE.BufferGeometry();
      const positions = new Float32Array(data.points.flatMap(p => [p.x, p.y, p.z]));
      geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      pointCloud.geometry.dispose();
      pointCloud.geometry = geometry;
    }
  };
}

window.addEventListener('DOMContentLoaded', () => {
  initPointCloudScene();
  connectPointCloudStream();
});



// 获取静态变换表单相关的 DOM
const tfParentFrameInput = document.getElementById('tfParentFrame');
const tfChildFrameInput = document.getElementById('tfChildFrame');
const tfXInput = document.getElementById('tfX');
const tfYInput = document.getElementById('tfY');
const tfZInput = document.getElementById('tfZ');
const tfRollInput = document.getElementById('tfRoll');
const tfPitchInput = document.getElementById('tfPitch');
const tfYawInput = document.getElementById('tfYaw');
const setTransformBtn = document.getElementById('setTransformBtn');
const transformStatus = document.getElementById('transformStatus');

setTransformBtn.addEventListener('click', () => {
  // 获取用户输入
  const parentFrame = tfParentFrameInput.value.trim();
  const childFrame = tfChildFrameInput.value.trim();
  const x = parseFloat(tfXInput.value);
  const y = parseFloat(tfYInput.value);
  const z = parseFloat(tfZInput.value);
  const roll = parseFloat(tfRollInput.value);
  const pitch = parseFloat(tfPitchInput.value);
  const yaw = parseFloat(tfYawInput.value);

  // 发送POST请求到后端
  fetch('/api/set_transform', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      parent_frame: parentFrame,
      child_frame: childFrame,
      x: x,
      y: y,
      z: z,
      roll: roll,
      pitch: pitch,
      yaw: yaw
    })
  })
    .then(response => response.json())
    .then(data => {
      if (data.status === 'success') {
        transformStatus.textContent = '✅ ' + data.message;
      } else {
        transformStatus.textContent = '❌ ' + data.message;
      }
    })
    .catch(err => {
      console.error(err);
      transformStatus.textContent = '❌ Failed to call /api/set_transform.';
    });
});






// ==============================
// 2) 节点状态功能
// ==============================
const refreshNodeStatusBtn = document.getElementById('refreshNodeStatusBtn');
const nodeTableBody = document.querySelector('#nodeTable tbody');

let nodeStateMap = {};

// 每次请求后端 /api/node_status
function fetchNodeStatus() {
  fetch("/api/node_status")
    .then(response => response.json())
    .then(data => {
      const now = Date.now();
      const onlineNodes = data.nodes;         // 所有在线节点列表
      const activeNodes = data.active_nodes;  // 后端判定为有数据传输的节点
      // 如果后端没返回 active_nodes，也可先设置空数组:
      // const activeNodes = data.active_nodes || [];

      // 1) 更新在线节点的信息
      onlineNodes.forEach(nodeName => {
        // 若首次见到，则创建默认记录
        if (!nodeStateMap[nodeName]) {
          nodeStateMap[nodeName] = {
            isOnline: true,
            lastSeen: now,
            isActive: false
          };
        }
        // 更新 lastSeen, isOnline, isActive
        nodeStateMap[nodeName].isOnline = true;
        nodeStateMap[nodeName].lastSeen = now;
        nodeStateMap[nodeName].isActive = activeNodes.includes(nodeName);
      });

      // 2) 标记不在在线列表中的节点为离线
      for (const [name, info] of Object.entries(nodeStateMap)) {
        if (!onlineNodes.includes(name)) {
          if (info.isOnline) {
            // 下线
            nodeStateMap[name].isOnline = false;
            // 不更新 lastSeen，用它表示最后在线时间
          }
        }
      }

      // 3) 清理离线超过 3 分钟的节点
      const OFFLINE_THRESHOLD = 3 * 60 * 1000; // 3分钟
      for (const [name, info] of Object.entries(nodeStateMap)) {
        if (!info.isOnline) {
          const offlineDuration = now - info.lastSeen;
          if (offlineDuration > OFFLINE_THRESHOLD) {
            // 彻底移除
            delete nodeStateMap[name];
          }
        }
      }

      // 4) 渲染表格
      renderNodeTable();
    })
    .catch(err => {
      console.error("Failed to fetch node status:", err);
      // 如果需要在表格显示错误信息，可在此处理
    });
}

// 根据 nodeStateMap 动态生成表格行
function renderNodeTable() {
  nodeTableBody.innerHTML = "";

  // 先按 在线/离线 排序，然后再按名字排序
  const entries = Object.entries(nodeStateMap);
  entries.sort((a, b) => {
    // a[1].isOnline, b[1].isOnline
    if (a[1].isOnline && !b[1].isOnline) return -1;
    if (!a[1].isOnline && b[1].isOnline) return 1;
    return a[0].localeCompare(b[0]); // 同等在线离线时按名字比较
  });

  const now = Date.now();
  entries.forEach(([name, info]) => {
    const tr = document.createElement("tr");

    // (1) Node Name
    const tdName = document.createElement("td");
    tdName.textContent = name;
    tr.appendChild(tdName);

    // (2) 状态列
    const tdStatus = document.createElement("td");
    if (info.isOnline) {
      // 在线 + 如果 isActive 标记为有数据，可再加一个提示
      let badgeHtml = '<span class="badge bg-success me-1">Online</span>';
      if (info.isActive) {
        badgeHtml += '<span class="badge bg-warning text-dark">Active</span>';
      }
      tdStatus.innerHTML = badgeHtml;
    } else {
      // 离线
      tdStatus.innerHTML = '<span class="badge bg-danger">Offline</span>';
    }
    tr.appendChild(tdStatus);

    // (3) 上次在线/离线时长
    const tdTime = document.createElement("td");
    if (info.isOnline) {
      // 当前在线 => "Just now" 或者你可以显示 "Last seen Xs ago"
      tdTime.textContent = "Just now";
    } else {
      const offlineSec = Math.floor((now - info.lastSeen) / 1000);
      tdTime.textContent = `Offline for ${offlineSec}s`;
    }
    tr.appendChild(tdTime);

    nodeTableBody.appendChild(tr);
  });
}

// 点击“Refresh”手动刷新
refreshNodeStatusBtn.addEventListener('click', fetchNodeStatus);

// 页面加载完毕后自动刷新一次
window.addEventListener('DOMContentLoaded', () => {
  fetchNodeStatus();
});
