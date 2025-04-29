/**
 * 上传文件函数
 */
function uploadFile() {
    const fileInput = document.getElementById("fileInput"); // 获取文件输入框
    const file = fileInput.files[0]; // 获取用户选择的文件
    const preview = document.getElementById("preview"); // 获取图片预览元素

    // 检查是否选择了文件
    if (!file) {
        alert("请选择一个文件！");
        return;
    }

    // 显示文件预览
    const reader = new FileReader();
    reader.onload = function (e) {
        preview.src = e.target.result; // 将图片加载到预览区域
    };
    reader.readAsDataURL(file);

    // 创建表单数据
    const formData = new FormData();
    formData.append("file", file);

    // 上传文件到后端API
    fetch("http://localhost:8000/api/upload", {
        method: "POST",
        body: formData,
    })
        .then((response) => response.json())
        .then((data) => {
            if (data.status === "success") {
                alert("文件上传成功！");
            } else {
                alert(`上传失败: ${data.message}`);
            }
        })
        .catch((err) => alert(`请求失败: ${err}`));
}
