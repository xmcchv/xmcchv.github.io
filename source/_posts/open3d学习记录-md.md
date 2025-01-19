---
title: open3d学习记录.md
date: 2025-01-19 11:26:08
tags:
- python
- open3d
- tkinter
---
# open3d学习记录
## 1. 安装环境
```bash
pip install open3d
```

## 2. open3d可视化点云

open3d.visualization类一共提供了四种不同形式的可视化方式：

- open3d.visualization.Visualizer <br>
最基础的可视化方式，提供了可视化窗口的创建、渲染、交互等功能。
- open3d.visualization.VisualizerWithEditing <br>
    有选点功能VisualizerWithEditing.get_picked_points()
- open3d.visualization.VisualizerWithKeyCallback <br>
    可以绑定按键回调函数如：register_mouse_button_callback()、register_key_action_callback()
- open3d.visualization.VisualizerWithVertexSelection <br>
    专门的选点的可视化方式。也可以绑定按键回调函数。<br>
    self.vis.register_selection_changed_callback(self.selection_changed_callback) <br>
    self.vis.register_selection_moved_callback(self.selection_moved_callback)


注意：run()函数是一个阻塞函数, 最好使用子线程来调用。然后手动刷新窗口。
```python
self.vis.update_geometry(self.point_cloud)
self.vis.poll_events()
self.vis.update_renderer()
```

### 2.1 可视化点云控制

- 调整点云大小和渲染方式
```python
self.vis.get_render_option().point_size = POINT_SIZE
self.vis.get_render_option().point_color_option = o3d.visualization.PointColorOption.YCoordinate
```

## 3. open3d.gui

### 3.1 窗口创建
open3d.gui.Application类用于创建和管理GUI应用程序。它提供了一个主事件循环，用于处理用户输入和渲染GUI元素。
```python
class PointCloudVisualizer:
    def __init__(self):
        self.app = gui.Application.instance
        self.app.initialize()
        self.window = self.app.create_window("Open3D - PointCloudVisualizer", 1024, 768)
        self.SceneWidget = gui.SceneWidget()
        self.window.add_child(self.SceneWidget)
        self.renderer = rendering.Open3DScene(self.window.renderer)
        self.SceneWidget.scene = self.renderer
        self.mat = rendering.MaterialRecord()
        self.mat.base_color = [1.0, 0.94, 0.96, 1.0]
        self.mat.shader = "defaultLit"
        self.mat.point_size = 5 * self.window.scaling
        self.point_cloud = None
        self.init_scene()
        self.SceneWidget.set_on_mouse(self.on_mouse)
```

### 3.2 可视化点云
```python
def visualize_point_cloud(self, points):
    if points.size == 0:
        print("No points to visualize.")
        return
    self.point_cloud = o3d.geometry.PointCloud()
    self.point_cloud.points = o3d.utility.Vector3dVector(points)
    self.renderer.add_geometry("PointCloud", self.point_cloud, self.mat)
    self.update_camera()
def update_camera(self):
    target = np.array([0.0, 0.0, self.position_info], dtype=np.float32)
    eye = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    up = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    self.SceneWidget.look_at(eye, target, up)
```

### 3.3 鼠标事件
```python
def on_mouse(self, event: gui.MouseEvent) -> gui.SceneWidget.EventCallbackResult:
    if event.is_modifier_down(gui.KeyModifier.SHIFT) and event.is_button_down(gui.MouseButton.LEFT) and event.type.value == gui.MouseEvent.BUTTON_DOWN:
        print("pick_points")
        mouse_x = event.x
        mouse_y = event.y
        print(f'mouse:({mouse_x},{mouse_y})')
        return gui.SceneWidget.EventCallbackResult.HANDLED
    return gui.SceneWidget.EventCallbackResult.IGNORED
```
但是gui类里面没有提供pick_points()函数。以及没有提供如何从鼠标位置获取点云的索引。

### 3.4 运行
```python
def run(self):
    self.app.run()

viewer.run()
```

## 4. tkinter

tkinter是Python的标准GUI库，用于创建图形用户界面(GUI)应用程序。它提供了一组工具和组件，使开发者能够轻松地创建各种窗口、按钮、文本框、标签等元素。
### 4.1 创建窗口
```python
self.root = tk.Tk()
self.root.geometry("960x480")
self.root.title("Point Cloud Visualizer")

# 创建文本框用于显示输出
self.output_text = scrolledtext.ScrolledText(self.root, width=100, height=20, 
                                            font=("Courier New", 20),  # 设置字体
                                            bg="white",  # 设置背景色
                                            fg="black",  # 设置前景色
                                            insertbackground='white',  # 光标颜色
                                            wrap=tk.WORD)  # 单词换行
self.output_text.pack(side=tk.RIGHT, padx=10, pady=10)
# 创建按钮
self.button = tk.Button(self.root, text="Start", command=self.start_visualization)
self.button.pack(side=tk.LEFT, padx=10, pady=10)
```
然后按钮可以触发自定的函数，实现功能。

### 4.2 绑定按键事件

如```self.start_visualization```函数，在其中可以绑定按键事件。

