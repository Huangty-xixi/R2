# 将彩色 `map.dae` 模型导入 Gazebo 简明步骤（对应当前目录）

## 当前目录结构
当前工作目录是 `D:\world`，已经整理成下面这套结构：

```text
D:\world
|-- map.dae
|-- map.stp
|-- worlds.md
|-- my_gazebo_models
|   `-- my_map_model
|       |-- model.config
|       |-- model.sdf
|       `-- meshes
|           `-- map.dae
`-- worlds
    `-- my_map.world
```

其中真正给 Gazebo 使用的是：
- `my_gazebo_models/my_map_model/model.config`
- `my_gazebo_models/my_map_model/model.sdf`
- `my_gazebo_models/my_map_model/meshes/map.dae`
- `worlds/my_map.world`

## 当前模型说明
当前根目录下的 `map.dae` 已经更新为彩色版本，并且包含多个材质和多个几何部件。
为了让 Gazebo 读取这个新模型，已经将它覆盖到下面这个实际使用路径：

```text
my_gazebo_models/my_map_model/meshes/map.dae
```

注意：
- 新的 `map.dae` 自带颜色信息。
- 因此 `model.sdf` 里不要再额外写统一的灰色材质覆盖，否则会把彩色效果盖掉。
- 新模型使用的是 `Z_UP` 轴向，如果后面在 Gazebo 里出现姿态不对、躺倒、朝向异常，再调整 `model.sdf` 里的 `<pose>`。

## 1. 把目录带到 Ubuntu
把整个 `D:\world` 目录复制到 Ubuntu 里，例如复制到：

```bash
/home/liupeng/my_project/src/vision/RC2026/src/world
```

复制完成后，Ubuntu 里的关键路径会变成：

```bash
/home/liupeng/my_project/src/vision/RC2026/src/world/my_gazebo_models
/home/liupeng/my_project/src/vision/RC2026/src/world/worlds
```

## 2. 设置 Gazebo 模型路径
先停止旧的 Gazebo 进程：

```bash
pkill gzserver
pkill gzclient
```

然后把模型路径加入当前终端：

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/liupeng/my_project/src/vision/RC2026/src/world/my_gazebo_models
```

如果确认可用，再写入 `~/.bashrc`：

```bash
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/liupeng/my_project/src/vision/RC2026/src/world/my_gazebo_models' >> ~/.bashrc
source ~/.bashrc
```

## 3. 检查模型文件
确认下面这些文件存在：

```bash
ls /home/liupeng/my_project/src/vision/RC2026/src/world/my_gazebo_models/my_map_model
ls /home/liupeng/my_project/src/vision/RC2026/src/world/my_gazebo_models/my_map_model/meshes
ls /home/liupeng/my_project/src/vision/RC2026/src/world/worlds
```

你应该能看到：
- `model.config`
- `model.sdf`
- `map.dae`
- `my_map.world`

## 4. 当前 `model.config`
当前已经写好的 `model.config` 内容如下：

```xml
<?xml version="1.0"?>
<model>
  <name>my_map_model</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>world</name>
    <email>user@example.com</email>
  </author>
  <description>Map imported from map.dae for Gazebo.</description>
</model>
```

## 5. 当前 `model.sdf`
当前已经写好的 `model.sdf` 内容如下：

```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.6">
  <model name="my_map_model">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
    <link name="body">
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://my_map_model/meshes/map.dae</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://my_map_model/meshes/map.dae</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

说明：
- `scale` 当前设为 `1 1 1`，表示先按原始尺寸导入。
- 已经去掉了统一材质覆盖，这样 Gazebo 才会尽量使用 `map.dae` 自带的颜色。
- 如果模型太大或太小，再改 `<scale>`。
- 如果姿态不对，再调 `<pose>`。

## 6. 当前世界文件 `my_map.world`
当前已经写好的世界文件在：

```bash
/home/liupeng/my_project/src/vision/RC2026/src/world/worlds/my_map.world
```

内容如下：

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="map_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://my_map_model</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
```

## 7. 启动 Gazebo
在 Ubuntu 终端运行：

```bash
gazebo /home/liupeng/my_project/src/vision/RC2026/src/world/worlds/my_map.world
```

如果只剩服务端在跑，也可以单独开界面：

```bash
gzclient
```

## 常见问题
- 报 `Unable to find uri[model://my_map_model]`：说明 `GAZEBO_MODEL_PATH` 没设好。
- 模型没有颜色：先确认 `meshes/map.dae` 用的是新版本，并且 `model.sdf` 没有统一材质覆盖。
- 模型躺倒或方向不对：这是新 `map.dae` 使用 `Z_UP` 导致的，去调 `model.sdf` 里的 `<pose>`。
- 模型太大或太小：改 `model.sdf` 中两个 `<scale>`。
- 启动时报端口占用：说明旧的 `gzserver` 还没关，先 `pkill gzserver`。

## 目前结论
当前这套文件已经切换到新的彩色 `map.dae` 版本，并同步更新了 Gazebo 相关配置。下一步主要是在 Ubuntu 里重新加载，确认颜色、尺度和姿态是否符合预期。
