# RoboSot

一個基於 ROS 2 的機器人導航系統，包含地圖發布和姿態導航功能。

## 專案概述

是一個試用於RoboSot競賽的  TurtleBot4 機器人的導航解決方案，提供以下核心功能：
- 自定義代價地圖發布
- 預定義位置的自動導航
- 基於名稱的姿態查詢和導航

## 套件說明

### 1. map_publisher

一個 C++ 編寫的 ROS 2 套件，用於從 PGM 和 YAML 文件發布地圖。

**功能：**
- 讀取 PGM 格式的地圖文件
- 解析 YAML 配置文件
- 發布 OccupancyGrid 地圖到 `/mapadd` 話題

**依賴：**
- rclcpp
- nav_msgs
- yaml-cpp

### 2. pose_navigator

一個 Python 編寫的 ROS 2 套件，用於管理預定義位置並執行導航任務。

**功能：**
- 從 JSON 配置文件載入預定義姿態
- 訂閱 `/pose_name` 話題接收目標位置名稱
- 使用 TurtleBot4Navigator 執行自動導航
- 支援方向控制（前進、左轉、右轉、後退）

**預定義位置：**
- home（原點）
- base（基地）
- left、right、up、up_center（方向位置）
- Yellow、Green、Blue（顏色標記位置）

**依賴：**
- rclpy
- geometry_msgs
- std_msgs
- turtlebot4_navigation
- ament_index_python

## 安裝與編譯

### 前置需求
- ROS 2（Humble 或更新版本）
- TurtleBot4 相關套件
- yaml-cpp（C++ YAML 解析器）

### 編譯步驟

```bash
# 進入工作空間
cd c:\robosot

# 編譯所有套件
colcon build

# 載入環境設定
source install/setup.bash  # Linux/Mac
# 或
call install\setup.bat  # Windows
```

## 使用方法

### 啟動地圖發布器

```bash
ros2 run map_publisher map_publisher_node \
  --ros-args \
  -p map_file:=/path/to/map.pgm \
  -p yaml_file:=/path/to/map.yaml
```

### 啟動姿態導航器

```bash
# 使用 launch 文件啟動
ros2 launch pose_navigator pose_navigator.launch.py

# 或直接運行節點
ros2 run pose_navigator pose_navigator_node
```

### 發送導航指令

```bash
# 導航到預定義位置
ros2 topic pub /pose_name std_msgs/msg/String "data: 'home'" --once
ros2 topic pub /pose_name std_msgs/msg/String "data: 'Yellow'" --once
ros2 topic pub /pose_name std_msgs/msg/String "data: 'base'" --once
```

## 配置文件

### poses.json

位於 `pose_navigator/pose_navigator/config/poses.json`，定義了所有預定義的導航位置：

```json
{
    "home": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
    },
    "Yellow": {
        "x": -0.2,
        "y": 0.84,
        "z": 0.0,
        "w": 1.0
    }
    ...
}
```

您可以根據實際環境自定義位置座標。

## 方向常數

PoseNavigator 類別提供了方向常數：
- `FORWARD` (1.0 rad) - 前方
- `trunLEFT` (2.571 rad) - 正左
- `trunRIGHT` (-0.571 rad) - 正右
- `BACKWARD` (4.142 rad) - 正後

## 開發者指南 (PoseNavigator API)

以下是 `PoseNavigator` 類別中主要方法的詳細說明：

### `load_poses(self)`
- **功能**: 從套件的 `config/poses.json` 載入預定義位置配置。
- **返回**: 包含位置數據的字典。若文件未找到或讀取錯誤則返回空字典。

### `navigate_to_pose(self, pose_name, direction=None)`
- **功能**: 導航到指定位置，支援自定義方向。這是核心導航邏輯。
- **參數**:
  - `pose_name` (str): 目標位置名稱（對應 `poses.json` 中的鍵）。
    - 特殊值 `"now"`: 使用機器人當前位置作為目標座標。
  - `direction` (float, optional): 目標方向的弧度值。
    - `None` (預設): 使用 `poses.json` 中定义的 `z`/`w` 方向。如果 `pose_name` 為 `"now"`，則使用當前朝向。
    - `"now"`: 強制使用機器人當前朝向。
    - `float` 值: 指定具體的目標朝向（弧度）。
- **示例**:
  ```python
  # 導航到 home 點，方向由 json 定義
  self.navigate_to_pose("home")
  
  # 在當前位置原地旋轉到指定方向
  self.navigate_to_pose("now", direction=1.57) # 轉向 90 度
  ```

### `turn_right_180_degrees(self)`
- **功能**: 發布 `cmd_vel` 指令控制機器人原地右轉 180 度。
- **實現細節**: 使用開環控制（基於時間計算），發送角速度 -1.0 rad/s 持續 π 秒。此方法為阻塞式調用。

### `pose_callback(self, msg)`
- **功能**: `/pose_name` 話題的回調函數。
- **參數**: `msg` (std_msgs.msg.String)，包含目標位置名稱。
- **行為**: 接收到位置名稱後自動調用 `navigate_to_pose`。

## 專案結構

```
robosot/
├── redme.md
├── map_publisher/              # C++ 地圖發布套件
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/
│   │   └── map_publisher/
│   └── src/
│       └── map_publisher_node.cpp
└── pose_navigator/             # Python 姿態導航套件
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── pose_navigator/
    │   ├── __init__.py
    │   ├── pose_navigator_node.py
    │   ├── config/
    │   │   └── poses.json     # 位置配置
    │   └── launch/
    │       └── pose_navigator.launch.py
    ├── resource/
    └── test/
```



## 故障排除

### 常見問題

1. **地圖無法載入**
   - 確認 PGM 和 YAML 文件路徑正確
   - 檢查文件權限

2. **導航失敗**
   - 確認 TurtleBot4 導航堆疊已啟動
   - 檢查目標位置是否在可達範圍內
   - 確認地圖已正確載入

3. **位置名稱無效**
   - 檢查 poses.json 中是否定義了該位置
   - 確認位置名稱拼寫正確（區分大小寫）



