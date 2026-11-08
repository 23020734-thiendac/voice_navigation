# TurtleBot 4 – Week 7: Semantic Map và Voice Navigation

Repository này chỉ chứa các thành phần của tuần 7: phân vùng bản đồ Small House, gán nhãn khu vực và điều hướng TurtleBot 4 bằng giọng nói tiếng Việt.

Không đưa vào repository: package tuần trước, file báo cáo, dataset, kết quả train, `build/`, `install/`, `log/` và `__pycache__/`.

## Pipeline

```text
PGM/YAML → phân vùng Semantic → nhãn khu vực trên RViz
Microphone → Vosk → câu lệnh tiếng Việt → tìm khu vực
→ NavigateToPose → Nav2 → TurtleBot 4 trong Gazebo
```

Các vùng Semantic gồm `Bedroom`, `Living Room`, `Kitchen` và `Fitness Area`. Người dùng nói tiếng Việt; nhãn hiển thị trên RViz dùng tiếng Anh.

## Cấu trúc package

```text
tb4_week7/
├── config/
│   └── zones.yaml                         # polygon, nhãn và tọa độ đích
├── launch/
│   ├── week7_voice_navigation.launch.py   # launch tổng tuần 7
│   └── small_house_sim.launch.py          # Gazebo Small House, không dock
├── maps/
│   ├── small_house_slam.pgm               # bản đồ điều hướng
│   ├── small_house_slam.yaml
│   ├── small_house_regions.pgm            # ảnh phân vùng trực quan
│   └── small_house_regions.yaml
├── rviz/
│   └── semantic_navigation.rviz
├── simulation/
│   ├── worlds/small_house.world           # world Small House
│   ├── models/                            # model và texture world tham chiếu
│   └── photos/                            # texture của world
├── tb4_week7/
│   ├── map_segmentation_node.py           # semantic map và marker vùng
│   ├── voice_recognition_node.py          # microphone + Vosk
│   ├── voice_navigation_node.py           # phân tích lệnh + Nav2
│   ├── voice_monitor_node.py              # in lệnh ra terminal
│   ├── initial_pose_publisher.py          # pose ban đầu cho AMCL
│   └── wheel_drop_state_publisher.py      # TF joint hỗ trợ mô phỏng
├── tools/generate_segmented_map.py        # tạo small_house_regions.pgm
├── package.xml
├── setup.py
└── README.md
```

Các file báo cáo không thuộc repository này.

## Khu vực và tọa độ đích

| Tên Semantic | Nhãn RViz | Điểm đích map | Ý nghĩa |
|---|---|---:|---|
| `bedroom` | Bedroom | `(-6.10, 0.25)` | vùng trống trung tâm phòng ngủ |
| `living_room` | Living Room | `(-0.15, -3.55)` | khoảng giữa sofa và TV |
| `kitchen` | Kitchen | `(6.20, -1.45)` | vùng trống trung tâm gian bếp |
| `fitness_area` | Fitness Area | `(3.10, 2.20)` | phía trước Fitness Equipment |

Các điểm được kiểm tra trên `small_house_slam.pgm`; node tự dịch điểm tới ô free gần nhất nếu điểm cấu hình nằm sát vật cản.

## Yêu cầu môi trường

- Ubuntu + ROS 2 Humble.
- TurtleBot 4 Simulation, Nav2, Gazebo Ignition và các package TurtleBot 4 liên quan.
- Python 3, PyYAML.
- Microphone và Vosk nếu muốn nhận giọng nói thật.

Cài Vosk và sounddevice:

```bash
python3 -m pip install --user vosk sounddevice
```

Tải model tiếng Việt Vosk và giải nén, ví dụ:

```text
/home/$USER/models/vosk-model-small-vn-0.4
```

## Clone và build

Clone repository vào thư mục source của ROS 2:

```bash
cd ~/ros2_ws/src
git clone https://github.com/23020734-thiendac/voice_navigation.git
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tb4_week7
source install/setup.bash
```

Nếu repository được clone ở vị trí khác trong workspace, chạy `colcon build` từ root workspace để colcon tự tìm package.

## Chạy toàn bộ hệ thống

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=68
ros2 launch tb4_week7 week7_voice_navigation.launch.py \
  ros_domain_id:=68 \
  x:=0.0 y:=0.0 yaw:=0.0 \
  show_rviz:=true \
  voice_backend:=auto \
  voice_model_path:=/home/$USER/models/vosk-model-small-vn-0.4
```

Launch tự mở Gazebo Small House, spawn TurtleBot 4, khởi động localization, Nav2, semantic map, nhận dạng giọng nói và RViz. World, model, texture và launch mô phỏng cần thiết đã nằm trong `tb4_week7/simulation`; không cần package gesture tuần 6 và không spawn dock sạc.

Nếu chỉ muốn kiểm tra node bằng văn bản, dùng:

```bash
ros2 launch tb4_week7 week7_voice_navigation.launch.py \
  voice_backend:=text show_rviz:=true
```

Sau đó publish lệnh:

```bash
ros2 topic pub --once /voice_command_text std_msgs/msg/String \
  "{data: 'Đi từ phòng ngủ qua phòng khách rồi tới phòng bếp'}"
```

## Câu lệnh giọng nói

Lệnh đơn:

```text
Đi tới phòng ngủ
Đi tới phòng khách
Đi tới phòng bếp
Đi tới chỗ tập
Dừng
```

Lệnh chuỗi:

```text
Đi từ phòng ngủ qua phòng khách rồi tới phòng bếp
```

Robot sẽ đi lần lượt `Bedroom → Living Room → Kitchen`. Khi nói `Dừng`, goal Nav2 bị hủy và robot nhận vận tốc bằng 0.

## Topic và action chính

| Kênh | Kiểu | Chức năng |
|---|---|---|
| `/voice_command` | `std_msgs/String` | câu đã nhận dạng từ microphone |
| `/voice_command_text` | `std_msgs/String` | kênh kiểm thử bằng văn bản |
| `/voice_navigation_status` | `std_msgs/String` | trạng thái phân tích và điều hướng |
| `/semantic_map/segmentation` | `nav_msgs/OccupancyGrid` | bản đồ có mã vùng Semantic |
| `/semantic_zones/markers` | `visualization_msgs/MarkerArray` | biên và nhãn vùng trên RViz |
| `/semantic_zones/route` | `visualization_msgs/MarkerArray` | các đích của chuỗi lệnh |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | action Nav2 nhận từng đích |
| `/cmd_vel` | `geometry_msgs/Twist` | vận tốc điều khiển robot |

## Tạo lại ảnh phân vùng

```bash
cd ~/ros2_ws/src/voice_navigation/tb4_week7
a=tools/generate_segmented_map.py
python3 "$a"
```

File `small_house_regions.pgm` là ảnh kiểm tra trực quan. Bản đồ Nav2 vẫn là `small_house_slam.pgm`; polygon và tọa độ semantic được khai báo trong `config/zones.yaml`.

## Ghi chú

- Nếu microphone không phải thiết bị số 0, truyền thêm `voice_device:=1` hoặc chỉ số tương ứng.
- Các terminal phải dùng cùng `ROS_DOMAIN_ID`.
- Có thể tăng `command_hold_sec:=3.5` nếu câu chuỗi bị cắt.
- Repository này chỉ dùng bản đồ Small House, không dùng map maze.
