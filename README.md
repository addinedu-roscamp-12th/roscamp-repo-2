# roscamp-repo-2

ROS2와 AI를 활용한 자율주행 로봇개발자 부트캠프 2팀 저장소.

---

## 목차

- [레포 구조](#레포-구조)
- [패키지 소개](#패키지-소개)
- [사전 패키지 설치](#사전-패키지-설치)
- [설치 및 빌드 방법](#설치-및-빌드-방법)
- [패키지 실행 방법](#패키지-실행-방법)
- [노드 추가 방법](#노드-추가-방법)

---

## 레포 구조

```
roscamp-repo-2/
└── src/
    ├── jetcobot1/     # Jetcobot 로봇 1번 제어 패키지
    ├── jetcobot2/     # Jetcobot 로봇 2번 제어 패키지
    ├── pinky1/        # Pinky 로봇 1번 제어 패키지
    └── db/            # MySQL DB 연결 및 쿼리 처리 패키지
```

> `src/` 폴더 안에 각 로봇을 제어하는 ROS2 패키지가 들어있습니다.

---

## 패키지 소개

각 패키지는 **Python(ament_python)** 기반의 ROS2 패키지입니다.

| 패키지 이름 | 대상 로봇 | 설명 |
|------------|----------|------|
| `jetcobot1` | Jetcobot 1호 | Jetcobot 1번 로봇 제어 노드 모음 |
| `jetcobot2` | Jetcobot 2호 | Jetcobot 2번 로봇 제어 노드 모음 |
| `pinky1`    | Pinky 1호   | Pinky 1번 로봇 제어 노드 모음 |
| `db`        | 공통        | MySQL DB 연결 및 쿼리 실행 노드 (mysql_node), DB 클라이언트 라이브러리 (db_client) |

### 공통 의존성

모든 패키지는 아래 ROS2 기본 패키지에 의존합니다. afsfsfdfeifeodsodsfdfdfdf

| 패키지 | 용도 |
|--------|------|
| `rclpy` | ROS2 Python 클라이언트 라이브러리 (노드 생성, 토픽/서비스 통신 등) |
| `std_msgs` | 기본 메시지 타입 (String, Int32, Bool 등) |
| `geometry_msgs` | 위치/속도 관련 메시지 (Twist, Pose 등) |
| `nav_msgs` | 내비게이션 관련 메시지 (Odometry, Path 등) |
| `sensor_msgs` | 센서 데이터 메시지 (LaserScan, Image, Imu 등) |

---

## 사전 패키지 설치

ROS2 패키지 외에 아래 Python 라이브러리를 별도로 설치해야 합니다.

### pymysql (db 패키지 사용 시 필수)

MySQL 데이터베이스에 연결하기 위한 Python 라이브러리입니다.

```bash
pip3 install pymysql
```

설치 확인:

```bash
python3 -c "import pymysql; print('pymysql 설치 완료')"
```

---

## 설치 및 빌드 방법

### 1. 레포 클론

```bash
git clone https://github.com/addinedu-roscamp-12th/roscamp-repo-2.git
cd roscamp-repo-2
```

### 2. 의존성 설치

```bash
# 워크스페이스 루트(roscamp-repo-2/)에서 실행
rosdep install --from-paths src --ignore-src -r -y
```

> `rosdep`은 `package.xml`에 명시된 의존성을 자동으로 설치해주는 도구입니다.

### 3. 빌드

```bash
# 워크스페이스 루트(roscamp-repo-2/)에서 실행
colcon build
```

### 4. 환경 변수 설정 (빌드 후 매번 실행하거나 ~/.bashrc에 추가)

```bash
source install/setup.bash
```
---

## 패키지 실행 방법

빌드와 source가 완료된 후 아래 명령어로 노드를 실행합니다.

```bash
ros2 run <패키지명> <노드명>

# 예시
ros2 run jetcobot1 my_node
ros2 run pinky1 my_node
```

---

## 노드 추가 방법

새 노드를 만들 때는 아래 순서를 따릅니다.

### 1. 노드 파일 생성

```
src/jetcobot1/jetcobot1/my_node.py  ← 여기에 Python 파일 추가
```

### 2. `setup.py`에 entry_points 등록

```python
entry_points={
    'console_scripts': [
        'my_node = jetcobot1.my_node:main',  # 추가
    ],
},
```

> 형식: `'실행할이름 = 패키지명.파일명:함수명'`

### 3. 다시 빌드

```bash
colcon build --packages-select jetcobot1
source install/setup.bash
```
