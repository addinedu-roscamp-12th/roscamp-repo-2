# 🏭 PPI Share - 스마트 공유 창고 시스템

> 자율주행과 로봇팔을 이용한 스마트 소형 공유 창고 시스템

<br>

## 📌 목차

- [프로젝트 개요](#-프로젝트-개요)
- [팀 구성 및 역할](#-팀-구성-및-역할)
- [기술 스택](#-기술-스택)
- [System Scenario](#-system-scenario)
- [System Architecture](#-system-architecture)
- [GUI - 사용자 & 관리자](#-gui---사용자--관리자)
- [각 로봇별 기능](#-각-로봇별-기능)
- [데모 영상](#-데모-영상)
- [레포 구조](#-레포-구조)
- [패키지 소개](#-패키지-소개)
- [사전 패키지 설치](#-사전-패키지-설치)
- [설치 및 빌드 방법](#-설치-및-빌드-방법)
- [패키지 실행 방법](#-패키지-실행-방법)
- [노드 추가 방법](#-노드-추가-방법)

<br>

## 📖 프로젝트 개요

<!-- 프로젝트 배경, 목적, 주요 기능 등을 작성해주세요 -->

| 항목 | 내용 |
|------|------|
| 프로젝트명 | 스마트 공유 창고 시스템 |
| 개발 기간 | 2026.04.22 ~ 2026.06.26 |
| 개발 인원 | 6명 |
| 프로젝트 소개 | 추후 작성 예정 |

<br>

## 👥 팀 구성 및 역할

<!-- 팀원 이름, 역할, GitHub 링크 등을 작성해주세요 -->

| 이름 | 역할 | 작업 | GitHub |
|------|------|---------------|--------|
| 채윤재 | 역할 | 작업 내용 나열 | [@username](https://github.com/username) |
| 김성은 | 역할 | 작업 내용 나열 | [@username](https://github.com/username) |
| 방석진 | 역할 | 작업 내용 나열 | [@username](https://github.com/username) |
| 서동찬 | 역할 | 작업 내용 나열 | [@username](https://github.com/username) |
| 이태경 | 역할 | 작업 내용 나열 | [@username](https://github.com/username) |
| 정창현 | 역할 | 작업 내용 나열 | [@username](https://github.com/username) |

<br>

## 🛠 기술 스택

| 분류 | 기술 |
|------|------|
| 개발환경 | ![Linux](https://img.shields.io/badge/Linux-FCC624?style=flat-square&logo=linux&logoColor=black) ![Ubuntu](https://img.shields.io/badge/Ubuntu_24.04-E95420?style=flat-square&logo=ubuntu&logoColor=white) ![VSCode](https://img.shields.io/badge/VSCode-007ACC?style=flat-square&logo=visualstudiocode&logoColor=white) |
| 언어 | ![Python](https://img.shields.io/badge/Python-3776AB?style=flat-square&logo=python&logoColor=white) ![C++](https://img.shields.io/badge/C++-00599C?style=flat-square&logo=cplusplus&logoColor=white) |
| UI | ![PyQt](https://img.shields.io/badge/PyQt-41CD52?style=flat-square&logo=qt&logoColor=white) |
| DBMS | ![MySQL](https://img.shields.io/badge/MySQL-4479A1?style=flat-square&logo=mysql&logoColor=white) |
| 인식 | ![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=flat-square&logo=opencv&logoColor=white) |
| LLM | ![Qwen](https://img.shields.io/badge/Qwen-7B68EE?style=flat-square&logoColor=white) |
| 자율주행 | ![ROS2](https://img.shields.io/badge/ROS2-22314E?style=flat-square&logo=ros&logoColor=white) ![SLAM&NAV](https://img.shields.io/badge/SLAM%26NAV-4CAF50?style=flat-square&logoColor=white) |
| 협업 | ![Git](https://img.shields.io/badge/Git-F05032?style=flat-square&logo=git&logoColor=white) ![GitHub](https://img.shields.io/badge/GitHub-181717?style=flat-square&logo=github&logoColor=white) ![Slack](https://img.shields.io/badge/Slack-4A154B?style=flat-square&logo=slack&logoColor=white) ![Confluence](https://img.shields.io/badge/Confluence-172B4D?style=flat-square&logo=confluence&logoColor=white) ![Jira](https://img.shields.io/badge/Jira-0052CC?style=flat-square&logo=jira&logoColor=white) |

<br>

## 🎬 System Scenario

<!-- 시스템 시나리오 다이어그램 또는 설명을 작성해주세요 -->

```
추후 작성 예정
```

<br>

## 🏗 System Architecture

<!-- 시스템 아키텍처 다이어그램을 삽입해주세요 -->

```
추후 작성 예정
```

<!-- 예시: ![Architecture](docs/images/architecture.png) -->

<br>

## 🖥 GUI - 사용자 & 관리자

<!-- GUI 스크린샷을 삽입해주세요 -->

### 사용자 화면

| 화면명 | 스크린샷 |
|--------|----------|
| 로그인 | 추후 추가 |
| 입고 신청 | 추후 추가 |
| 출고 신청 | 추후 추가 |

### 관리자 화면

| 화면명 | 스크린샷 |
|--------|----------|
| 대시보드 | 추후 추가 |
| 랙 관리 | 추후 추가 |
| 사용자 관리 | 추후 추가 |

<br>

## 🤖 각 로봇별 기능

<!-- 각 로봇의 기능 사진 및 설명을 작성해주세요 -->

### IR (입고 로봇) - JetcoBot

| 기능 | 설명 | 사진 |
|------|------|------|
| QR 인식 | 입고 물품 QR 코드 스캔 | 추후 추가 |
| ArUco 마커 인식 | 정렬 마커(PMK) 인식 | 추후 추가 |
| 적재 | 랙에 물품 적재 | 추후 추가 |

### AMR (이송 로봇) - PinkyPro

| 기능 | 설명 | 사진 |
|------|------|------|
| 자율주행 | IZ → BZ → SZ 이동 | 추후 추가 |
| 물품 이송 | 구역 간 물품 운반 | 추후 추가 |

<br>

## 🎥 데모 영상

<!-- 데모 영상 링크 또는 GIF를 삽입해주세요 -->

[![Demo Video](https://img.shields.io/badge/YouTube-Demo-red)](https://youtube.com/링크추가예정)

> 또는 아래에 GIF 삽입
>
> ![Demo GIF](docs/images/demo.gif)

<br>

## 📁 레포 구조

```
roscamp-repo-2/
└── src/
    ├── jetcobot1/     # Jetcobot 로봇 1번 제어 패키지
    ├── jetcobot2/     # Jetcobot 로봇 2번 제어 패키지
    ├── pinky1/        # Pinky 로봇 1번 제어 패키지
    └── db/            # MySQL DB 연결 및 쿼리 처리 패키지
```

`src/` 폴더 안에 각 로봇을 제어하는 ROS2 패키지가 들어있습니다.

<br>

## 📦 패키지 소개

각 패키지는 Python(`ament_python`) 기반의 ROS2 패키지입니다.

| 패키지 이름 | 대상 로봇 | 설명 |
|------------|----------|------|
| jetcobot1 | Jetcobot 1호 | Jetcobot 1번 로봇 제어 노드 모음 |
| jetcobot2 | Jetcobot 2호 | Jetcobot 2번 로봇 제어 노드 모음 |
| pinky1 | Pinky 1호 | Pinky 1번 로봇 제어 노드 모음 |
| db | 공통 | MySQL DB 연결 및 쿼리 실행 노드 (`mysql_node`), DB 클라이언트 라이브러리 (`db_client`) |

### 공통 의존성

| 패키지 | 용도 |
|--------|------|
| rclpy | ROS2 Python 클라이언트 라이브러리 (노드 생성, 토픽/서비스 통신 등) |
| std_msgs | 기본 메시지 타입 (String, Int32, Bool 등) |
| geometry_msgs | 위치/속도 관련 메시지 (Twist, Pose 등) |
| nav_msgs | 내비게이션 관련 메시지 (Odometry, Path 등) |
| sensor_msgs | 센서 데이터 메시지 (LaserScan, Image, Imu 등) |

<br>

## ⚙️ 사전 패키지 설치

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

<br>

## 🚀 설치 및 빌드 방법

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

### 4. 환경 변수 설정

빌드 후 매번 실행하거나 `~/.bashrc`에 추가:

```bash
source install/setup.bash
```

<br>

## ▶️ 패키지 실행 방법

빌드와 `source`가 완료된 후 아래 명령어로 노드를 실행합니다.

```bash
ros2 run <패키지명> <노드명>

# 예시
ros2 run jetcobot1 my_node
ros2 run pinky1 my_node
```

<br>

## ➕ 노드 추가 방법

새 노드를 만들 때는 아래 순서를 따릅니다.

### 1. 노드 파일 생성

```
src/jetcobot1/jetcobot1/my_node.py  ← 여기에 Python 파일 추가
```

### 2. `setup.py`에 `entry_points` 등록

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
