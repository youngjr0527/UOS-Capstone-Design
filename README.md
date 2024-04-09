# UOS-Capstone-Design-in-Creative-Engineering
2023, University of Seoul, Mechanical Information Engineering, Team Project

# 1. 프로젝트 소개

## 1.1. 프로젝트 명

### 지능형 사족 보행 로봇을 이용한 캠퍼스 투어 및 경로 안내 시스템 
### (Intelligent Quadruped Robot-Based Campus Tour and Path Guidance System)<br>
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/e253df83-807d-420e-bfb5-9ffbc3bd7f7c" width="380px">
<div style="background-color: #800080; color: #ffffff; margin: 10px 0; padding: 10px; border-radius: 5px;">
  <strong>🤖프로젝트 개요:</strong> 실외에서 움직일 수 있는 사족보행 로봇과 음성 AI 서비스를 개발해 학교 투어 및 길 안내를 수행하는 로봇의 소프트웨어 시스템을 구현하였다.  학교 투어는 로봇이 정해진 경로를 이동하고 길 안내는 현재 위치부터 사용자가 설정한 목적지까지 계산된 경로를 이동한다. 로봇은 스스로 장애물 회피하고, 사용자와 멀리 떨어질 시 기다리는 기능을 제공한다. 또한 음성 AI 기능을 탑재해 사용자와 질의응답을 하며 학교와 관련된 정보를 제공해준다.
</div>
<br>
<details>
  <summary>📝프로젝트 A1포스터</summary>
  <img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/a4f5585a-5d51-4e98-85e3-5d170c4a5c73" width="550px"><br>
</details>
<details>
  <summary>🏆캡스톤 디자인 우수작품상</summary>
  <img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/98cde30d-a661-4d00-a314-d2a919871a69" width="450px"><br>
</details>

## 1.2. 프로젝트 기간

📅 2023.09.01 ~ 2023.12.22<br>
**Project Workspace** :📒 [Notion](https://www.notion.so/skipper0527/9934fbcaccc94075b9d5d3b15c98b6cf?pvs=4)<br>

## 1.3. 팀 소개

### 팀명 : 안내봇 이루멍

🎯 **역할 분배**
| 학교 | 학과 | 이름 | 역할 |
|:---:|:---:|:---:|:---:|
| 서울시립대 | 기계정보공학과  | 이용재 | 로봇제어 |
| 서울시립대 | 기계정보공학과  | 박정현 | 로봇제어 |
| 서울시립대 | 기계정보공학과  | 김영민 | 인식 및 센서 데이터 처리 |
| 서울시립대 | 기계정보공학과  | 김영준 | 음성 AI |
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/1790ba18-b075-47e8-bf10-9b60752bfa8b" width="340px">

# 2. 프로젝트 개요

## 2.1	개발 배경
&nbsp;&nbsp;대형 건물이나 박물관 같은 곳에는 사람을 대신해 길 안내 및 궁금증을 해결해 주는 로봇들이 많이 사용되고 있다. 일례로 현재 인천공항에서 사용하는 안내로봇 ‘에어스타’는 키오스크 모니터와 음성인식 기능으로 공항 이용객들에게 많은 편의를 제공하고 있다. 하지만 이와 같은 로봇들은 두 가지의 문제점이 있다. 첫번째는 바퀴로 움직이기 때문에 계단이나 연석등이 있는 곳에서 이동이 어려워 제한된 실내 구간에서만 동작이 가능하다. 두번째는 키오스크 화면에 제시된 질문 이외에 궁금한 사항에 대해서는 사용자가 직접 인터넷에 검색하여 찾아야 하는 번거로움이 있다.<br>
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/ee72a55a-9920-450f-a036-dabf95eef154" width="500px"><br>

## 2.2	기대효과
&nbsp;&nbsp;사족보행 로봇을 활용하면 계단과 연석 등이 있는 실외 환경에서 원활하게 작동할 수 있으므로 관광지, 학교 캠퍼스, 도시 공원 등에서도 방문객들이 쉽게 길을 찾을 수 있게 도와줄 수 있다. 또한, 음성인식 기술을 활용하여 사용자와 쌍방향 의사소통이 가능해진다. 이것은 사용자의 질문에 대답하고 추가 정보를 제공하며, 사용자와의 자연스러운 대화를 통해 정보 전달을 더 효과적으로 할 수 있다. 그리고 인공지능을 활용하여 사용자의 선호도와 필요에 따라 맞춤형 서비스를 제공할 수 있다. 예를 들어, 관광지에서 사용자가 관심을 가지는 특정 명소나 학교 투어에서 사용자가 방문하고자 하는 특정 건물에 대한 정보를 제공할 수 있다. 마지막으로 서울시립대에 도입했을 때 좋은 홍보 수단이 될 수 있다. 학교에 사족보행 로봇을 도입함으로써 매력적인 홍보 요소로 작용하고, 방문객들에게 새로운 경험을 제공하여 더 많은 관심을 끌 수 있다.

# 3. 시스템 구성

### 3.1 제공 서비스

사족보행 로봇인 GO1을 활용하여 학교 캠퍼스 투어 및 길 안내 서비스를 제공하며 음성인식 및 인공지능을 활용하여 사람 안내원과 비슷하게 사용자와 의사소통이 가능한 로봇을 개발하여 서비스를 제공한다.

 **3.1.1 학교 투어**<br>
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/7a6ff2c4-7201-464d-b41e-8b21611a0b63" width="500px"><br>
사용자와 함께 캠퍼스를 돌아다니며 학교에 대해 소개한다.

**3.1.2 길 안내**<br>
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/c99f4124-2225-49a6-b02b-f815d9f6b492" width="500px"><br>
사용자가 특정한 건물에 대한 위치를 물으면 로봇이 그 건물까지 안내한다.<br>
두 기능 모두 이동시 사용자와의 거리 유지 기능을 제공하며, 학교에 대한 질문 답변 기능을 제공한다.


### 3.2 개발 내용

**3.2.1 사족보행 로봇 구동**<br>
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/39efeb90-4082-449b-a1a5-49c3ccfbb99d" width="250px"><br>
투어 및 길 안내 서비스를 제공하는 로봇으로 그림 6의 Unitree사의 GO1 모델을 사용하여 기본적인 움직임과 장애물 회피 등의 움직임을 구현하였다.

**3.2.2 자율주행 시스템**<br>
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/d12e8d16-c758-4296-b442-b65c3e33df40" width="580px"><br>
&nbsp;&nbsp;ROS(Robot Operating System)에서 위와 같은 구조를 활용해 자율주행 시스템을 제작한다. 주요 기능으로는 자신의 위치 정보와 도착지점이 정해져 있을 때 경로를 만드는 알고리즘, 장애물을 회피할 알고리즘이 있다. 로봇의 위치정보는 GPS를 활용하여 파악하며, 경로를 만드는 알고리즘 및 장애물 회피 알고리즘은 오픈소스을 활용해 문제환경에 맞는 알고리즘을 제작하였다.


# 4. 시스템 설계 및 구현

### 4.1 조립도

부품 조립 사진 <br>
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/cb804610-59de-44eb-a763-bee7b41ab9a9" width="280px"><br>
### 4.2 제어부 설계
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/9ccb3913-b899-4e38-b357-37318c5d5929" width="580px"><br>
&nbsp;&nbsp;GO1 내부구조는 라즈베리파이, Jetson Nano, Jetson Xavier등의 다른 다수의 제어들이 이더넷으로 연결되어 구성된다. 라즈베리 파이는 다른 제어기들과의 통신을 주로 담당하고, GO1에서는 부분적으로 ROS통신을 제공하는데, 이때 ROS Master의 IP가 된다. 다른 노드들은 라즈베리파이의  IP를 ROS Master로 설정하여 ROS원격 통신을 할 수 있다. Jetson Xavier에서는 로봇의 모터 제어에 관한 작업을 수행한다. Unitree legged real 패키지를 통해 ROS로 로봇에게 속도 명령을 전달한다. Jetson Nano에서는 GO1머리부분의 LED를 제어할 수 있다.
개발한 소프트웨어 대부분은 GO1 외부에 추가로 장착하는 미니 PC인 NUC에서 동작한다. NUC에서는 라이다, GPS, 아두이노, 인터넷 연결을 위한 핸드폰 핫스팟과 연결되어 센서 값을 받고, 음성 AI와 자율주행 기능의 결과를 통해 로봇의 속도와 각속도를 최종적으로 계산하여 Jetson Xavier로 내보내어 로봇을 동작시킨다.<br>

### 4.3 소프트웨어 설계

**4.3.1 음성 AI**<br>
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/521704d2-c9af-415c-bbcc-2e091c04a3b2" width="580px"><br>
&nbsp;&nbsp;서울시립대학교 관련 지식을 통합한 대화형 언어 모델을 개발하여, 정확하고 신뢰할 수 있는 대학 캠퍼스 정보를 제공하기 위해 설계되었다. 이 과정에서 사용되는 Langchain 프레임워크는 언어 모델을 기반으로 한 애플리케이션 개발을 용이하게 하며, 특히 Word Embedding 기술을 통해 텍스트 데이터를 수치적 벡터로 변환, 의미론적인 정보를 딥러닝 모델이 이해할 수 있는 형태로 인코딩한다.

&nbsp;&nbsp;이러한 시스템은 서울시립대학교의 공식 문서와 데이터를 학습 자료로 사용하여, 이루멍이 사용자로부터 속도 조절, 목적지 설정, 학교 관련 질문 등의 쿼리를 받았을 때, 정확한 정보를 기반으로 응답할 수 있도록 한다. 이루멍의 LLM은 사용자의 쿼리를 분석하고, 벡터 데이터베이스에서 추출한 관련 문서의 정보와 결합하여, 사용자의 질문에 대한 구체적이고 정확한 답변을 생성한다.<br>

<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/993cbe9c-713f-4930-a78e-3352d3bb81b8" width="630px"><br>

&nbsp;&nbsp;**이벤트 트리거 프로세스**는 GPS 콜백을 기반으로 작동한다. 이 시스템은 로봇이 특정 건물 위치에 도달했을 때, 해당 건물에 대한 사전 녹음된 설명을 자동으로 재생하는 기능을 포함한다. 또한, 사용자와 로봇 사이의 거리가 일정 기준을 벗어날 경우 경고 메시지를 재생하여 안전 유지를 도모한다. 이러한 프로세스는 로봇이 캠퍼스 내에서 자율적으로 활동하는 동안 필요한 정보를 제공하고 사용자의 안전을 확보하는 역할을 한다.
&nbsp;&nbsp;**동적 상호작용 프로세스**는 사용자의 호출 명령어를 인식하고, 이를 기반으로 1) 속도 조절, 2) 목적지 설정, 3) 학교 관련 질문 등 세 가지 주요 작업으로 분류하여 수행한다.  이 프로세스는 사용자와 로봇 간의 상호작용을 더욱 풍부하고 유연하게 만들어, 사용자의 경험을 향상시키는데 기여한다. 

&nbsp;&nbsp;본 프로젝트에서는 OpenAI가 제공하는 ChatGPT 3.5 Turbo 모델을 API로 사용하여, 다양한 사용자 요구를 분류하고 적절한 반응을 생성한다. 특히, 서울시립대학교 관련 문서로 사전 학습된 LLM 모델은 학교 관련 질문에 대한 높은 정확도를 제공한다.

&nbsp;&nbsp;분류된 텍스트는 로봇의 각 기능에 맞는 프로세스로 전환된다. 예를 들어, 속도 조절이나 목적지 안내 요청이 있을 경우, 이러한 요구는 ROS 토픽으로 전송되어 로봇이 실시간으로 반응할 수 있도록 한다. 또한, 학교 관련 질문에 대해서는 해당 질문을 LLM 모델에 전달하고, 모델은 질문에 대한 답변을 생성한다. 이 답변은 TTS(Text-to-Speech) 과정을 거쳐 음성으로 변환되고, 스피커를 통해 사용자에게 전달한다.<br>

**4.3.2 로봇 자율주행**

**a) 자율주행 구조<br>**
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/ef424319-eb03-410a-9982-18b0f3b591ed" width="550px"><br>
&nbsp;&nbsp;로봇의 자율주행을 위해 ros_navigation 패키지를 변형하거나 일부 직접 개발한 알고리즘으로 대체하여 활용하였다. 기존의 Navigation은 로봇이 모터의 엔코터 값으로부터 역산한 변위인 오도메트리(odometry), 각 링크간의 상대 위치인 상대위치변환(tfMessage), 거리 센서(sensor topics), 지도(map), 목적 좌표(move_base), 속도 명령(cmd_vel) 절차로 이루어진다. 하지만 사족 보행의 특성상 보통의 바퀴형 로봇에서 얻을 수 있었던 오도메트리를 얻기 힘들다. 그리고 기존의 실내로봇은 라이다 등의 센서로 SLAM으로 작성한 map에서 오도메트리와 amcl을 통해 자신의 위치를 추정하지만, 이번 프로젝트에서는 실외에서 정확한 위치를 GPS와 IMU를 통해서 얻을 수 있기 때문에, 이부분을 변경해야 한다. 그리고, 기존의 global planner는 SLAM에서 작성한 map에서 경로를 찾지만, 이번에는 map이 없기 때문에 새로운 Global Planner를 작성해야 한다.<br>

**b) 글로벌 플래너<br>**
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/64bb3d7e-f118-4997-8c47-26d57cb7a223" width="480px"><br>
&nbsp;&nbsp;Global Planner는 그래프기반의 경로 탐색 알고리즘인 Dijkstra’s algorithm을 사용했다. 학교의 건물들과 보행로의 갈림길을 가상의 노드(node)로 만들고, 노드간 이동할 수 있는 길이 있으면 그 노드 사이를 잇는 엣지(edge)를 추가한다. 경로 탐색이 발생하면, 현재 위치를 임시 노드로 추가하고, 현재 위치에서 가장 가까운 엣지위의 점도 임시노드로 추가한다. 그리고 그 두 노드를 잇는 임시 엣지를 추가한다. 현재 위치에서 목표 노드까지 Dijkstra’s alogrithm을 사용하여 경로를 탐색하면, 목적지까지 이동할 수 있는 일련의 엣지들의 나열을 얻을 수 있다. 이때 각 edge는 실제 학교 GPS좌표로 기록된 경유점(waypoint)들과 매칭되어 있는데, 엣지들의 나열 순서대로 경유점들을 찾아 이으면 실제 로봇이 추종할 수 있는 global path가 된다.<br>

**c) 로컬 플래너<br>**
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/91c9b7f3-8032-4a07-ad8a-9a027f1725d9" width="320px"><br>
&nbsp;&nbsp;로컬 플래너로는 ROS 패키지 move_base에서 제공하는 DWA 로컬 플래너를 사용하였다. 위 사진은 DWA를 도식화한 그림이다. 그림과 같이 로봇은 현재 위치를 기준으로 X, Y방향의 속도와 각속도의 최대, 최소값을 샘플링하여 가능한 모든 조합을 생성한 후 장애물 정보를 이용해 장애물에 부딧히지 않는 경로를 생성한다. 이때, 로봇에게 주어지는 파라미터는 속도, 가속도의 최대, 최솟값 뿐만 아니라 경로를 얼마나 잘 따라갈지, 목표점 도착 시 목표 지점에서 얼마나 떨어져도 되는지 등 다양하다. 속도의 최대값은 Y축으로는 0으로 설정하였으며, X축으로는 사람이 걸을 때의 평균 속도인 1.3m/s로 설정하였다. 또한, 목표지점에서의 거리 차이는 초기에 목표 삼았던 1m까지 가능하도록 설정하였다. 나머지 파라미터들은 실험을 진행하며 최적의 파라미터로 설정하였다.<br>
 
**4.3.3 칼만 필터를 통한 거리 측정<br>**
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/67a52767-8d67-41b0-a0b6-f4e2d09feb24" width="320px"><br>
&nbsp;&nbsp;길 안내에 있어서 사용자와 로봇 사이의 거리 유지가 필요하다. 이에 블루투스 신호 세기를 이용해 로봇과 사용자 사이의 거리를 예측하는 시스템을 만들었다. 블루투스 모듈 사이의 신호 세기는 RSSI를 이용해서 측정이 가능하다. RSSI값과 실제 거리 사이의 관계는 아래 식과 같다.<br>
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/e82e0a22-9e58-4dd4-9a88-e826f102f113" width="220px"><br>
&nbsp;&nbsp;d는 거리를 의미하고, 𝛼는 1m 거리에서 측정된 RSSI 값이다. N은 장애물 등과 같은 전파 환경에 따라 설정이 가능하고, 일반적으로 2~4의 값을 가진다. RSSI값은 noise가 많아 noise 제거가 필요하다. 본 프로젝트에서는 noise제거를 위해 칼만 필터를 사용했다.

&nbsp;&nbsp;칼만 필터는 잡음이 포함되어 있는 센서의 측정치에서 선형 역학계의 상태를 추정하는 재귀 필터이다. 칼만 필터 알고리즘은 예측과 업데이트 두 단계로 이뤄진다. 예측 단계에서는 현재 상태 변수의 값과 정확도를 예측한다. 업데이트 단계에서는 이전에 추정한 상태 변수를 기반으로 예측한 측정치와 실제 측정치를 반영해 현재의 상태 변수를 업데이트한다.<br>

<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/2ef4b73b-263d-4ee5-a839-46faf8c3b9a0" width="330px"><br>
<img src="https://github.com/youngjr0527/UOS-Capstone-Design/assets/83463280/08d4b564-6b03-4f61-be2d-e03d0bdf72a1" width="330px"><br>


위 그림은 Raw RSSI 데이터이고, 아래는 칼만 필터를 이용해 noise를 제거한 그림이다.<br>

# File Tree 구조
```
├─GO1-Twist-to-Highcmd
│  └─src
├─GPS-Global-Planner
│  ├─include
│  ├─launch
│  ├─rviz
│  ├─src
│  └─waypoints
├─GPS-IMU-odometry
│  ├─cfg
│  │  ├─diff_drive
│  │  │  └─dyn_obst
│  │  └─dwa
│  ├─include
│  │  └─move_go1
│  ├─launch
│  ├─rviz
│  ├─script
│  └─src
├─GO1-Twist-to-Highcmd
│  └─src
├─GPS-Global-Planner
│  ├─include
│  ├─launch
│  ├─rviz
│  ├─src
│  └─waypoints
├─GPS-IMU-odometry
│  ├─cfg
│  │  ├─diff_drive
│  │  │  └─dyn_obst
│  │  └─dwa
│  ├─include
│  │  └─move_go1
│  ├─launch
│  ├─rviz
│  ├─script
│  └─src
├─LangchainforQnA
└─Bluetooth
```
### GO1-twist to Highcmd

Ros move_base의 속도 명령 토픽인 **[geometry_msgs](https://docs.ros.org/en/noetic/api/geometry_msgs/html/index-msg.html)/Twist**  을 GO1의 자체 명령 토픽 [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real)으로 변환하는 패키지.
 
### GPS-Global-Planner

각 건물까지 이동할 수 있는 전역 경로를 생성하는 패키지. GPS좌표로 이루어진 경로를 생성, ROS navigation stack의 Global Planner Plugin으로 작성됨.

### GPS-IMU-odometry

사족보행 로봇은 바퀴형 로봇과 다르게 오도메트리를 얻기 어려워, GPS와 IMU등의 외부 센서를 기반으로 오도메트리와 tf정보 생성.

### LangchainforQnA

안내로봇과 사람이 음성 인터페이스로 소통하기 위한 LLM 모델. 학교에 대한 특정 정보를 답할 수 있음.  목적지 설정, 질문에 응답하는 기능을 수행.
