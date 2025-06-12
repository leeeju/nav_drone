# Drone Navigation Package

드론에서서 사용할 수 있는 nav2 패키지 입니다

기존의 로봇 2D 환경에서 드론이 사용한 3D 환경으로 변환하는 작업을 하고 있습니다

아직 오류가 매우 많음.

2025.06.12 기준 10개의 패키지 생성 및 빌드 진행중

Nav2 Drone Navigation Stack

이 저장소는 드론(Aerial Robot)을 위한 ROS 2 Navigation2(Nav2) 확장 패키지 모음입니다. 기존의 2D 기반 네비게이션에서 확장되어, 3D 환경을 고려한 플래닝, 3D 코스트맵, MPC 제어까지 포함합니다.

패키지 구조
```
nav2_drone_bt_navigator/    # Behavior Tree 기반 상위 레벨 네비게이터
nav2_drone_common/         # 공통 런치 파일 및 설정
nav2_drone_controller/     # Controller 서버 노드 (pluginlib 로드)
nav2_drone_core/           # Planner/Controller 코어 인터페이스 정의
nav2_drone_costmap_3d/     # OctoMap 기반 3D Costmap 구현
nav2_drone_dummy_planner/  # 테스트용 단순 플래너(plugin)
nav2_drone_mpc_controller/ # 드론용 MPC 제어기(plugin)
nav2_drone_msgs/           # 맞춤형 Message/Action 정의 (ComputePathToPose 등)
nav2_drone_planner/        # Planner 서버 노드 (pluginlib 로드)
nav2_drone_util/           # 드론 네비게이션 유틸리티 라이브러리
README.md                  # 프로젝트 개요 및 패키지 설명
```
# nav2_drone_msgs

드론 네비게이션용 ROS 2 메시지, 서비스, 액션(Action) 타입 정의

주로 ComputePathToPose 액션을 통해 3D 목표 위치 경로 생성

# nav2_drone_common

여러 패키지에서 공통으로 사용할 런치 파일, 파라미터 설정 등을 포함

# nav2_drone_util

수학, 좌표 변환, 경로 처리 등 드론 네비게이션에서 자주 사용되는 유틸리티 함수 제공

# nav2_drone_core

Planner, Controller 추상 인터페이스 정의

플래너/컨트롤러를 pluginlib 로딩할 수 있는 기반

# nav2_drone_costmap_3d

OctoMap을 이용한 3D voxel 기반 코스트맵 레이어 구현

CostmapServer, LayeredCostmap3D 등

# nav2_drone_planner / nav2_drone_dummy_planner

# nav2_drone_planner: Planner 서버 노드

pluginlib 통해 여러 플래너(plugin) 로딩

기본 DumbPlanner, ThetaStarPlanner 포함

# nav2_drone_dummy_planner: 예제용 단순 플래너

# nav2_drone_mpc_controller / nav2_drone_controller

# nav2_drone_mpc_controller: Model Predictive Control 기반 드론 속도 명령 생성기

# nav2_drone_controller: Controller 서버 노드

pluginlib 통해 MPC 이외의 컨트롤러 로드 가능

nav2_drone_bt_navigator

Behavior Tree 기반 상위 네비게이션 노드

플래너, 컨트롤러를 순차/병렬 실행하여 전체 경로 추종 관리

빌드 및 실행

# 워크스페이스 루트에서
```
colcon build --symlink-install
source install/setup.bash
```
# BT Navigator 실행 예시
```
ros2 launch nav2_drone_bt_navigator bt_navigator_launch.py
```
# 기여

이슈 등록, PR 환영합니다!

코딩 컨벤션: ROS 2, Nav2 스타일 가이드 준수

