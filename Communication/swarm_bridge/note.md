# 计划列表

## 通信

1. 开发TCP的通信, 以及UDP的可视化通信
   1. TCP：稳定且关键的数据传输, IP为本机IP
      - 机器人-机器人
        - Y 探索: frontier? + 地图?, include drone_id_
        - Y 规划: 里程计 + 轨迹, include drone_id_
      - 机器人-地面站
        - NONE
      - 地面站-机器人
        - Y 强制目标点, include drone_id_
        - Y 是否遥控控制, include drone_id_
        - Y 是否可视化展示, include drone_id_
        - Y 是否急停, include drone_id_
   2. UDP：可视化信息展示, IP一定是XXX.XXX.XXX.255
      - 机器人-机器人
        - NONE
      - 机器人-地面站
        - 原始数据：压缩图像, include drone_id_
        - 规划：里程计+轨迹, include drone_id_
        - 探索：构建地图 + frontier, include drone_id_
        - state
      - 地面站-机器人
        - 强制遥控控制, include drone_id_

## 可视化

1. 展示
   1. 探索+规划信息
      1. RVIZ内展示
   2. 无人机基础信息
      1. 信号?电池?等
2. 交互
   1. 多机快速选中
      1. RVIZ多机选中
   2. 强制目标(探索区域更新)
      1. 3D GOAL
   3. 第一视角展示
      1. ？



sudo apt install libzmqpp-dev



TODO1: add a new package for message