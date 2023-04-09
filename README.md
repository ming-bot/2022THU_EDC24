# 2022THU_EDC24
2022THU_EDC24特等奖软件部分



### 1. 系统设计概述

本次系统上层采用串级PID调节，一层为轮速PID，一层为欧几里得距离的P调节（同时做了避免死区的设计内容。为了发挥麦克纳姆轮的优势，通过上位机传达的位置坐标，结合自身IMU计算的姿态角度，将世界坐标系中的方位转换为自身IMU坐标系下的方位角度，方便运动学正逆运动学的解算。同时，鉴于上位机帧率不低，本次的卡尔曼滤波较为简单（仅使用轮速的逆运动学解算计算速度）。

主函数则采用了有限状态机的设定，通过取--送--决策之间状态的转换来达到一个小车的闭环控制。加入了包括回合时间快结束时尽量送光外卖等等的小trick。最终也是取得了THU_EDC24特等奖的成绩。

其余妙处皆在代码中，可自行判别，取其精华，去其糟粕。



### 2. 特别鸣谢

硬件PCB板和大部分硬件方面的设定皆由吴宗桓同学完成，居功至伟！非常感谢也佩服得五体投地哈哈哈希望他下届比赛也能斩获No.1！@HUANsic

软件部分感谢队友闫梦蓓同学的调试支持！

特别感谢清华大学智能车车队焊接场地，还有队员们的支持！

特别感谢自动化系学生科协和电子系学生科协工作人员对于这次比赛的辛勤付出！
