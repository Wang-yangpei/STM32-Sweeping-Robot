# STM32 Autonomous Sweeping Robot (自主移动扫地机器人)

<p align="center">
  <img src="docs/robot_af.jpg" alt="面包板纯手工搭建的扫地机器人原型全景" width="600">
</p>
<p align="center">
  <i>基于基础玩具底盘与面包板的快速原型验证机 (V1.0 裸机版 -> V2.0 RTOS版)</i>
</p>

<p align="center">
  <img src="docs/robot_details.jpg" alt="硬件细节特写" width="600">
</p>
<p align="center">
  <i>硬件细节：面包板走线、电机驱动 (TB6612)、六轴姿态传感器 (MPU6050) 与超声波传感器布局</i>
</p>

## 项目简介 (Introduction)

本项目主打**快速原型开发 (Rapid Prototyping)** 与 **底层硬核重构**。初期基于基础玩具底盘，通过面包板与杜邦线完成了纯手工的硬件拓扑搭建。随后在软件架构上，实现了从“传统裸机时间片轮询”到“**基于 FreeRTOS 的工业级强抢占式多任务内核**”的史跨越，最终实现了一台具备环境感知、姿态闭环与弓字形路径规划能力的智能机器人。

## V2.0 核心架构大升级：全面拥抱 FreeRTOS (CMSIS_V2)

为了解决裸机架构下复杂的时序冲突与传感器死等阻塞问题，本项目进行了底层的全面重构，充分展现了解决 RTOS 并发冲突的工程能力：

* **核心调度解耦 (Task Management)**：
  * **Task_Control (高优先级 / 10ms)**：机器人底层小脑，独占执行速度 PID 闭环与 MPU6050 姿态积分，保证底盘运动的绝对丝滑。
  * **Task_Sensor (中优先级 / 50ms)**：环境雷达，非阻塞式轮询超声波阵列。
  * **Task_Decision (普通优先级 / 50ms)**：导航司令部，运行**弓字形路径规划状态机**，处理避障、平移换道与 U 型弯逻辑。
* **攻克 RTOS 典型天坑 (Hardcore Debugging)**：
  * **时间片融合**：开启 `configUSE_TICK_HOOK`，通过滴答钩子桥接 FreeRTOS 与 STM32 HAL 库，完美解决 `HAL_Delay` 与内核 SysTick 的硬件冲突死锁。
  * **绝对非阻塞架构**：剔除所有死等逻辑。决策层下发目标状态后主动 `osDelay` 休眠让出 CPU，由后台控制任务接管电机动作，实现业务逻辑与底层控制的“状态解耦”。
  * **传感器工业级滤噪**：
    * **超声波防死锁**：重构微秒级读取逻辑，加入 30ms 绝对时钟保护，防止断线导致的系统级卡死。
    * **中断硬件防抖**：为 Bumper (碰撞) 和 Cliff (悬崖) 传感器的 EXTI 外部中断注入 20ms 硬件级防抖滤噪，保护 RTOS 内核免受机械弹片高频抖动的轰炸。
    * **防丢脉冲里程计**：彻底废弃定时器归零法，重构为“无符号 16 位差分法”读取编码器，完美杜绝因高优先级任务抢占导致的编码器脉冲永久丢失隐患。

## 硬件架构与核心外设 (Hardware Stack)

* **主控大脑**：STM32F103C8T6  (基于 STM32 HAL 库 + CMSIS_RTOS_V2 接口开发)
* **底盘与动力系统**：
  * **执行器**：双路 GA12-N20 微型减速电机，附带高精度**霍尔正交编码器**。
  * **驱动器**：TB6612FNG 双路电机驱动模块。
* **环境感知与姿态监测网**：
  * **姿态解算**：MPU6050 六轴传感器，基于 I2C 实现偏航角 (Yaw) 积分与零漂动态校准，确保直线巡航不偏航。
  * **中远探测**：HC-SR04 超声波测距。
  * **生死底线**：红外防跌落 (Cliff Sensor) + 机械碰撞开关 (Bumper)，触发 EXTI 中断实现“最高优先级物理级瞬间刹车保命”。

## 工程目录结构 (Project Structure)

本项目严格遵守标准嵌入式工程规范，已配置 `.gitignore` 过滤所有编译中间产物，保证仓库纯净度。

```text
├── Core/               # 核心应用层代码 (main.c, PID 算法, 导航状态机, 传感器底层重构库)
├── Drivers/            # 硬件底层驱动库 (STM32F1xx_HAL_Driver, CMSIS)
├── Middlewares/        # 中间件库 (FreeRTOS Source)
├── docs/               # 项目文档与实物展示图
├── MDK-ARM/            # Keil uVision5 工程配置文件 (.uvprojx)
└── .gitignore          # Git 提交流程规范配置 (已过滤 .o, .d, .axf 等 800+ 编译垃圾)