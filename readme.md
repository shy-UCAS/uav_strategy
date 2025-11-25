# BlueSingleAgent程序

## 运行配置要求

- 代码位置：`/examples/uavs_strategy/BlueSingleAgent `

- 代码运行之前首先需要为Spade BDI框架配置Openfire服务器和MySQL数据库
  
  - 启动数据库：` mysql -u root -p  `之后再启动Openfire服务器

- 需要在本地配置Redis数据库用于存储所有无人机以及地图数据，并用可视化
  
  - 在Win11系统中安装wsl2并安装Redis
  
  - 启动Redis服务器：`sudo service redis-server start`

- 在最外层目录`uav_strategy`下启动代码：
  
  - ```
    # 启动主程序
    python -m examples.uavs_strategy.BlueSingleAgent --server 127.0.0.1 --password 202127
    ```
  
  - 启动可视化：`python -m examples.uavs_strategy.redis_data_visualize`

## 实现的功能

- 实现了一个基于BDI 框架的无人机Agent任务规划及执行管理系统
  
  - 首先在ASL文件定义任务序列
  
  - 每个无人机通过 `BlueUAVAgent` 类实例化并初始化参数，加入BDI运行框架中
  
  - 从ASL中获取当前轨迹任务指令，并且从`add_custom_actions` 方法注册各种轨迹规划相关的实现方法，得到一条预规划轨迹
  
  - 注册周期性行为，从`Redis` 获取蓝方和红方 UAV 的态势信息，通过 `APFStep` 使得无人机Agent可以周期性地根据势场规划其下一步的位置，进而更新Redis中的无人机信息
  
  - 集群轨迹规划及编队控制

- `class BlueUAVAgent(BDIAgent)`继承自 SPADE 的 BDIAgent
  
  - 在初始化阶段：
    
    - 加载一个 ASL文件，定义了Agent的行为规则和决策逻辑
    
    - 绑定 Redis 通信通道，通过`UavRedisIO`类连接。
    
    - 加载默认设施信息、将经纬度起点转换成 UTM 轨迹
    
    - 同步初始位置、轨迹与 lookahead 等状态到 Redis
    
    - 添加编队状态描述信息
  
  - `register_planning_actions`封装与无人机轨迹规划和高度插值相关的自定义动作，并通过`add_custom_actions`方法把这些动作注册到 BDI 规划器，使 Agent能够调用这些动作实现轨迹规划
  
  - 在 setup 中为Agent加载`FetchWorldState`、`APFStep` 等周期行为,定期从Redis服务器中读取、刷新红蓝方无人机的坐标数据，并执行APF人工势场轨迹推理
    
    - `FetchWorldState`从Redis服务器中读取数据
    
    - `APFStep`执行人工势场法实现轨迹推理并将新的无人机数据写入Redis服务器
  
  - 在`main`函数中，创建并启动多个 `BlueUAVAgent` 实例，测试功能
  
  ## 

## 库函数

- `redis_modules`
  
  - `uav_redis_io.py`:封装了与 Redis 数据库交互的功能，主要用于管理无人机（蓝方和红方）的位置信息、轨迹、参考轨迹和预瞄点等数据
    
    1. **无人机ID管理**：
       
       * `add_uav_id`: 将无人机ID添加到蓝方或红方的ID集合中。
       
       * `remove_uav_id`: 从蓝方或红方的ID集合中移除无人机ID。
       
       * `get_ids`: 获取蓝方或红方所有的无人机ID。
       
       * `scan_ids_by_key`: 通过扫描键名获取ID集合，用于当没有维护ID集合时通过键推断。
    
    2. **单机位置的读写**：
       
       * `set_pos`: 设置某个无人机的位置（包括坐标和时间戳）。
       
       * `get_pos`: 获取某个无人机的最新位置。
       
       * `mget_pos`: 批量获取多个无人机的位置。
    
    3. **轨迹管理**：
       
       * `set_traj`: 设置某个无人机的轨迹，覆盖写入新的轨迹。
       
       * `append_traj_points`: 向某个无人机的轨迹中追加新的轨迹点。
       
       * `get_traj`: 获取某个无人机的完整轨迹。
       
       * `mget_traj`: 批量获取多个无人机的轨迹。
       
       * `clear_traj`: 清除某个无人机的轨迹。
    
    4. **参考轨迹管理**：
       
       * `set_ref_traj`: 设置某个无人机的参考轨迹。
       
       * `get_ref_traj`: 获取某个无人机的参考轨迹。
    
    5. **预瞄点管理**：
       
       * `set_lookahead`: 设置某个无人机在参考轨迹中当前走到的预瞄点索引。
       
       * `get_lookahead`: 获取某个无人机的预瞄点索引。
    
    6. **计算距离**：
       
       * `get_dist_2d`: 计算某个无人机当前位置与参考轨迹终点的平面距离。
* behaviors_modules:
  
  * `uav_periodic_behaviours.py`: 定义了几个主要的周期性行为类，分别是 `FormationAPFStep`、`APFStep` 和 `FetchWorldState`，它们都继承自 `PeriodicBehaviour`，用于控制无人机的运动和状态更新
    
    * `APFStep` (单机势场步进行为)
      
      * **功能**：控制单机无人机根据参考轨迹进行运动。它通过以下步骤执行：
        
        * **获取参考轨迹**：获取当前参考轨迹中的目标点（预瞄点）。
        
        * **计算引力**：根据无人机当前位置与目标点之间的距离，计算引力，将无人机吸引向目标点。
        
        * **计算斥力**：根据其他无人机的位置，计算避障的斥力，避免碰撞。
        
        * **合成引力和斥力**：将引力和斥力合成，得到总的运动方向。
        
        * **更新位置**：更新无人机的位置，确保移动步长符合最大步长要求。
        
        * **写入位置**：更新无人机的当前位置，并将其写入 Redis。
      
      * **适用场景**：适用于单机任务执行，尤其是在无人机需要跟踪参考轨迹并避开障碍物时
    
    * `FetchWorldState` (世界状态查询行为)
      
      * **功能**：定期查询蓝方和红方无人机的位置信息，并将这些信息更新到代理的 `world` 属性中。具体步骤包括：
        
        * **获取无人机ID**：查询蓝方和红方无人机的 ID。
        
        * **批量获取位置信息**：使用 Redis 批量获取蓝方和红方无人机的位置数据。
        
        * **更新世界状态**：将获取到的位置信息更新到代理的 `world["blue_pos"]` 和 `world["red_pos"]` 中。
      
      * **适用场景**：适用于定期获取并更新环境中所有无人机的位置信息，确保每个代理都能保持对环境的实时感知。
        
        
- `uav_planning_actions.py` 这个库定义了一些与轨迹规划和BDI动作相关的功能，主要用于无人机在不同飞行任务中的行为控制。
  
  - **轨迹规划工具类 `PlanningLib`**
    
    * **功能**：为无人机提供轨迹规划与高度插值功能。它通过 `self.agent` 访问无人机的状态（如轨迹、设施和高度范围等），并实现了以下方法：
      
      * **高度插值 (`insert_height_val`)**：给二维轨迹的起点和终点添加高度，并根据轨迹类型（如突破、逃逸、绕行）选择不同的插值方法。
      
      * **突破规划 (`plan_breakthrough_target` 和 `plan_breakthrough_targettype`)**：根据目标设施（如防空、指挥所、探测设施等）生成一个二维轨迹。
      
      * **迂回规划 (`plan_detour`)**：通过获取目标设施的多边形边界，计算一个绕行路径。
      
      * **逃逸规划 (`plan_escape`)**：在威胁区域内计算从危险区域到最近边界点的逃逸轨迹。
  
  - **轨迹规划动作注册**
    
    * **功能**：通过 `register_planning_actions` 方法，将轨迹规划与BDI动作结合。每个动作的执行由特定的命令触发，具体实现包括：
      
      * **突破 (`.act_breakthrough`)**：执行突破动作，计算从起点到目标（如防空设施、指挥所等）的轨迹。
      
      * **逃逸 (`.act_escape`)**：执行逃逸动作，计算从危险区域到逃逸边界的轨迹。
      
      * **迂回 (`.act_detour`)**：执行绕行动作，计算绕过目标设施的轨迹。
      
      * **攻击 (`.act_attack`)**：一个占位动作，打印攻击目标（暂未实现详细行为）。
      
      * **获取位置 (`.act_get_position`)**：获取无人机当前位置，并将其赋值给ASL中的变量。
      
      * **编队相关动作**：
        
        * **加入编队 (`.act_join_formation`)**：设置无人机为编队成员并指定偏移量。
        
        * **离开编队 (`.act_leave_formation`)**：将无人机从编队中移除，恢复独立飞行状态。
