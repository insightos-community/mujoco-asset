# Copyright 2026 InsightOS
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import mujoco
import mujoco.viewer
import time

# 加载机器人模型和数据
model = mujoco.MjModel.from_xml_path("r1_pro_001.xml")
data = mujoco.MjData(model)

def robot_idle_controller(model: mujoco.MjModel, data: mujoco.MjData):
    """
    机器人待命控制器：
    1. 覆盖所有关节的控制向量（轮子、转向、躯干、双臂、夹爪）
    2. 所有控制量设为0，无任何主动指令
    3. 仅维持被动动力学（重力、阻尼），机器人保持待命静止
    """
    # 校验控制维度（鲁棒性保障，避免索引越界）
    if model.nu > 0:
        # 核心逻辑：所有控制量置0，无任何主动指令
        data.ctrl[:] = 0.0
        
        # 可选：打印控制状态（验证所有控制量为0）
        if int(data.time * 10) % 10 == 0:  # 每1秒打印一次
            print(f"仿真时间: {data.time:.1f}s | 所有控制量均为0，机器人待命")

# 绑定全局控制回调（覆盖所有关节）
mujoco.set_mjcb_control(robot_idle_controller)

# ==============================
# 标准仿真循环（step1+step2模式）
# ==============================
if __name__ == "__main__":
    with mujoco.viewer.launch(model, data) as viewer:
        # 仿真参数
        sim_frequency = 60 
        dt = 1.0 / sim_frequency
        
        # 仿真主循环
        while viewer.is_running():
            # Step1：计算位置、速度、接触等状态
            mujoco.mj_step1(model, data)
            
            # Step2：计算加速度、约束 + 积分推进
            mujoco.mj_step2(model, data)
            
            # 可视化同步 + 帧率控制
            viewer.sync()
            time.sleep(dt)

    print("仿真结束，机器人全程处于待命状态，无任何主动控制指令")
