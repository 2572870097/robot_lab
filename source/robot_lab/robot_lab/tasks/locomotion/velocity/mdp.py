# 添加关于无命令时自转和移动的惩罚函数

def ang_vel_without_cmd(env_ptr, command_name, command_threshold=0.05):
    """
    无命令时的角速度惩罚函数
    
    Args:
        env_ptr: 环境指针
        command_name: 命令名称
        command_threshold: 命令阈值，小于该阈值视为无命令状态
    
    Returns:
        rewards: 角速度惩罚
    """
    # 获取命令
    commands = env_ptr.get_commands(command_name)
    # 获取机器人角速度
    ang_vel = env_ptr.get_base_ang_vel()
    
    rewards = []
    for i in range(len(commands)):
        # 计算命令大小
        cmd_lin_vel_x = commands[i][0]  # x方向线速度
        cmd_lin_vel_y = commands[i][1]  # y方向线速度
        cmd_ang_vel_z = commands[i][2]  # z方向角速度
        cmd_magnitude = (cmd_lin_vel_x**2 + cmd_lin_vel_y**2 + cmd_ang_vel_z**2)**0.5
        
        if cmd_magnitude < command_threshold:
            # 无命令时，惩罚任何角速度
            ang_vel_magnitude = (ang_vel[i, 0]**2 + ang_vel[i, 1]**2 + ang_vel[i, 2]**2)**0.5
            rewards.append(-ang_vel_magnitude)
        else:
            rewards.append(0.0)  # 有命令时不惩罚
    
    # 如果没有奖励，返回0
    if not rewards:
        return 0.0
    
    return sum(rewards) / len(rewards)

def lin_vel_without_cmd(env_ptr, command_name, command_threshold=0.05):
    """
    无命令时的线速度惩罚函数
    
    Args:
        env_ptr: 环境指针
        command_name: 命令名称
        command_threshold: 命令阈值，小于该阈值视为无命令状态
    
    Returns:
        rewards: 线速度惩罚
    """
    # 获取命令
    commands = env_ptr.get_commands(command_name)
    # 获取机器人线速度
    lin_vel = env_ptr.get_base_lin_vel()
    
    rewards = []
    for i in range(len(commands)):
        # 计算命令大小
        cmd_lin_vel_x = commands[i][0]  # x方向线速度
        cmd_lin_vel_y = commands[i][1]  # y方向线速度
        cmd_ang_vel_z = commands[i][2]  # z方向角速度
        cmd_magnitude = (cmd_lin_vel_x**2 + cmd_lin_vel_y**2 + cmd_ang_vel_z**2)**0.5
        
        if cmd_magnitude < command_threshold:
            # 无命令时，惩罚任何线速度
            lin_vel_magnitude = (lin_vel[i, 0]**2 + lin_vel[i, 1]**2 + lin_vel[i, 2]**2)**0.5
            rewards.append(-lin_vel_magnitude)
        else:
            rewards.append(0.0)  # 有命令时不惩罚
    
    # 如果没有奖励，返回0
    if not rewards:
        return 0.0
    
    return sum(rewards) / len(rewards)

def stay_still_reward(env_ptr, command_name, command_threshold=0.05, velocity_threshold=0.05):
    """
    保持静止的奖励函数，在无命令时奖励静止状态
    
    Args:
        env_ptr: 环境指针
        command_name: 命令名称
        command_threshold: 命令阈值，小于该阈值视为无命令状态
        velocity_threshold: 速度阈值，小于该阈值视为静止
    
    Returns:
        rewards: 静止奖励
    """
    # 获取命令
    commands = env_ptr.get_commands(command_name)
    # 获取机器人线速度和角速度
    lin_vel = env_ptr.get_base_lin_vel()
    ang_vel = env_ptr.get_base_ang_vel()
    
    rewards = []
    for i in range(len(commands)):
        # 计算命令大小
        cmd_lin_vel_x = commands[i][0]  # x方向线速度
        cmd_lin_vel_y = commands[i][1]  # y方向线速度
        cmd_ang_vel_z = commands[i][2]  # z方向角速度
        cmd_magnitude = (cmd_lin_vel_x**2 + cmd_lin_vel_y**2 + cmd_ang_vel_z**2)**0.5
        
        if cmd_magnitude < command_threshold:
            # 无命令时，奖励保持静止
            lin_vel_magnitude = (lin_vel[i, 0]**2 + lin_vel[i, 1]**2 + lin_vel[i, 2]**2)**0.5
            ang_vel_magnitude = (ang_vel[i, 0]**2 + ang_vel[i, 1]**2 + ang_vel[i, 2]**2)**0.5
            
            # 如果线速度和角速度都小于阈值，给予奖励；速度越小，奖励越大
            velocity_factor = (lin_vel_magnitude + ang_vel_magnitude) / (2 * velocity_threshold)
            if velocity_factor < 1.0:
                rewards.append(1.0 - velocity_factor)  # 奖励越静止越大
            else:
                rewards.append(0.0)  # 速度超过阈值不给奖励
        else:
            rewards.append(0.0)  # 有命令时不给奖励
    
    # 如果没有奖励，返回0
    if not rewards:
        return 0.0
    
    return sum(rewards) / len(rewards) 