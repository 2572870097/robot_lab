```
ln -s ~/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/humanoid_amp ~/IsaacLab/

ln -s ~/IsaacLab/scripts/reinforcement_learning/skrl ~/IsaacLab/
```
Train:
```
./isaaclab.sh -p ~/IsaacLab/skrl/train.py --task Isaac-G1-AMP-Walk-Direct-v0 --headless

or

./isaaclab.sh -p ~/IsaacLab/skrl/train.py --task Isaac-G1-AMP-Dance-Direct-v0 --headless
```
Eval：
```
./isaaclab.sh -p ~/IsaacLab/skrl/play.py --task Isaac-G1-AMP-Walk-Direct-v0 --num_envs 32 
```
TensorBoard:
```
./isaaclab.sh -p -m tensorboard.main --logdir logs/skrl/
```
The parameters of the code in this repository have not been fine-tuned. Currently, the walk performance is acceptable, but the dance performance is quite poor. Due to personal bussiness, I will not begin to debug until summer.

The dataset and URDF files are from [Hugging Face](https://huggingface.co/datasets/unitreerobotics/LAFAN1_Retargeting_Dataset). 

**Contributions**, **discussions**, and stars are all welcome! ❥(^_-)



# RUN process
# Dance code
## train
```
python scripts/skrl/train.py --task Isaac-G1-AMP-Dance-Direct-v0 --headless --algorithm AMP 
```
## play
```
python scripts/skrl/play.py --task Isaac-G1-AMP-Dance-Direct-v0 --algorithm AMP --num_envs 1 --checkpoint 
```

# Walk code
## train
```
python scripts/skrl/train.py --task Isaac-G1-AMP-Walk-Direct-v0 --headless --algorithm AMP 
```
## play
```
python scripts/skrl/play.py --task Isaac-G1-AMP-Walk-Direct-v0 --algorithm AMP --num_envs 1 --checkpoint 
```

# 对于unitreee_g1_amp的学习步骤
## 项目结构
整个项目主要是由五个部分组成：usd agent motions，env，skrl_rl
 1. usd提供机器人结构
 2. agent配置机器人的rl属性
 3. motions提供运动文件
 4. env配置机器人、环境和奖励
 5. skrl_rl训练

## 第一步，直接看train文件
 流程为：
  1. override configurations with non-hydra CLI arguments
  2. set the agent and environment seed from command line
  3. create isaac environment
  4. wrap around environment for skrl
  5. run training
  6. close the simulator

## 第二步，看__init__. 注册器
```
gym.register(
    id="Isaac-G1-AMP-Walk-Direct-v0",
    entry_point=f"{__name__}.g1_amp_env:G1AmpEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.g1_amp_env_cfg:G1AmpWalkEnvCfg",
        "skrl_amp_cfg_entry_point": f"{agents.__name__}:skrl_g1_walk_amp_cfg.yaml",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_g1_walk_amp_cfg.yaml",
    },
)
```
id是传入train中的task的任务id

## 第三步，看entry_point 
```
entry_point=f"{__name__}.g1_amp_env:G1AmpEnv",
```
就是g1_amp_env 文件下的G1AmpEnv类
