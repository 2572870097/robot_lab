import numpy as np
import argparse
np.set_printoptions(threshold=.100)

def verify_motion_file(npz_file):
    # 加载npz文件
    data = np.load(npz_file, allow_pickle=True)
    
    # 输出所有键名及对应的数据类型和形状
    print("数据内容：")
    for key in data:
        arr = data[key]
        print(f"{key}: dtype = {arr.dtype}, shape = {arr.shape}")
        if key == 'dof_names':
            print(f"关节名称: {arr}")
        if key == 'body_names':
            print(f"身体部位名称: {arr}")
        if key == "body_positions":
            print(f"身体部位位置: {arr}")
    
    # 示例：读取FPS（每秒帧数）
    if 'fps' in data:
        fps = data['fps'].item()  # 如果存储的是标量，使用 .item() 转换为 Python 数值
        print("\nFPS:", fps)
    else:
        print("\n警告：未找到 'fps' 键！")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="验证motion文件的内容")
    parser.add_argument("--file", type=str, required=True, help="npz文件路径")
    args = parser.parse_args()
    
    verify_motion_file(args.file)
