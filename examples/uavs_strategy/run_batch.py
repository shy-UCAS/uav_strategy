import subprocess
import sys
import time
import os

# 配置项
TOTAL_RUNS = 100
SCRIPT_MODULE = "examples.uavs_strategy.uav_dynamic_agents02"

# 获取项目根目录 (假设当前脚本在 examples/uavs_strategy/ 下)
# 向上两级找到 workspace root
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, "..", ".."))

def run_batch():
    print(f"Start batch processing: {TOTAL_RUNS} runs.")
    print(f"Target script: {SCRIPT_MODULE}")
    print(f"Project root: {project_root}")

    for i in range(1, TOTAL_RUNS + 1):
        print(f"\n{'='*20} Run {i}/{TOTAL_RUNS} {'='*20}")
        start_time = time.time()
        
        try:
            # 使用 sys.executable 确保使用相同的 Python 解释器
            # cwd 设置为项目根目录，以便模块导入正确工作
            cmd = [sys.executable, "-m", SCRIPT_MODULE]
            result = subprocess.run(cmd, cwd=project_root, check=True)
            
        except subprocess.CalledProcessError as e:
            print(f"[Run {i}] Failed with error: {e}")
            # 如果需要出错停止，可以 break
            # break 
        except KeyboardInterrupt:
            print("\nBatch processing interrupted by user.")
            break
            
        end_time = time.time()
        print(f"[Run {i}] Finished in {end_time - start_time:.2f} seconds.")
        
        # 短暂休眠，确保文件句柄释放和端口清理
        time.sleep(2)

    print("\nBatch processing completed.")

if __name__ == "__main__":
    run_batch()
