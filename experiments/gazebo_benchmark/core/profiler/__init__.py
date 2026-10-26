"""ROS-independent hardware queries and scaling resource sampling."""
import subprocess
import time


def query_gpu():
    try:
        out = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used", "--format=csv,noheader,nounits"],
            text=True,
        )
        gpu_util, mem_used = out.strip().split(",")
        return float(gpu_util), float(mem_used)
    except Exception:
        return 0.0, 0.0


def query_gpu_int():
    try:
        res = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used", "--format=csv,noheader,nounits"],
            text=True, timeout=1.0,
        ).strip().split(',')
        return int(res[0]), int(res[1])
    except Exception:
        return 0, 0


def sample_resources(start, duration, cpu_percent, read_gpu, read_breakdown=None):
    """Sample at the historical 0.5s cadence, in CPU/GPU/breakdown order."""
    cpu, gpu, vram, breakdown = [], [], [], []
    while time.time() - start < duration:
        cpu.append(cpu_percent(interval=None))
        gpu_util, memory_used = read_gpu()
        gpu.append(gpu_util)
        vram.append(memory_used)
        if read_breakdown is not None:
            gz, bridge, nav2, ros = read_breakdown()
            breakdown.append((gz, bridge, nav2, ros))
        time.sleep(0.5)
    return cpu, gpu, vram, breakdown
