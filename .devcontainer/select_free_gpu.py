#!/usr/bin/env python3
"""Select the least-loaded CUDA GPU and optionally export it via CUDA_VISIBLE_DEVICES."""
import subprocess
import sys
import os
import re


def get_free_gpu() -> int:
    result = subprocess.run(
        ["nvidia-smi",
         "--query-gpu=index,memory.free,memory.total,utilization.gpu",
         "--format=csv,noheader,nounits"],
        capture_output=True, text=True, check=True
    )
    gpus = []
    for line in result.stdout.strip().splitlines():
        idx, mem_free, mem_total, util = [x.strip() for x in line.split(",")]
        gpus.append((int(idx), int(mem_free), int(mem_total), int(util)))
    # Primary sort: most free memory; secondary sort: lowest utilization
    best = max(gpus, key=lambda g: (g[1], -g[3]))
    return best[0]


def set_bashrc(gpu_id: int) -> None:
    bashrc = os.path.expanduser("~/.bashrc")
    marker = "# CUDA_VISIBLE_DEVICES auto-set by select_free_gpu.py"
    try:
        with open(bashrc, "r") as f:
            content = f.read()
    except FileNotFoundError:
        content = ""
    # Remove any previously injected block (marker line + export line)
    content = re.sub(
        rf"{re.escape(marker)}\nexport CUDA_VISIBLE_DEVICES=\d+\n", "", content
    )
    content = content.rstrip("\n") + f"\n{marker}\nexport CUDA_VISIBLE_DEVICES={gpu_id}\n"
    with open(bashrc, "w") as f:
        f.write(content)
    print(f"Set CUDA_VISIBLE_DEVICES={gpu_id} in ~/.bashrc")


if __name__ == "__main__":
    gpu_id = get_free_gpu()
    if "--set-bashrc" in sys.argv:
        set_bashrc(gpu_id)
    else:
        print(gpu_id)
