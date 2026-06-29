#!/bin/bash
# Zephyr 构建和烧录脚本

set -e

# Zephyr 环境
source ~/zephyr-env/bin/activate

# 项目目录
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_DIR"

# 命令
CMD=${1:-build}

case "$CMD" in
    build)
        echo "=== Building ==="
        rm -rf build
        west build -b robomaster_board_c -p auto
        ;;
    flash)
        echo "=== Flashing ==="
        west flash --runner openocd
        ;;
    all)
        echo "=== Building and Flashing ==="
        rm -rf build
        west build -b robomaster_board_c -p auto
        west flash --runner openocd
        ;;
    clean)
        echo "=== Cleaning ==="
        rm -rf build
        ;;
    *)
        echo "用法: $0 [build|flash|all|clean]"
        echo "  build - 编译项目"
        echo "  flash - 烧录到设备"
        echo "  all   - 编译并烧录"
        echo "  clean - 清理 build 目录"
        exit 1
        ;;
esac
