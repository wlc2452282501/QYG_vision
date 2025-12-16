sleep 5

# 切换到当前工程目录（请确认路径是否与实际一致）
cd /home/rm/rm_vision_2025 || exit 1

# 使用 screen 后台运行 watchdog.sh，并把标准输出写入日志
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d -m \
    bash -c "./watchdog.sh"
