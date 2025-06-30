#!/bin/bash

# 로그 디렉토리 생성
source /home/hycon/autoware-1/install/setup.bash

mkdir -p /home/hycon/autoware_logs

echo "Starting Autoware with logging..."

# Autoware 실행 (백그라운드에서 로그와 함께)
ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/gangseo-gu \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit \
    > /home/hycon/autoware_logs/autoware_run.log 2>&1 &

AUTOWARE_PID=$!
echo "Autoware started with PID: $AUTOWARE_PID"
echo "Autoware logs saved to: /home/hycon/autoware_logs/autoware_run.log"

# Autoware가 시작될 때까지 대기
echo "Waiting for Autoware to start..."
sleep 30

echo "Starting benchmark test..."

# 벤치마크 실행
source /home/hycon/autoware-1/install/setup.bash
ros2 run my_package autoware_loop_node --ros-args -p trial_count:=100 -p timeout_seconds:=300.0

echo "Benchmark completed!"
echo "Benchmark logs saved to: /home/hycon/autoware_logs/driving_benchmark_log.log"
echo "Results saved to: /home/hycon/autoware_logs/driving_benchmark_results.csv"