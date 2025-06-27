#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import numpy as np
import threading
import os
import csv
import logging
from datetime import datetime

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from autoware_auto_system_msgs.msg import AutowareState
from std_srvs.srv import Trigger
from tier4_external_api_msgs.srv import Engage
from tier4_external_api_msgs.msg import ResponseStatus
from autoware_adapi_v1_msgs.srv import ClearRoute

class AutowareBenchmarkController(Node):
    """Autoware Autonomous Driving Benchmark Controller"""

    def __init__(self):
        super().__init__('autoware_benchmark_controller')
        
        # Declare ROS2 parameters
        self.declare_parameter('trial_count', 100)  # Default 100 trials
        self.declare_parameter('timeout_seconds', 300.0)  # Default 5 minutes timeout
        
        # Publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pose_pub = self.create_publisher(
            PoseStamped, '/planning/mission_planning/goal', 10)
        
        # Subscriber
        self.state_sub = self.create_subscription(
            AutowareState, '/autoware/state', self.state_callback, 10)
        
        # Service clients
        self.engage_client = self.create_client(
            Engage, '/api/autoware/set/engage')
        self.clear_route_client = self.create_client(
            ClearRoute, '/api/routing/clear_route')
        
        # State tracking
        self.current_state = None
        self.is_driving = False
        self.driving_started = False
        self.goal_reached = threading.Event()
        
        # Driving time tracking
        self.autonomous_start_time = None
        self.autonomous_end_time = None
        self.actual_driving_time = 0.0
        
        # Configuration from parameters
        self.total_trials = self.get_parameter('trial_count').get_parameter_value().integer_value
        self.max_trial_time = self.get_parameter('timeout_seconds').get_parameter_value().double_value
        
        # Test poses
        self.initial_pose = {
            'x': 6747.92529296875,
            'y': 60173.984375,
            'z': 0.0,
            'qx': 0.0, 'qy': 0.0,
            'qz': 0.7189788872253966,
            'qw': 0.6950319127379191
        }
        
        self.goal_pose = {
            'x': 6747.75,
            'y': 60135.046875,
            'z': 0.0,
            'qx': 0.0, 'qy': 0.0,
            'qz': 0.6900725510622704,
            'qw': 0.7237401980478977
        }
        
        # Results tracking
        self.results = []
        # CSV 결과 파일 경로
        self.csv_file_path = '/home/hycon/autoware_logs/driving_benchmark_results.csv'
        
        # 로그 디렉토리 생성
        os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)
        
        self.get_logger().info("Autoware Benchmark Controller started")
        self.get_logger().info(f"Trial count: {self.total_trials}")
        self.get_logger().info(f"Timeout: {self.max_trial_time} seconds")
        self.get_logger().info(f"CSV file path: {self.csv_file_path}")

    def state_callback(self, msg):
        """Autoware state callback - simplified for goal detection only"""
        prev_state = self.current_state
        self.current_state = msg.state
        
        # Detect DRIVING state (only numeric format for autoware_auto_system_msgs)
        is_driving_now = self._is_driving_state(self.current_state)
        was_driving_before = self._is_driving_state(prev_state) if prev_state is not None else False
        
        # Log state changes for debugging
        if prev_state != self.current_state:
            log_msg = f"State change: {prev_state} -> {self.current_state} (driving: {was_driving_before} -> {is_driving_now})"
            self.get_logger().info(log_msg)
        
        # 목표 도달 감지 조건 디버깅 - 이미 저장된 테스트는 제외
        if was_driving_before and not is_driving_now and not getattr(self, 'trial_already_saved', False):
            print(f"*** GOAL REACHED CONDITION MET ***")
            self.goal_reached.set()
            log_msg = "*** GOAL REACHED! ***"
            self.get_logger().info(log_msg)
            
            # 이 테스트 결과가 이미 저장되었음을 표시
            self.trial_already_saved = True
            
            try:
                # 목표 도달 즉시 CSV에 결과 저장
                print(f"*** GOAL REACHED - TRYING TO SAVE TO CSV ***")
                
                # 현재 시간을 기준으로 저장 (간단하게)
                print(f"*** STEP 1: Getting time ***")
                end_time = time.time()
                end_timestamp = datetime.now().strftime("%H%M%S")
                
                # 속성이 있으면 사용, 없으면 기본값
                print(f"*** STEP 2: Getting attributes ***")
                trial_num = getattr(self, 'current_trial_num', 1)
                start_time = getattr(self, 'trial_start_time', end_time - 60)  # 기본 1분
                start_timestamp = getattr(self, 'trial_start_timestamp', end_timestamp)
                
                print(f"*** STEP 3: Calculating duration ***")
                trial_duration = end_time - start_time
                
                print(f"*** STEP 4: Creating result dict ***")
                result = {
                    'trial': trial_num,
                    'duration': trial_duration,
                    'actual_driving_time': self.actual_driving_time,
                    'success': True,  # 목표 도달했으므로 성공
                    'start_time': start_timestamp,
                    'end_time': end_timestamp
                }
                
                print(f"*** STEP 5: About to call save_single_trial_to_csv ***")
                print(f"*** GOAL REACHED - SAVING TRIAL {trial_num} TO CSV ***")
                self.save_single_trial_to_csv(trial_num, result)
                print(f"*** STEP 6: CSV SAVE COMPLETED ***")
                
            except Exception as e:
                print(f"*** ERROR DURING CSV SAVE: {e} ***")
                print(f"*** ERROR TYPE: {type(e).__name__} ***")
                import traceback
                print(f"*** TRACEBACK: {traceback.format_exc()} ***")
                self.get_logger().error(f"Error during CSV save: {e}")
        else:
            # 조건이 맞지 않는 경우 디버깅
            if prev_state != self.current_state:  # 상태 변화가 있을 때만
                print(f"*** GOAL NOT REACHED - was_driving_before: {was_driving_before}, is_driving_now: {is_driving_now} ***")

    def _is_driving_state(self, state):
        """Check if state is DRIVING - autoware_auto_system_msgs uses numeric values"""
        if state is None:
            return False
        
        # For autoware_auto_system_msgs, state is always numeric
        # Based on observations: 
        # 2=waiting/arrived, 4=driving_start, 5=driving, 6=arrived_at_goal
        if isinstance(state, int):
            # States 3, 4, 5 are considered driving (3=planning, 4=driving_start, 5=driving)
            # State 6 is arrival/completion, not driving
            is_driving = state in [3, 4, 5]
            return is_driving
        
        return False

    def wait_for_service(self, client, service_name, timeout=10.0):
        """Wait for service availability"""
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"{service_name} service not found (continuing anyway)")
            return False
        return True

    def clear_route(self):
        """Clear existing route"""
        if self.wait_for_service(self.clear_route_client, "Clear Route", 3.0):
            try:
                request = ClearRoute.Request()
                future = self.clear_route_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.result() and future.result().status.success:
                    self.get_logger().info("Existing route cleared successfully")
                else:
                    self.get_logger().warn("Failed to clear route")
            except Exception as e:
                self.get_logger().warn(f"Failed to clear route: {e}")

    def set_initial_pose(self):
        """Set initial pose"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.pose.position.x = self.initial_pose['x']
        msg.pose.pose.position.y = self.initial_pose['y']
        msg.pose.pose.position.z = self.initial_pose['z']
        
        msg.pose.pose.orientation.x = self.initial_pose['qx']
        msg.pose.pose.orientation.y = self.initial_pose['qy']
        msg.pose.pose.orientation.z = self.initial_pose['qz']
        msg.pose.pose.orientation.w = self.initial_pose['qw']
        
        # Set covariance matrix (for localization stability)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.1   # x
        msg.pose.covariance[7] = 0.1   # y
        msg.pose.covariance[35] = 0.1  # yaw
        
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f"Initial pose set: ({self.initial_pose['x']:.2f}, {self.initial_pose['y']:.2f})")

    def set_goal_pose(self):
        """Set goal pose"""
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.position.x = self.goal_pose['x']
        msg.pose.position.y = self.goal_pose['y']
        msg.pose.position.z = self.goal_pose['z']
        
        msg.pose.orientation.x = self.goal_pose['qx']
        msg.pose.orientation.y = self.goal_pose['qy']
        msg.pose.orientation.z = self.goal_pose['qz']
        msg.pose.orientation.w = self.goal_pose['qw']
        
        self.goal_pose_pub.publish(msg)
        self.get_logger().info(f"Goal pose set: ({self.goal_pose['x']:.2f}, {self.goal_pose['y']:.2f})")

    def engage_autonomous(self):
        """Start autonomous driving mode using Engage service"""
        if not self.wait_for_service(self.engage_client, "Engage", 10.0):
            self.get_logger().error("Engage service not found!")
            return False
        
        try:
            # Create engage request
            request = Engage.Request()
            request.engage = True
            
            self.get_logger().info("Requesting autonomous engage...")
            future = self.engage_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result():
                response = future.result()
                if response.status.code == ResponseStatus.SUCCESS:
                    self.get_logger().info("Autonomous engage successful!")
                    return True
                else:
                    self.get_logger().error(f"Engage failed: {response.status.message} (code: {response.status.code})")
                    return False
            else:
                self.get_logger().error("Engage service no response")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Engage service error: {e}")
            return False

    def disengage_autonomous(self):
        """Disengage autonomous driving mode using Engage service"""
        if not self.wait_for_service(self.engage_client, "Disengage", 10.0):
            self.get_logger().error("Disengage service not found!")
            return False
        
        try:
            # Create disengage request
            request = Engage.Request()
            request.engage = False
            
            self.get_logger().info("Requesting autonomous disengage...")
            future = self.engage_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result():
                response = future.result()
                if response.status.code == ResponseStatus.SUCCESS:
                    self.get_logger().info("Autonomous disengage successful!")
                    return True
                else:
                    self.get_logger().error(f"Disengage failed: {response.status.message} (code: {response.status.code})")
                    return False
            else:
                self.get_logger().error("Disengage service no response")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Disengage service error: {e}")
            return False

    def emergency_stop(self):
        """Emergency stop the vehicle and force complete system reset"""
        self.get_logger().info("Emergency stop initiated - forcing complete Autoware reset")
        
        # Try multiple times to ensure disengage
        for attempt in range(5):
            self.get_logger().info(f"Emergency disengage attempt {attempt + 1}/5")
            if self.disengage_autonomous():
                self.get_logger().info(f"Emergency disengage successful on attempt {attempt + 1}")
                break
            time.sleep(2.0)
        else:
            self.get_logger().error("Emergency disengage failed after 5 attempts")
        
        # Clear routes multiple times
        for i in range(3):
            self.clear_route()
            time.sleep(1.0)
        
        # Wait longer for complete system reset
        time.sleep(10.0)
        self.get_logger().info("Emergency stop completed - Autoware should be fully reset")

    def run_single_trial(self, trial_num):
        """Run a single autonomous driving trial with enhanced error recovery"""
        self.get_logger().info(f"TRIAL #{trial_num}: Starting trial")
        
        max_recovery_attempts = 3
        
        for attempt in range(max_recovery_attempts):
            try:
                self.get_logger().info(f"TRIAL #{trial_num}: Attempt {attempt + 1}/{max_recovery_attempts}")
                
                # Step 1: Force clean state at start of each trial
                self.is_driving = False
                self.driving_started = False
                self.goal_reached.clear()
                self.autonomous_start_time = None
                self.autonomous_end_time = None
                self.actual_driving_time = 0.0
                self.trial_already_saved = False  # 테스트 결과 저장 플래그 초기화
                
                reset_msg = f"TRIAL #{trial_num}: State forcefully reset - is_driving={self.is_driving}, driving_started={self.driving_started}"
                self.get_logger().info(reset_msg)
                
                # Step 2: Force system reset (especially important after errors)
                self.get_logger().info(f"TRIAL #{trial_num}: Starting system reset (attempt {attempt + 1})")
                
                # CRITICAL: Force disengage Autoware autonomous mode first
                self.get_logger().info(f"TRIAL #{trial_num}: Force disengaging autonomous mode...")
                for disengage_attempt in range(3):  # Try multiple times
                    if self.disengage_autonomous():
                        self.get_logger().info(f"TRIAL #{trial_num}: Successfully disengaged autonomous mode")
                        break
                    else:
                        self.get_logger().warn(f"TRIAL #{trial_num}: Disengage attempt {disengage_attempt + 1} failed, retrying...")
                        time.sleep(2.0)
                else:
                    self.get_logger().error(f"TRIAL #{trial_num}: Failed to disengage after 3 attempts")
                
                # Wait for Autoware to fully stop autonomous mode
                time.sleep(5.0)
                
                # Verify that Autoware is actually disengaged
                self.get_logger().info(f"TRIAL #{trial_num}: Verifying system state after disengage...")
                self.get_logger().info(f"TRIAL #{trial_num}: Current Autoware state: {self.current_state}")
                
                # If still in driving state, this is a problem
                if self._is_driving_state(self.current_state):
                    self.get_logger().error(f"TRIAL #{trial_num}: WARNING - Autoware still in driving state after disengage!")
                    # Try emergency stop
                    self.emergency_stop()
                
                # Clear existing route multiple times to ensure cleanup
                self.get_logger().info(f"TRIAL #{trial_num}: Clearing routes...")
                for i in range(3):  # Increased to 3 attempts
                    self.clear_route()
                    time.sleep(1.5)
                
                # Step 3: Set initial pose with verification
                self.get_logger().info(f"TRIAL #{trial_num}: Setting initial pose")
                self.set_initial_pose()
                time.sleep(4.0)  # Increased wait time for localization
                
                # Step 4: Set goal pose with verification
                self.get_logger().info(f"TRIAL #{trial_num}: Setting goal pose")
                self.set_goal_pose()
                time.sleep(3.0)  # Increased wait time for path planning
                
                # Step 5: Start autonomous driving
                self.get_logger().info(f"TRIAL #{trial_num}: Starting autonomous mode")
                if not self.engage_autonomous():
                    self.get_logger().error(f"TRIAL #{trial_num}: Failed to start autonomous mode")
                    if attempt < max_recovery_attempts - 1:
                        self.get_logger().warn(f"TRIAL #{trial_num}: Retrying system reset...")
                        continue
                    else:
                        return False
                
                # CRITICAL: Mark test as in progress ONLY after engage is successful
                self.test_in_progress = True
                # 실제 주행 시작 시간 기록 (autonomous 전환 성공 시점)
                self.autonomous_start_time = time.time()
                self.get_logger().info(f"TRIAL #{trial_num}: Test marked as IN PROGRESS - state monitoring active")
                self.get_logger().info(f"TRIAL #{trial_num}: Autonomous driving started at {datetime.now().strftime('%H:%M:%S')}")
                
                # Step 6: Wait for goal arrival with periodic status check
                self.get_logger().info(f"TRIAL #{trial_num}: Waiting for goal arrival (timeout: {self.max_trial_time}s)")
                
                # Check status every 30 seconds during wait
                check_interval = 30.0
                remaining_time = self.max_trial_time
                
                while remaining_time > 0:
                    wait_time = min(check_interval, remaining_time)
                    
                    if self.goal_reached.wait(timeout=wait_time):
                        # 실제 주행 종료 시간 기록 (성공 시점)
                        self.autonomous_end_time = time.time()
                        
                        success_msg = f"TRIAL #{trial_num}: SUCCESS - Goal reached!"
                        self.get_logger().info(success_msg)
                        
                        # 실제 주행시간 계산
                        if self.autonomous_start_time:
                            self.actual_driving_time = self.autonomous_end_time - self.autonomous_start_time
                            driving_time_msg = f"TRIAL #{trial_num}: Actual driving time: {self.actual_driving_time:.2f} seconds"
                            self.get_logger().info(driving_time_msg)
                        else:
                            self.actual_driving_time = 0.0
                        
                        # Mark test as completed
                        self.test_in_progress = False
                        
                        # Ensure proper cleanup after success - CRITICAL for next test
                        self.get_logger().info(f"TRIAL #{trial_num}: Performing cleanup after success...")
                        for cleanup_attempt in range(3):
                            if self.disengage_autonomous():
                                self.get_logger().info(f"TRIAL #{trial_num}: Successfully disengaged after success")
                                break
                            time.sleep(1.0)
                        else:
                            self.get_logger().warn(f"TRIAL #{trial_num}: Failed to disengage after success")
                        
                        time.sleep(3.0)  # Wait for complete stop
                        return True
                    
                    remaining_time -= wait_time
                    
                    # Log periodic status
                    if remaining_time > 0:
                        self.get_logger().info(f"TRIAL #{trial_num}: Still driving... {remaining_time:.0f}s remaining")
                
                # Timeout occurred
                # 실제 주행 종료 시간 기록 (실패 시점)
                self.autonomous_end_time = time.time()
                
                error_msg = f"TRIAL #{trial_num}: TIMEOUT ({self.max_trial_time} seconds)"
                self.get_logger().error(error_msg)
                
                error_msg = f"TRIAL #{trial_num}: Current state at timeout: {self.current_state}, is_driving: {self.is_driving}"
                self.get_logger().error(error_msg)
                
                # 실제 주행시간 계산 (실패한 경우에도)
                if self.autonomous_start_time:
                    self.actual_driving_time = self.autonomous_end_time - self.autonomous_start_time
                    driving_time_msg = f"TRIAL #{trial_num}: Actual driving time (failed): {self.actual_driving_time:.2f} seconds"
                    self.get_logger().info(driving_time_msg)
                else:
                    self.actual_driving_time = 0.0
                
                # 타임아웃 즉시 CSV에 실패 결과 저장
                end_timestamp = datetime.now().strftime("%H%M%S")
                trial_duration = self.autonomous_end_time - self.trial_start_time
                
                result = {
                    'trial': trial_num,
                    'duration': trial_duration,
                    'actual_driving_time': self.actual_driving_time,
                    'success': False,  # 타임아웃이므로 실패
                    'start_time': self.trial_start_timestamp,
                    'end_time': end_timestamp
                }
                
                print(f"*** TIMEOUT - SAVING TRIAL {trial_num} TO CSV ***")
                self.save_single_trial_to_csv(trial_num, result)
                self.trial_already_saved = True  # 타임아웃으로 저장했음을 표시
                
                # Mark test as completed
                self.test_in_progress = False
                
                # Force stop after timeout AND reset state variables
                self.emergency_stop()
                
                # CRITICAL: Force reset state variables after timeout to prevent chain failures
                self.is_driving = False
                self.driving_started = False
                self.goal_reached.clear()
                self.get_logger().info(f"TRIAL #{trial_num}: State forcefully reset after timeout")
                
                return False
                
            except Exception as e:
                error_msg = f"TRIAL #{trial_num}: ERROR on attempt {attempt + 1}: {e}"
                self.get_logger().error(error_msg)
                
                error_msg = f"TRIAL #{trial_num}: Current state at error: {self.current_state}, is_driving: {self.is_driving}"
                self.get_logger().error(error_msg)
                
                # Mark test as completed
                self.test_in_progress = False
                
                # Emergency recovery
                try:
                    self.emergency_stop()
                except:
                    pass
                
                # CRITICAL: Force reset state variables after exception to prevent chain failures  
                self.is_driving = False
                self.driving_started = False
                self.goal_reached.clear()
                self.get_logger().info(f"TRIAL #{trial_num}: State forcefully reset after exception")
                
                if attempt < max_recovery_attempts - 1:
                    self.get_logger().warn(f"TRIAL #{trial_num}: Attempting recovery...")
                    time.sleep(5.0)  # Wait longer before retry
                    continue
                else:
                    self.get_logger().error(f"TRIAL #{trial_num}: FAILED after {max_recovery_attempts} attempts")
                    return False
        
        return False

    def run_benchmark(self):
        """Run the complete benchmark suite"""
        
        # Ensure log directory exists
        os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)
        
        # Log benchmark start
        self.get_logger().info("=== BENCHMARK EXECUTION STARTED ===")
        self.get_logger().info(f"Total trials: {self.total_trials}")
        self.get_logger().info(f"Timeout per trial: {self.max_trial_time} seconds")
        
        # Initialize CSV file with header
        self.initialize_csv_file()
        
        print("\n" + "="*50)
        print("AUTOWARE DRIVING BENCHMARK START")
        print("="*50)
        print(f"Total trials: {self.total_trials}")
        print(f"Timeout per trial: {self.max_trial_time} seconds")
        print(f"CSV file: {self.csv_file_path}")
        print("="*50)
        
        all_test_start_time = time.time()
        
        for trial_num in range(1, self.total_trials + 1):
            print(f"\n----------")
            print(f"test {trial_num} start")
            
            start_time = time.time()
            start_timestamp = datetime.now().strftime("%H%M%S")
            print(f"start time : {start_timestamp}")
            
            # 현재 테스트 정보를 state_callback에서 사용할 수 있도록 저장
            self.current_trial_num = trial_num
            self.trial_start_time = start_time
            self.trial_start_timestamp = start_timestamp
            
            success = self.run_single_trial(trial_num)
            
            end_time = time.time()
            end_timestamp = datetime.now().strftime("%H%M%S")
            trial_duration = end_time - start_time
            
            print(f"finish time : {end_timestamp}")
            print(f"total time : {int(trial_duration//3600):02d}{int((trial_duration%3600)//60):02d}{int(trial_duration%60):02d}")
            
            # 성공한 경우만 결과 보관 (CSV는 이미 상태변화/타임아웃 시점에 저장됨)
            if success:
                result = {
                    'trial': trial_num,
                    'duration': trial_duration,
                    'actual_driving_time': self.actual_driving_time,
                    'success': success,
                    'start_time': start_timestamp,
                    'end_time': end_timestamp
                }
                self.results.append(result)
            
            if success:
                print(f"Status: SUCCESS")
            else:
                print(f"Status: FAILED")
            
            print("----------")
            
            # Wait between trials with adaptive recovery time
            if trial_num < self.total_trials:
                base_wait_time = 3.0  # Increased base wait time
                
                # If this trial failed, add extra recovery time
                if not success:
                    recovery_time = 10.0
                    self.get_logger().info(f"Trial failed - adding {recovery_time}s recovery time")
                    time.sleep(recovery_time)
                
                # Check for consecutive failures and add more recovery time
                recent_failures = 0
                for i in range(max(0, len(self.results) - 5), len(self.results)):
                    if not self.results[i]['success']:
                        recent_failures += 1
                
                if recent_failures >= 3:
                    extra_recovery = 15.0
                    self.get_logger().warn(f"Multiple consecutive failures detected - adding {extra_recovery}s extra recovery time")
                    time.sleep(extra_recovery)
                
                # Critical failure check - if too many consecutive failures, pause for manual intervention
                if recent_failures >= 5:
                    self.get_logger().error("CRITICAL: 5+ consecutive failures detected!")
                    self.get_logger().error("System may need manual intervention. Pausing for 60 seconds...")
                    self.get_logger().error("Press Ctrl+C to stop the benchmark if needed.")
                    time.sleep(60.0)
                
                time.sleep(base_wait_time)
        
        # Calculate final statistics - 전체 테스트 카운트 업데이트
        self.total_trial_count = self.total_trials
        
        # 콘솔과 로그 파일에 간단한 완료 메시지
        print("\n" + "="*50)
        print("ALL TESTS COMPLETED")
        print("="*50)
        
        # 터미널과 로그에 요약 출력
        self.print_summary()
        
        # CSV에 최종 요약 저장
        self.save_results_csv()
        
        return self.results

    def save_results_csv(self):
        """Save final summary to CSV file"""
        try:
            with open(self.csv_file_path, 'a', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                
                # Write final summary only
                total_trials = self.total_trial_count
                successful_trials = len(self.results)
                failed_trials = total_trials - successful_trials
                success_rate = (successful_trials / total_trials) * 100 if total_trials > 0 else 0
                
                if self.results:
                    avg_total_duration = sum(r['duration'] for r in self.results) / len(self.results)
                    avg_driving_duration = sum(r['actual_driving_time'] for r in self.results) / len(self.results)
                    
                    avg_test_time_formatted = f'{int(avg_total_duration//3600):02d}{int((avg_total_duration%3600)//60):02d}{int(avg_total_duration%60):02d}'
                    avg_driving_time_formatted = f'{int(avg_driving_duration//3600):02d}{int((avg_driving_duration%3600)//60):02d}{int(avg_driving_duration%60):02d}'
                else:
                    avg_test_time_formatted = '000000'
                    avg_driving_time_formatted = '000000'
                
                # Write final summary
                writer.writerow(['=== FINAL RESULTS ==='])
                writer.writerow([f'all test finished'])
                writer.writerow([f'success : {successful_trials}/{total_trials}'])
                writer.writerow([f'fail : {failed_trials}/{total_trials}'])
                writer.writerow([f'success rate : {success_rate:.1f}%'])
                writer.writerow([f'average test time (successful) : {avg_test_time_formatted}'])
                writer.writerow([f'average driving time (successful) : {avg_driving_time_formatted}'])
                writer.writerow([''])
            
            self.get_logger().info(f"Final summary saved to: {self.csv_file_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save final summary: {e}")

    def save_single_trial_to_csv(self, trial_num, result):
        """Save single trial result to CSV file immediately"""
        print(f"*** SAVING TRIAL {trial_num} TO CSV ***")
        self.get_logger().info(f"Attempting to save trial {trial_num} to CSV")
        
        try:
            with open(self.csv_file_path, 'a', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                
                # Write individual test result
                writer.writerow([f'----------'])
                writer.writerow([f'test {trial_num} start'])
                writer.writerow([f'start time : {result["start_time"]}'])
                writer.writerow([f'finish time : {result["end_time"]}'])
                
                # Format duration as HHMMSS
                duration = result['duration']
                duration_formatted = f'{int(duration//3600):02d}{int((duration%3600)//60):02d}{int(duration%60):02d}'
                writer.writerow([f'total time : {duration_formatted}'])
                
                # Format actual driving time as HHMMSS
                driving_time = result['actual_driving_time']
                driving_time_formatted = f'{int(driving_time//3600):02d}{int((driving_time%3600)//60):02d}{int(driving_time%60):02d}'
                writer.writerow([f'actual driving time : {driving_time_formatted}'])
                
                status = 'SUCCESS' if result['success'] else 'FAILED'
                writer.writerow([f'Status: {status}'])
                writer.writerow([f'----------'])
                writer.writerow([''])
            
            print(f"*** TRIAL {trial_num} SAVED TO CSV SUCCESSFULLY ***")
            self.get_logger().info(f"Single trial result saved to: {self.csv_file_path}")
            
        except Exception as e:
            print(f"*** ERROR SAVING TRIAL {trial_num} TO CSV: {e} ***")
            self.get_logger().error(f"Failed to save single trial result to CSV: {e}")

    def initialize_csv_file(self):
        """Initialize CSV file with header"""
        try:
            with open(self.csv_file_path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                
                # Write header only
                writer.writerow(['=== AUTOWARE DRIVING BENCHMARK RESULTS ==='])
                writer.writerow([f'Total trials: {self.total_trials}'])
                writer.writerow([f'Timeout per trial: {self.max_trial_time} seconds'])
                writer.writerow([''])
            
            self.get_logger().info(f"CSV file initialized: {self.csv_file_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CSV: {e}")

    def print_summary(self):
        """최종 요약 출력"""
        if not self.results:
            print("No successful trials to summarize")
            return
            
        total_trials = self.total_trial_count
        successful_trials = len(self.results)
        failed_trials = total_trials - successful_trials
        success_rate = (successful_trials / total_trials) * 100 if total_trials > 0 else 0
        
        # 성공한 테스트들의 평균 계산
        total_durations = [r['duration'] for r in self.results]
        driving_times = [r['actual_driving_time'] for r in self.results]
        
        avg_total_duration = sum(total_durations) / len(total_durations)
        avg_driving_time = sum(driving_times) / len(driving_times)
        
        print("\n" + "="*50)
        print("BENCHMARK SUMMARY")
        print("="*50)
        print(f"Total Trials: {total_trials}")
        print(f"Successful: {successful_trials}")
        print(f"Failed: {failed_trials}")
        print(f"Success Rate: {success_rate:.1f}%")
        print(f"Average Test Duration (successful): {avg_total_duration:.2f} seconds")
        print(f"Average Driving Time (successful): {avg_driving_time:.2f} seconds")
        print("="*50)

def main(args=None):
    rclpy.init(args=args)
    
    controller = AutowareBenchmarkController()
    
    # Run ROS2 spinning in separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()
    
    try:
        # Execute benchmark
        controller.run_benchmark()
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        controller.get_logger().error(f"Unexpected error: {e}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        print("Benchmark finished")

if __name__ == '__main__':
    main()
