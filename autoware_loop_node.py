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
        self.declare_parameter('trial_count', 100)
        self.declare_parameter('timeout_seconds', 300.0)
        
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
        
        # Time tracking
        self.autonomous_start_time = None
        self.autonomous_end_time = None
        self.actual_driving_time = 0.0
        
        # Configuration
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
        self.csv_file_path = '/home/hycon/autoware_logs/driving_benchmark_results.csv'
        
        # Create log directory
        os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)
        
        self.get_logger().info("Autoware Benchmark Controller started")
        self.get_logger().info(f"Trial count: {self.total_trials}")
        self.get_logger().info(f"Timeout: {self.max_trial_time} seconds")

    def state_callback(self, msg):
        """Autoware state callback for goal detection only"""
        prev_state = self.current_state
        self.current_state = msg.state
        
        is_driving_now = self._is_driving_state(self.current_state)
        was_driving_before = self._is_driving_state(prev_state) if prev_state is not None else False
        
        # Log state changes
        if prev_state != self.current_state:
            self.get_logger().info(f"State change: {prev_state} -> {self.current_state}")
        
        # Goal reached detection - only signal, no CSV saving
        if was_driving_before and not is_driving_now:
            self.goal_reached.set()
            self.get_logger().info("Goal reached!")

    def _is_driving_state(self, state):
        """Check if state is DRIVING"""
        if state is None:
            return False
        
        if isinstance(state, int):
            return state in [3, 4, 5]
        
        return False

    def wait_for_service(self, client, service_name, timeout=10.0):
        """Wait for service availability"""
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"{service_name} service not found")
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
                    self.get_logger().info("Route cleared successfully")
                else:
                    self.get_logger().warn("Failed to clear route")
            except Exception as e:
                self.get_logger().warn(f"Failed to clear route: {e}")

    def set_initial_pose(self):
        """Set initial pose"""
        try:
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
            
            # Set covariance matrix
            msg.pose.covariance = [0.0] * 36
            msg.pose.covariance[0] = 0.1
            msg.pose.covariance[7] = 0.1
            msg.pose.covariance[35] = 0.1
            
            self.initial_pose_pub.publish(msg)
            self.get_logger().info(f"Initial pose set successfully - Position: ({self.initial_pose['x']:.2f}, {self.initial_pose['y']:.2f})")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to set initial pose: {e}")
            return False

    def set_goal_pose(self):
        """Set goal pose"""
        try:
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
            self.get_logger().info(f"Goal pose set successfully - Position: ({self.goal_pose['x']:.2f}, {self.goal_pose['y']:.2f})")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to set goal pose: {e}")
            return False

    def engage_autonomous(self):
        """Start autonomous driving mode"""
        if not self.wait_for_service(self.engage_client, "Engage", 10.0):
            self.get_logger().error("Engage service not found!")
            return False
        
        try:
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
                    self.get_logger().error(f"Engage failed: {response.status.message}")
                    return False
            else:
                self.get_logger().error("Engage service no response")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Engage service error: {e}")
            return False

    def disengage_autonomous(self):
        """Disengage autonomous driving mode"""
        if not self.wait_for_service(self.engage_client, "Disengage", 10.0):
            self.get_logger().error("Disengage service not found!")
            return False
        
        try:
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
                    self.get_logger().error(f"Disengage failed: {response.status.message}")
                    return False
            else:
                self.get_logger().error("Disengage service no response")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Disengage service error: {e}")
            return False

    def emergency_stop(self):
        """Emergency stop and system reset"""
        self.get_logger().info("Emergency stop initiated")
        
        # Try multiple times to ensure disengage
        for attempt in range(5):
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
        
        time.sleep(10.0)
        self.get_logger().info("Emergency stop completed")

    def run_single_trial(self, trial_num):
        """Run a single autonomous driving trial"""
        self.get_logger().info(f"Starting trial #{trial_num}")
        
        max_recovery_attempts = 3
        
        for attempt in range(max_recovery_attempts):
            try:
                # Reset state
                self.is_driving = False
                self.driving_started = False
                self.goal_reached.clear()
                self.driving_start_time = 0.0
                self.actual_driving_time = 0.0
                
                # Force disengage
                for disengage_attempt in range(3):
                    if self.disengage_autonomous():
                        break
                    time.sleep(2.0)
                else:
                    self.get_logger().error("Failed to disengage after 3 attempts")
                
                time.sleep(5.0)
                
                # Emergency stop if still in driving state
                if self._is_driving_state(self.current_state):
                    self.emergency_stop()
                
                # Clear routes
                for i in range(3):
                    self.clear_route()
                    time.sleep(1.5)
                
                # Set poses
                if not self.set_initial_pose():
                    self.get_logger().error(f"Trial #{trial_num}: Failed to set initial pose")
                    if attempt < max_recovery_attempts - 1:
                        continue
                    else:
                        return False
                time.sleep(4.0)
                
                if not self.set_goal_pose():
                    self.get_logger().error(f"Trial #{trial_num}: Failed to set goal pose")
                    if attempt < max_recovery_attempts - 1:
                        continue
                    else:
                        return False
                time.sleep(3.0)
                
                # Start autonomous driving
                if not self.engage_autonomous():
                    self.get_logger().error(f"Trial #{trial_num}: Failed to engage autonomous mode")
                    if attempt < max_recovery_attempts - 1:
                        continue
                    else:
                        return False
                
                # Record driving start time (engage success point)
                self.test_in_progress = True
                self.driving_start_time = time.time()
                driving_start_timestamp = datetime.now().strftime("%H:%M:%S")
                self.get_logger().info(f"Trial #{trial_num}: Driving started at {driving_start_timestamp}")
                
                # Wait for goal arrival
                check_interval = 30.0
                remaining_time = self.max_trial_time
                
                while remaining_time > 0:
                    wait_time = min(check_interval, remaining_time)
                    
                    if self.goal_reached.wait(timeout=wait_time):
                        # Success
                        test_end_time = time.time()
                        driving_end_timestamp = datetime.now().strftime("%H:%M:%S")
                        self.get_logger().info(f"Trial #{trial_num}: Goal reached! Driving ended at {driving_end_timestamp}")
                        
                        self.test_in_progress = False
                        
                        # Cleanup
                        for cleanup_attempt in range(3):
                            if self.disengage_autonomous():
                                break
                            time.sleep(1.0)
                        
                        # Calculate and save result
                        self.calculate_and_save_trial_result(trial_num, True, test_end_time)
                        
                        time.sleep(3.0)
                        return True
                    
                    remaining_time -= wait_time
                
                # Timeout
                test_end_time = time.time()
                timeout_timestamp = datetime.now().strftime("%H:%M:%S")
                self.get_logger().error(f"Trial #{trial_num}: TIMEOUT ({self.max_trial_time} seconds) at {timeout_timestamp}")
                
                # Calculate and save timeout result
                self.calculate_and_save_trial_result(trial_num, False, test_end_time)
                
                self.test_in_progress = False
                self.emergency_stop()
                
                # Reset state
                self.is_driving = False
                self.driving_started = False
                self.goal_reached.clear()
                
                return False
                
            except Exception as e:
                self.get_logger().error(f"Trial #{trial_num}: ERROR on attempt {attempt + 1}: {e}")
                
                self.test_in_progress = False
                
                try:
                    self.emergency_stop()
                except:
                    pass
                
                # Reset state
                self.is_driving = False
                self.driving_started = False
                self.goal_reached.clear()
                
                if attempt < max_recovery_attempts - 1:
                    time.sleep(5.0)
                    continue
                else:
                    return False
        
        return False

    def calculate_and_save_trial_result(self, trial_num, success, test_end_time):
        """Calculate driving time and save trial result"""
        try:
            # Simple time calculation
            test_duration = test_end_time - self.trial_start_time if hasattr(self, 'trial_start_time') else 0.0
            
            # Actual driving time = test end time - engage success time
            if self.driving_start_time > 0:
                self.actual_driving_time = test_end_time - self.driving_start_time
            else:
                self.actual_driving_time = 0.0
                self.get_logger().warn(f"Trial #{trial_num}: No driving start time recorded")
            
            # Create result for CSV
            end_timestamp = datetime.now().strftime("%H%M%S")
            test_end_readable = datetime.now().strftime("%H:%M:%S")
            
            result = {
                'trial': trial_num,
                'duration': test_duration,
                'actual_driving_time': self.actual_driving_time,
                'success': success,
                'start_time': getattr(self, 'trial_start_timestamp', end_timestamp),
                'end_time': end_timestamp
            }
            
            # Log detailed trial result
            status_text = "SUCCESS" if success else "FAILED"
            self.get_logger().info(f"Trial #{trial_num}: {status_text} - Test ended at {test_end_readable}")
            self.get_logger().info(f"Trial #{trial_num}: Test duration: {test_duration:.2f}s, Driving time: {self.actual_driving_time:.2f}s")
            
            self.save_single_trial_to_csv(trial_num, result)
            
        except Exception as e:
            self.get_logger().error(f"Error in calculate_and_save_trial_result: {e}")

    def run_benchmark(self):
        """Run the complete benchmark suite"""
        os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)
        
        self.get_logger().info("Benchmark execution started")
        self.initialize_csv_file()
        
        print("\n" + "="*50)
        print("AUTOWARE DRIVING BENCHMARK START")
        print("="*50)
        print(f"Total trials: {self.total_trials}")
        print(f"Timeout per trial: {self.max_trial_time} seconds")
        print("="*50)
        
        # Log benchmark start info
        self.get_logger().info(f"Starting benchmark - Total trials: {self.total_trials}, Timeout: {self.max_trial_time}s")
        
        for trial_num in range(1, self.total_trials + 1):
            print(f"\n----------")
            print(f"test {trial_num} start")
            
            start_time = time.time()
            start_timestamp = datetime.now().strftime("%H%M%S")
            print(f"start time : {start_timestamp}")
            
            self.current_trial_num = trial_num
            self.trial_start_time = start_time
            self.trial_start_timestamp = start_timestamp
            
            # Log trial start
            self.get_logger().info(f"Trial #{trial_num} started at {start_timestamp}")
            
            success = self.run_single_trial(trial_num)
            
            end_time = time.time()
            end_timestamp = datetime.now().strftime("%H%M%S")
            trial_duration = end_time - start_time
            
            print(f"finish time : {end_timestamp}")
            print(f"total time : {int(trial_duration//3600):02d}{int((trial_duration%3600)//60):02d}{int(trial_duration%60):02d}")
            
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
            
            print(f"Status: {'SUCCESS' if success else 'FAILED'}")
            print("----------")
            
            # Wait between trials
            if trial_num < self.total_trials:
                base_wait_time = 3.0
                
                if not success:
                    recovery_time = 10.0
                    self.get_logger().info(f"Trial failed - adding {recovery_time}s recovery time")
                    time.sleep(recovery_time)
                
                # Check for consecutive failures
                recent_failures = 0
                for i in range(max(0, len(self.results) - 5), len(self.results)):
                    if not self.results[i]['success']:
                        recent_failures += 1
                
                if recent_failures >= 3:
                    extra_recovery = 15.0
                    self.get_logger().warn(f"Multiple consecutive failures - adding {extra_recovery}s recovery time")
                    time.sleep(extra_recovery)
                
                if recent_failures >= 5:
                    self.get_logger().error("CRITICAL: 5+ consecutive failures detected!")
                    self.get_logger().error("System may need manual intervention. Pausing for 60 seconds...")
                    time.sleep(60.0)
                
                time.sleep(base_wait_time)
        
        self.total_trial_count = self.total_trials
        
        print("\n" + "="*50)
        print("ALL TESTS COMPLETED")
        print("="*50)
        
        self.print_summary()
        self.save_results_csv()
        
        return self.results

    def save_results_csv(self):
        """Save final summary to CSV file"""
        try:
            with open(self.csv_file_path, 'a', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                
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
                
                writer.writerow(['=== FINAL RESULTS ==='])
                writer.writerow([f'all test finished'])
                writer.writerow([f'success : {successful_trials}/{total_trials}'])
                writer.writerow([f'fail : {failed_trials}/{total_trials}'])
                writer.writerow([f'success rate : {success_rate:.1f}%'])
                writer.writerow([f'average test time (successful) : {avg_test_time_formatted}'])
                writer.writerow([f'average driving time (successful) : {avg_driving_time_formatted}'])
                writer.writerow([''])
            
            self.get_logger().info("Final summary saved to CSV")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save final summary: {e}")

    def save_single_trial_to_csv(self, trial_num, result):
        """Save single trial result to CSV file"""
        try:
            with open(self.csv_file_path, 'a', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                
                writer.writerow([f'----------'])
                writer.writerow([f'test {trial_num} start'])
                writer.writerow([f'start time : {result["start_time"]}'])
                writer.writerow([f'finish time : {result["end_time"]}'])
                
                duration = result['duration']
                duration_formatted = f'{int(duration//3600):02d}{int((duration%3600)//60):02d}{int(duration%60):02d}'
                writer.writerow([f'total time : {duration_formatted}'])
                
                driving_time = result['actual_driving_time']
                driving_time_formatted = f'{int(driving_time//3600):02d}{int((driving_time%3600)//60):02d}{int(driving_time%60):02d}'
                writer.writerow([f'actual driving time : {driving_time_formatted}'])
                
                status = 'SUCCESS' if result['success'] else 'FAILED'
                writer.writerow([f'Status: {status}'])
                writer.writerow([f'----------'])
                writer.writerow([''])
            
            self.get_logger().info(f"Trial {trial_num} result saved to CSV")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save trial {trial_num} to CSV: {e}")

    def initialize_csv_file(self):
        """Initialize CSV file with header"""
        try:
            with open(self.csv_file_path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                
                writer.writerow(['=== AUTOWARE DRIVING BENCHMARK RESULTS ==='])
                writer.writerow([f'Total trials: {self.total_trials}'])
                writer.writerow([f'Timeout per trial: {self.max_trial_time} seconds'])
                writer.writerow([''])
            
            self.get_logger().info("CSV file initialized")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CSV: {e}")

    def print_summary(self):
        """Print final benchmark summary"""
        if not self.results:
            print("No successful trials to summarize")
            return
            
        total_trials = self.total_trial_count
        successful_trials = len(self.results)
        failed_trials = total_trials - successful_trials
        success_rate = (successful_trials / total_trials) * 100 if total_trials > 0 else 0
        
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
