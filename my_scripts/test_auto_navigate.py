import time
import math
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

# Distance calibration factor - adjust if robot moves too short/far
DISTANCE_SCALE = 4


class Go2WaypointNavigator:
    def __init__(self, network_interface="enp0s2"):
        ChannelFactoryInitialize(0, network_interface)

        # 位置追蹤
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.state_ready = False  # Flag to track if state callback has fired

        # 訂閱狀態
        self.sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.sub.Init(self._on_state, 10)

        # Sport Client
        self.sport = SportClient()
        self.sport.SetTimeout(10.0)
        self.sport.Init()

        # Wait for first state update
        print("Waiting for state updates...")
        timeout = 5.0  # 5 seconds timeout
        start_time = time.time()
        while not self.state_ready:
            time.sleep(0.1)
            if time.time() - start_time > timeout:
                print("WARNING: State update timeout! Robot may not be connected.")
                break
        if self.state_ready:
            print(f"State ready! Initial position: ({self.current_x:.2f}, {self.current_y:.2f}), yaw={math.degrees(self.current_yaw):.1f}°")

    def _on_state(self, msg):
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_yaw = msg.imu_state.rpy[2]
        self.state_ready = True

    def get_position(self):
        return self.current_x, self.current_y, self.current_yaw

    def normalize_angle(self, angle):
        """將角度正規化到 -pi ~ pi"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def distance_to(self, target_x, target_y):
        x, y, _ = self.get_position()
        return math.sqrt((target_x - x)**2 + (target_y - y)**2)

    def angle_to(self, target_x, target_y):
        """計算到目標點的角度"""
        x, y, _ = self.get_position()
        return math.atan2(target_y - y, target_x - x)

    def rotate_to_target(self, target_x, target_y, tolerance=0.25, timeout=30.0):
        """先旋轉面向目標點"""
        print("  Rotating to face target...")

        # Calculate target yaw ONCE at the start (don't recalculate during rotation)
        target_yaw = self.angle_to(target_x, target_y)
        print(f"    [Rotate] Target heading: {math.degrees(target_yaw):.1f}°")

        start_time = time.time()
        iteration = 0
        last_progress_check_time = start_time
        last_yaw_error = None

        while True:
            _, _, yaw = self.get_position()
            yaw_error = self.normalize_angle(target_yaw - yaw)

            # Debug logging every 50 iterations (~1 second)
            if iteration % 50 == 0:
                print(f"    [Rotate] yaw={math.degrees(yaw):.1f}°, target={math.degrees(target_yaw):.1f}°, error={math.degrees(yaw_error):.1f}°")

            if abs(yaw_error) < tolerance:
                print(f"    [Rotate] Rotation complete! Final error: {math.degrees(yaw_error):.1f}°")
                break

            # Progress check: if no improvement in 5 seconds, skip rotation
            if time.time() - last_progress_check_time > 5.0:
                if last_yaw_error is not None and abs(abs(yaw_error) - abs(last_yaw_error)) < 0.05:
                    print(f"    [Rotate] No progress detected, skipping to forward movement (error: {math.degrees(yaw_error):.1f}°)")
                    break
                last_yaw_error = abs(yaw_error)
                last_progress_check_time = time.time()

            # Timeout check
            if time.time() - start_time > timeout:
                print(f"    [Rotate] WARNING: Rotation timeout after {timeout}s! Error: {math.degrees(yaw_error):.1f}°")
                break

            # 旋轉速度，根據誤差調整
            # Add dead zone to prevent micro-corrections
            if abs(yaw_error) < 0.05:  # ~2.8 degrees dead zone
                vyaw = 0
            else:
                # Use constant speed like working test (overcoming friction threshold)
                vyaw = 0.8 if yaw_error > 0 else -0.8
            self.sport.Move(0, 0, vyaw)
            time.sleep(0.02)
            iteration += 1

        self.sport.StopMove()
        time.sleep(0.3)

    def move_forward_to_target(self, target_x, target_y, tolerance=0.25, speed=0.3, timeout=60.0):
        """直線前進到目標點，途中微調方向"""
        print("  Moving forward...")

        start_time = time.time()
        iteration = 0
        last_x, last_y = None, None
        stuck_counter = 0

        while self.distance_to(target_x, target_y) > tolerance:
            x, y, yaw = self.get_position()
            distance = self.distance_to(target_x, target_y)

            # Debug logging every 50 iterations (~1 second)
            if iteration % 50 == 0:
                print(f"    [Move] pos=({x:.2f}, {y:.2f}), distance={distance:.2f}m, yaw={math.degrees(yaw):.1f}°")

            # Timeout check
            if time.time() - start_time > timeout:
                print(f"    [Move] WARNING: Movement timeout after {timeout}s! Distance remaining: {distance:.2f}m")
                break

            # Stuck detection - check if position hasn't changed
            if last_x is not None and last_y is not None:
                position_change = math.sqrt((x - last_x)**2 + (y - last_y)**2)
                if position_change < 0.01:  # Less than 1cm movement
                    stuck_counter += 1
                    if stuck_counter > 100:  # Stuck for 2 seconds (100 iterations * 0.02s)
                        print(f"    [Move] Robot appears stuck, accepting current position (distance: {distance:.2f}m)")
                        break
                else:
                    stuck_counter = 0  # Reset if moving
            last_x, last_y = x, y

            # 計算方向誤差
            target_yaw = self.angle_to(target_x, target_y)
            yaw_error = self.normalize_angle(target_yaw - yaw)

            # 前進速度，接近時減速
            vx = min(speed, distance * 0.8)
            vx = max(vx, 0.15)  # 最小速度

            # 微調方向 (P 控制)
            vyaw = yaw_error * 0.8
            vyaw = max(min(vyaw, 0.3), -0.3)  # 限制旋轉速度

            self.sport.Move(vx, 0, vyaw)
            time.sleep(0.02)
            iteration += 1

        self.sport.StopMove()
        print(f"    [Move] Movement complete! Final distance: {self.distance_to(target_x, target_y):.3f}m")

    def move_to_point(self, target_x, target_y, tolerance=0.25):
        """移動到指定座標：先轉向，再前進"""
        x, y, yaw = self.get_position()
        print(f"  Current: ({x:.2f}, {y:.2f}), Target: ({target_x:.2f}, {target_y:.2f})")

        # 如果距離太近，跳過
        if self.distance_to(target_x, target_y) < tolerance:
            print("  Already at target!")
            return

        # Step 1: 旋轉面向目標
        self.rotate_to_target(target_x, target_y)

        # Step 2: 直線前進
        self.move_forward_to_target(target_x, target_y, tolerance)

        print("  Arrived!")

    def navigate_waypoints(self, waypoints, pause_seconds=2.0, wait_for_enter=True):
        """導航多個路徑點"""

        print("=" * 50)
        print("Waypoint Navigation")
        print("=" * 50)

        # 站起來
        print("\nStanding up...")
        self.sport.StandUp()
        time.sleep(3)

        # 啟用移動模式
        print("Activating FreeWalk mode...")
        self.sport.FreeWalk()
        time.sleep(1)

        # 獲取起始位置，將相對路徑點轉換為絕對座標
        start_x, start_y, start_yaw = self.get_position()
        print(f"\nStarting position: ({start_x:.3f}, {start_y:.3f}), yaw={math.degrees(start_yaw):.1f}°")
        print("Converting relative waypoints to absolute coordinates...")

        # 將相對座標轉換為絕對座標 (應用距離校正係數)
        absolute_waypoints = []
        for i, waypoint in enumerate(waypoints):
            if len(waypoint) == 3:
                rel_x, rel_y, rel_yaw = waypoint
                abs_x = start_x + (rel_x * DISTANCE_SCALE)
                abs_y = start_y + (rel_y * DISTANCE_SCALE)
                abs_yaw = start_yaw + rel_yaw
                absolute_waypoints.append((abs_x, abs_y, abs_yaw))
                print(f"  Waypoint {i+1}: ({rel_x:+.3f}, {rel_y:+.3f}) × {DISTANCE_SCALE} → ({abs_x:.3f}, {abs_y:.3f})")
            else:
                rel_x, rel_y = waypoint
                abs_x = start_x + (rel_x * DISTANCE_SCALE)
                abs_y = start_y + (rel_y * DISTANCE_SCALE)
                absolute_waypoints.append((abs_x, abs_y))
                print(f"  Waypoint {i+1}: ({rel_x:+.3f}, {rel_y:+.3f}) × {DISTANCE_SCALE} → ({abs_x:.3f}, {abs_y:.3f})")

        # 使用轉換後的絕對座標
        waypoints = absolute_waypoints

        print(f"\nTotal waypoints: {len(waypoints)}")
        print("=" * 50)

        try:
            for i, waypoint in enumerate(waypoints):
                # Handle both (x, y) and (x, y, yaw) formats
                if len(waypoint) == 3:
                    target_x, target_y, target_yaw = waypoint
                    print(f"\n>>> Waypoint {i+1}/{len(waypoints)}: ({target_x}, {target_y}, yaw={math.degrees(target_yaw):.1f}°)")
                else:
                    target_x, target_y = waypoint
                    target_yaw = None
                    print(f"\n>>> Waypoint {i+1}/{len(waypoints)}: ({target_x}, {target_y})")

                # 移動到目標點
                self.move_to_point(target_x, target_y)

                # 顯示到達位置
                x, y, yaw = self.get_position()
                print(f"  Position after arrival: ({x:.3f}, {y:.3f}), yaw={math.degrees(yaw):.1f}°")

                # 停留
                if pause_seconds > 0:
                    print(f"  Pausing for {pause_seconds} seconds...")
                    time.sleep(pause_seconds)

                # 等待 Enter 繼續
                if wait_for_enter and i < len(waypoints) - 1:
                    input("  Press Enter to continue to next waypoint...")

            print("\n" + "=" * 50)
            print("Navigation complete!")
            print("=" * 50)

        except KeyboardInterrupt:
            print("\n\nNavigation interrupted!")

        finally:
            # 停止並趴下
            print("\nStopping...")
            self.sport.StopMove()
            time.sleep(1)

            print("Recovery stand...")
            self.sport.RecoveryStand()
            time.sleep(1)

            print("Standing down...")
            self.sport.StandDown()
            print("Done!")

if __name__ == "__main__":
    nav = Go2WaypointNavigator("enp0s2")

    # 相對路徑點 (相對於起點 #1，已移除重複點)
    # 這些座標是相對位移，會在導航開始時加上機器人當前位置
    waypoints = [
        (0.0, 0.0, 0.0),         # Start point (current position)
        (0.954, 0.25, 0.488),    # +0.954m x, +0.25m y, 28.0°
        (1.843, 0.852, -0.133),  # +1.843m x, +0.852m y, -7.6°
        (2.055, 0.445, -1.203),  # +2.055m x, +0.445m y, -68.9°
        (2.447, -0.21, -1.168),  # +2.447m x, -0.21m y, -66.9°
        (2.43, -0.524, -2.037),  # +2.43m x, -0.524m y, -116.7°
        (2.142, -0.745, -2.752), # +2.142m x, -0.745m y, -157.7°
        (1.034, -0.15, 2.764),   # +1.034m x, -0.15m y, 158.4°
        (0.419, -0.097, 2.803),  # +0.419m x, -0.097m y, 160.6°
    ]

    # 開始導航
    # wait_for_enter=True: 每個點到達後等你按 Enter
    # wait_for_enter=False: 自動連續走完
    nav.navigate_waypoints(waypoints, pause_seconds=2.0, wait_for_enter=True)
