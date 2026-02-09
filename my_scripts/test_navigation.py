import time                                                                                                                                                            
import threading                                                                                                                                                       
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber                                                                                    
from unitree_sdk2py.go2.sport.sport_client import SportClient                                                                                                          
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient                                                                             
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_                                                                                                     
import math                                                                                                                                                            
                                                                                                                                                                        
class Go2Navigator:                                                                                                                                                    
    def __init__(self, network_interface="enp0s2"):                                                                                                                    
        ChannelFactoryInitialize(0, network_interface)                                                                                                                 
                                                                                                                                                                        
        # 位置追蹤                                                                                                                                                     
        self.current_x = 0.0                                                                                                                                           
        self.current_y = 0.0                                                                                                                                           
        self.current_yaw = 0.0                                                                                                                                         
        self.position_lock = threading.Lock()                                                                                                                          
                                                                                                                                                                        
        # 訂閱狀態                                                                                                                                                     
        self.sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)                                                                                             
        self.sub.Init(self._on_state, 10)                                                                                                                              
                                                                                                                                                                        
        # Sport Client                                                                                                                                                 
        self.sport = SportClient()                                                                                                                                     
        self.sport.SetTimeout(10.0)                                                                                                                                    
        self.sport.Init()                                                                                                                                              
                                                                                                                                                                        
        # Obstacle Avoidance Client                                                                                                                                    
        self.obstacle = ObstaclesAvoidClient()                                                                                                                         
        self.obstacle.SetTimeout(3.0)                                                                                                                                  
        self.obstacle.Init()                                                                                                                                           
                                                                                                                                                                        
        # 控制 flag                                                                                                                                                    
        self.running = True                                                                                                                                            
        self.print_position = True                                                                                                                                     
                                                                                                                                                                        
    def _on_state(self, msg):                                                                                                                                          
        """Callback: 更新位置"""                                                                                                                                       
        with self.position_lock:                                                                                                                                       
            self.current_x = msg.position[0]                                                                                                                           
            self.current_y = msg.position[1]                                                                                                                           
            self.current_yaw = msg.imu_state.rpy[2]                                                                                                                    
                                                                                                                                                                        
    def get_position(self):                                                                                                                                            
        """取得目前位置"""                                                                                                                                             
        with self.position_lock:                                                                                                                                       
            return self.current_x, self.current_y, self.current_yaw                                                                                                    
                                                                                                                                                                        
    def _position_printer(self):                                                                                                                                       
        """背景執行緒：每 2 秒印出位置"""                                                                                                                              
        while self.running and self.print_position:                                                                                                                    
            x, y, yaw = self.get_position()                                                                                                                            
            print(f"[Position] x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad ({math.degrees(yaw):.1f}°)")                                                                    
            time.sleep(2)                                                                                                                                              
                                                                                                                                                                        
    def distance_to(self, target_x, target_y):                                                                                                                         
        """計算到目標的距離"""                                                                                                                                         
        x, y, _ = self.get_position()                                                                                                                                  
        return math.sqrt((target_x - x)**2 + (target_y - y)**2)                                                                                                        
                                                                                                                                                                        
    def enable_obstacle_avoidance(self):                                                                                                                               
        """開啟避障功能"""                                                                                                                                             
        print("Enabling obstacle avoidance...")                                                                                                                        
        while not self.obstacle.SwitchGet()[1]:                                                                                                                        
            self.obstacle.SwitchSet(True)                                                                                                                              
            time.sleep(0.1)                                                                                                                                            
        print("Obstacle avoidance enabled!")                                                                                                                           
                                                                                                                                                                        
    def disable_obstacle_avoidance(self):                                                                                                                              
        """關閉避障功能"""                                                                                                                                             
        print("Disabling obstacle avoidance...")                                                                                                                       
        self.obstacle.SwitchSet(False)                                                                                                                                 
        time.sleep(0.5)                                                                                                                                                
                                                                                                                                                                        
    def move_to_point(self, target_x, target_y, tolerance=0.15, speed=0.3):                                                                                            
        """移動到指定座標"""                                                                                                                                           
        print(f"Moving to ({target_x}, {target_y})...")                                                                                                                
                                                                                                                                                                        
        while self.distance_to(target_x, target_y) > tolerance:                                                                                                        
            x, y, yaw = self.get_position()                                                                                                                            
                                                                                                                                                                        
            # 計算目標方向                                                                                                                                             
            dx = target_x - x                                                                                                                                          
            dy = target_y - y                                                                                                                                          
            target_yaw = math.atan2(dy, dx)                                                                                                                            
                                                                                                                                                                        
            # 計算 yaw 誤差                                                                                                                                            
            yaw_error = target_yaw - yaw                                                                                                                               
            # 正規化到 -pi ~ pi                                                                                                                                        
            while yaw_error > math.pi:                                                                                                                                 
                yaw_error -= 2 * math.pi                                                                                                                               
            while yaw_error < -math.pi:                                                                                                                                
                yaw_error += 2 * math.pi                                                                                                                               
                                                                                                                                                                        
            # 如果角度誤差大，先轉向                                                                                                                                   
            if abs(yaw_error) > 0.3:  # ~17 度                                                                                                                         
                vx = 0.1                                                                                                                                               
                vyaw = 0.5 if yaw_error > 0 else -0.5                                                                                                                  
            else:                                                                                                                                                      
                vx = speed                                                                                                                                             
                vyaw = yaw_error * 0.5  # P 控制                                                                                                                       
                                                                                                                                                                        
            self.sport.Move(vx, 0, vyaw)                                                                                                                               
            time.sleep(0.02)                                                                                                                                           
                                                                                                                                                                        
        self.sport.StopMove()                                                                                                                                          
        print(f"Arrived at ({target_x}, {target_y})!")                                                                                                                 
                                                                                                                                                                        
    def navigate_waypoints(self, waypoints, pause_seconds=2.0):                                                                                                        
        """                                                                                                                                                            
        依序導航多個路徑點                                                                                                                                             
        waypoints: [(x1, y1), (x2, y2), ...]                                                                                                                           
        """                                                                                                                                                            
        # 開始印出位置的執行緒                                                                                                                                         
        position_thread = threading.Thread(target=self._position_printer)                                                                                              
        position_thread.start()                                                                                                                                        
                                                                                                                                                                        
        try:                                                                                                                                                           
            # 站起來並啟用移動模式                                                                                                                                     
            print("Standing up...")                                                                                                                                    
            self.sport.StandUp()                                                                                                                                       
            time.sleep(3)                                                                                                                                              
                                                                                                                                                                        
            print("Activating FreeWalk mode...")                                                                                                                       
            self.sport.FreeWalk()                                                                                                                                      
            time.sleep(1)                                                                                                                                              
                                                                                                                                                                        
            # 開啟避障                                                                                                                                                 
            self.enable_obstacle_avoidance()                                                                                                                           
                                                                                                                                                                        
            # 記錄起始位置                                                                                                                                             
            start_x, start_y, _ = self.get_position()                                                                                                                  
            print(f"Starting position: ({start_x:.3f}, {start_y:.3f})")                                                                                                
                                                                                                                                                                        
            # 依序走到每個點                                                                                                                                           
            for i, (x, y) in enumerate(waypoints):                                                                                                                     
                print(f"\n=== Waypoint {i+1}/{len(waypoints)}: ({x}, {y}) ===")                                                                                        
                self.move_to_point(x, y)                                                                                                                               
                                                                                                                                                                        
                if pause_seconds > 0:                                                                                                                                  
                    print(f"Pausing for {pause_seconds} seconds...")                                                                                                   
                    time.sleep(pause_seconds)                                                                                                                          
                                                                                                                                                                        
            print("\n=== Navigation complete! ===")                                                                                                                    
                                                                                                                                                                        
        finally:                                                                                                                                                       
            # 停止                                                                                                                                                     
            self.sport.StopMove()                                                                                                                                      
            self.running = False                                                                                                                                       
            self.print_position = False                                                                                                                                
            position_thread.join()                                                                                                                                     
                                                                                                                                                                        
            # 趴下                                                                                                                                                     
            print("Standing down...")                                                                                                                                  
            self.sport.StandDown()                                                                                                                                     
                                                                                                                                                                        
                                                                                                                                                                        
if __name__ == "__main__":                                                                                                                                             
    nav = Go2Navigator("enp0s2")                                                                                                                                       
                                                                                                                                                                        
    # 定義路徑點 (相對於起始位置的座標)                                                                                                                                
    # 注意：這些是絕對座標，不是相對位移                                                                                                                               
    waypoints = [                                                                                                                                                      
        (0.5, 0.0),    # 前進 0.5m                                                                                                                                     
        (0.5, 0.5),    # 右移到 (0.5, 0.5)                                                                                                                             
        (0.0, 0.5),    # 回到 (0, 0.5)                                                                                                                                 
        (0.0, 0.0),    # 回到原點                                                                                                                                      
    ]                                                                                                                                                                  
                                                                                                                                                                        
    # 每個點停留 2 秒                                                                                                                                                  
    nav.navigate_waypoints(waypoints, pause_seconds=2.0)  