import time                                                                                                                                                            
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber                                                                                    
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_                                                                                                     
import math                                                                                                                                                            
                                                                                                                                                                        
class PositionRecorder:                                                                                                                                                
    def __init__(self, network_interface="enp0s2"):                                                                                                                    
        ChannelFactoryInitialize(0, network_interface)                                                                                                                 
                                                                                                                                                                        
        self.current_x = 0.0                                                                                                                                           
        self.current_y = 0.0                                                                                                                                           
        self.current_yaw = 0.0                                                                                                                                         
                                                                                                                                                                        
        self.sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)                                                                                             
        self.sub.Init(self._on_state, 10)                                                                                                                              
                                                                                                                                                                        
        self.waypoints = []  # 儲存記錄的點                                                                                                                            
                                                                                                                                                                        
    def _on_state(self, msg):                                                                                                                                          
        self.current_x = msg.position[0]                                                                                                                               
        self.current_y = msg.position[1]                                                                                                                               
        self.current_yaw = msg.imu_state.rpy[2]                                                                                                                        
                                                                                                                                                                        
    def run(self):                                                                                                                                                     
        print("=" * 50)                                                                                                                                                
        print("Position Recorder")                                                                                                                                     
        print("=" * 50)                                                                                                                                                
        print("Commands:")                                                                                                                                             
        print("  [Enter] - Record current position as waypoint")                                                                                                       
        print("  [p]     - Print all recorded waypoints")                                                                                                              
        print("  [c]     - Clear all waypoints")                                                                                                                       
        print("  [q]     - Quit and print final waypoints")                                                                                                            
        print("=" * 50)                                                                                                                                                
                                                                                                                                                                        
        while True:                                                                                                                                                    
            # 顯示目前位置                                                                                                                                             
            yaw_deg = math.degrees(self.current_yaw)                                                                                                                   
            print(f"\rPosition: x={self.current_x:7.3f}, y={self.current_y:7.3f}, yaw={yaw_deg:6.1f}°  ", end="", flush=True)                                          
                                                                                                                                                                        
            # 非阻塞式輸入檢查                                                                                                                                         
            import sys                                                                                                                                                 
            import select                                                                                                                                              
                                                                                                                                                                        
            if select.select([sys.stdin], [], [], 0.5)[0]:                                                                                                             
                cmd = input().strip().lower()                                                                                                                          
                                                                                                                                                                        
                if cmd == "":                                                                                                                                          
                    # 記錄當前點                                                                                                                                       
                    point = (round(self.current_x, 3), round(self.current_y, 3), round(self.current_yaw, 3))                                                                                       
                    self.waypoints.append(point)                                                                                                                       
                    yaw_deg = math.degrees(self.current_yaw)
                    print(f"\n>>> Recorded waypoint #{len(self.waypoints)}: x={point[0]}, y={point[1]}, yaw={yaw_deg:.1f}°")                                                                                  
                                                                                                                                                                        
                elif cmd == "p":                                                                                                                                       
                    print(f"\n>>> Recorded waypoints: {self.waypoints}")                                                                                               
                                                                                                                                                                        
                elif cmd == "c":                                                                                                                                       
                    self.waypoints = []                                                                                                                                
                    print("\n>>> Cleared all waypoints")                                                                                                               
                                                                                                                                                                        
                elif cmd == "q":                                                                                                                                       
                    break                                                                                                                                              
                                                                                                                                                                        
        # 輸出最終結果                                                                                                                                                 
        print("\n" + "=" * 50)                                                                                                                                         
        print("Final waypoints (copy this to your navigation script):")                                                                                                
        print("=" * 50)                                                                                                                                                
        print(f"waypoints = {self.waypoints}")                                                                                                                         
        print("=" * 50)                                                                                                                                                
                                                                                                                                                                        
                                                                                                                                                                        
if __name__ == "__main__":                                                                                                                                             
    recorder = PositionRecorder("enp0s2")                                                                                                                              
    recorder.run()   