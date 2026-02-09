import time                                                                                                                                                                  
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize                                                                                          
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_                                                                                                           
                                                                                                                                                                            
# 全域變數儲存最新狀態                                                                                                                                                       
latest_state = None                                                                                                                                                          
                                                                                                                                                                            
def on_state(msg):                                                                                                                                                           
    global latest_state                                                                                                                                                      
    latest_state = msg  # 只儲存，不印出                                                                                                                                     
                                                                                                                                                                            
if __name__ == "__main__":                                                                                                                                                   
    ChannelFactoryInitialize(0, "enp0s2")                                                                                                                                    
                                                                                                                                                                            
    sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)                                                                                                            
    sub.Init(on_state, 10)                                                                                                                                                   
                                                                                                                                                                            
    print("Listening for robot state... (Ctrl+C to stop)")                                                                                                                   
                                                                                                                                                                            
    while True:                                                                                                                                                              
        time.sleep(2)  # 每 2 秒                                                                                                                                             
                                                                                                                                                                            
        if latest_state is not None:                                                                                                                                         
            msg = latest_state                                                                                                                                               
            print(f"Position: x={msg.position[0]:.3f}, y={msg.position[1]:.3f}, z={msg.position[2]:.3f}")                                                                    
            print(f"Velocity: vx={msg.velocity[0]:.3f}, vy={msg.velocity[1]:.3f}")                                                                                           
            print(f"Yaw: {msg.imu_state.rpy[2]:.3f} rad")                                                                                                                    
            print("-" * 40)                                                                                                                                                  
        else:                                                                                                                                                                
            print("Waiting for data...") 