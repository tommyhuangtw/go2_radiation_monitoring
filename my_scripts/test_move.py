import time                                                                                                                                                                  
from unitree_sdk2py.core.channel import ChannelFactoryInitialize                                                                                                             
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient                                                                                  
from unitree_sdk2py.go2.sport.sport_client import SportClient                                                                                                                
                                                                                                                                                                            
if __name__ == "__main__":                                                                                                                                                   
    ChannelFactoryInitialize(0, "enp0s2")                                                                                                                                    
                                                                                                                                                                            
    # 先切換到 normal 模式                                                                                                                                                   
    print("Switching to normal mode...")                                                                                                                                     
    msc = MotionSwitcherClient()                                                                                                                                             
    msc.SetTimeout(5.0)                                                                                                                                                      
    msc.Init()                                                                                                                                                               
    ret = msc.SelectMode("advanced")  # 或試試 "advanced"                                                                                                                      
    print(f"Mode switch ret: {ret}")                                                                                                                                         
    time.sleep(2)                                                                                                                                                            
                                                                                                                                                                            
    # 然後控制                                                                                                                                                               
    client = SportClient()                                                                                                                                                   
    client.SetTimeout(10.0)                                                                                                                                                  
    client.Init()                                                                                                                                                            
                                                                                                                                                                            
    print("Standing up...")                                                                                                                                                  
    client.StandUp()                                                                                                                                                         
    time.sleep(3)                                                                                                                                                            
                                                                                                                                                                            
    print("Moving forward...")                                                                                                                                               
    duration = 4                                                                                                                                                             
    start_time = time.time()                                                                                                                                                 
    while time.time() - start_time < duration:                                                                                                                               
        client.Move(0.3, 0, 0)                                                                                                                                               
        time.sleep(0.1)                                                                                                                                                      
                                                                                                                                                                            
    print("Stopping...")                                                                                                                                                     
    client.StopMove()                                                                                                                                                        
                                                                                                                                                                            
    print("Standing down...")                                                                                                                                                
    client.StandDown()                                                                                                                                                       
    print("Done!") 