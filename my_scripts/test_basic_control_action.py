import time                                                                                                                                                                  
from unitree_sdk2py.core.channel import ChannelFactoryInitialize                                                                                                             
from unitree_sdk2py.go2.sport.sport_client import SportClient                                                                                                                
                                                                                                                                                                            
if __name__ == "__main__":                                                                                                                                                   
    ChannelFactoryInitialize(0, "enp0s2")                                                                                                                                    
                                                                                                                                                                            
    client = SportClient()                                                                                                                                                   
    client.SetTimeout(10.0)                                                                                                                                                  
    client.Init()                                                                                                                                                            
                                                                                                                                                                            
    # 1. 趴下                                                                                                                                                                
    print("Standing down...")                                                                                                                                                
    client.StandDown()                                                                                                                                                       
    time.sleep(3)                                                                                                                                                            
                                                                                                                                                                            
    # 2. 用 BalanceStand 而不是 StandUp                                                                                                                                      
    print("Standing up...")                                                                                                                                                  
    client.StandUp()                                                                                                                                                         
    time.sleep(3) 

    print("Activating move mode...")      
    ret = client.FreeWalk()                                                                                                                                                                 

    print("Rotating Clockwise...")                                                                                                                                               
    duration = 2                                                                                                                                                         
    start_time = time.time()                                                                                                                                                 
    while time.time() - start_time < duration:                                                                                                                               
        client.Move(0, 0, -1.0)                                                                                                                                               
        time.sleep(0.02)  

    # print("Rotating Clockwise...")                                                                                                                                               
    # duration = 4                                                                                                                                                   
    # start_time = time.time()                                                                                                                                                 
    # while time.time() - start_time < duration:                                                                                                                               
    #     client.Move(0, 0, -0.5)                                                                                                                                               
    #     time.sleep(0.02)        

                                                                                                                                                    
    # 3. 前進                                                                                                                                                                
    # print("Moving forward...")                                                                                                                                               
    # duration = 4                                                                                                                                                           
    # start_time = time.time()                                                                                                                                                 
    # while time.time() - start_time < duration:                                                                                                                               
    #     client.Move(0.3, 0, 0)                                                                                                                                               
    #     time.sleep(0.02)  

    # print("Moving lateral LEFT...")                                                                                                                                               
    # duration = 1.5                                                                                                                                                             
    # start_time = time.time()                                                                                                                                                 
    # while time.time() - start_time < duration:                                                                                                                               
    #     client.Move(0, 0.3, 0)                                                                                                                                               
    #     time.sleep(0.02)           

    # print("Moving lateral RIGHT...")                                                                                                                                               
    # duration = 1.5                                                                                                                                                           
    # start_time = time.time()                                                                                                                                                 
    # while time.time() - start_time < duration:                                                                                                                               
    #     client.Move(0, -0.3, 0)                                                                                                                                               
    #     time.sleep(0.02)  

                                                                                                                                              
                                                                                                                                                                            
    # 4. 停止並趴下                                                                                                                                                          
    print("Stopping...")                                                                                                                                                     
    client.StopMove()                                                                                                                                                        
    time.sleep(1)                                                                                                                                                         
                                                                                                                                                                            
    print("Standing down...")                                                                                                                                                
    client.StandDown()                                                                                                                                                       
    print("Done!") 