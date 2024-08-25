There are 3 nodes for the working of the butler bot.

 1. order_manager.py
 2. confirmation.py
 3. robot_controller.py
 
 
 order_manager.py 
   
     -> This node manages the incomming order and publishes a Int32MultiArray data of the same to robot_controller.py.
    
    Usage:
    
        -> cd ~/butlerbot_ws
        -> source the setup file
        -> ros2 run butler_bot order_manager
        
    Example input:
       
        -> Single order
               > a1
         a stands for add, 1 is the number of the table and order.
         This adds and publishes order [1] to order_queue topic.
         
        -> Group order
               > a1 a2 a3 
          This adds and publishes order [1,2,3] to order_queue topic.
          
          
          
 confirmation.py
 
      -> This is a service node which gets the confirmation from table and kitchen and responds to the client node (robot_controller.py).
      
      
      Usage:
      
        -> cd ~/butlerbot_ws
        -> source the setup file
        -> ros2 run butler_bot confirmation
        
        
       Example input:
       
        -> depending upon the robot_controller's needs it will ask for confirmations 
        -> To respond with confirm 
                     > 1
        -> To NOT confirm 
                     > 0
        
        
 robot_controller.py
 
       -> This node manages the robot movement from home_state to kitchen to table using the confirmation, cancellation order and order_queue values.
       
       Usage:
      
        -> cd ~/butlerbot_ws
        -> source the setup file
        -> ros2 run robot_controller
        
        
       Example input:
       
        -> It will display us the robots various movements inside the cafe, no dynamic input from user is needed for this node.
                     
       
 
        
        
        
         
          
               
          
                 
        
         
