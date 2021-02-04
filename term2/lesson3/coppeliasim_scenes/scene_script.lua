joystick_x = 0
joystick_y = 0
joystick_z = 0

function sysCall_init()
    -- do some initialization here
    h_joint_1 = sim.getObjectHandle('Revolute_joint_1')
    h_joint_2 = sim.getObjectHandle('Revolute_joint_2')
    
    -- Enable a subscriber:
    if simROS then
        print("<font color='#0F0'>ROS interface was found.</font>@html")        
        sub=simROS.subscribe('/read_joystick', 'geometry_msgs/Point', 'joystick_callback',1)
    else
        print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
    end
end

function joystick_callback(msg)
    --interfaces ROS sub callback
    joystick_x = msg.x
    joystick_y = msg.y
    trigger_value = msg.z
end

function sysCall_actuation()
    -- put your actuation code here
    
    --actuate each joint according to the two joystick values
    sim.setJointPosition(h_joint_1, 0.0025*(joystick_x-510))
    sim.setJointPosition(h_joint_2, 0.0025*(joystick_y-490))
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details

