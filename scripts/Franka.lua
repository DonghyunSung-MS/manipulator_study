function joint_set_msg_callback(msg)
    local msg_length = table.getn(msg['name'])
    local target_position = {}
    local target_velocity = {}
    local target_torque = {}
    for i=1,msg_length do
        for j=1,number_of_joint do
            if (msg['name'][i] == joint_names[j]) then
                sim.setJointTargetPosition(joint_handles[j], msg['position'][i])
            end
        end
    end
end

if (sim_call_type==sim.syscb_init) then
    joint_names={-1,-1,-1,-1,-1,-1,-1}
    joint_handles = joint_names
    for i=1,7,1 do
        joint_names[i]='Franka_joint'..i
        joint_handles[i]=sim.getObjectHandle('Franka_joint'..i)
    end
    number_of_joint = 7
    base = sim.getObjectHandle("Franka_link1")
    joint_pub = simROS.advertise('/Franka/joint_states', 'sensor_msgs/JointState')
    simROS.publisherTreatUInt8ArrayAsString(joint_pub)
    joint_sub = simROS.subscribe('/Franka/joint_set', 'sensor_msgs/JointState', 'joint_set_msg_callback')
end
if (sim_call_type==sim.syscb_sensing) then
    local frame_stamp = simROS.getTime() -- Use ROS Time
    local position = {}
    local velocity = {}
    local torque = {}
    for i=1,number_of_joint do
        position[i] = sim.getJointPosition(joint_handles[i])
        r, velocity[i] = sim.getObjectFloatParameter(joint_handles[i],sim.jointfloatparam_velocity)
        torque[i] = sim.getJointForce(joint_handles[i])
    end
    local joint_msg_data = {}
    joint_msg_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "Franka_link1"}
    joint_msg_data['name'] = joint_names
    joint_msg_data['position'] = position
    joint_msg_data['velocity'] = velocity
    joint_msg_data['effort'] = torque
    simROS.publish(joint_pub,joint_msg_data)
end


if (sim_call_type==sim.syscb_cleanup) then
    simROS.shutdownPublisher(joint_pub)
    simROS.shutdownSubscriber(joint_sub)
end
