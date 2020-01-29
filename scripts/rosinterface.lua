function startSimulation_callback(msg)
  sim.startSimulation()
end

function pauseSimulation_callback(msg)
  sim.pauseSimulation()
end

function stopSimulation_callback(msg)
  sim.stopSimulation()
end

function enableSyncMode_callback(msg)
  rosInterfaceSynModeEnabled=msg.data
  sim.setBoolParameter(sim.boolparam_rosinterface_donotrunmainscript,rosInterfaceSynModeEnabled)
end

function triggerNextStep_callback(msg)
  sim.setBoolParameter(sim.boolparam_rosinterface_donotrunmainscript,false)
end

function aux_callback(msg)
  simROS.publish(simStepDonePub,{data=true})
end

function publishSimState()
  local state=0 -- simulation not running
  local s=sim.getSimulationState()
  if s==sim.simulation_paused then
    state=2 -- simulation paused
  elseif s==sim.simulation_stopped then
    state=0 -- simulation stopped
  else
    state=1 -- simulation running
  end
  simROS.publish(simStatePub,{data=state})
end

if (sim_call_type==sim.syscb_init) then
-- Check if the RosInterface is available:
  moduleName=0
  moduleVersion=0
  index=0
  pluginFound=false
  while moduleName do
    moduleName,moduleVersion=sim.getModuleName(index)
    if (moduleName=='ROSInterface') then
      pluginFound=true
    end
  index=index+1
  end

  if pluginFound then
    --Subscribe rostopic
    --simROS.subscribe('topic_name','msgs_type','callback_function')
    startSub=simROS.subscribe('/startSimulation', 'std_msgs/Bool', 'startSimulation_callback')
    pauseSub=simROS.subscribe('/pauseSimulation', 'std_msgs/Bool', 'pauseSimulation_callback')
    stopSub=simROS.subscribe('/stopSimulation', 'std_msgs/Bool', 'stopSimulation_callback')
    enableSynModeSub=simROS.subscribe('/enableSyncMode', 'std_msgs/Bool', 'enableSyncMode_callback')
    triggerNextStepSub=simROS.subscribe('/triggerNextStep', 'std_msgs/Bool', 'triggerNextStep_callback')
    auxSub=simROS.subscribe('/privateMsgAux', 'std_msgs/Bool', 'aux_callback')
    --Publish rostopic
    --simROS.advertise('topic_name','msgs_type')
    simStepDonePub=simROS.advertise('/simulationStepDone', 'std_msgs/Bool')
    simStatePub=simROS.advertise('/simulationState','std_msgs/Int32')
    simTimePub=simROS.advertise('/simulationTime','std_msgs/Float32')
    auxPub=simROS.advertise('/privateMsgAux', 'std_msgs/Bool')
    --Unenable SyncMode with ros_controller_node
    sim.setScriptAttribute(sim.handle_self,sim.customizationscriptattribute_activeduringsimulation,true)
    rosInterfaceSynModeEnabled=false
  else
      sim.displayDialog('Error','The RosInterface was not found.',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
  end
end

if (sim_call_type==sim.syscb_nonsimulation) then
  if pluginFound then
    publishSimState()
  end
end

if (sim_call_type==sim.syscb_actuation) then
  if pluginFound then
    publishSimState()
    simROS.publish(simTimePub,{data=sim.getSimulationTime()})
  end
end

if (sim_call_type==sim.syscb_sensing) then
  if pluginFound then
    simROS.publish(auxPub,{data=true})
    sim.setBoolParameter(sim.boolparam_rosinterface_donotrunmainscript,rosInterfaceSynModeEnabled)
  end
end

if (sim_call_type==sim.syscb_suspended) then
  if pluginFound then
    publishSimState()
  end
end

if (sim_call_type==sim.syscb_aftersimulation) then
  if pluginFound then
    publishSimState()
  end
end

if (sim_call_type==sim.syscb_cleanup) then
  if pluginFound then
    simROS.shutdownSubscriber(startSub)
    simROS.shutdownSubscriber(pauseSub)
    simROS.shutdownSubscriber(stopSub)
    simROS.shutdownSubscriber(enableSynModeSub)
    simROS.shutdownSubscriber(triggerNextStepSub)
    simROS.shutdownSubscriber(auxSub)
    simROS.shutdownPublisher(auxPub)
    simROS.shutdownPublisher(simStepDonePub)
    simROS.shutdownPublisher(simStatePub)
    simROS.shutdownPublisher(simTimePub)
  end
end
