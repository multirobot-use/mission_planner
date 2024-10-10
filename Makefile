task_id = i
human_target = human_target_1
distance = 1.5
number = 1
waypoints = 15 -6 1
tool = hammer
ugv = jackal
height = 2
# geo_wp = 92.6349 -0.073258 10 18.1892 -1.076 10
# geo_wp = 45 -13 2 5 -13 2
geo_wp = 45 -16 2 5 -16 2
geo_wp_short = 25 -6 2 5 -6 2

agent_prefix = uav
agent_id = 1
x = 0
y = 0
z = 0.3
h = 0

agent_debugger = none
planner_debugger = none

.PHONY: launch

main:
	@cd ~/grvc_planning_workspace; catkin build mission_planner

clean:
	@cd ~/grvc_planning_workspace; catkin clean mission_planner
	
gesture_info:
	@sed -n 8,21p src/gesture_recognition_faker.cpp

launch:
	@roslaunch mission_planner simulation.launch number_UAV:=$(number) planner_debugger:=$(planner_debugger) agent_debugger:=$(agent_debugger)

monitor:
	@rosrun mission_planner gesture_recognition_faker task_$(task_id) M $(human_target) $(distance) $(number)

monitorUGV:
	@rosrun mission_planner gesture_recognition_faker task_$(task_id) F $(ugv) $(height)

inspect:
	@rosrun mission_planner gesture_recognition_faker task_$(task_id) I $(waypoints)

inspectPVArray:
	@rosrun mission_planner gesture_recognition_faker task_$(task_id) A $(geo_wp)

deliver:
	@rosrun mission_planner gesture_recognition_faker task_$(task_id) D $(tool) $(human_target)

battery:
	@rostopic echo /$(agent_prefix)$(agent_id)/battery_fake

battery_off:
	@rostopic pub /$(agent_prefix)$(agent_id)/battery_fake/control mission_planner/BatteryControl 2 0.2 0.01 0.01

battery_ok:
	@rostopic pub /$(agent_prefix)$(agent_id)/battery_fake/control mission_planner/BatteryControl 2 1 0.01 0.01

battery_static:
	@rostopic pub /$(agent_prefix)$(agent_id)/battery_fake/control mission_planner/BatteryControl 0 0.9 0.01 0.01

mission_over:
	@rostopic pub /mission_over mission_planner/MissionOver "value: true"

groot:
	@rosrun groot Groot &

ros_simulation_tasks:
	@rosrun mission_planner gesture_recognition_faker PVArrayInspectionShort A $(geo_wp_short)
	@rosrun mission_planner gesture_recognition_faker Monitoring F $(ugv) $(height)
	#@rosrun mission_planner gesture_recognition_faker Monitoring D $(tool) $(human_target)
	@rosrun mission_planner gesture_recognition_faker PVArrayInspectionLong A $(geo_wp)

ros_simulation_failures:
	@rostopic pub /$(agent_prefix)2/battery_fake/control mission_planner/BatteryControl 2 0.2 0.01 0.01

#Phase 1 are the tests executted to validate the Agent Behavior Manager in simulations with a single UAV
phase_1_tasks_1:
	@echo "Mission Start. 3 Tasks requested, 1 of each type"
	@rosrun mission_planner gesture_recognition_faker task_1 D hammer human_target_1
	@rosrun mission_planner gesture_recognition_faker task_2 I 0 7 3 7 7 3 7 14 3 7 21 3 0 21 3 -7 21 3 -7 14 3 -7 7 3
	@rosrun mission_planner gesture_recognition_faker task_3 M human_target_1 1.5 4

phase_1_tasks_2:
	@echo "Unforeseen event: New Task"
	@rosrun mission_planner gesture_recognition_faker task_4 D hammer human_target_1

phase_1_tasks_3:
	@echo "Unforeseen event: New Task with a duplicated ID. Tool Delivery Task is overwrited."
	@rosrun mission_planner gesture_recognition_faker task_4 I 0 7 3 7 7 3 7 14 3 7 21 3 0 21 3 -7 21 3 -7 14 3 -7 7 3

phase_1_unforeseen_event_1:
	@echo "Unforeseen event: $(agent_prefix)$(agent_id) has low battery"
	@rostopic pub /$(agent_prefix)$(agent_id)/battery_fake/control mission_planner/BatteryControl "2" "0.2" "0.01" "0.01"

phase_1_unforeseen_event_2:
	@echo "Unforeseen event: $(agent_prefix)$(agent_id) has its battery charged"
	@rostopic pub /$(agent_prefix)$(agent_id)/battery_fake/control mission_planner/BatteryControl "2" "1" "0.01" "0.01"

phase_1_unforeseen_event_3:
	@echo "Unforeseen event: High-Level Planner block disconnection"
	@rosnode kill /high_level_planner

phase_1_unforeseen_event_4:
	@echo "Unforseen event: mission over"
	@rostopic pub /mission_over mission_planner/MissionOver "value: true"

#Phase 2 are the tests executted to validate the High-Level Planner in simulations with multiple UAVs
phase_2_tasks_1:
	@echo "Mission Start. 3 Tasks requested, 1 of each type"
	@rosrun mission_planner gesture_recognition_faker task_1 D hammer human_target_2
	@rosrun mission_planner gesture_recognition_faker task_2 I 0 7 3 7 7 3 7 14 3 7 21 3 0 21 3 -7 21 3 -7 14 3 -7 7 3
	@rosrun mission_planner gesture_recognition_faker task_3 M human_target_1 1.5 1

phase_2_tasks_2:
	@echo "Unforeseen event: New Tasks"
	@rosrun mission_planner gesture_recognition_faker task_4 D hammer human_target_2
	@rosrun mission_planner gesture_recognition_faker task_3 I 0 7 3 7 7 3 7 14 3 7 21 3 0 21 3 -7 21 3 -7 14 3 -7 7 3

phase_2_tasks_3:
	@echo "Unforeseen event: task params update"
	@rosrun mission_planner gesture_recognition_faker task_3 M human_target_1 3 4

phase_2_unforeseen_event_1:
	@echo "Unforeseen event: UAV2 has low battery"
	@rostopic pub /uav2/battery_fake/control mission_planner/BatteryControl "0" "0.2" "0.01" "0.01"

phase_2_unforeseen_event_2:
	@echo "Unforeseen event: UAV3 has low battery"
	@rostopic pub /uav3/battery_fake/control mission_planner/BatteryControl "0" "0.2" "0.01" "0.01"

phase_2_unforeseen_event_3:
	@echo "Unforeseen event: UAV2 has its battery charged"
	@rostopic pub /uav2/battery_fake/control mission_planner/BatteryControl "0" "1" "0.01" "0.01"

phase_2_unforeseen_event_4:
	@echo "Unforeseen event: UAV3 has its battery charged"
	@rostopic pub /uav3/battery_fake/control mission_planner/BatteryControl "0" "1" "0.01" "0.01"

phase_2_unforeseen_event_5:
	@echo "Unforeseen event: UAV2 disconnection"
	@rosnode kill /uav2/agent_behaviour_manager

phase_2_unforeseen_event_6:
	@echo "Unforeseen event: UAV4 disconnection"
	@rosnode kill /uav4/agent_behaviour_manager

phase_2_unforeseen_event_7:
	@echo "Unforeseen event: UAV2 reconnection"
	@rosrun mission_planner agent_behaviour_manager __ns:=uav2

phase_2_unforeseen_event_8:
	@echo "Unforeseen event: UAV4 reconnection"
	@rosrun mission_planner agent_behaviour_manager __ns:=uav4

phase_2_unforeseen_event_9:
	@echo "Unforseen event: mission over"
	@rostopic pub /mission_over mission_planner/MissionOver "value: true"
