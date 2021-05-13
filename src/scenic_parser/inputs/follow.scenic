""" Scenario Description
The ego vehicle follows the lead car which suddenly stops
The av_ego follows the lane
"""

#SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../third-party/scenic/CARLA/Town03.xodr') 
param carla_map = 'Town03'
model scenic.domains.driving.model

#CONSTANTS
MAX_BREAK_THRESHOLD = 1
SAFETY_DISTANCE = 10
INITIAL_DISTANCE_APART = -1*Uniform(5, 10)
STEPS_PER_SEC = 10

##DEFINING BEHAVIORS
behavior LeadCarBehavior():
	try:
		do FollowLaneBehavior()
	interrupt when simulation().currentTime > 15 * STEPS_PER_SEC:
		take SetBrakeAction(MAX_BREAK_THRESHOLD)


behavior CollisionAvoidance():
	while withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
		take SetBrakeAction(MAX_BREAK_THRESHOLD)


behavior FollowLeadCarBehavior():
	try: 
		do FollowLaneBehavior()

	interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
		do CollisionAvoidance()


##DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py

roads = network.roads

# make sure to put '*' to uniformly randomly select from all elements of the list, 'network.roads'
select_road = network.roads[0]
select_lane = select_road.lanes[0]

lead_point = OrientedPoint on select_lane.centerline

lead = Car at lead_point offset along roadDirection by 0 @ 20,
		with behavior LeadCarBehavior()


ego = Car following roadDirection from lead for INITIAL_DISTANCE_APART,
		with behavior FollowLeadCarBehavior()


av_ego = Car at lead_point offset along roadDirection by 0 @ 2 ,
		with rolename "AV_EGO",
		with behavior FollowLaneBehavior(10)