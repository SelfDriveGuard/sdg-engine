""" Scenario Description
Voyage OAS Scenario Unique ID: 2-2-XX-CF-STR-CAR:Pa>E:03
The car ahead of ego that is badly parked over the sidewalk cuts into ego vehicle's lane.
This scenario may fail if there exists any obstacle (e.g. fences) on the sidewalk 
"""

#SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../third-party/scenic/CARLA/Town03.xodr') 
param carla_map = 'Town03'
model scenic.domains.driving.model

EGO_MODEL = "vehicle.lincoln.mkz2017"
EGO_SPEED = 10

# EGO BEHAVIOR: Follow lane and brake when reaches threshold distance to obstacle
behavior EgoBehavior(speed=10):
    try:
        do FollowLaneBehavior(speed)
    interrupt when withinDistanceToObjsInLane(self, 10):
        take SetBrakeAction(1.0)

## DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py 

# Background activity
background_vehicles = []
for _ in range(25):
    lane = Uniform(*network.lanes)
    spot = OrientedPoint on lane.centerline

    background_car = Car at spot,
        with behavior AutopilotBehavior()
    background_vehicles.append(background_car)

background_walkers = []
for _ in range(10):
    sideWalk = Uniform(*network.sidewalks)
    background_walker = Pedestrian in sideWalk,
        with behavior WalkBehavior()
    background_walkers.append(background_walker)


ego = Car following roadDirection from spot for Range(-30, -20),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED)

lane = Uniform(*network.lanes)
start = OrientedPoint on lane.centerline
av_ego = Car at start offset along roadDirection by 0 @ 7 ,
		with rolename "AV_EGO",
		with behavior FollowLaneBehavior(10)