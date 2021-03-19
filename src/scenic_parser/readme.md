# User Guide

## Set map

Parameter `map` and `carla_map` shall be set at the beginning:

```
param map = localPath('../third-party/scenic/CARLA/{CARLA_MAP_NAME}.xodr') 
param carla_map = {CARLA_MAP_NAME}
```

{CARLA_MAP_NAME} could be:

```
Town01
Town02
Town03
Town04
Town05
Town06
Town07
Town10
```

## Set autonomous ego

If you want to create a car controlled by AV System such as Autoware, specify its rolename as "AV_EGO". For example:

```
......(ignore previous code)
av_ego = Car at start,
		with rolename "AV_EGO"
```