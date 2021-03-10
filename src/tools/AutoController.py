from src.tools.agents.navigation.behavior_agent import BehaviorAgent
class AutoCotroller():
    def __init__(self,adapted_vehicle,world):
        agent = BehaviorAgent(adapted_vehicle.carla_actor,ignore_traffic_light=False, behavior='normal')
        destination_list = [t.location for t in adapted_vehicle.path_transform_list]
        agent.set_many_destinations(destination_list, clean=True)
        while True:
            agent.update_information(world)
            if len(agent.get_local_planner().waypoints_queue) == 0:
                print("[{}]:reached".format(adapted_vehicle.name))
                adapted_vehicle.stop()
                break
            else:
                control = agent.run_step()
                adapted_vehicle.carla_actor.apply_control(control)
