import scenic


def parse(code_file, params):
    scenario = scenic.scenarioFromFile(
        path=code_file, model="scenic.simulators.carla.model", params=params)
    return scenario
