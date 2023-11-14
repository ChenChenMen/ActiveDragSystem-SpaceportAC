import json
import define_types
import ads

def main():
    # load a json file with system parameters
    json_file_name = "system_param.json"
    with open(json_file_name,"r",encoding="utf-8") as file:
        system_param = json.load(file)

    # create an Active Drag System object
    # initialize loop variables
    system = ads.ADS(system_param=system_param)
    ads_state = system.get_state()
    ads_clock = system.get_mission_time()

    # system operation loop, terminated by
    # 1) LV/system reached descent phase
    # 2) abort signal is sent from the ADS object
    while ads_state is not define_types.State.DESCENT:
        dt = system.get_mission_time()-ads_clock
        deployment, ads_state, abort = system.update(dt)
        ads_clock = system.get_mission_time()
        if abort: break

    system.clean()
        
if __name__ == "__main__":
    main()
