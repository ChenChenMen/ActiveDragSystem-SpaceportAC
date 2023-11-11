import ads
import json
import define_types

def main():
    json_file_name = "system_param.json"
    with open(json_file_name,"r",encoding="utf-8") as file:
        system_param = json.load(file)

    system = ads.ADS(system_param=system_param)
    ads_state = system.get_state()
    ads_clock = system.get_mission_time()
    prev_time = 0

    while ads_state is not define_types.State.DESCENT:
        prev_time = ads_clock
        ads_clock = system.get_mission_time()
        deployment, ads_state, abort = system.update(ads_clock-prev_time)
        if abort: break
    system.clean()
        
if __name__ == "__main__":
    main()
