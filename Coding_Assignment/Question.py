def monitor_following_distance(distances: list[float], speeds: list[float]) -> tuple[int, float, int]:
    incident_counter = 0
    total_tailgating_secs = 0
    tailgating_now = False  #no actual use, helps me track state
    Running_incident = False
    if len(distances) ==0 :
        return (0, 0.0 ,0)
    else:
        min_distance = distances [0]
        for i in range(len(distances)):
            my_distance = distances [i]
            my_speed = speeds [i]
            safe_distance = my_speed* 0.5
            if my_distance < min_distance:
                min_distance = my_distance
            if my_distance >= safe_distance:
                tailgating_now = False
                Running_incident = False
            else:
                tailgating_now = True
                if Running_incident == True:
                    total_tailgating_secs = total_tailgating_secs +1
                else:
                    Running_incident = True
                    incident_counter = incident_counter +1
                    total_tailgating_secs = total_tailgating_secs +1
    return (total_tailgating_secs, min_distance,incident_counter)



