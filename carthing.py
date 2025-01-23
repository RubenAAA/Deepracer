import math


reward_max = 5.0
reward_min = 0.001
last_progress = 0

######################################################################

def reward_function(params):
    reward = 0.0

    # Define the line (from point (7, -7) to (-6, 6))
    line_x = [7, -6]
    line_y = [-7, 6]

    # Calculate the slope
    m = (line_y[1] - line_y[0]) / (line_x[1] - line_x[0])
    # Calculate the y-value on the line for the car's x-coordinate
    line_y_at_car_x = m * params["x"] + line_y[0] - m * line_x[0]

    # Adjust sight based on position relative to the line
    if params["y"] < line_y_at_car_x:  # Below the line
        sight = 0.25  # Decrease sight for tighter focus
    else:  # Above the line
        sight = 0.5  # Increase sight for further lookahead

    racing_track = get_racing_track()

    target_idx, min_idx = draw_ray(params, racing_track, sight)

    reward += score_steering(params, racing_track[target_idx])

    reward += score_speed(params, racing_track[min_idx])

    reward += score_center(params, racing_track[min_idx])
    
    reward += score_progress(params)
    
    reward += score_speed_steering(params)

    return float(reward)

######################################################################

def get_distance(coor1, coor2):
    return math.sqrt(
        (coor1[0] - coor2[0]) * (coor1[0] - coor2[0])
        + (coor1[1] - coor2[1]) * (coor1[1] - coor2[1])
    )
######################################################################


def get_difference_degrees(angle1, angle2):
    diff = (angle1 - angle2) % (2.0 * math.pi)

    if diff >= math.pi:
        diff -= 2.0 * math.pi    
    return math.degrees(diff)


def get_distance_list(car, waypoints):
    dist_list = []
    min_dist = float("inf")
    min_idx = -1

    for i, waypoint in enumerate(waypoints):
        dist = get_distance(car, waypoint)
        if dist < min_dist:
            min_dist = dist
            min_idx = i
        dist_list.append(dist)

    return dist_list, min_dist, min_idx, len(waypoints)


######################################################################

def draw_ray(params, waypoints, sight):
    car = [params["x"], params["y"]]

    target_dist = params["track_width"] * sight

    dist_list, _, min_idx, length = get_distance_list(car, waypoints)

    target_idx = min_idx

    for i in range(5, int(length * 0.1)):
        index = (min_idx + i) % length
        target_idx = index
        if dist_list[index] >= target_dist:
            break

    return target_idx, min_idx
######################################################################

def score_steering(params, target):
    car = [params["x"], params["y"]]

    # target angle
    target_angle = math.atan2((target[1] - car[1]), (target[0] - car[0]))

    heading = params["heading"]
    steering = params["steering_angle"]

    # target steering
    target_steering = get_difference_degrees(target_angle, math.radians(heading))
    target_steering = max(-30, min(30, target_steering))

    # diff steering
    diff = abs(steering - target_steering)

    if diff > 30:
        score = 0.001  # Severe punishment for large steering differences
    else:
        score = 1.001 - (diff / 30)


    return max(min(score, reward_max), reward_min)


def score_speed(params, target):
    speed = params["speed"]

    # diff speed
    diff = abs(speed - target[2])

    score = 1.001 - (diff / 3)

    return max(min(score, reward_max), reward_min)


def score_speed_steering(params):
    speed = params["speed"]
    steering_angle = abs(params["steering_angle"])  # Absolute value to handle both left and right turns
    
    # Define thresholds
    max_speed = 4.0  # Maximum speed (example value, adjust for your track)
    max_steering = 30.0  # Maximum steering angle

    # Normalize speed and steering
    normalized_speed = speed / max_speed
    normalized_steering = steering_angle / max_steering

    # Reward formula: penalize high steering at high speed, reward high steering at low speed
    if speed >= 2.5:  # High speed case
        reward = 1.0 - (normalized_steering * normalized_speed)
    elif speed <= 1.5:  # Low speed case
        reward = 1.0 + (normalized_steering * (1.0 - normalized_speed))
    else:
        reward = 1.0 - (normalized_steering / 2.0)

    # Ensure reward is within bounds
    return max(reward_min, min(reward, reward_max))  # Reward clamped between 0.001 and 5.0



def score_center(params, target):
    track_width = params["track_width"]

    car = [params["x"], params["y"]]

    center = [target[0], target[1]]

    dist = get_distance(car, center)

    score = 1.001 - (abs(track_width - dist) / track_width)

    return max(min(score, reward_max), reward_min)
    
    
def score_progress(params):
    
    global last_progress
    
    progress = params["progress"]
        
    score = ((progress - last_progress)*reward_max) 
    
    last_progress = progress
    
    return max(score, reward_min)


def get_racing_track():
    return [
[1.17885, -0.02819, 4],
[0.97428, 0.19281, 4],
[0.77039, 0.41465, 4],
[0.56694, 0.63702, 4],
[0.36371, 0.85965, 3.5],
[0.16067, 1.08251, 3.5],
[-0.04223, 1.30555, 3.5],
[-0.24496, 1.5288, 3],
[-0.4475, 1.75228, 3],
[-0.64984, 1.97599, 3],
[-0.85192, 2.20003, 2.5],
[-1.05344, 2.42472, 2.5],
[-1.25411, 2.6504, 2.22318],
[-1.45594, 2.87373, 1.987],
[-1.66165, 3.09018, 1.80388],
[-1.87335, 3.29467, 1.5668],
[-2.09255, 3.48153, 1.3907],
[-2.31965, 3.64525, 1.23406],
[-2.55394, 3.78055, 1.09776],
[-2.79366, 3.8824, 1.09776],
[-3.03592, 3.94668, 1.09776],
[-3.27641, 3.96537, 1.09776],
[-3.50797, 3.93144, 1.0977],
[-3.71896, 3.83638, 1.0977],
[-3.88698, 3.67145, 1.21977],
[-4.0029, 3.46131, 1.35655],
[-4.06619, 3.23045, 1.47171],
[-4.08237, 2.99662, 1.61364],
[-4.06132, 2.76898, 1.74679],
[-4.01102, 2.55088, 1.89145],
[-3.93766, 2.34296, 1.55965],
[-3.84538, 2.14508, 1.26288],
[-3.73745, 1.95666, 1.26288],
[-3.61589, 1.77732, 1.26288],
[-3.46559, 1.58986, 1.26288],
[-3.3348, 1.39364, 1.26288],
[-3.24176, 1.18168, 1.2628],
[-3.20714, 0.94929, 1.45877],
[-3.22122, 0.70365, 1.73181],
[-3.27557, 0.44914, 2.10352],
[-3.36195, 0.1884, 2.5],
[-3.47031, -0.07663, 2.5],
[-3.59276, -0.35194, 2.5],
[-3.70757, -0.6305, 2.45447],
[-3.81255, -0.91281, 2.26315],
[-3.90467, -1.19896, 2.10558],
[-3.98055, -1.48823, 1.95433],
[-4.03675, -1.77896, 1.8058],
[-4.07006, -2.06871, 1.61297],
[-4.07796, -2.35438, 1.42626],
[-4.05785, -2.63218, 1.27572],
[-4.00829, -2.89786, 1.1279],
[-3.92841, -3.14667, 1.0],
[-3.81797, -3.37333, 1.0],
[-3.67586, -3.56992, 1.0],
[-3.50254, -3.72585, 1.0],
[-3.30246, -3.82919, 1.0],
[-3.0846, -3.8639, 1.0],
[-2.86921, -3.80985, 1.1893],
[-2.67215, -3.69283, 1.31765],
[-2.50144, -3.5234, 1.47389],
[-2.36194, -3.31176, 1.65465],
[-2.25566, -3.0676, 1.76875],
[-2.18244, -2.79938, 1.61197],
[-2.14008, -2.51429, 1.4814],
[-2.127, -2.22076, 1.30142],
[-2.08005, -1.94006, 1.17403],
[-1.99865, -1.67826, 1.0412],
[-1.88265, -1.44243, 1.0412],
[-1.73341, -1.24052, 1.04125],
[-1.55471, -1.08023, 1.04125],
[-1.35065, -0.97424, 1.04125],
[-1.13142, -0.93427, 1.04125],
[-0.91566, -0.97615, 1.16222],
[-0.71978, -1.08038, 1.27537],
[-0.55152, -1.2341, 1.38928],
[-0.41548, -1.42792, 1.52292],
[-0.31396, -1.65377, 1.69263],
[-0.2468, -1.90419, 1.5618],
[-0.21119, -2.17228, 1.2590],
[-0.2013, -2.45172, 1.05429],
[-0.21099, -2.73807, 1.05429],
[-0.23212, -3.0039, 1.05429],
[-0.22807, -3.25729, 1.05429],
[-0.18217, -3.48688, 1.05429],
[-0.08656, -3.68183, 1.05429],
[0.06315, -3.82604, 1.07185],
[0.25374, -3.91471, 1.19],
[0.47108, -3.94962, 1.3079],
[0.70518, -3.9303, 1.44545],
[0.9466, -3.85721, 1.5091],
[1.18401, -3.72675, 1.5792],
[1.43537, -3.64681, 1.54486],
[1.6916, -3.61315, 1.47565],
[1.9489, -3.62299, 1.37028],
[2.20403, -3.67601, 1.2804],
[2.45328, -3.77354, 1.202],
[2.69005, -3.92338, 1.02595],
[2.94411, -4.01746, 1.02595],
[3.19298, -4.05176, 1.02595],
[3.42603, -4.0289, 1.02595],
[3.63458, -3.95263, 1.02595],
[3.81, -3.82567, 1.02595],
[3.92786, -3.64044, 1.13441],
[3.98445, -3.41809, 1.27721],
[3.98138, -3.17723, 1.41599],
[3.92288, -2.9333, 1.576],
[3.81718, -2.69741, 1.76607],
[3.67368, -2.47517, 2.00827],
[3.50131, -2.26778, 2.37831],
[3.30856, -2.07314, 3],
[3.10167, -1.88845, 3],
[2.88408, -1.71115, 3.5],
[2.65936, -1.51577, 3.5],
[2.43924, -1.31391, 4],
[2.22303, -1.10716, 4],
[2.01001, -0.89649, 4],
[1.79956, -0.68266, 4],
[1.59116, -0.46633, 4],
[1.38438, -0.24802, 4]]