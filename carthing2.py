import math

# Define overall clamping boundaries
reward_max = 10.0
reward_min = 0.001

######################################################################
# MAIN REWARD FUNCTION
######################################################################
def reward_function(params):
    """
    AWS DeepRacer reward function with:
      - Off-track check
      - Weighted sub-rewards (steering, speed, center, progress)
      - 'sight' logic based on a line from (7, -7) to (-6, 6)
    """
    
    # 1) Immediately penalize if off track
    if params["is_offtrack"]:
            return 1e-3
        
    # 2) Base reward initialization
    reward = 0.0
    
    # --------------------------
    # 3) Adjust 'sight' based on line from (7, -7) to (-6, 6)
    # --------------------------
    line_x = [7, -6]
    line_y = [-7, 6]
    
    # Calculate slope (m) and intercept for line
    m = (line_y[1] - line_y[0]) / (line_x[1] - line_x[0])  # slope
    line_y_at_car_x = m * params["x"] + line_y[0] - m * line_x[0]
    
    # If car is below the line, narrower "sight", else broader
    if params["y"] < line_y_at_car_x:
        sight = 0.5
    else:
        sight = 1

    # 4) Get racing track (the set of reference waypoints)
    racing_track = get_racing_track()
    
    # 5) Identify relevant waypoint indices based on 'sight'
    target_idx, min_idx = draw_ray(params, racing_track, sight)
    
    # 6) Calculate sub-rewards
    steering_score = score_steering(params, racing_track[target_idx])
    speed_score    = score_speed(params)
    center_score   = score_center(params, racing_track[min_idx])
    progress_score = score_progress(params)
    
    # --------------------------
    # 7) Combine sub-rewards with weights
    # --------------------------
    # Adjust these weights to emphasize/de-emphasize certain behaviors
    steering_weight = 0.1
    speed_weight    = 0.5
    center_weight   = 0.3
    progress_weight = 0.1
    
    reward = (
        steering_weight * steering_score
        + speed_weight    * speed_score
        + center_weight   * center_score
        + progress_weight * progress_score
    )
    
    # 8) Final clamp
    reward = max(min(reward, reward_max), reward_min)
    
    return float(reward)


######################################################################
# HELPER FUNCTIONS
######################################################################
def get_distance(coor1, coor2):
    return math.sqrt(
        (coor1[0] - coor2[0]) ** 2
        + (coor1[1] - coor2[1]) ** 2
    )


def get_difference_degrees(angle1_radians, angle2_radians):
    """
    Returns the difference (angle1 - angle2) in degrees, between -180 and 180.
    """
    diff = (angle1_radians - angle2_radians) % (2.0 * math.pi)
    if diff >= math.pi:
        diff -= 2.0 * math.pi
    return math.degrees(diff)


def get_distance_list(car, waypoints):
    """
    Returns:
      dist_list: list of distances from car to each waypoint
      min_dist:  smallest distance
      min_idx:   index of the closest waypoint
      length:    number of waypoints
    """
    dist_list = []
    min_dist = float("inf")
    min_idx = -1

    for i, waypoint in enumerate(waypoints):
        dist = get_distance(car, waypoint)
        dist_list.append(dist)
        if dist < min_dist:
            min_dist = dist
            min_idx = i

    return dist_list, min_dist, min_idx, len(waypoints)


######################################################################
# LOGIC FOR FINDING A TARGET WAYPOINT (draw_ray)
######################################################################
def draw_ray(params, waypoints, sight):
    """
    Finds a 'target_idx' a certain distance ahead based on 'sight'
    relative to track width.
    """
    car = [params["x"], params["y"]]
    target_dist = params["track_width"] * sight
    
    dist_list, _, min_idx, length = get_distance_list(car, waypoints)

    # Start from the closest waypoint
    target_idx = min_idx
    
    # Look ahead through some fraction of waypoints
    for i in range(5, int(length * 0.1)):
        index = (min_idx + i) % length
        target_idx = index
        if dist_list[index] >= target_dist:
            break

    return target_idx, min_idx


######################################################################
# SUB-REWARD: STEERING
######################################################################
def score_steering(params, target_wp):
    """
    Encourages matching steering angle to the direction from car to the target waypoint.
    """
    car = [params["x"], params["y"]]
    
    # target angle (radians)
    target_angle = math.atan2(
        (target_wp[1] - car[1]),
        (target_wp[0] - car[0])
    )
    
    # Car's heading is in degrees, steering is also in degrees
    heading_deg = params["heading"]
    steering_deg = params["steering_angle"]
    
    # Convert heading_deg to radians, compute difference in degrees
    target_steering_deg = get_difference_degrees(target_angle, math.radians(heading_deg))
    # Clip the target steering to [-30, 30]
    target_steering_deg = max(-30, min(30, target_steering_deg))

    # Difference
    diff = abs(steering_deg - target_steering_deg)
    
    score = 1.1 - (diff / 30.0)
    
    score *= reward_max
    
    # Clamp
    return max(min(score, reward_max), reward_min)


######################################################################
# SUB-REWARD: SPEED
######################################################################
def score_speed(params):
    """
    Encourages speed above 1 m/s, but not too high that it deviates from good racing lines.
    """
    speed = params["speed"]
    
    # Example: base 0.1, plus 1 unit reward per 3 m/s above 1
    # So speed=4 => 0.1 + (3/3)= 1.1
    score = 0.1 + ((speed - 1.0) / 3.0)
    
    score *= reward_max
    
    # Clamp
    return max(min(score, reward_max), reward_min)


######################################################################
# SUB-REWARD: DISTANCE TO RACING LINE
######################################################################
def score_center(params, closest_wp):
    """
    Encourages being close to the 'closest_wp' (which can represent the ideal racing line).
    """
    car = [params["x"], params["y"]]
    line_point = [closest_wp[0], closest_wp[1]]
    
    dist_to_line = get_distance(car, line_point)
    
    # Reward is better the closer we are
    score = reward_max - dist_to_line
    
    # Clamp
    return max(min(score, reward_max), reward_min)


######################################################################
# SUB-REWARD: PROGRESS
######################################################################
def score_progress(params):
    """
    Rewards partial progress. 'progress' is in [0..100], percentage of track completed.
    """
    progress_percent = params["progress"]  # e.g., 0..100
    
    # A straightforward approach:
    score = (progress_percent / 100.0) * reward_max
    
    # Clamp
    return max(min(score, reward_max), reward_min)


######################################################################
# RACING TRACK WAYPOINTS
######################################################################
def get_racing_track():
    """
    Example reference path (waypoints). 
    Replace or adapt as needed for your specific track.
    """
    return [
       [ 1.17885, -0.02819],
       [ 0.97428,  0.19281],
       [ 0.77039,  0.41465],
       [ 0.56694,  0.63702],
       [ 0.36371,  0.85965],
       [ 0.16067,  1.08251],
       [-0.04223,  1.30555],
       [-0.24496,  1.5288 ],
       [-0.4475 ,  1.75228],
       [-0.64984,  1.97599],
       [-0.85192,  2.20003],
       [-1.05344,  2.42472],
       [-1.25411,  2.6504 ],
       [-1.45594,  2.87373],
       [-1.66165,  3.09018],
       [-1.87335,  3.29467],
       [-2.09255,  3.48153],
       [-2.31965,  3.64525],
       [-2.55394,  3.78055],
       [-2.79366,  3.8824 ],
       [-3.03592,  3.94668],
       [-3.27641,  3.96537],
       [-3.50797,  3.93144],
       [-3.71896,  3.83638],
       [-3.88698,  3.67145],
       [-4.0029 ,  3.46131],
       [-4.06619,  3.23045],
       [-4.08237,  2.99662],
       [-4.06132,  2.76898],
       [-4.01102,  2.55088],
       [-3.93766,  2.34296],
       [-3.84538,  2.14508],
       [-3.73745,  1.95666],
       [-3.61589,  1.77732],
       [-3.46559,  1.58986],
       [-3.3348 ,  1.39364],
       [-3.24176,  1.18168],
       [-3.20714,  0.94929],
       [-3.22122,  0.70365],
       [-3.27557,  0.44914],
       [-3.36195,  0.1884 ],
       [-3.47031, -0.07663],
       [-3.59276, -0.35194],
       [-3.70757, -0.6305 ],
       [-3.81255, -0.91281],
       [-3.90467, -1.19896],
       [-3.98055, -1.48823],
       [-4.03675, -1.77896],
       [-4.07006, -2.06871],
       [-4.07796, -2.35438],
       [-4.05785, -2.63218],
       [-4.00829, -2.89786],
       [-3.92841, -3.14667],
       [-3.81797, -3.37333],
       [-3.67586, -3.56992],
       [-3.50254, -3.72585],
       [-3.30246, -3.82919],
       [-3.0846 , -3.8639 ],
       [-2.86921, -3.80985],
       [-2.67215, -3.69283],
       [-2.50144, -3.5234 ],
       [-2.36194, -3.31176],
       [-2.25566, -3.0676 ],
       [-2.18244, -2.79938],
       [-2.14008, -2.51429],
       [-2.127  , -2.22076],
       [-2.08005, -1.94006],
       [-1.99865, -1.67826],
       [-1.88265, -1.44243],
       [-1.73341, -1.24052],
       [-1.55471, -1.08023],
       [-1.35065, -0.97424],
       [-1.13142, -0.93427],
       [-0.91566, -0.97615],
       [-0.71978, -1.08038],
       [-0.55152, -1.2341 ],
       [-0.41548, -1.42792],
       [-0.31396, -1.65377],
       [-0.2468 , -1.90419],
       [-0.21119, -2.17228],
       [-0.2013 , -2.45172],
       [-0.21099, -2.73807],
       [-0.23212, -3.0039 ],
       [-0.22807, -3.25729],
       [-0.18217, -3.48688],
       [-0.08656, -3.68183],
       [ 0.06315, -3.82604],
       [ 0.25374, -3.91471],
       [ 0.47108, -3.94962],
       [ 0.70518, -3.9303 ],
       [ 0.9466 , -3.85721],
       [ 1.18401, -3.72675],
       [ 1.43537, -3.64681],
       [ 1.6916 , -3.61315],
       [ 1.9489 , -3.62299],
       [ 2.20403, -3.67601],
       [ 2.45328, -3.77354],
       [ 2.69005, -3.92338],
       [ 2.94411, -4.01746],
       [ 3.19298, -4.05176],
       [ 3.42603, -4.0289 ],
       [ 3.63458, -3.95263],
       [ 3.81   , -3.82567],
       [ 3.92786, -3.64044],
       [ 3.98445, -3.41809],
       [ 3.98138, -3.17723],
       [ 3.92288, -2.9333 ],
       [ 3.81718, -2.69741],
       [ 3.67368, -2.47517],
       [ 3.50131, -2.26778],
       [ 3.30856, -2.07314],
       [ 3.10167, -1.88845],
       [ 2.88408, -1.71115],
       [ 2.65936, -1.51577],
       [ 2.43924, -1.31391],
       [ 2.22303, -1.10716],
       [ 2.01001, -0.89649],
       [ 1.79956, -0.68266],
       [ 1.59116, -0.46633],
       [ 1.38438, -0.24802],
       [ 1.17885, -0.02819]
    ]
