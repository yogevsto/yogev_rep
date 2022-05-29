import airsim
import numpy as np
from modules.Lidar import Lidar
from modules.PotentialField import PotentialField
from modules.Controller import distance, car_reverse, prepare_waypoint

client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
client.reset()
car_controls = airsim.CarControls()

lidar = Lidar(client)

state = client.getCarState()
waypoint = np.zeros(2)
waypoint[0] = state.kinematics_estimated.position.x_val
waypoint[1] = state.kinematics_estimated.position.y_val

while client.ping():

    points = []
    state = client.getCarState()
    for i in range(20):
        points = lidar.get_points()

    pot = PotentialField(client, points, 10, 1, 1, 5, 0.5)
    q = state.kinematics_estimated.orientation
    roll, pitch, yaw = pot.quaternion2euler(q)
    current_pos = state.kinematics_estimated.position

    if distance(current_pos, waypoint) < 9:
        waypoint = prepare_waypoint(current_pos, yaw, 10)
    dir_vec = pot.get_direction_vec(waypoint)
    car_controls.throttle = dir_vec[0]*0.5

    if dir_vec[0] > 0:
        car_controls.steering = dir_vec[1]
    else:
        car_controls.steering = 1*np.sign(dir_vec[1])
        car_reverse(car_controls)

    client.setCarControls(car_controls)
    car_controls.is_manual_gear = False
    car_controls.manual_gear = 0
