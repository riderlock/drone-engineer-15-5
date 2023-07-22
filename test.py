from dronekit import connect, LocationGlobalRelative, VehicleMode
import time
import math


def reached_target_location(target_location):
    """
    目標の位置に到達したかどうかを判定する関数
    """
    current_location = vehicle.location.global_relative_frame
    distance = get_distance_metres(current_location, target_location)
    return distance < 150  # 目標に1.5m以内であれば到達したとみなす


def get_distance_metres(location1, location2):
    """
    2つの位置情報間の距離をメートル単位で計算する関数
    """
    dlat = (location2.lat - location1.lat) * 1.113195e5
    dlong = (location2.lon - location1.lon) * 1.113195e5
    dalt = location2.alt - location1.alt 
    distance_2d = math.sqrt((dlat * dlat) + (dlong * dlong))  # 直線距離を計算する
    distance_3d = math.sqrt((distance_2d * distance_2d) + (dalt * dalt))  # 3次元距離を計算する
    return distance_3d


main_p_loc = LocationGlobalRelative(35.878275,140.338069, 100)
boat_p_loc = LocationGlobalRelative(35.879768, 140.348495)
copter_p_loc = LocationGlobalRelative(35.867003, 140.305987)
namegawa_loc = LocationGlobalRelative(35.876991, 140.348026)
seven_loc = LocationGlobalRelative(35.877518, 140.295439)

plane_departure_flg = False
plane_arrive_flg = False

copter_departure_flg = False
copter_arrive_flg = False
rover1_departure_flg = False
rover1_arrive_flg = False

boat_departure_flg = False
boat_arrive_flg = False
rover2_departure_flg = False
rover2_arrive_flg = False

vehicle_plane  = connect('tcp:127.0.0.1:5763', wait_ready=False, timeout=30)

vehicle_copter = connect('tcp:127.0.0.1:5770', wait_ready=False, timeout=30)
vehicle_rover1 = connect('tcp:127.0.0.1:5762', wait_ready=False, timeout=30)

vehicle_boat   = connect('tcp:127.0.0.1:5762', wait_ready=False, timeout=30)
vehicle_rover2 = connect('tcp:127.0.0.1:5762', wait_ready=False, timeout=30)

vehicle_plane.parameters['WPNAV_SPEED'] = 2000
vehicle_plane.parameters['WPNAV_SPEED'] = 2000
vehicle_plane.parameters['WPNAV_SPEED'] = 2000
vehicle_plane.parameters['WPNAV_SPEED'] = 2000
vehicle_plane.parameters['WPNAV_SPEED'] = 2000


while True:
    if(False == plane_departure_flg):
        print('Plane出発')
        vehicle_plane.arm()
        vehicle_plane.mode = VehicleMode('GUIDED')
                                         
        vehicle_plane.wait_simple_takeoff(20, timeout=20)
        vehicle_plane.simple_goto(main_p_loc)

        plane_departure_flg = True

    if(True == plane_departure_flg and False == plane_arrive_flg ):
        print('Plane到着')
        loc_plane = vehicle_plane.location.global_relative_frame
        loc_copter = vehicle_copter.location.global_relative_frame
        distance = get_distance_metres(loc_plane, loc_copter)
        if(distance < 150):
            # Planeが到着
            plane_arrive_flg = True

    if(False == copter_depature_flg and True == plane_arrive_flg):
        print('Copter出発')
        vehicle_copter.arm()
        vehicle_copter.mode = VehicleMode('GUIDED')
                                         
        vehicle_copter.wait_simple_takeoff(5, timeout=20)
        vehicle_copter.simple_goto(copter_p_loc)

        copter_departure_flg = True

    if(False == boat_departure_flg and True == plane_arrive_flg):
        print('Boat出発')
        vehicle_boat.arm()
        vehicle_boat.mode = VehicleMode('GUIDED')
                                         
        vehicle_boat.wait_simple_takeoff(0, timeout=20)
        vehicle_boat.simple_goto(boat_p_loc)

        copter_departure_flg = True


    time.sleep(1.0)








