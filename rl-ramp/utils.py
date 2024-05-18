

import sys
import os
import random
import math
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib
import traci
from plexe import POS_X, POS_Y, ENGINE_MODEL_REALISTIC


# lane change state bits
bits = {
    0: 'LCA_NONE',
    1 << 0: 'LCA_STAY',
    1 << 1: 'LCA_LEFT',
    1 << 2: 'LCA_RIGHT',
    1 << 3: 'LCA_STRATEGIC',
    1 << 4: 'LCA_COOPERATIVE',
    1 << 5: 'LCA_SPEEDGAIN',
    1 << 6: 'LCA_KEEPRIGHT',
    1 << 7: 'LCA_TRACI',
    1 << 8: 'LCA_URGENT',
    1 << 9: 'LCA_BLOCKED_BY_LEFT_LEADER',
    1 << 10: 'LCA_BLOCKED_BY_LEFT_FOLLOWER',
    1 << 11: 'LCA_BLOCKED_BY_RIGHT_LEADER',
    1 << 12: 'LCA_BLOCKED_BY_RIGHT_FOLLOWER',
    1 << 13: 'LCA_OVERLAPPING',
    1 << 14: 'LCA_INSUFFICIENT_SPACE',
    1 << 15: 'LCA_SUBLANE',
    1 << 16: 'LCA_AMBLOCKINGLEADER',
    1 << 17: 'LCA_AMBLOCKINGFOLLOWER',
    1 << 18: 'LCA_MRIGHT',
    1 << 19: 'LCA_MLEFT',
    1 << 30: 'LCA_UNKNOWN'
}


def add_vehicle(plexe, vid, position, lane, speed, vtype="vtypeauto"):
    if plexe.version[0] >= 1:
        traci.vehicle.add(vid, "platoon_route", departPos=str(position),
                          departSpeed=str(speed), departLane=str(lane),
                          typeID=vtype)
    else:
        traci.vehicle.add(vid, "platoon_route", pos=position, speed=speed,
                          lane=lane, typeID=vtype)
def add_joiner(plexe, vid, position, lane, speed, vtype="vtypeauto"):
    if plexe.version[0] >= 1:
        traci.vehicle.add(vid, "join_route", departPos=str(position),
                          departSpeed=str(speed), departLane=str(lane),
                          typeID=vtype)
    else:
        traci.vehicle.add(vid, "join_route", pos=position, speed=speed,
                          lane=lane, typeID=vtype)

def add_hv_vehicle(plexe, vid, position, lane, speed, cacc_spacing,
                           vtype, real_engine=False):

    add_vehicle(plexe, vid, position, lane, speed, vtype)

    plexe.set_path_cacc_parameters(vid, cacc_spacing, 2, 1, 0.5)
    plexe.set_cc_desired_speed(vid, speed)
    plexe.set_acc_headway_time(vid, 1.5)

    if real_engine:
        plexe.set_engine_model(vid, ENGINE_MODEL_REALISTIC)
        plexe.set_vehicles_file(vid, "vehicles.xml")
        plexe.set_vehicle_model(vid, "alfa-147")
    # set color for HV
    if vtype == "passenger":
        traci.vehicle.setColor(vid, ((0, 205, 102)))
    else :
        traci.vehicle.setColor(vid, ((154, 255, 154)))

def add_platooning_vehicle(plexe, vid, position, lane, speed, cacc_spacing,
                           real_engine=False, vtype="vtypeauto"):

    add_vehicle(plexe, vid, position, lane, speed, vtype)

    plexe.set_path_cacc_parameters(vid, cacc_spacing, 2, 1, 0.5)
    plexe.set_cc_desired_speed(vid, speed)
    plexe.set_acc_headway_time(vid, 1.5)  # 车头时距
    if real_engine:
        plexe.set_engine_model(vid, ENGINE_MODEL_REALISTIC)
        plexe.set_vehicles_file(vid, "vehicles.xml")
        plexe.set_vehicle_model(vid, "alfa-147")
    traci.vehicle.setColor(vid, (random.uniform(0, 255),
                                 random.uniform(0, 255),
                                 random.uniform(0, 255), 255))


def add_joiner_vehicle(plexe, vid, jposition, lane, speed, cacc_spacing,
                           real_engine=False, vtype="vtypeauto"):

    add_joiner(plexe, vid, jposition, lane, speed, vtype)

    plexe.set_path_cacc_parameters(vid, cacc_spacing, 2, 1, 0.5)
    plexe.set_cc_desired_speed(vid, speed)
    plexe.set_acc_headway_time(vid, 1.5)
    if real_engine:
        plexe.set_engine_model(vid, ENGINE_MODEL_REALISTIC)
        plexe.set_vehicles_file(vid, "vehicles.xml")
        plexe.set_vehicle_model(vid, "alfa-147")
    traci.vehicle.setColor(vid, (random.uniform(0, 255),
                                 random.uniform(0, 255),
                                 random.uniform(0, 255), 255))
def get_distance(plexe, v1, v2):

    v1_data = plexe.get_vehicle_data(v1)
    v2_data = plexe.get_vehicle_data(v2)
    return math.sqrt((v1_data[POS_X] - v2_data[POS_X])**2 +
                     (v1_data[POS_Y] - v2_data[POS_Y])**2) - 4


def communicate(plexe, topology):

    for vid, l in topology.items():
        if "leader" in l.keys():
            # get data about platoon leader
            ld = plexe.get_vehicle_data(l["leader"])
            # pass leader vehicle data to CACC
            plexe.set_leader_vehicle_data(vid, ld)
            # pass data to the fake CACC as well, in case it's needed
            plexe.set_leader_vehicle_fake_data(vid, ld)
        if "front" in l.keys():
            # get data about platoon leader
            fd = plexe.get_vehicle_data(l["front"])
            # pass front vehicle data to CACC
            plexe.set_front_vehicle_data(vid, fd)
            # compute GPS distance and pass it to the fake CACC
            distance = get_distance(plexe, vid, l["front"])
            plexe.set_front_vehicle_fake_data(vid, fd, distance)


def start_sumo(config_file, already_running, gui=True):
    """
    Starts or restarts sumo with the given configuration file

    """
    arguments = ["--lanechange.duration", "3", "-c"]
    sumo_cmd = [sumolib.checkBinary('sumo-gui' if gui else 'sumo')]
    arguments.append(config_file)
    if already_running:
        traci.load(arguments)
    else:
        sumo_cmd.extend(arguments)
        traci.start(sumo_cmd)


def running(demo_mode, step, max_step):

    if demo_mode:
        return True
    else:
        return step <= max_step


def get_status(status):

    st = ""
    for i in range(32):
        mask = 1 << i
        if status & mask:
            if mask in bits.keys():
                st += " " + bits[mask]
            else:
                st += " 2^" + str(i)
    return st
