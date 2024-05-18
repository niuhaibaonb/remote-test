import os
import sys

sys.path.append("/")
import plexe
from plexe import Plexe, ACC, CACC, FAKED_CACC, RPM, GEAR, ACCELERATION, SPEED, PLOEG
import random
from utils import add_platooning_vehicle, communicate, get_distance, start_sumo, running, add_joiner_vehicle

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

from parameter import *
# 该程序与 brakedemo.py 中类似，向场景中加入车辆，这里包含 1 个 platoon 和 1 个单独的车辆
# 返回 platoon 的 topology，这个 topology 中不包含后加入的单个车辆


def add_vehicles(plexe, n, real_engine=False, Jposition=609):

    # add a platoon of n vehicles
    # plexe = Plexe()
    topology = {}

    for i in range(n):
        vid = "v.%d" % i
        # 生成车排车辆
        add_platooning_vehicle(plexe, vid, (n - i + 1) * (DISTANCE + LENGTH) +
                               800, 0, SPEED_platoon, DISTANCE, real_engine) #2120
        plexe.set_vehicles_file( vid, filename='vehicles.xml')

        plexe.set_vehicle_model( vid, model='truck')
        plexe.set_fixed_lane(vid, 0, safe=True) # 车排生成位置 本来是2270
        # traci.vehicle.changeLane(vid, 0, 0)
        traci.vehicle.setSpeedMode(vid, 0)
        if i == 0:
            plexe.set_active_controller(vid, ACC)
        else:
            plexe.set_active_controller(vid, CACC)
        if i > 0:
            topology[vid] = {"front": "v.%d" % (i - 1), "leader": LEADER}
    # add a vehicle that wants to join the platoon
    vid = "v.%d" % n

    add_joiner_vehicle(plexe, vid, Jposition, 0, JOINer_speed, DISTANCE, real_engine)
    plexe.set_vehicles_file(vid, filename='vehicles.xml')

    plexe.set_vehicle_model(vid, model='truck')
    plexe.set_fixed_lane(vid, 0, safe=True)
    traci.vehicle.setSpeedMode(vid, 0)
    plexe.set_active_controller(vid, ACC)  # merge vehicle的控制器
    # 传入cacc算法需要的参数
    plexe.set_path_cacc_parameters(vid, distance=JOIN_DISTANCE)
    return topology

# def get_joiner_pos(plexe,J_ID):
#     target_vehicle_pos = traci.vehicle.getPosition(J_ID)
#     return (f"Current position of {J_ID}: ({target_vehicle_pos[0]}, {target_vehicle_pos[1]})")
#
# def set_interdistance(topology,n):
#     for i in range(n):
#         vid = "v.%d" % i




# 该函数的作用是通过改变待加入车辆的通讯拓扑，令其到达指定位置，准备变道加入 platoon
def get_in_position(plexe, jid, fid, topology,Join_speed):

    topology[jid] = {"leader": LEADER, "front": fid}
    plexe.set_cc_desired_speed(jid, Join_speed) # 此处速度的设置有待考虑
    # plexe.set_active_controller(jid, FAKED_CACC)
    plexe.set_active_controller(jid, ACC)

    return topology


# 该函数的作用是当待加入车辆到达指定位置时，后车为其留出足够的距离
# 在此过程中，platoon 分裂，后车成为新的 leader，带领它的 follower 减速，为待加入车辆留出足够的空间
def open_gap(plexe, vid, jid, topology, n,d_current, v_merge, v_follower):
    """
    Makes the vehicle that will be behind the joiner open a gap
    """
    d_trigger = 20
    index = int(vid.split(".")[1])
    for i in range(index + 1, n):
        # temporarily change the leader
        topology["v.%d" % i]["leader"] = vid

    # the front vehicle if the vehicle opening the gap is the joiner
    topology[vid]["front"] = jid
    # plexe.set_active_controller(vid, FAKED_CACC)
    plexe.set_active_controller(vid, ACC)

    if d_current < d_trigger:
        v_relative = v_follower - v_merge
        s_extra = max(0, v_relative * 0.8)  # SAFETY_DISTANCE_INCREMENT_FACTOR 是一个您需要设置的常数
        g_dynamic = JOIN_DISTANCE + s_extra
    else:
        g_dynamic = JOIN_DISTANCE  # 如果 d_current >= d_trigger, 使用基础JOIN_DISTANCE
    plexe.set_path_cacc_parameters(vid, distance=g_dynamic)
    # plexe.set_path_cacc_parameters(vid, distance=g_dynamic+2)

    return topology


# 该函数的作用是，完成 join 操作后，修改通讯网络拓扑，回归最初的 CACC 情形，都以头车作为 leader
def reset_leader(vid, topology, n):
    """
    After the maneuver is completed, the vehicles behind the one that opened

    """
    index = int(vid.split(".")[1])
    for i in range(index + 1, n):
        # restore the real leader
        topology["v.%d" % i]["leader"] = LEADER
    return topology
def join_final(plexe,JOIN_POSITION,J_ID,step,state, N_VEHICLES, topology,join_speed,v_merge,merging_head_position,front_tail_position):
    F_ID = "v.%d" % (JOIN_POSITION - 1)  # 前车
    if state != COMPLETED:
        topology = get_in_position(plexe, J_ID, F_ID, topology, Join_speed=join_speed)
        # 当车辆达到指定位置，令后车留出足够的空间，以便汇入。
    if state == GOING_TO_POSITION and step > 0:
        # when the distance of the joiner is small enough, let the others
        # open a gap to let the joiner enter the platoon
        d_current = get_distance(plexe, J_ID, F_ID)

        if get_distance(plexe, J_ID, F_ID) < JOIN_DISTANCE*2:
            state = OPENING_GAP
    if state == OPENING_GAP:
        if front_tail_position >merging_head_position +1:
            # 换道
            plexe.set_fixed_lane(J_ID, 1, safe=False)
            # traci.vehicle.changeLane(J_ID, 1, duration=0)
            # DATA= plexe.get_vehicle_data(J_ID)
            # print(DATA[SPEED])
            plexe.set_active_controller(J_ID, CACC)
            plexe.set_path_cacc_parameters(J_ID, distance=DISTANCE)
            # plexe.set_active_controller(BEHIND_JOIN, CACC)
            # plexe.set_path_cacc_parameters(BEHIND_JOIN, distance=DISTANCE)
            topology = reset_leader(J_ID, topology, N_VEHICLES)
            state = COMPLETED
    else:
        state = state
        topology = topology

    return state, topology


def join_platoon(plexe,JOIN_POSITION,J_ID, step, state, N_VEHICLES, topology,join_speed,is_merge,v_merge,v_follower,merging_tail_position,follower_head_position,merging_head_position,front_tail_position):
    F_ID = "v.%d" % (JOIN_POSITION - 1)
    BEHIND_JOIN = "v.%d" % JOIN_POSITION
    if is_merge == True:


        # 程序执行 100 步 (0.01s * 100 = 1s) 后，令单个车辆驶近目标位置

        # if step == 50:
            # at 1 second, let the joiner get closer to the platoon
        if state !=COMPLETED:
            topology = get_in_position(plexe, J_ID, F_ID, topology,Join_speed=join_speed)

        # 当车辆达到指定位置，令后车留出足够的空间，以便汇入。
        if state == GOING_TO_POSITION and step > 0:
            # when the distance of the joiner is small enough, let the others
            # open a gap to let the joiner enter the platoon
            d_current = get_distance(plexe, J_ID, F_ID)

            if get_distance(plexe, J_ID, F_ID) < JOIN_DISTANCE*2:


                topology = open_gap(plexe, BEHIND_JOIN, J_ID, topology, N_VEHICLES,d_current,v_merge,v_follower)
                state = OPENING_GAP

        # 当空间足够大时，完成汇入
        if state == OPENING_GAP:
            # when the gap is large enough, complete the maneuver
            # if (get_distance(plexe, BEHIND_JOIN, F_ID) > 2.1 * JOIN_DISTANCE)  :


            if (get_distance(plexe, BEHIND_JOIN, F_ID) > 1.5 * JOIN_DISTANCE):
                plexe.set_fixed_lane(J_ID, 1, safe=False)
                # traci.vehicle.changeLane(J_ID, 1, duration=0)
                # DATA= plexe.get_vehicle_data(J_ID)
                # print(DATA[SPEED])
                plexe.set_active_controller(J_ID, CACC)
                plexe.set_path_cacc_parameters(J_ID, distance=DISTANCE)
                plexe.set_active_controller(BEHIND_JOIN, CACC)
                plexe.set_path_cacc_parameters(BEHIND_JOIN, distance=DISTANCE)
                topology = reset_leader(BEHIND_JOIN, topology, N_VEHICLES)
                state = COMPLETED
    else:
        state = state
        topology = topology

    return state,topology



