
import gym
from gym import spaces
from sb3_contrib import RecurrentPPO
import numpy as np
import traci
from join_platoon import *
from parameter import *
from plexe import POS_X
from plexe import Plexe, SPEED


# 合并位置取值列表
merge_pos_list = [0.1, 0.2, 0.3, 0.4]
# 合并速度取值列表
merge_speed_list = [10, 20, 30, 40]
class SumoGymEnv(gym.Env):
    def __init__(self, sumo_cmd):
        super().__init__()
        self.sumo_cmd = sumo_cmd
        self.sumo_config_file = "cfg/freeway.sumo.cfg"
        self.N_VEHICLES = N_VEHICLES
        self.feature = 5
        self.join_vec = 1
        self.end_pos = 2700
        self.pos = 0
        # self.state = state
        self.veh_list = []
        self.speed = 0
        self.obser = np.zeros((9, self.feature))
        self.oberstion = []
        self.i = 0
        self.observation_space = spaces.Box(low=-float('inf'), high=float('inf'),
                                            shape=((self.N_VEHICLES + 1), self.feature))
        self.reward_range = (-float('inf'), float('inf'))
        self.endGap=0
        self.Join1_speed = 12

        min_position = 1  # 最小位置
        max_position = 7  # 最大位置
        min_speed = 23  # 最小速度
        max_speed = 40  # 最大速度

        # 定义合并位置动作空间为离散空间
        position_space = spaces.Discrete(max_position - min_position + 1)

        # 定义合并速度动作空间为连续空间
        speed_space = spaces.Box(low=min_speed, high=max_speed, shape=(1,), dtype=np.float32)

        # 定义二维动作空间为合并位置和合并速度的组合
        action_space = spaces.Tuple((position_space, speed_space))
        self.action_space = action_space

    def reset(self):
        try:
            traci.close()
        except:
            pass
        start_sumo("./cfg/freeway.sumo.cfg", False)
        step = 0
        state = GOING_TO_POSITION
        random.seed(1)
        self.step_counter = 0
        self.veh_list = []

        return self._get_observation()

    def seed(self, seed):  # 种子
        np.random.seed(seed)

    def _apply_action(self, action, step, state, real_engine, setter, topology, Plexe, merge_position, merge_speed,v_follower):
        plexe = Plexe()
        traci.addStepListener(plexe)

        state,topology = join_platoon(plexe,JOIN_POSITION=merge_position,join_speed=merge_speed,J_ID =JOINER,step = step , state = state ,N_VEHICLES=8 ,topology=topology,is_merge=True,v_merge=self.Join1_speed,v_follower=v_follower)
        return state,topology
    def _get_observation(self, Plexe):

        plexe = Plexe()
        J_speed = 0
        acceleration = 0

        # route_id = traci.route.getIDList()[0]
        num_vehicles = len(traci.vehicle.getIDList())  # ID
        all_vehicles = traci.vehicle.getIDList()
        selected_vehicles = [veh for veh in all_vehicles if veh.startswith('v.') and int(veh.split('.')[1]) in range(9)]
        self.veh_list = selected_vehicles

        join_data = plexe.get_vehicle_data(J_ID)  # 新卡车的vehicle—data
        join_vehicle_pos = join_data[POS_X]  # 新卡车的pos——x
        self.endGap = self.end_pos - join_vehicle_pos  # 新卡车与end_pos的间距、
        J_speed = join_data[SPEED]  # 新卡车的速度
        self.Join1_speed = J_speed

        # for vehicle_id in self.veh_list:
        #     # 获取车辆的速度和位置
        #     speed = traci.vehicle.getSpeed(vehicle_id)
        #     x_position = traci.vehicle.getPosition(vehicle_id)[0]
        #
        #     # 将速度和位置添加到观测列表
        #     self.oberstion.append(speed)
        #     self.oberstion.append(x_position)
        # return self.oberstion
        for j, veh_id in enumerate(self.veh_list):
            # if self.i >= self.N_VEHICLES:
            #     break
            # else:
                # 局部特征
            platoon_data = plexe.get_vehicle_data(veh_id)
            # speed = platoon_data[SPEED]
            # acceleration = traci.vehicle.getAcceleration(veh_id)

            self.speed = traci.vehicle.getSpeed(veh_id)
            self.pos = platoon_data[POS_X]
            acceleration = platoon_data[ACCELERATION]
            self.obser[j] = [self.speed, self.pos, self.endGap, J_speed, join_vehicle_pos]
        self.i += 1
        return acceleration


def main(demo_mode, real_engine=False, setter=None):
    # used to randomly color the vehicles
    random.seed(1)
    start_sumo("./cfg/freeway.sumo.cfg", False)
    plexe = Plexe()
    traci.addStepListener(plexe)
    step = 0
    state = GOING_TO_POSITION
    env = SumoGymEnv("sumo")

    while running(demo_mode, step, 6000):
        # a = plexe.get_vehicle_data("v.1")
        # print(a)
        # when reaching 60 seconds, reset the simulation when in demo_mode
        if demo_mode and step == 5000:
            start_sumo("./cfg/freeway.sumo.cfg", False)
            step = 0
            state = GOING_TO_POSITION
            random.seed(1)

        traci.simulationStep()
        if step == 0:
            # create vehicles and track the joiner
            topology = add_vehicles(plexe, N_VEHICLES, real_engine)
            traci.gui.trackVehicle("View #0", JOINER)
            traci.gui.setZoom("View #0", 1000)
        if step % 10 == 1:
            # simulate vehicle communication every 100 ms
            communicate(plexe, topology)
        ober = env._get_observation(Plexe)
        # print(ober)


        action = [4,40]
        merge_position,merge_speed = action
        BEHIND_JOIN = "v.%d" % merge_position
        v_follower = traci.vehicle.getSpeed(BEHIND_JOIN)
        lane_change_state = traci.vehicle.getLaneChangeState("v.8",0)
        # print(lane_change_state)


        # if real_engine and setter is not None:
        #     # if we are running with the dashboard, update its values
        #     tracked_id = traci.gui.getTrackedVehicle("View #0")
        #     if tracked_id != "":
        #         ed = plexe.get_engine_data(tracked_id)
        #         vd = plexe.get_vehicle_data(tracked_id)
        #         setter(ed[RPM], ed[GEAR], vd.speed, vd.acceleration)

        step += 1


        # print(topology)



        # if step == 0:
        #     # create vehicles and track the joiner
        #     topology = add_vehicles(plexe, N_VEHICLES, real_engine)
        #     traci.gui.trackVehicle("View #0", JOINER)
        #     traci.gui.setZoom("View #0", 2000)
        # if step % 10 == 1:
        #     # simulate vehicle communication every 100 ms
        #     communicate(plexe, topology)
        #
        #
        #
        # state,topology = join_platoon(plexe,JOIN_POSITION,J_ID =JOINER,step = step , state = state ,N_VEHICLES=8 ,topology=topology)
        #
        # # obser = env._get_observation(Plexe)
        # # print(obser)
        # print("ddd")
        # if real_engine and setter is not None:
        #     # if we are running with the dashboard, update its values
        #     tracked_id = traci.gui.getTrackedVehicle("View #0")
        #     if tracked_id != "":
        #         ed = plexe.get_engine_data(tracked_id)
        #         vd = plexe.get_vehicle_data(tracked_id)
        #         setter(ed[RPM], ed[GEAR], vd.speed, vd.acceleration)
        #
        # step += 1


    traci.close()

if __name__ == "__main__":
    main(True, False)




