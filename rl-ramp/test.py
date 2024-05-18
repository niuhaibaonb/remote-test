import math
import gym
from gym import spaces
from gym.utils.env_checker import check_env
from gym.wrappers import FlattenObservation
from join_platoon import *
from parameter import *
from plexe import POS_X, Plexe, SPEED
import numpy as np
import time
from GetAcceleration import *
from TruckDictionary import *




class SumoGymEnv(gym.Env):
    def __init__(self, sumo_cmd):
        super(SumoGymEnv, self).__init__()
        self.sumo_cmd = sumo_cmd
        self.sumo_config_file = "cfg/freeway.sumo.cfg"
        self.N_VEHICLES = N_VEHICLES
        self.feature = 4
        self.join_vec = 1
        self.end_pos = 2700
        self.veh_list = []
        self.i = 0
        self.reward_given = False
        self.reward_given1 = False
        self.reward_given2 = False
        self.observation_space = spaces.Box(low=-float('inf'), high=float('inf'), shape=((self.N_VEHICLES + 2), self.feature))
        self.reward_range = (-float('inf'), float('inf'))
        self.merge_position = None
        self.fake_merge_position = 2
        self.merge_speed = 24
        self.punish_change_action = -1.0
        self.reward_1 = 0
        self.endGap = 5
        self.x = 0
        self.J_speed = None
        self.Join1_speed = 12
        self.join_vehicle_pos = None
        self.Join1_vehicle_pos = 620
        self.join_vehicle_laneid = 0
        self.epso = 0
        self.merging_tail_position = self.Join1_vehicle_pos - 9
        self.follower_head_position = 800
        self.follower_speed = 30
        self.front_head_position = 800
        self.front_speed = 30
        self.front_acc = 0
        self.j_pos = 608
        self.collision = False
        self.prev_velocity = 0.0
        self.action_space = spaces.MultiDiscrete([7, 38])
        self.obser = np.zeros((10, self.feature))
        self.step_counter = 0
        self.state = GOING_TO_POSITION
        self.topology = {}
        self.off_list = {"v.0": 400, "v.1": 390, "v.2": 390, "v.3": 360, "v.4": 290, "v.5": 250, "v.6": 220, "v.7": 200}
        self.CAT_i = {"v.8": 300}
        self.current_distance = 0
        self.speed = 33

    def reset(self):
        try:
            traci.close()
        except traci.TraCIException:
            pass

        start_sumo(self.sumo_config_file, False)
        self.plexe = Plexe()

        self.reward_given = False
        self.reward_given1 = False
        self.reward_given2 = False
        self.step_counter = 0
        self.state = GOING_TO_POSITION
        random.seed(1)
        self.veh_list = []
        self.topology = {}
        self.epso += 1
        random.seed(time.time())

        if self.epso % 5 == 0:
            self.j_pos = random.randint(600, 610)

        If_add = random.randint(1, 2)
        pos1 = random.randint(80, 100)
        pos2 = random.randint(970, 980)

        if If_add == 1:
            traci.vehicle.add(vehID="Ann", routeID="normal_route", departPos=pos1, departSpeed=33, departLane=1, typeID="passenger")
        else:
            traci.vehicle.add(vehID="Ann", routeID="platoon_route", departPos=pos2, departSpeed=26, departLane=0, typeID="passenger")

        self.topology = add_vehicles(self.plexe, self.N_VEHICLES, True, Jposition=self.j_pos)

        obs = self._get_observation()
        return obs.astype(np.float32)

    def seed(self, seed):
        np.random.seed(seed)

    def _apply_action(self, action):
        action_valid = True
        traci.simulationStep()

        self.prev_velocity = self.merge_speed
        action_valid, self.reward_1 = self.safety_supervisor(action, speed_difference_threshold=5)

        if action_valid and self.step_counter % 10 == 0:
            self.merge_speed = action[1]

        self.safe_supervisor(action)

        if self.merge_position != 8:
            BEHIND_JOIN = f"v.{self.fake_merge_position}"
            v_follower = traci.vehicle.getSpeed(BEHIND_JOIN)

            self.state, self.topology = join_platoon(self.plexe, JOIN_POSITION=self.merge_position,
                                                     join_speed=self.merge_speed, J_ID=JOINER,
                                                     step=self.step_counter, state=self.state,
                                                     N_VEHICLES=self.N_VEHICLES, topology=self.topology,
                                                     is_merge=True, v_merge=self.Join1_speed,
                                                     v_follower=v_follower,
                                                     merging_tail_position=self.merging_tail_position,
                                                     follower_head_position=self.follower_head_position,
                                                     merging_head_position=self.Join1_vehicle_pos,
                                                     front_tail_position=self.front_head_position - 7)
        else:
            self.state, self.topology = join_final(self.plexe, JOIN_POSITION=self.merge_position, J_ID=JOINER,
                                                   step=self.step_counter, state=self.state, N_VEHICLES=self.N_VEHICLES,
                                                   topology=self.topology, join_speed=self.merge_speed,
                                                   v_merge=self.Join1_speed,
                                                   merging_head_position=self.Join1_vehicle_pos,
                                                   front_tail_position=self.front_head_position - 7)
        return self.state, self.topology

    def _get_observation(self):
        F_ID = f"v.{self.fake_merge_position - 1}"
        if self.fake_merge_position != 8:
            BEHIND_JOIN = f"v.{self.fake_merge_position}"

        all_vehicles = traci.vehicle.getIDList()
        selected_vehicles = [veh for veh in all_vehicles if veh.startswith('v.') and int(veh.split('.')[1]) in range(9)]
        self.veh_list = selected_vehicles

        for j, veh_id in enumerate(self.veh_list):
            self.plexe.get_crashed(veh_id)
            platoon_data = self.plexe.get_vehicle_data(veh_id)
            speed = traci.vehicle.getSpeed(veh_id)

            pos = platoon_data[POS_X]
            join_data = self.plexe.get_vehicle_data(J_ID)
            join_vehicle_pos = join_data[POS_X]

            if self.fake_merge_position != 8:
                BEHIND_JOIN_data = self.plexe.get_vehicle_data(BEHIND_JOIN)
                self.follower_head_position = BEHIND_JOIN_data[POS_X]
                self.follower_speed = BEHIND_JOIN_data[SPEED]

            FRONT_JOIN_data = self.plexe.get_vehicle_data(F_ID)
            self.front_head_position = FRONT_JOIN_data[POS_X]
            self.front_speed = FRONT_JOIN_data[SPEED]
            self.front_acc = FRONT_JOIN_data[ACCELERATION]
            self.merging_tail_position = join_vehicle_pos - 9
            endGap = self.end_pos - join_vehicle_pos
            J_speed = join_data[SPEED]
            J_acceleration = join_data[ACCELERATION]
            acceleration = platoon_data[ACCELERATION]
            self.endGap = endGap
            self.J_speed = J_speed
            self.Join1_speed = J_speed
            self.join_vehicle_pos = join_vehicle_pos
            self.x = join_vehicle_pos - 2400
            self.Join1_vehicle_pos = join_vehicle_pos
            self.obser[j] = [speed / 10, acceleration, pos / 1000, 0]
            leader_info = traci.vehicle.getLeader("v.0", 500)
            leader_hv_id = leader_info[0]
            leader_hv_acceleration = traci.vehicle.getAcceleration(leader_hv_id)
            leader_hv_speed = traci.vehicle.getSpeed(leader_hv_id)
            position1 = traci.vehicle.getPosition(leader_hv_id)
            leader_hv_pos = position1[0]
            self.obser[8] = [J_speed / 10, J_acceleration, join_vehicle_pos / 1000, endGap]
            self.obser[9] = [leader_hv_speed / 10, leader_hv_acceleration, leader_hv_pos / 1000, 0]
            self.current_distance = get_distance(self.plexe, J_ID, F_ID)

        return self.obser

    def step(self, action):
        if self.merge_position is None or self.merge_position == 8:
            self.merge_position = self.fake_merge_position

        self._apply_action(action)
        obs = self._get_observation()

        reward = self.compute_reward(obs, action)
        done = self._is_done()
        info = {}
        self.step_counter += 1

        if self._is_collision():
            self.collision = True
            done = True

        return obs.astype(np.float32), reward, done, info

    def render(self, mode='human'):
        pass

    def close(self):
        traci.close()

    def _done(self):

        if self.step_counter >= 2000:
            return True
        else:
            return False

    def slow_down_at_step(self,car_id, step,slow_acceleration, duration, original_speed=30):

        if step == 200:
            # 降速到20
            # traci.vehicle.setSpeed(car_id, slow_speed)
            traci.vehicle.setAcceleration(car_id,slow_acceleration,duration)
        # elif step == 200 + (duration * traci.simulation.getDeltaT()):
        elif step == 800:

            if original_speed is not None:
                # traci.vehicle.setSpeed(car_id, original_speed)
                traci.vehicle.slowDown(car_id,28,duration=4)
            else:
                # 如果没有提供原始速度，就取消速度限制，让车辆恢复其默认行为
                traci.vehicle.setSpeed(car_id, -1)

    def gaussian_function(self, x):
        return -np.exp(-((x - 350) ** 2) / (10 * 350))

    def merge_pos(self,x,M_erge):
        return -0.1* np.exp(-((x / 2))*(x-M_erge))


    def _get_reward(self, action):
        reward = 0
        L = 350
        # 创建实例并调用函数
        truck_dict = TruckDictionary()
        v8_sort=truck_dict.insert_and_sort()


        if self.join_vehicle_laneid != "E1_1":  # 合并未完成
            if self.endGap < 2 and self.reward_given2 == False:  # 未合并完成且末端距离小于10
                reward -= 5
                self.reward_given2 = True
            reward -= 1  # 促使尽快合

        a = self.gaussian_function(self.x)
        reward += a
        reward +=self.merge_pos(self,v8_sort)

        if self.join_vehicle_laneid == "E1_1" and self.collision == False and self.state == COMPLETED:
            if not self.reward_given:  # 检查是否已经给予了奖励
                reward += 1000
                self.reward_given = True  # 更新标志变量# 2
        if self.collision:  # 发生碰撞
            if not self.reward_given1:  # 检查是否已经给予了奖励
                reward -= 100
                self.reward_given1 = True  # 2


        if self.step_counter > 300:

            if self.Join1_vehicle_pos >= self.front_head_position - 17:
                reward_diff = 0.5*self.calculate_speed_difference_reward(action[1],self.follower_speed)
                reward = reward + reward_diff


        accelerations = self.get_vehicle_accelerations()  # jerk
        reward = reward - (self.get_average_acceleration_reward(accelerations)) * 0.8

        return reward
    def calculate_speed_difference_reward(self,vehicle_speed, rear_vehicle_speed):

        # 目标速度范围的中值
        target_speed = rear_vehicle_speed

        # 计算当前速度与目标速度的差异
        speed_difference = abs(vehicle_speed - target_speed)

        # 计算奖励，这里使用负的平方差作为奖励，速度差异越小，奖励越大
        # 注意：这里的 -1 是为了确保差异越小奖励越大，你可能需要根据实际情况调整系数
        reward = -speed_difference ** 2

        return reward
    def check_collision(self):
        all_vehicles = traci.vehicle.getIDList()
        veh_list = [veh for veh in all_vehicles if veh.startswith('v.') and int(veh.split('.')[1]) in range(9)]
        for veh_id in veh_list:
            if self.plexe.get_crashed(veh_id):
                return True
        return False

    def speed_reward(self,action):
        if abs(action[1]-self.follower_speed)<5:
            return 2

        else:
            return 0

    def check_lanechange(self):
        lane_change_state = traci.vehicle.getLaneChangeState("v.8", 0)
        if lane_change_state == traci.constants.LANECHANGE_TAKEN:
            return True
        return False

    def _decode_action(self, action):
        if action[0] == 0:  # 动作为1表示不合并
            return None
        elif 2 <= action[0] <= 4:  # 动作解码为表示对应的合并位置
            merge_position = action[0]
            return merge_position

    def get_vehicle_accelerations(self):
        """
        获取车辆的加速度列表
        :param vehicle_ids: 车辆ID列表
        :return: 车辆的加速度列表
        """
        all_vehicles = traci.vehicle.getIDList()
        veh_list = [veh for veh in all_vehicles if veh.startswith('v.') and int(veh.split('.')[1]) in range(9)]


        accelerations = []
        for vehicle_id in veh_list:
            acceleration = traci.vehicle.getAcceleration(vehicle_id)  # 获取车辆加速度的具体实现
            accelerations.append(acceleration)
        return accelerations

    def speed_diff_reward(self, merge_vehicle_speed, front_vehicle_speed, behind_vehicle_speed, merge_distance):
        # 定义β值
        BETA = 0.2

        # 直接计算速度差异
        speed_diff_front = abs(merge_vehicle_speed - front_vehicle_speed)
        speed_diff_behind = abs(merge_vehicle_speed - behind_vehicle_speed)

        # 使用指数衰减函数调整速度差异
        decay_factor = math.exp(-BETA * merge_distance)
        adjusted_diff_front = decay_factor * speed_diff_front
        adjusted_diff_behind = decay_factor * speed_diff_behind

        # 惩罚是速度差异的总和，所以需要用负号
        total_penalty = -(adjusted_diff_front + adjusted_diff_behind)/2  # 奖励为负值，表示惩罚

        return total_penalty

    def get_average_acceleration_reward(self, accelerations):
        """
        计算相邻时间步的加速度差值并求和取平均值作为奖励

        """
        acceleration_diffs = [abs(accelerations[i] - accelerations[i - 1]) for i in range(1, len(accelerations))]
        average_reward = sum(acceleration_diffs) / len(acceleration_diffs)
        return average_reward




# 创建环境实例
num_steps = 2000
env = SumoGymEnv("sumo")
env = FlattenObservation(env)
check_env(env)
# print(torch.cuda.is_available())

# model = RecurrentPPO("MlpLstmPolicy", env, verbose=1, learning_rate=2e-4, policy_kwargs= dict(lstm_hidden_size=128,enable_critic_lstm=True,net_arch=dict(pi=[11,128,128,128,2], vf=[11,128,128,128,1])),
#                      batch_size=256, gamma=0.96,
#                      , device="cuda")
