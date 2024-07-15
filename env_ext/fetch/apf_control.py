import numpy as np

from .vanilla import VanillaGoalEnv


class APFControlGoalEnv(VanillaGoalEnv):
    total_reward = 0
    collisions = 0
    obj_distance_threshold = 0.04

    def __init__(self, args):
        VanillaGoalEnv.__init__(self, args)

    def get_obs(self):
        obs = super().get_obs()
        obs = self.extend_obs(obs, self.prev_obs)
        return obs

    def disable_action_limit(self):
        self.env.env.env.limit_action = 1.0

    @property
    def time(self):
        return self.sim.get_state().time

    def step(self, action):
        # Extend the observation with a collision information
        prev_obs = self.prev_obs
        obs, reward, done, info = super().step(action)
        obs = self.extend_obs(obs, prev_obs)
        reward = self._extend_reward(reward, obs)
        info = self._extend_info(info)
        return obs, reward, done, info

    def _extend_info(self, info):
        new_info = info.copy()
        new_info['Collisions'] = self.collisions
        new_info['ExReward'] = self.total_reward
        return new_info

    def _extend_reward(self, reward, obs):
        new_reward = reward
        if obs['collision_check']:
            new_reward = -4.0
        if obs['object_dis'] > self.obj_distance_threshold:
            new_reward = -10.0  # object is no more in the grip

        self.total_reward += new_reward

        return new_reward

    def extend_obs(self, obs, prev_obs):
        sim = self.sim
        exists_collision = False
        # object_id = self.sim_env.geom_id_object
        for i in range(sim.data.ncon):
            contact = sim.data.contact[i]

            for obstacle_id in self.sim_env.geom_ids_obstacles:
                if (contact.geom1 == obstacle_id) or (contact.geom2 == obstacle_id):
                    # if contact.geom1 != 16 and contact.geom2 != 16:
                    exists_collision = True
                    break
        obs['collision_check'] = exists_collision
        if exists_collision:
            self.collisions += 1

        # calculate the velocities of the objects
        if self.sim_env.env.n_obstacles:
            real_obstacle_info = obs['real_obstacle_info']
            real_obstacle_info_pos = real_obstacle_info[:, 0:3]
            prev_real_obstacle_info_pos = prev_obs['real_obstacle_info'][:, 0:3]
            dt = self.dt
            obj_vels = (real_obstacle_info_pos - prev_real_obstacle_info_pos) / dt
            obs['obj_vels'] = obj_vels
            ob = obs['observation'].copy()

            # extend the observation with obstacle positions and velocities
            obs['observation'] = np.concatenate([ob, real_obstacle_info.ravel(), obj_vels.ravel()])

        return obs

    def reset(self):
        self.collisions = 0
        self.total_reward = 0
        obs = super().reset()
        obs = self.extend_obs(obs, self.prev_obs)
        return obs

    @property
    def dt(self):
        return self.env.env.dt