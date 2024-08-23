import os
import sys

from gym.wrappers.monitoring.video_recorder import VideoRecorder
from tqdm import tqdm

import mppi_constants
from algorithm.replay_buffer import goal_based_process
from common import get_args
from env_ext import make_env
from envs import register_custom_envs
from mpc import MPCDebugPlot
from policies import Policy, make_policy


class Player:
    policy: Policy = None
    debug_plot: MPCDebugPlot = None
    # debug_plot: MPCDebugPlotSmall = None
    mpc_policy = False

    def __init__(self, args):
        # initialize environment
        self.args = args
        self.env = make_env(args)
        self.info = []
        self.test_rollouts = 1
        self.mode = args.play_mode
        self.policy = make_policy(args)
        self.policy.set_envs(envs=[self.env])
        if args.play_policy in ['MPCRLPolicy', 'MPCPolicy']:
            self.mpc_policy = True
            if self.mode == 'plot':
                self.sim_length = args.timesteps
                self.debug_plot = MPCDebugPlot(args, sim_length=self.sim_length, model=self.policy.model)
                # self.debug_plot = MPCDebugPlotSmall(args, sim_length=self.sim_length, model=self.policy.model)

    def play(self):
        # play policy on env
        env = self.env
        acc_sum, obs = 0.0, []
        err_sum = 0

        for i in tqdm(range(self.test_rollouts)):
            self.policy.reset()
            ob = env.reset()
            obs.append(goal_based_process(ob))

            if args.play_policy in ['MPCPolicy']:
                self.policy.set_sub_goals([ob['desired_goal']])

            if self.debug_plot:
                info = self.policy.initial_info([ob])[0]
                self.debug_plot.createPlot(xinit=info['xinit'], pred_x=info['pred_x'],
                                           pred_u=info['pred_u'], k=0, parameters=info['parameters'], obs=ob)

            prev_ob = ob
            sim_timestep = 0
            for timestep in range(args.timesteps):
                # start = time.time()

                actions, infos = self.policy.predict(obs=[ob])
                action = actions[0]
                ob, _, _, env_info = env.step(action)

                # end = time.time()
                # print('Execution time {}: {} ({} Hz)'.format(timestep, end - start, 1 / (end - start)))

                if self.mpc_policy:
                    info = infos[0]
                    # sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n" \
                    #                  .format(info['info'].it, info['info'].solvetime))
                    mpc_info = info
                    next_x = mpc_info['next_x']
                    target_x = mpc_info['target_x']
                    pred_x = mpc_info['pred_x']
                    pred_u = mpc_info['pred_u']
                    parameters = mpc_info['parameters']

                    if self.debug_plot:
                        print('t:', sim_timestep)
                        self.debug_plot.updatePlots(next_x=next_x, target_x=target_x, pred_x=pred_x, pred_u=pred_u,
                                                    k=sim_timestep,
                                                    parameters=parameters, ob=prev_ob, new_ob=ob)
                    sim_timestep = sim_timestep + 1

                    # print('Reward: ', env_info['Rewards'])
                    # print('MPC flag: ', mpc_info['exitflag'])

                    if mpc_info['exitflag'] != 1:
                        print(f"""FORCESPRO error {mpc_info['exitflag']}""")
                        err_sum += 1

                    if ob['collision_check']:
                        sys.stderr.write("Collision on {}")
                        if self.debug_plot:
                            self.debug_plot.show()
                        else:
                            env.render()
                            pass

                    if env_info['Success']:
                        print('\n Success. Exiting. Time steps: ', timestep)
                        if self.debug_plot:
                            self.debug_plot.show()
                        else:
                            env.render()
                            pass
                        break

                    if self.debug_plot:
                        if timestep == args.timesteps - 1:
                            self.debug_plot.show()
                        else:
                            self.debug_plot.draw()
                            # input('check')
                    else:
                        env.render()
                        pass
                else:
                    # env.render()
                    if env_info['Success']:
                        print('Success. Exiting. Time steps: ', timestep)
                        break
                    prev_ob = ob
            print('Collisions: ', env.collisions)
            print('Errors: ', err_sum)

    def record_video(self, raw_path, test_rollouts):
        directory = os.path.dirname(raw_path)
        if not os.path.exists(directory):
            os.makedirs(directory)

        env = self.env
        test_col_tolerance = 0
        # play policy on env
        recorder = VideoRecorder(env.env.env, base_path=raw_path)
        recorder.frames_per_sec = 30
        acc_sum, obs = 0.0, []
        tol_acc_sum = 0.0
        for i in range(test_rollouts):
            print("Rollout {} / {}".format(i + 1, test_rollouts))
            self.policy.reset()
            ob = env.reset()
            env_info = None
            next_capture_time = 0.0
            for _ in tqdm(range(self.args.timesteps)):
                actions, infos = self.policy.predict(obs=[ob])
                action = actions[0]
                ob, _, _, env_info = env.step(action)

                time = env.sim.get_state().time
                if time >= next_capture_time:
                    recorder.capture_frame()
                    next_capture_time += 1.0 / recorder.frames_per_sec

                if env_info['Success']:
                    print('Success. Exiting.')
                    break
            acc_sum += env_info['Success']
            if env.collisions <= test_col_tolerance:
                tol_acc_sum += env_info['Success']
            print('Collisions: ', env.collisions)
        print('Success rate: {}'.format(acc_sum / test_rollouts))
        print('Success rate (no col): {}'.format(tol_acc_sum / test_rollouts))
        recorder.close()


if __name__ == "__main__":
    args = get_args()
    mppi_constants.update_k(args.mppi_k, args.device)

    register_custom_envs()
    player = Player(args)
    # player.play()
    player.record_video(raw_path="./videos/rollouts_{}_{}".format(args.env, args.play_policy),
                        test_rollouts=args.rollouts)
