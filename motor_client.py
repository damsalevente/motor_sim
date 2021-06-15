import socket
import os
import sys
import gym
import numpy as np
import torch
from gym import spaces
from stable_baselines3 import PPO, SAC, TD3
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.results_plotter import load_results, ts2xy, plot_results
from stable_baselines3.common.callbacks import BaseCallback

class SaveOnBestTrainingRewardCallback(BaseCallback):
    """
    Callback for saving a model (the check is done every ``check_freq`` steps)
    based on the training reward (in practice, we recommend using ``EvalCallback``).

    :param check_freq: (int)
    :param log_dir: (str) Path to the folder where the model will be saved.
      It must contains the file created by the ``Monitor`` wrapper.
    :param verbose: (int)
    """
    def __init__(self, check_freq: int, log_dir: str,extra_stuff='', verbose=1):
        super(SaveOnBestTrainingRewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, extra_stuff)
        self.best_mean_reward = -np.inf

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:

          # Retrieve training reward
          x, y = ts2xy(load_results(self.log_dir), 'timesteps')
          if len(x) > 0:
              # Mean training reward over the last 100 episodes
              mean_reward = np.mean(y[-100:])
              if self.verbose > 0:
                print("Num timesteps: {}".format(self.num_timesteps))
                print("Best mean reward: {:.2f} - Last mean reward per episode: {:.2f}".format(self.best_mean_reward, mean_reward))

              # New best model, you could save the agent here
              if mean_reward > self.best_mean_reward:
                  self.best_mean_reward = mean_reward
                  # Example for saving best model
                  if self.verbose > 0:
                    print("Saving new best model to {}".format(self.save_path))
                  self.model.save(self.save_path)

        return True

class Motor:
    def __init__(self, s):
        self.ud = 0  # idk
        self.uq = 0
        self.id = 0
        self.iq = 0
        self.wr = 0
        self.pos = 0
        self.s = s  # socket
        self.UMAX = 350
        self.IMAX = 1000
        self.WMAX = 150
        self.THETAMAX = 360

    def obs(self):
        # todo: work with bytes no need to convert
        line = self.s.recv(256).decode()
        line = line.split(',')
        if len(line) > 6:
            self.ud = float(line[3]) / self.UMAX # time is not needed
            self.uq = float(line[4]) / self.UMAX
            self.id = float(line[1]) / self.IMAX
            self.iq = float(line[2]) / self.IMAX
            self.wr = float(line[5]) / self.WMAX
            self.pos = float(line[6]) / self.THETAMAX

    def reset(self):
        self.s.sendall(b'X')

    def close(self):
        self.s.sendall(b'close')

    def control(self, ud, uq):
        self.ud = ud
        self.uq = uq

    def log(self):
        print('--------------\ncurrents> id=>{} A | iq=>{} A\nvoltages> ud=> {} V | uq=>{} V\nmotor> w_r => {}rad/sec | theta => {}'.format(self.id, self.iq, self.ud, self.uq, self.wr, self.pos))

    def send_ctrl(self):
        pkg = '{},{}'.format(self.ud, self.uq)
        self.s.sendall(bytes(pkg, 'utf-8'))

class MotorEnv(gym.Env):
  """Custom Environment that follows gym interface"""
  metadata = {'render.modes': ['human']}

  def __init__(self, s):
    super(MotorEnv, self).__init__()
    self.motor = Motor(s)
    self.ref_speed = 30 / self.motor.WMAX
    self.goodone = 0
    self.counter = 0
    # 0: ud, 1: uq, both has the same range, -350, 350 
    self.action_space = spaces.Box(np.array([-1,-1]), np.array([1,1]))
    # ud, uq, id, iq, wr, position, ref speed
    self.observation_space = spaces.Box(np.array([-1,-1,-1,-1,-1,0,-1]),np.array([1,1,1,1,1,1,1])) 


  def step(self, action):
    # Execute one time step within the environment
    self.motor.control(action[0] * 350,action[1] * 350)
    self.motor.send_ctrl()
    self.motor.obs()
    self.counter += 1
    state = np.array([self.motor.ud, self.motor.uq, self.motor.id, self.motor.iq, self.motor.wr, self.motor.pos, self.ref_speed])
    reward, done, info  = self.calc_reward()
    if self.counter > 20:
        done = True
    return state, reward, done, info

  def calc_reward(self):
    err = 1 - (np.abs(self.motor.wr - self.ref_speed)) 
    done = False
    succ = False
    if err > 0.98:
        self.goodone += 1
        
    if self.counter > 20:
        if self.goodone > 15:
            err += 5
        if self.goodone > 19:
            succ = True
            err += 5
    return err, done, {'success':succ}
    

  def reset(self):
    # Reset the state of the environment to an initial state
    self.motor.reset()
    self.counter = 0
    self.goodone = 0
    self.motor.obs()
    state = np.array([self.motor.ud, self.motor.uq, self.motor.id, self.motor.iq, self.motor.wr, self.motor.pos, self.ref_speed])
    return state

  def render(self, mode='human', close=False):
    # Render the environment to the screen
    self.motor.log()

  def close(self):
    self.motor.close()



# setting up webserver 
host = '127.0.0.1'
port = 9099

ud = 5
uq = 5

s = None

callback = SaveOnBestTrainingRewardCallback(
            check_freq=4096, log_dir='./logs/', extra_stuff='_check')

for res in socket.getaddrinfo(host, port, socket.AF_UNSPEC, socket.SOCK_STREAM):
    af, socktype, proto, canonname, sa = res

    try:
        s = socket.socket(af, socktype, proto)
    except OSError as msg:
        s = None
        continue
    try:
        s.connect(sa)
    except OSError as msg:
        s.close()
        s = None
        continue
    break

if s is None:
    print("could not open sokcet")
    sys.exit(1)
train = False
with s:
    env = MotorEnv(s)  # create motor with socket
    env = Monitor(env, './logs/')
    obs = env.reset()
    if train:
        policy_kwargs = dict(activation_fn=torch.nn.Tanh,
                net_arch=[dict(pi=[32,32, 16], vf=[32,32, 16])])
        model = PPO("MlpPolicy", env,learning_rate= 3e-3,policy_kwargs = policy_kwargs, tensorboard_log='./logs', verbose = 2)
        model.learn(total_timesteps=1e6, callback = [callback])
    else:
        done = False
        model = PPO.load('./logs/_check.zip')
        while not done:
            action = model.predict(obs)
            obs, reward, done, info = env.step(action[0])
            env.render()
    env.close()
