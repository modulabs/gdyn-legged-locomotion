import gym
import numpy as np
from EnvOpenDogStand import EnvOpenDogStand
import time

env = EnvOpenDogStand()
env.seed(0)

episode_count = 100
reward = 0

for i in range(episode_count):
    print("episode : ")
    print(i)
    ob = env.reset()
    while True:
        #Random continuous action
        action = np.random.randn(12)*0.01
        ob, reward, done, _ = env.step(action)
        #We sleep for the timestep duration
        time.sleep(env.timeStep)
        if done:
            print( reward )
            break

env.close()
