import gym
import numpy as np
from EnvOpenDogStand import EnvOpenDogStand
from EnvOpenDogForward import EnvOpenDogForward
from EnvOpenDogForwardSimple import EnvOpenDogForwardSimple
import time
from es import CMAES
import tensorflow as tf
import gc
import pickle
from tensorforce.agents import PPOAgent


render = True
env = EnvOpenDogForwardSimple(renders=render)
ENV_NAME = "ForwardPPOSimple"

env.seed(0)




import os
os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'


class ForwardActor:
    def __init__(self):

        actions = {}
        for i in range(12):
            actions[str(i)] = {'type': 'float'}# 'num_actions': 10

        network_spec = [
            dict(type='dense', size=100, activation='relu'),
            dict(type='dense', size=100, activation='relu')
        ]

        self.agent = PPOAgent(
            states=dict(type='float', shape=(12,)),
            actions=actions,
            batching_capacity=2000,
            network=network_spec,
            step_optimizer=dict(
                type='adam',
                learning_rate=1e-4
            ),
        )


    def act(self, state):
        jp = np.expand_dims( np.nan_to_num( np.array(state["JointPosition"] ) ),axis=0)
        jv = np.expand_dims(np.array(state["JointVelocity"]), axis=0)

        #actiondict = self.agent.act( np.concatenate([jp,jv],axis=1))
        actiondict = self.agent.act(jp)

        action = np.zeros(12)
        for i in range(12):
            action[i] = actiondict[str(i)][0]
        action = np.nan_to_num(action)
        #print(action)
        return np.clip( action,-1.0,1.0)

    def observe(self, reward, terminal):
        self.agent.observe(reward=reward,terminal=terminal)

    def save(self,directory):
        self.agent.save_model(directory=directory)

    def restore(self,directory):
        self.agent.restore_model(directory=directory)


from tensorforce.core.networks.network import Network

class CustomNetwork(Network):
    def tf_apply(self, x, internals, update, return_internals=False):
        #print(x)
        #input()
        x = next(iter(x.values()))
        layerSize = 100
        nbreslayers = 3

        x = tf.layers.Dense(layerSize)(x)

        out = x
        for i in range(nbreslayers):
            h = tf.nn.selu( tf.layers.Dense(layerSize)(out) )
            h = tf.nn.selu( tf.layers.Dense(layerSize)(h) )
            out = x+h

        out = tf.layers.Dense( layerSize )(h)

        if return_internals:
            return out, dict()
        else:
            return out

class ForwardActorSimple:
    def __init__(self):

        actions = {}
        actions_exp = {}
        for i in range(12):
            actions[str(i)] = {'type': 'float'}# 'num_actions': 10
            actions_exp[str(i)] = dict(
            type='ornstein_uhlenbeck',
            sigma=0.1,
            mu=0.0,
            theta=0.1
)

        preprocessing_config = [ {
                "type": "standardize"
            }
        ]

        preprocessing_config = None

        customnet = dict(type=CustomNetwork)
        layerSize = 300
        network_spec = [
            dict(type='dense', size=100),
            dict(type='lstm',size=100)
        ]
        '''
        network_spec = [
                            dict(type='dense', size=100),
                           dict(type='internal_lstm', size=100)
                       ]
       
        '''

        network_spec = [
            dict(type='dense', size=layerSize, activation='selu'),
            dict(type='dense', size=layerSize, activation='selu'),
            dict(type='dense', size=layerSize, activation='selu')
        ]


        self.agent = PPOAgent(
            states=dict(type='float', shape=(12+9,)),
            actions=actions,
            batching_capacity=1000,
            network=network_spec,
            states_preprocessing=preprocessing_config,
            actions_exploration=actions_exp,
            step_optimizer=dict(
                type='adam',
                learning_rate=1e-5
            ),

        )


    def act(self, state):
        jp = np.expand_dims( np.nan_to_num( np.array(state["JointPosition"] ) ),axis=0)
        #jv = np.expand_dims(np.array(state["JointVelocity"]), axis=0)
        orient = np.expand_dims( np.array(state["bodyRot"]), axis=0 )
        actiondict = self.agent.act( np.nan_to_num( np.concatenate([jp,orient],axis=1) ) / 5.0 )
        #actiondict = self.agent.act(jp)

        action = np.zeros(12)
        for i in range(12):
            action[i] = actiondict[str(i)][0]
        action = np.nan_to_num(action)
        #print(action)
        return np.clip(action,-1.0,1.0)

    def observe(self, reward, terminal):
        self.agent.observe(reward=reward,terminal=terminal)

    def save(self,directory):
        self.agent.save_model(directory=directory)

    def restore(self,directory):
        self.agent.restore_model(directory=directory)



agent = ForwardActorSimple()



def runEpisode( ):
    ob = env.reset()
    totreward = 0
    while True:
        action = agent.act(ob)
        # Add experience, agent automatically updates model according to batch size

        ob, reward, done, _ = env.step(action)
        totreward = totreward +reward
        agent.observe(reward=np.nan_to_num(reward), terminal=done)
        time.sleep(0.025)
        if done:
            #print("episode done")
            #print(reward)
            gc.collect()
            # Only return last reward
            return totreward

MY_REQUIRED_REWARD = 300.0

try:
    agent.restore("./"+ENV_NAME+"/")
except:
    print("No file found starting from scratch")

bestScore = - np.Infinity
ep = 0
while True:
  # ask the ES to give us a set of candidate solutions
  if( ep % 100 == 0) :
      agent.save("./"+ENV_NAME+"/")
  score = runEpisode()
  if( score > bestScore):
      bestScore = score
  # get best parameter, reward from ES

  print("Episode " + str(ep))
  print( "score : ")
  print( score )
  print("current Best : ")
  print( bestScore)
  ep = ep+1



# Close the env and write monitor result info to disk
env.close()