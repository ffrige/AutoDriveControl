import gym
import gym_car

from keras.models import load_model
from keras.models import Sequential, Model
from keras.layers import Dense, Activation, Flatten, Input, Concatenate
from keras.optimizers import Adam

from rl.agents import DDPGAgent
from rl.memory import SequentialMemory
from rl.random import OrnsteinUhlenbeckProcess


# get the environment
ENV_NAME = 'Car-v0'
env = gym.make(ENV_NAME)

env.reset()

nb_actions = env.action_space.shape[0]

#load actor policy network
actor = load_model('model_car.h5')

#build critic value network
action_input = Input(shape=(nb_actions,), name='action_input')
observation_input = Input(shape=(1,) + env.observation_space.shape, name='observation_input')
flattened_observation = Flatten()(observation_input)
x = Dense(400)(flattened_observation)
x = Activation('relu')(x)
x = Concatenate()([x, action_input])
x = Dense(300)(x)
x = Activation('relu')(x)
x = Dense(1)(x)
x = Activation('linear')(x)
critic = Model(inputs=[action_input, observation_input], outputs=x)

#configure DDPG agent
memory = SequentialMemory(limit=100000, window_length=1)
random_process = OrnsteinUhlenbeckProcess(size=nb_actions, theta=.15, mu=0., sigma=.1)
agent = DDPGAgent(nb_actions=nb_actions, actor=actor, critic=critic, critic_action_input=action_input,
                  memory=memory, nb_steps_warmup_critic=1000, nb_steps_warmup_actor=1000,
                  random_process=random_process, gamma=.99, target_model_update=1e-3)
agent.compile([Adam(lr=1e-4), Adam(lr=1e-3)], metrics=['mae'])

#load old weights
#agent.load_weights('ddpg_{}_weights.h5f'.format(ENV_NAME))

#learn
agent.fit(env, nb_steps=1000, visualize=False, verbose=1, nb_max_episode_steps=30)

#save new weights
agent.save_weights('ddpg_{}_weights.h5f'.format(ENV_NAME), overwrite=True)

env.close() #close TCP socket
