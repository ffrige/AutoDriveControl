import gym
import gym_car

from PIL import Image

from keras.models import load_model
from keras.models import Sequential, Model
from keras.layers import Dense, Activation, Flatten, Input, Conv2D, MaxPooling2D, Dropout, Concatenate, Cropping2D, Lambda
from keras.optimizers import Adam

from rl.agents import DDPGAgent
from rl.memory import SequentialMemory
from rl.random import OrnsteinUhlenbeckProcess
from rl.core import Processor

class ImageProcessor(Processor):
    def process_state_batch(self, batch):
        processed_batch = batch[0,:,:,:,:] #remove action from batch, so that actor can work correctly
        return processed_batch


# get the environment
ENV_NAME = 'Car-v0'
env = gym.make(ENV_NAME)

env.reset()

nb_actions = env.action_space.shape[0]

#load actor policy network
actor = load_model('model_car.h5')

#build critic value network
action_input = Input(shape=(nb_actions,), name='action_input')
observation_input = Input(shape=env.observation_space.shape, name='observation_input')

cropped_input = Cropping2D(cropping=((70,50), (0,0)))(observation_input)
normalized_input = Lambda(lambda x: x/255. - 0.5)(cropped_input)

x = Conv2D(32,(3,3), activation='relu')(normalized_input)
x = MaxPooling2D((2, 2))(x)
x = Dropout(0.5)(x)

x = Conv2D(64,(3,3), activation='relu')(x)
x = MaxPooling2D((2, 2))(x)
x = Dropout(0.5)(x)

x = Flatten()(x)
x = Concatenate()([x, action_input])
x = Dense(300)(x)
x = Activation('relu')(x)
x = Dense(1)(x)
x = Activation('linear')(x)
critic = Model(inputs=[action_input, observation_input], outputs=x)

#configure DDPG agent
memory = SequentialMemory(limit=100000, window_length=1)
processor = ImageProcessor()
random_process = OrnsteinUhlenbeckProcess(size=nb_actions, theta=.15, mu=0., sigma=.1)
agent = DDPGAgent(nb_actions=nb_actions, actor=actor, critic=critic, critic_action_input=action_input,
                  memory=memory, nb_steps_warmup_critic=1000, nb_steps_warmup_actor=1000,
                  random_process=random_process, gamma=.99, target_model_update=1e-3,processor=processor)
agent.compile([Adam(lr=1e-4), Adam(lr=1e-3)], metrics=['mae'])

#load old weights
#agent.load_weights('ddpg_{}_weights.h5f'.format(ENV_NAME))

#learn
agent.fit(env, nb_steps=10000, visualize=False, verbose=1, nb_max_episode_steps=100)

#save new weights
agent.save_weights('ddpg_{}_weights.h5f'.format(ENV_NAME), overwrite=True)

env.close() #close TCP socket
