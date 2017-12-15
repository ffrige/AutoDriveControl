from gym.envs.registration import register

register(
    id='Car-v0',
    entry_point='gym_car.envs:CarEnv',
)
