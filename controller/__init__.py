from gym.envs.registration import register
register(
    id='Quadrotor-v0',
    entry_point='controller.Quadrotor:Quadrotor',
)