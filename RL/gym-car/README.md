# gym-car

The car environment can be used to run reinforcement learning algorithms on the Udacity Self-Driving Car Simulator.

The action is a steering value.

The observation is a tuple of (steering_angle, throttle, speed, image).

The reward is -log(1-|cte|), where cte is the cross-track-error. A high cte causes a large "negative reward" (i.e. penalty), while a low cte causes a small penalty. No positive rewards are given.

# Installation

```
cd gym-car
pip install -e .
```
