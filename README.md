# Autonomous drive control methods
Standard control vs. machine learning

---

Collections of algorithms to solve the autonomous drive control problem in the Udacity car simulator. The goal is to build a controller that is able to safely drive the car around the track, i.e. without going offroad.

We implemented and present here four different approaches, two from classical control theory and two from a machine learning approach.

The simulator can be downloaded here: https://github.com/udacity/self-driving-car-sim/releases

## PID (Proportional, Integral, Derivative controller)

The observed variables are cross-track error, current steering angle and current speed.

The controlled variables are steering value and throttle value.

We used a three-level cascaded PID controller for steering, and a single PID for throttle. Integral action has anti-windup limits; we use gain scheduling for the proportional action to recover quickly from extreme cross-track errors; a dead zone is used in throttle control to avoid slowing down too often; all output actions are clamped to the physical limits of the actuators.

PID parameters were optimized for lap speed at the cost of driving smoothness. We reach a maximum of about 65mph on the longest straight and we go down to about 30mph in the turns.

## MPC (Model Predictive Controller)

The observed variables are current position and orientation of the car along the track.

The controlled variables are steering value and throttle value.

The target path is given in terms of waypoints along the middle of the track. We use a polynomial fit to generate a continuous function along them.

The optimal actions are obtained by solving a non-linear optimization problem based on the kinematic model of the vehicle.

The cost function to be minimized by the optimizer is built-up adding several parameters: error in orientation (responsible for the highest cost to keep the car mostly parallel to the track); cross-track error (to avoid running off-track); magnitude of steering angle plus first and second derivatives (to avoid large or sudden turning and guarantee a smoother driving experience); difference from maximum speed (to avoid moving too slow or even stopping).

The car behaves much better than the drive with PID control: it follows a smoother trajectory and stays closer to the middle of the track. We also reach much higher speeds: above 95mph on the straight and down to 50mph on the curves.

## SL (Supervised Learning)

The controller is a neural network with two convolutional layers at the bottom and two fully connected layers at the top, capped by a softmax layer for classification.

All layers include ReLU non-linear activation functions. Convolutions are all done with (3x3) kernel size, the first layer with 32 filters and the second with 64. Pooling layers are added to downscale by a factor of 2. Each convolutional layer also has a 50% dropout to reduce overfitting.

Input images are scaled to a 320x40 pixels size and flattened to one gray channel. We filter the intensity gradients according to their magnitude and direction in order to highlight the lateral road boundaries.

For simplicity we only use three output classes (turn left/right, go straight).

The learning rate is adapted by an Adam optimizer. We trained for only five epochs to avoid overfitting, given the low size of the training data, and achieved 93% accuracy on the training set and 78% on the validation set.

The only controlled variable is steering value. Speed is kept constant at 30mph.

## RL (Reinforcement Learning)

TODO