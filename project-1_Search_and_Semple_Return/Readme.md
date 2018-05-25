# A project similar to NASA’s planetary rover challenge.

## Writeup
### Simulator environment settings
- Resolution: 1024x768
- Graphics quality: good
- Windowed: true
- FPS: ~18 avg

### Notebook Analysis
Function `obstacle_thresh` is used for obstacles detection. It basically does `logical not` operation on warped terrain image.

To effectively detect rock additional investigation was done to find out threshold values for different color channels. It leads to the next threshold values:
- Red channel: 130-150 
- Geen channel: 110-130

Red and green channels should be enough to detect rocks, therefore there is no reason to check green channel.

Function `rock_thresh` is used for rocks detection. For input image it creates a logical mask of rock by comparing every pixel of the image with boundaries defined above.

Function `process_image` puts all the image processing steps together:
- Defines source and destination points (picked manually) for perspective transform
- Applies perspective transform
- Applies color threshold to identify navigable terrain/obstacles/rock samples
- Converts thresholded image pixel values to rover-centric coordinates
- Converts rover-centric pixel values to world coordinates
- Updates the worldmap
- Makes a mosaic image

### Autonomous Navigation and Mapping
`perception_step()` function in `perception.py` script was updated. Step-by-step explanation:
1. Manually defined source the boundaries and calculated destination boundaries for perspective transform to switch from actual to desirable coordinates.
2. Applied perspective transformation.
3. Applied color threshold to identify navigable terrain/obstacles/rock samples. To identify obstacles and rocks two new functions were defined: `obstacle_thresh()` and `rock_thresh()` correspondingly (explained above).
4. Undated "rover vision image", which is present on left side of screen. Masks are multiplied by 255 as original values are binary.
5. Converted all images to rover-centric coordinates.
6. Rover-centric coordinates in turn converted to world coordinates.
7. Rover worldmap updated using world coordinates.
8. Finally rover-centric coordinates converted to polar coordinates for navigation purposes.

`decision_step()` function in `decision.py`
The function consists of 5 main parts:
1. Updating historical data (last 100 positions)
2. Stuck and looping checks
3. Strategy for 'forward' and 'stop' modes
4. Pick up condition
5. Returning result `Rover` object

To be able to detect stuck state some historical data required which we can compare with current rover position. For this purpose two new variables of deque type were added to `Rover` object: `last_100_x_positions` and `last_100_y_positions`.
Here `Rover.steer` variable updated with it's previous value taken from another deque `last_100_steer_positions`. For some reason value of `Rover.steer` could be rewritten with data from telemetry, which sometimes simply return 0 instead of actual value.
History of steering is needed to be able to detect simple loops of circular motion.

Now it is possible check if rover stuck in one place or in a loop. It is not needed to do such checks at every step. To determine when check must be made two new variables added: `Rover.check_stuck_time` and `Rover.check_loop_time`. They are re-initialized with random values to add some non-determinism to avoid more complex loops.
If rover stuck decision about steering will be made immediately. `Rover.looping` and `Rover.turning` variables added to be able to track state when rover escaping a loop and stuck situation correspondingly. In this state the same steering action will be repeated until `Rover.steering_repeat_counter` value gets to 0. This variable will be re-initialized with some random value from the specified range.

Next strategy for more 'normal' cases may be applied.
If rover gets close to rock, mode will be switched to 'stop'.
If rover is in 'forward' mode and there are no enough navigable terrain in front of it, mode will be switched to 'stop'. In opposite case (or if rock is detected and rover needs to continue moving to it) throttle and steering variables will be updated with new values.
If rover is in 'stop' mode and has high velocity, it must to continue braking.
If velocity is low enough there are a few cases:
- if the nearest path is blocked or the rover stuck (except when getting close to rock and need to finish maneuver), then switch to turning mode;
- if not enough space to move forward and not steering already, then turn around (assign new steer value);
- otherwise move forward.

Finally if rover is near sample, velocity is 0 and rover is not in picking up state, send pickup command and return `Rover` object with new state calculated during decision step.

## Further improvements
There are many ways to improve current decision approach.

Main problems of current approach:
* it exploits simulated environment: some rules are designed to deal with environment's characteristics
* rover still tend to stuck in a complex loop in some areas for a long time
* some areas left unvisited for a long time (or even forever)
* sometimes rules conflict with each other
* it is hard to track how rover state is changing with time

Quick improvements:
* preserve turning direction: currently direction chosen randomly
* another stuck escape strategy: move one direction gradually decreasing angles until escape
* better distinguishing behaviour for turning in case of stuck and in case of dead end
* increase steering history deque (e.g. to store 1000 last values)

Strategy change:
* as suggested in the lesson, one of possible strategies is to move close to walls, though it may bring a bunch of new issues to solve

Design changes:
* involve geometry to track complex loops (analyze shapes instead of pixels)
* make rover aware about its direction and environment
* increase randomness, especially with time; this will bring some noise to applied decisions forcing non-standard behaviour (increasing behavioral entropy), need to be careful though
* introduce and visualise model of finite-state machine to ease modelling process

Completely different approaches:
* fuzzy logic
* generative algorithms
* reinforcement deep neural network