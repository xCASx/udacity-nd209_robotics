import numpy as np
import math
import random


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # First of all update position history
    Rover.last_100_x_positions.append(math.modf(Rover.pos[0])[1])
    Rover.last_100_y_positions.append(math.modf(Rover.pos[1])[1])

    # Do not allow external changes to steer value (data object from telemetry overrides Rover.steer for some reason)
    Rover.steer = get_previous_steering_value(Rover)

    # Check if stuck
    if Rover.total_time > Rover.check_stuck_time:
        # Randomly set time of next check. Here and below randomness helps us to get away of more complex loops
        Rover.check_stuck_time = Rover.total_time + np.random.randint(5, 10)
        if Rover.mode == 'forward' and not Rover.stuck:
            if not are_coordinates_changed(Rover):
                Rover.stuck = True
                Rover.looping = False
                stop_rover(Rover)
                return track_steering(Rover)

    # Check if stuck in a simple round-moving loop
    if Rover.total_time > Rover.check_loop_time:
        Rover.check_loop_time = Rover.total_time + np.random.randint(15, 30)
        if Rover.mode == 'forward' and not Rover.stuck and not Rover.looping:
            if is_looping(Rover):
                # We are moving in a loop, change steering direction
                Rover.looping = True
                Rover.steer = -Rover.steer
                return track_steering(Rover)

    # Hold opposite steering with some delay to escape the loop
    if Rover.looping:
        Rover.steering_repeat_counter -= 1
        if Rover.steering_repeat_counter > 0 and not is_rock_detected(Rover):
            # continue steering, no state change required
            return track_steering(Rover)
        else:
            Rover.steering_repeat_counter = np.random.randint(15, 75)
            Rover.looping = False

    # Continue turning with some delay to find a way out
    # It is not enough to check amount of pixels of navigable terrain to switch to move forward
    # as rover may have plenty of them, but still be stuck near obstacle wall
    if Rover.turning:
        Rover.steering_repeat_counter -= 1
        if Rover.steering_repeat_counter > 0 and not is_rock_detected(Rover):
            return track_steering(Rover)
        else:
            Rover.steering_repeat_counter = np.random.randint(15, 75)
            Rover.turning = False

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Change mode to stop if we're getting close to a rock
        if Rover.near_sample or is_getting_close_to_rock(Rover):
            Rover.mode = 'stop'

        if Rover.mode == 'forward':
            # Something preventing rover from continue moving forward
            if should_stop_forward(Rover):
                stop_rover(Rover)
            # Check the extent of navigable terrain
            else:
                Rover.throttle = get_throttle(Rover)
                Rover.steer = get_steering_angle(Rover)
                Rover.brake = 0

        elif Rover.mode == 'stop':
            # Keep braking
            if Rover.vel > 0.2 or Rover.near_sample:
                stop_rover(Rover)
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                # Check if getting close to a rock to exclude some false positives happening right near rock
                if (is_nearest_path_blocked(Rover) or Rover.stuck) and not is_getting_close_to_rock(Rover):
                    turn_around(Rover)
                    Rover.turning = True
                # Turn around if not enough place
                elif not should_go_forward(Rover):
                    if Rover.steer == 0:
                        turn_around(Rover)
                # If we're stopped but see sufficient navigable terrain in front then go!
                else:
                    move_forward(Rover)
    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return track_steering(Rover)


def track_steering(Rover):
    Rover.last_100_steer_positions.append(math.modf(Rover.steer)[1])
    return Rover


def get_throttle(Rover):
    if Rover.vel < Rover.max_vel:
        # Set throttle value to throttle setting
        return Rover.throttle_set
    else:  # Else coast
        return 0


def move_forward(Rover):
    # Set throttle back to stored value
    Rover.throttle = Rover.throttle_set
    # Release the brake
    Rover.brake = 0
    # Set steer to mean angle
    Rover.steer = get_steering_angle(Rover)
    Rover.mode = 'forward'
    Rover.stuck = False


def stop_rover(Rover):
    # Set mode to "stop" and hit the brakes!
    Rover.throttle = 0
    # Set brake to stored brake value
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    Rover.mode = 'stop'


def turn_around(Rover):
    Rover.mode = 'stop'
    Rover.throttle = 0
    # Release the brake to allow turning
    Rover.brake = 0
    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
    Rover.steer = 15 * random.choice([-1, 1]) # Randomly choose which way to turn
    Rover.stuck = False


def get_steering_angle(Rover):
    if is_rock_detected(Rover):
        return np.clip(np.mean(Rover.rock_nav_angles * 180 / np.pi), -30, 30)
    else:
        return np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)


def is_nearest_path_blocked(Rover):
    return len(Rover.nav_dists[Rover.nav_dists < 15]) < 100


def should_stop_forward(Rover):
    return (len(Rover.nav_angles) < Rover.stop_forward or is_nearest_path_blocked(Rover)) \
           and not is_rock_detected(Rover)


def is_getting_close_to_rock(Rover):
    return Rover.rock_nav_dists.any() and np.mean(Rover.rock_nav_dists) < 15


def is_rock_detected(Rover):
    return len(Rover.rock_nav_angles) >= 3


def should_go_forward(Rover):
    return len(Rover.nav_angles) >= Rover.go_forward


def is_looping(Rover):
    # Consider that we can't looping when rock detected
    return not Rover.stuck \
           and Rover.steer != 0 \
           and np.mean(Rover.last_100_steer_positions) == Rover.steer \
           and not is_rock_detected(Rover)


def get_previous_steering_value(Rover):
    val = Rover.last_100_steer_positions.pop()
    Rover.last_100_steer_positions.append(val)
    return val


def are_coordinates_changed(Rover):
    # If rover hadn't (significantly) change its position for last 100 decision steps
    return np.mean(Rover.last_100_x_positions) != math.modf(Rover.pos[0])[1] \
           or np.mean(Rover.last_100_y_positions) != math.modf(Rover.pos[1])[1]
