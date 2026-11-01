"""Planning-interval reachability, not a continuous physical acceleration model.

The adapter executes each validated displacement as a straight segment. Its
previous segment velocity is authoritative; native preferred speed is not.
Safety may require lateral travel or a larger velocity change. Such exceptions
are reported by the projection, never hidden by post-projection smoothing.
"""
import math

DEFAULT_LIMITS = dict(acceleration_mps2=.8, deceleration_mps2=1.6)


def validate_limits(value):
    if set(value) != set(DEFAULT_LIMITS) or any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in value.values()):
        raise ValueError('Motion acceleration/deceleration must be finite positive m/s²')
    return dict(value)


def preferred_velocity(velocity, yaw, max_yaw_rate, dt):
    """Turn the preference within the yaw budget, braking for large direction error.

    This is a preference, not a nonholonomic restriction on safety sidesteps.
    """
    speed = math.hypot(*velocity)
    if speed < 1e-9:
        return (0., 0.)
    error = math.atan2(math.sin(math.atan2(velocity[1], velocity[0])-yaw),
                       math.cos(math.atan2(velocity[1], velocity[0])-yaw))
    turn = max(-max_yaw_rate*dt, min(max_yaw_rate*dt, error))
    speed *= max(0., math.cos(error-turn))
    return speed*math.cos(yaw+turn), speed*math.sin(yaw+turn)


def velocity_change_limits(previous, desired, limits):
    """Bound vector delta-v; use the braking budget when opposing prior velocity."""
    validate_limits(limits)
    return [limits['deceleration_mps2'] if sum((d-p)*p for p,d in zip(old,new)) < -1e-12
            else limits['acceleration_mps2'] for old,new in zip(previous,desired)]


def final_heading(previous_yaw, velocity, native_yaw, max_yaw_rate, dt, stationary=False):
    # Preserve conversation facing; preserve moving-person heading when stopped.
    target = native_yaw if stationary else math.atan2(velocity[1], velocity[0]) if math.hypot(*velocity) > 1e-9 else previous_yaw
    error = math.atan2(math.sin(target-previous_yaw), math.cos(target-previous_yaw))
    turn = max(-max_yaw_rate*dt, min(max_yaw_rate*dt, error))
    return math.atan2(math.sin(previous_yaw+turn), math.cos(previous_yaw+turn))
