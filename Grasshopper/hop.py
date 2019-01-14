import krpc
import time

conn = krpc.connect(name='Grasshopper Test')
space_center = conn.space_center
vessel = space_center.active_vessel
g0 = 9.80655
mu = vessel.orbit.body.gravitational_parameter

inertial_reference_frame = vessel.orbit.body.non_rotating_reference_frame
body_reference_frame = vessel.orbit.body.reference_frame

def thrust_to_weight():
    return vessel.thrust / (vessel.mass * g0)


def max_thrust_to_weight():
    return vessel.max_thrust / (vessel.mass * g0)


# Test 1: First very short hop.
# The vehicle will launch at full thrust of 1 engine (out of 9 total)
# After 0.5s-1.0s will shut down its engine and land (well... drop really)

vessel.control.activate_next_stage()
vessel.control.throttle = 1
while thrust_to_weight() < 1.2:
    time.sleep(0.1)
