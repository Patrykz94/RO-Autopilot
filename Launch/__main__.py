import sys
import time

import numpy as np
import utilities as utils

from Launch import globals
from Launch import upfg

conn = globals.conn
space_center = globals.space_center
vessel = globals.vessel
vehicle = upfg.analyze_vehicle()

position = globals.orbital_position
velocity = globals.orbital_velocity
surface_speed = globals.surface_speed

g0 = globals.g0
mu = globals.mu

target_apoapsis = sys.argv[1]
target_periapsis = sys.argv[2]
target_inclination = sys.argv[3]
target_lan = sys.argv[4]
target_true_anomaly = sys.argv[5]
sys.argv[6].upper()
if sys.argv[6] == 'RTLS':
    meco_speed = 1700
    turn_speed = 30
    g_lim = 3
    q_lim = 27000
elif sys.argv[6] == 'ASDS':
    meco_speed = 2300
    turn_speed = 30
    g_lim = 5
    q_lim = 30000
else:
    meco_speed = vehicle[0].l1 - 1000
    turn_speed = 30
    g_lim = 5
    q_lim = 30000
[azimuth, launch_time, target] = upfg.launch_targeting(target_periapsis,
                                                       target_apoapsis,
                                                       target_inclination,
                                                       target_lan,
                                                       target_true_anomaly)

game_launch_time = space_center.ut + launch_time
space_center.warp_to(game_launch_time - 20)

strongback_retracted = False
engines_ignited = False
engines_ignition_time = -4.5

while (space_center.ut - game_launch_time) < 0:
    print('Time to launch %i' % int(space_center.ut - game_launch_time))
    if not strongback_retracted and (space_center.ut - game_launch_time) > -15:
        vessel.control.set_action_group(9, True)
        strongback_retracted = True
    if not engines_ignited and (space_center.ut - game_launch_time) > engines_ignition_time:
        vessel.control.throttle = 1
        vessel.control.activate_next_stage()
        engines_ignited = True
    time.sleep(1)

while globals.state_thrust() < vessel.available_thrust * 0.95:
    time.sleep(0.1)

print(space_center.ut - game_launch_time)

vessel.auto_pilot.engage()
vessel.auto_pilot.target_heading = azimuth
vessel.auto_pilot.target_roll = azimuth
vessel.auto_pilot.target_pitch = 90
vessel.control.activate_next_stage()
vessel.auto_pilot.wait()

print('Proceeding Launch..')

while surface_speed() < turn_speed:
    time.sleep(0.1)

print('Clear from launch tower..')
print('Begin Pitch Program..')

vessel.auto_pilot.target_heading = azimuth

while True:
    vessel.control.throttle = upfg.throttle_control(vehicle, g_lim, q_lim)

    pitch1 = utils.atand((1000 - 2 * turn_speed) /
                         (surface_speed() - turn_speed))
    pitch2 = upfg.angle_from_vec(globals.surface_velocity(),
                                 globals.body_reference_frame,
                                 'pitch')
    vessel.auto_pilot.target_pitch = min(pitch1, pitch2)
    if vessel.available_thrust == 0:
        break
    time.sleep(0.01)

print('Main Engine Cutoff')

upfg.stage_controller(vehicle)

fairing_jettison = False
cser = upfg.Struct()
cser.dtcp = 0
cser.xcp = 0
cser.a = 0
cser.d = 0
cser.e = 0

rdinit = utils.rodrigues(utils.unit(position()), -target.normal, 20)
rdinit = np.multiply(rdinit, target.radius)
vdinit = np.multiply(target.velocity, utils.unit(
    utils.cross(-target.normal, rdinit)))
vdinit = vdinit - velocity()
upfg_internal = upfg.Struct()
upfg_internal.cser = cser
upfg_internal.rbias = [0, 0, 0]
upfg_internal.rd = rdinit
upfg_internal.rgrav = np.multiply(
    np.multiply(-(mu / 2), position()), 1 / utils.norm(position()) ** 3)
upfg_internal.tb = 0
upfg_internal.time = globals.universal_time()
upfg_internal.tgo = 0
upfg_internal.v = velocity()
upfg_internal.vgo = vdinit
converged = False
upfg_guided = upfg.Struct()
iteration = 0

while converged is False:
    [upfg_internal, upfg_guided] = upfg.upfg(vehicle, target, upfg_internal)
    t1 = upfg_guided.tgo
    [upfg_internal, upfg_guided] = upfg.upfg(vehicle, target, upfg_internal)
    t2 = upfg_guided.tgo
    if abs(t1 - t2) / t2 < 0.01:
        converged = True
    iteration += 1

print('Guidance converged after %i iteration' % int(iteration))

while True:
    vessel.control.throttle = upfg.throttle_control(vehicle, g_lim, q_lim)
    [upfg_internal, upfg_guided] = upfg.upfg(vehicle, target, upfg_internal)
    if upfg_guided.tgo > 1:
        vessel.auto_pilot.target_heading = upfg_guided.yaw
        vessel.auto_pilot.target_pitch = upfg_guided.pitch
    if upfg_guided.tgo < 0.1:
        vessel.control.throttle = 0
        break
    if globals.orbital_speed() > target.velocity:
        vessel.control.throttle = 0
        break
    if globals.surface_altitude() > 120000 and not fairing_jettison:
        for fairing in vessel.parts.fairings:
            for mod in fairing.part.modules:
                if mod.has_event('Jettison'):
                    mod.trigger_event('Jettison')
                    fairing_jettison = True
        if not fairing_jettison:
            vessel.control.activate_next_stage()
            fairing_jettison = True

print('Mission Success')
