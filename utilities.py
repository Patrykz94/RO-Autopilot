import numpy as np
import time

from Launch import globals

conn = globals.conn
space_center = globals.space_center
vessel = globals.vessel

# Trigonometric function in degrees :)


def r2d(x):
    return x * 180 / np.pi


def d2r(x):
    return x * np.pi / 180


def cosd(x):
    return np.cos(d2r(x))


def acosd(x):
    np.clip(x, -1, 1)
    return r2d(np.arccos(x))


def sind(x):
    return np.sin(d2r(x))


def asind(x):
    np.clip(x, -1, 1)
    return r2d(np.arcsin(x))


def tand(x):
    return np.tan(d2r(x))


def atand(x):
    return r2d(np.arctan(x))


def atan2d(x, y):
    return r2d(np.arctan2(x, y))


# Another simplified function, and yeah i am that lazy
# This one is for vector operation
def norm(x):
    return np.linalg.norm(x)


def unit(x):
    x = np.asarray(x)
    if norm(x) == 0:
        return x
    else:
        return x / norm(x)


def cross(x, y):
    return np.cross(x, y)


def dot(x, y):
    return np.vdot(x, y)


def vang(x, y):
    x = unit(x)
    y = unit(y)
    return acosd(np.clip(dot(x, y), -1, 1))

    # Vector rotation function


def rodrigues(vector, axis, angle):
    vector = np.asarray(vector)
    axis = unit(axis)
    rotated = vector * cosd(angle)
    rotated += cross(axis, vector) * sind(angle)
    rotated += axis * dot(axis, vector) * (1 - cosd(angle))

    return rotated


# Vector Exclude (or vector projection) function known from kOS
def vxcl(v1, v2):
    v1 = np.asarray(v1)
    v2 = np.asarray(v2)
    return v2 - dot(v2, v1) * v1


# Vector Angle function known from kOS
def vang2(v1, v2):
    v1 = np.asarray(v1)
    v2 = np.asarray(v2)
    return np.arccos((dot(v1, v2)/(magnitude(v1)*magnitude(v2))))


# Returns magnitude (or length) of a vector
def magnitude(v1):
    v1 = np.asarray(v1)
    return np.sqrt(dot(v1, v1))


# Normalizes the vector (makes it length of 1 / unit vector)
def normalize(v1):
    v1 = np.asarray(v1)
    return v1 / (np.sqrt(dot(v1, v1)))


# Returns Solar Prime Vector (Prime Meridian of the Solar System)
def solar_prime_vector(reference_frame):
    sun = space_center.bodies["Sun"]
    seconds_per_degree = sun.rotational_period / 360
    rotation_offset = np.mod(space_center.ut/seconds_per_degree, 360)
    sun_position = sun.position(reference_frame)
    sun_position2 = sun.surface_position(
        0, 0 - rotation_offset, reference_frame)

    prime_vector = np.asarray(sun_position2) - np.asarray(sun_position)

    return normalize(prime_vector)


def nodeVector(inclination, descending=False):
    # From right spherical triangle composed of inclination, latitude and "b",
    # which is angular difference between the desired node vector and projection
    # of the vector pointing at the launch site onto the equatorial plane.
    b = tand(90-inclination) * tand(vessel.flight().latitude)
    b = asind(min(max(-1, b), 1))
    longitudeVector = normalize(vxcl(np.asarray(
        [0, 1, 0]), -np.asarray(vessel.orbit.body.position(vessel.surface_reference_frame))))
    if descending == True:
        # This can be easily derived from spherical triangle if one draws a half
                # of an orbit, from node to node. It is obvious that distance from node to
                # peak equals 90 degrees, and from that the following results.
        return rodrigues(longitudeVector, np.asarray([0, 1, 0]), 180-b)
    else:
        return rodrigues(longitudeVector, np.asarray([0, 1, 0]), b)
