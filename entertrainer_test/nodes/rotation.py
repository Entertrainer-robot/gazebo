from math import *

def rx(theta, x, y, z):
    _x = 1 * x          + 0 * y             + 0 * z
    _y = 0 * x          + cos(theta) * y    + -sin(theta) * z
    _z = 0 * x          + sin(theta) * y    + cos(theta) * z
    return [_x, _y , _z]
def ry(theta, x, y, z):
    _x = cos(theta) * x  + 0 * y             + sin(theta) * z
    _y = 0 * x           + 1 * y             + 0 * z
    _z = -sin(theta) * x + 0 * y             + cos(theta) * z
    return [_x, _y , _z]
def rz(theta, x, y, z):
    _x = cos(theta) * x  + -sin(theta) * y   + 0 * z
    _y = sin(theta) * x  + cos(theta) * y    + 0 * z
    _z = 0 * x           + 0 * y             + 1 * z
    return [_x, _y , _z]

def rzyx(tx, ty, tz, x, y, z):
    _x, _y, _z = rz(tz, x, y, z)
    _x, _y, _z = ry(ty, _x, _y, _z)
    _x, _y, _z = rx(tx, _x, _y, _z)
    return [_x, _y, _z]

def ryzx(tx, ty, tz, x, y, z):
    _x, _y, _z = ry(ty, x, y, z)
    _x, _y, _z = rz(tz, _x, _y, _z)
    _x, _y, _z = rx(tx, _x, _y, _z)
    return [_x, _y, _z]

def rxyz(tx, ty, tz, x, y, z):
    _x, _y, _z = rx(tx, x, y, z)
    _x, _y, _z = ry(ty, _x, _y, _z)
    _x, _y, _z = rz(tz, _x, _y, _z)
    return [_x, _y, _z]
###
def rzxy(tx, ty, tz, x, y, z):
    _x, _y, _z = rz(tz, x, y, z)
    _x, _y, _z = rx(tx, _x, _y, _z)
    _x, _y, _z = ry(ty, _x, _y, _z)
    return [_x, _y, _z]

def ryxz(tx, ty, tz, x, y, z):
    _x, _y, _z = ry(ty, x, y, z)
    _x, _y, _z = rx(tx, _x, _y, _z)
    _x, _y, _z = rz(tz, _x, _y, _z)
    return [_x, _y, _z]

def rxzy(tx, ty, tz, x, y, z):
    _x, _y, _z = rx(tx, x, y, z)
    _x, _y, _z = rz(tz, _x, _y, _z)
    _x, _y, _z = ry(ty, _x, _y, _z)
    return [_x, _y, _z]

def calc_euler_from_quaternion(q):
    # expect q is x,y,z,w like the pose.rotation value in rospy
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if (abs(sinp) >= 1):
        pitch = copysign(pi / 2, sinp) # use 90 degrees if out of range
    else:
        pitch = asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = atan2(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]


def calc_quaternion_from_euler(yaw, pitch, roll): # yaw (Z), pitch (Y), roll (X)
    # Abbreviations for the various angular functions
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);

    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;

    return [x, y, z, w]
