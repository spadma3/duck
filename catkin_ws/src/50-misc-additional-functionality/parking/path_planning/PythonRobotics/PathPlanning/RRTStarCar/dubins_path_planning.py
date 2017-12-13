#! /usr/bin/python
# -*- coding: utf-8 -*-
"""

Dubins path planner sample code

author Atsushi Sakai(@Atsushi_twi)
adapted Samuel Nyffenegger (samueln@ethz.ch)

License MIT

"""

import math
from math import sin, cos, sqrt, atan2, degrees, radians, pi
from numpy import sign

def mod2pi(theta):
    return theta - 2.0 * math.pi * math.floor(theta / 2.0 / math.pi)


def pi_2_pi(angle):
    while(angle >= math.pi):
        angle = angle - 2.0 * math.pi

    while(angle <= -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def LSL(alpha, beta, d, allow_backwards_on_circle=False):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    tmp0 = d + sa - sb

    mode = ["L", "S", "L"]
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb))
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = math.atan2((cb - ca), tmp0)
    t = mod2pi(-alpha + tmp1)
    p = math.sqrt(p_squared)
    q = mod2pi(beta - tmp1)
    #  print(math.degrees(t), p, math.degrees(q))

    if allow_backwards_on_circle:
        if t > pi:
            t = t - 2*pi
            mode[0] = "l"
        if q > pi:
            q = q - 2*pi
            mode[2] = "l"

    return t, p, q, mode


def RSR(alpha, beta, d, allow_backwards_on_circle=False):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    tmp0 = d - sa + sb
    mode = ["R", "S", "R"]
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa))
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = math.atan2((ca - cb), tmp0)
    t = mod2pi(alpha - tmp1)
    p = math.sqrt(p_squared)
    q = mod2pi(-beta + tmp1)
    # print(math.degrees(t), p, math.degrees(q))

    if allow_backwards_on_circle:
        if t > pi:
            t = t - 2*pi
            mode[0] = "r"
        if q > pi:
            q = q - 2*pi
            mode[2] = "r"

    return t, p, q, mode


def LSR(alpha, beta, d, allow_backwards_on_circle=False):
    sa = sin(alpha)
    sb = sin(beta)
    ca = cos(alpha)
    cb = cos(beta)
    c_ab = cos(alpha-beta)

    p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb))
    mode = ["L", "S", "R"]
    if p_squared < 0:
        return None, None, None, mode
    p = sqrt(p_squared)
    tmp2 = atan2((-ca - cb), (d + sa + sb)) - atan2(-2.0, p)
    t = mod2pi(-alpha + tmp2)
    q = mod2pi(-mod2pi(beta) + tmp2)

    if allow_backwards_on_circle:
        if t > pi:
            t = t - 2*pi
            mode[0] = "l"
        if q > pi:
            q = q - 2*pi
            mode[2] = "r"

    return t, p, q, mode


def RSL(alpha, beta, d, allow_backwards_on_circle=False):
    sa = sin(alpha)
    sb = sin(beta)
    ca = cos(alpha)
    cb = cos(beta)
    c_ab = cos(alpha - beta)
    
    p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb))
    mode = ["R", "S", "L"]
    if p_squared < 0:
        return None, None, None, mode
    p = sqrt(p_squared)
    tmp2 = atan2((ca + cb), (d - sa - sb)) - atan2(2.0, p)
    t = mod2pi(alpha - tmp2)
    q = mod2pi(beta - tmp2)

    if allow_backwards_on_circle:
        if t > pi:
            t = t - 2*pi
            mode[0] = "r"
        if q > pi:
            q = q - 2*pi
            mode[2] = "l"

    return t, p, q, mode

# not tested when driving backwards
def RLR(alpha, beta, d, allow_backwards_on_circle=False):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    mode = ["R", "L", "R"]
    tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
    if abs(tmp_rlr) > 1.0:
        return None, None, None, mode

    p = mod2pi(2 * math.pi - math.acos(tmp_rlr))
    t = mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + mod2pi(p / 2.0))
    q = mod2pi(alpha - beta - t + mod2pi(p))

    if allow_backwards_on_circle:
        if t > pi:
            t = t - 2*pi
            mode[0] = "r"
        if q > pi:
            q = q - 2*pi
            mode[2] = "r"

    return t, p, q, mode

# not tested when driving backwards
def LRL(alpha, beta, d, allow_backwards_on_circle=False):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    mode = ["L", "R", "L"]
    tmp_lrl = (6. - d * d + 2 * c_ab + 2 * d * (- sa + sb)) / 8.
    if abs(tmp_lrl) > 1:
        return None, None, None, mode
    p = mod2pi(2 * math.pi - math.acos(tmp_lrl))
    t = mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.)
    q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p))

    if allow_backwards_on_circle:
        if t > pi:
            t = t - 2*pi
            mode[0] = "l"
        if q > pi:
            q = q - 2*pi
            mode[2] = "l"

    return t, p, q, mode


def dubins_path_planning_from_origin(ex, ey, eyaw, c, allow_backwards_on_circle=False):
    
    # nomalize
    dx = ex
    dy = ey
    D = sqrt(dx ** 2.0 + dy ** 2.0)
    d = D / c
    # print(dx, dy, D, d)

    theta = mod2pi(atan2(dy, dx))
    alpha = mod2pi(-theta)
    beta = mod2pi(eyaw - theta)
    # print(degrees(theta), degrees(alpha), degrees(beta), d)

    planners = [LSL, RSR, LSR, RSL, RLR, LRL]

    bcost = float("inf")
    bt, bp, bq, bmode = None, None, None, None

    for planner in planners:
        t, p, q, mode = planner(alpha, beta, d, allow_backwards_on_circle)
        if t is None:
            print("".join(mode) + " cannot generate path")
            continue

        cost = (abs(t) + abs(p) + abs(q)) # cost is either normalized distance or angle in rad
        if bcost > cost:
            bt, bp, bq, bmode = t, p, q, mode
            bcost = cost

        # plot all paths (only correct if start pose is origin)
        px, py, pyaw = generate_course([t, p, q], mode, c)
        plt.plot(px, py, label="".join(mode))

    # print(bt, math.degrees(bp), math.degrees(bq), bmode)
    px, py, pyaw = generate_course([bt, bp, bq], bmode, c)

    return px, py, pyaw, bmode, bcost


def dubins_path_planning(sx, sy, syaw, ex, ey, eyaw, c, allow_backwards_on_circle=False):
    """
    Dubins path plannner

    input:
        sx x position of start point [m]
        sy y position of start point [m]
        syaw yaw angle of start point [rad]
        ex x position of end point [m]
        ey y position of end point [m]
        eyaw yaw angle of end point [rad]
        c curvature [1/m]

    output:
        px
        py
        pyaw
        mode

    """

    ex = ex - sx
    ey = ey - sy

    lex = math.cos(syaw) * ex + math.sin(syaw) * ey
    ley = - math.sin(syaw) * ex + math.cos(syaw) * ey
    leyaw = eyaw - syaw
    
    # print(lex,ley,math.degrees(leyaw))

    lpx, lpy, lpyaw, mode, clen = dubins_path_planning_from_origin(
        lex, ley, leyaw, c, allow_backwards_on_circle)

    px = [math.cos(-syaw) * x + math.sin(-syaw) *
          y + sx for x, y in zip(lpx, lpy)]
    py = [- math.sin(-syaw) * x + math.cos(-syaw) *
          y + sy for x, y in zip(lpx, lpy)]
    pyaw = [pi_2_pi(iyaw + syaw) for iyaw in lpyaw]
    #  print(syaw)
    #  pyaw = lpyaw

    #  plt.plot(pyaw, "-r")
    #  plt.plot(lpyaw, "-b")
    #  plt.plot(eyaw, "*r")
    #  plt.plot(syaw, "*b")
    #  plt.show()

    return px, py, pyaw, mode, clen


def generate_course(length, mode, c):
    px=[0.0]
    py=[0.0]
    pyaw=[0.0]
    
    # length = [t, p, q], mode = ["r","S","l"]
    
    for m, l in zip(mode, length):
        pd = 0.0
        if m is "S":
            # straight plotting resolution
            d = 1.0 / c /10.0
        else:  # turning couse
            # radial plotting resolution
            d = radians(3.0)
        
        while pd < abs(l - d*sign(l)):
            # print(pd, l)
            if m is "L":  # left turn forward
                px.append(px[-1] + d * c * cos(pyaw[-1]))
                py.append(py[-1] + d * c * sin(pyaw[-1]))
                pyaw.append(pyaw[-1] + d)
            elif m is "l":  # left turn backwards
                px.append(px[-1] - d * c * cos(pyaw[-1]))
                py.append(py[-1] - d * c * sin(pyaw[-1]))
                pyaw.append(pyaw[-1] - (+d))
            elif m is "S":  # Straight forward
                px.append(px[-1] + d * c * cos(pyaw[-1]))
                py.append(py[-1] + d * c * sin(pyaw[-1]))
                pyaw.append(pyaw[-1])
            elif m is "R":  # right turn forward
                px.append(px[-1] + d * c * cos(pyaw[-1]))
                py.append(py[-1] + d * c * sin(pyaw[-1]))
                pyaw.append(pyaw[-1] - d)
            elif m is "r":  # right turn backwards
                px.append(px[-1] - d * c * cos(pyaw[-1]))
                py.append(py[-1] - d * c * sin(pyaw[-1]))
                pyaw.append(pyaw[-1] - (-d))
            pd += d
        else:
            # difference (|d| < resolution)
            d = l - pd*sign(l)
            if m is "L":  # left turn forward
                px.append(px[-1] + d * c * cos(pyaw[-1]))
                py.append(py[-1] + d * c * sin(pyaw[-1]))
                pyaw.append(pyaw[-1] + d)
            elif m is "l":  # left turn backwards
                d = -d
                px.append(px[-1] - d * c * cos(pyaw[-1]))
                py.append(py[-1] - d * c * sin(pyaw[-1]))
                pyaw.append(pyaw[-1] - (+d))
            elif m is "S":  # Straight forward
                px.append(px[-1] + d * c * cos(pyaw[-1]))
                py.append(py[-1] + d * c * sin(pyaw[-1]))
                pyaw.append(pyaw[-1])
            elif m is "R":  # right turn forward
                px.append(px[-1] + d * c * cos(pyaw[-1]))
                py.append(py[-1] + d * c * sin(pyaw[-1]))
                pyaw.append(pyaw[-1] - d)
            elif m is "r":  # right turn backwards
                d = -d
                px.append(px[-1] - d * c * cos(pyaw[-1]))
                py.append(py[-1] - d * c * sin(pyaw[-1]))
                pyaw.append(pyaw[-1] - (-d))
            pd += d

    return px, py, pyaw


def plot_arrow(x, y, yaw, length=0.1, width=0.06, fc="k", ec="k"):
    """
    Plot arrow
    """
    import matplotlib.pyplot as plt

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


if __name__ == '__main__':
    
    
    print("Dubins path planner sample start!!")
    import matplotlib.pyplot as plt

    start_x = 0.0  # [m]
    start_y = 0.0  # [m]
    start_yaw = math.radians(0.0)  # [rad]

    end_x = 0.0  # [m]
    end_y = 0.1  # [m]
    end_yaw = math.radians(0.0)  # [rad]

    curvature = 0.25

    allow_backwards_on_circle = True

    px, py, pyaw, mode, clen = dubins_path_planning(start_x, start_y, start_yaw,
                                                    end_x, end_y, end_yaw, curvature,
                                                    allow_backwards_on_circle)

    plt.plot(px, py, label="final course " + "".join(mode))

    # plotting
    plot_arrow(start_x, start_y, start_yaw, 0.1, 0.06, fc="r", ec="r")
    plot_arrow(end_x, end_y, end_yaw, 0.1, 0.06, fc="g", ec="g")

#    for (ix, iy, iyaw) in zip(px, py, pyaw):
#        plot_arrow(ix, iy, iyaw, fc="b")

    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()
