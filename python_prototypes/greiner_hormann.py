import numpy as np
import matplotlib.pyplot as plt
import random
tol = 1e-9


def sign(x):
    if x > tol:
        return 1
    elif x < -tol:
        return -1
    else:
        return 0


def plot_lines(vertices):
    plt.plot([v[0] for v in vertices], [v[1] for v in vertices])


def point_in_polygon(p, poly):
    # poly is given in ccw order
    # for i in range(len(poly)):
    #     u = poly[i-1] - p
    #     v = poly[i] - p
    #     if np.cross(u, v) < -tol:
    #         return False
    # return True
    ang = random.uniform(-np.pi/2, np.pi/2)
    u = np.array([np.cos(ang), np.sin(ang)])

    number_of_intersections = 0
    for i in range(len(poly) - 1):
        v = poly[i+1] - poly[i]
        q = np.cross(u, v)
        t = np.cross(poly[i] - p, v) / q
        s = np.cross(poly[i] - p, u) / q
        if (t > -tol) and  (-tol < s < 1 + tol):
            number_of_intersections += 1
    return number_of_intersections % 2 == 1


def intersect_segments(a, b, c, d):
    u = b - a
    v = d - c
    q = np.cross(u, v)
    if -tol < q < tol:
        # Parallel / overlapping do not intersect =(
        return False,
    t = np.cross(c - a, v) / q
    s = np.cross(c - a, u) / q

    if (-tol < t < 1 + tol) and (-tol < s < 1 + tol):
        return True, a + t * u, t, s
    else:
        return False,


def intersect_polygons(poly1, poly2):
    # Add a label to points: -1 <= t <= 1 such that |t| specifies the position on the polygon edge
    # (e.g., polygon vertices have t= 0) and the sign marks
    # if it is an entry point(positive) or an exit point (negative).
    # Add an index to intersection points that gives its position in the second list (-1 for vertices)
    vert = [[[p, 0, -1, False] for p in poly1], [[p, 0, -1, False] for p in poly2]]

    # Find pairwise intersections of edges
    number_of_intersections2 = [0 for p in poly2]
    shift1 = 0
    number_of_intersections1 = 0
    for i in range(len(poly1) - 1):
        shift2 = 0
        shift1 += number_of_intersections1
        number_of_intersections1 = 0
        for j in range(len(poly2) - 1):
            intersection = intersect_segments(vert[0][i + shift1][0],
                                              vert[0][i+1 + shift1 + number_of_intersections1][0],
                                              vert[1][j + shift2][0],
                                              vert[1][j+1 + shift2 + number_of_intersections2[j]][0])
            shift2 += number_of_intersections2[j]
            if intersection[0]:
                vert[0].insert(i+1 + shift1, [intersection[1], intersection[2], j+1 + shift2, False])
                vert[1].insert(j+1 + shift2, [intersection[1], intersection[3], i+1 + shift1, False])
                shift2 += 1
                number_of_intersections2[j] += 1
                number_of_intersections1 += 1

    # Sort intersection points
    vertex_indices1 = [i for i in range(len(vert[0])) if vert[0][i][1] == 0]
    vertex_indices2 = [i for i in range(len(vert[1])) if vert[1][i][1] == 0]
    for j in range(len(vertex_indices1) - 1):
        vert[0][vertex_indices1[j]:vertex_indices1[j+1]] = sorted(vert[0][vertex_indices1[j]:vertex_indices1[j+1]],
                                                                  key=lambda x: x[1])
    for j in range(len(vertex_indices2) - 1):
        vert[1][vertex_indices2[j]:vertex_indices2[j+1]] = sorted(vert[1][vertex_indices2[j]:vertex_indices2[j+1]],
                                                                  key=lambda x: x[1])

    # Mark intersection points as enter/exit for poly1
    inside = point_in_polygon(poly1[0], poly2)
    label = -1 if inside else 1
    for v in vert[0]:
        if v[1] != 0:
            v[1] *= label
            label *= -1

    # Mark intersection points as enter/exit for poly2
    inside = point_in_polygon(poly2[0], poly1)
    label = -1 if inside else 1
    for v in vert[1]:
        if v[1] != 0:
            v[1] *= label
            label *= -1

    # Generate result
    cur_vert = 0
    index = next(iter([i for i in range(len(vert[cur_vert])) if vert[cur_vert][i][1] != 0]))
    dir = sign(vert[cur_vert][index][1])
    result = [vert[cur_vert][index][0]]
    vert[cur_vert][index][3] = True
    while not vert[cur_vert][(index + dir) % len(vert[cur_vert])][3]:
        index += dir
        vert[cur_vert][index][3] = True
        result.append(vert[cur_vert][index][0])
        if vert[cur_vert][index][1] != 0:
            cur_vert = (cur_vert + 1) % 2
            index = next(iter([i for i in range(len(vert[cur_vert])) if np.allclose(vert[cur_vert][i][0], result[-1])]))
            dir = sign(vert[cur_vert][index][1])
    return result


d = np.array([-1, - 1])
c = np.array([-1, 1])
b = np.array([1, 1])
a = np.array([1, -1])

A = np.array([2, 0])
B = np.array([-0.3, 2])
C = np.array([0.4, 0.5])
D = np.array([-1.5, -0.2])

p1 = [a,b,c,d,a]
p2 = [A,B,C,D,A]

inter = intersect_polygons(p1, p2)
plot_lines(p1)
plot_lines(p2)
plot_lines(inter)
plt.show()