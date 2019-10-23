import math
import cv2 as cv
import numpy as np
import skimage.morphology
# from mathutils.geometry import intersect_point_line
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
# import gridmap_2d

min_branch_length = 5
min_enclosing_radius = 5
connectivity_kernel = np.uint8([[1, 1, 1],
                                [1, 10, 1],
                                [1, 1, 1]])
reg_bounding_box_size = 20
max_angle_diff = 60
max_line_dist = 30

def prune_skeleton(skel):
    filtered = cv.filter2D(skel, -1, connectivity_kernel)

    branches = np.zeros_like(skel)
    branches[np.where((11 <= filtered) & (filtered <= 12))] = 255

    _, contours, _ = cv.findContours(branches, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)
    big_branches = np.zeros_like(filtered)
    for contour in contours:
        if cv.arcLength(contour, False) > min_branch_length:
            cv.drawContours(big_branches, [contour], 0, 255, 1)

    big_branches[np.where(filtered >= 13)] = 255
    pruned_skel = skimage.morphology.skeletonize(big_branches // 255).astype(np.uint8)
    return pruned_skel

def intersect_point_line(pntA, pntB1, pntB2, bound=False):
    pntA, pntB1, pntB2 = np.array([pntA, pntB1, pntB2])
    n = np.subtract(pntB2, pntB1) / np.linalg.norm(np.subtract(pntB2, pntB1))
    ap = np.subtract(pntA, pntB1)
    t = np.dot(ap, n)
    x = pntB1 + t * n

    if bound and np.linalg.norm(x - pntB1) + np.linalg.norm(x - pntB2) > np.linalg.norm(pntB2 - pntB1):
        endpnts = [pntB1, pntB2]
        return endpnts[np.argmin([np.linalg.norm(pntA - pnt) for pnt in endpnts])]
    return x

def connectable(current_contour_line, prev_contour_line):
    endpntA, pntA, vxA, vyA = current_contour_line
    endpntB, pntB, vxB, vyB = prev_contour_line

    mA, mB = vyA / vxA, vyB / vxB
    angle_between_lines = abs(math.degrees(math.atan((mB - mA) / (1 + (mB * mA)))))

    distance = 300
    point_a = (int(endpntA[0] + distance * vxA), int(endpntA[1] + distance * vyA))
    point_b = (int(endpntB[0] + distance * vxB), int(endpntB[1] + distance * vyB))

    closest_pnt_lineA = intersect_point_line(endpntB, endpntA, point_a, True)
    closest_pnt_lineB = intersect_point_line(endpntA, endpntB, point_b, True)
    dist_pnt_lineA = np.linalg.norm(np.subtract(closest_pnt_lineA, endpntB))
    dist_pnt_lineB = np.linalg.norm(np.subtract(closest_pnt_lineB, endpntA))

    return angle_between_lines < max_angle_diff and dist_pnt_lineB < max_line_dist and dist_pnt_lineA < max_line_dist


# img = cv.imread('track.png')
def show_fake_walls(img):
    _, binary = cv.threshold(img, 40, 1, cv.THRESH_BINARY | cv.THRESH_OTSU)

    # All Skel Img are in binary so must (skel * 255) to visualize
    skel = skimage.morphology.skeletonize(binary).astype(np.uint8)
    skel = prune_skeleton(skel)

    debug_img = cv.cvtColor(skel * 255, cv.COLOR_GRAY2BGR)

    _, contours, _ = cv.findContours(skel, cv.RETR_CCOMP, cv.CHAIN_APPROX_NONE)
    endpnt_line = []
    for contour in contours:
        _, radius = cv.minEnclosingCircle(contour)
        if radius > min_enclosing_radius:  # If small (Likely car or random obstacle) ignore
            contour_img = np.zeros_like(skel)
            cv.drawContours(contour_img, [contour], 0, 1, 1)

            filtered = cv.filter2D(contour_img, -1, connectivity_kernel)
            endpoints = np.where(filtered == 11)

            for endpnt in tuple(zip(endpoints[1], endpoints[0])):
                half = reg_bounding_box_size // 2
                tl, br = (endpnt[0] - half, endpnt[1] - half), (endpnt[0] + half, endpnt[1] + half)
                cv.rectangle(debug_img, tl, br, (255,), 1)

                endpnt_box = np.zeros_like(contour_img)
                endpnt_box[tl[1]:br[1], tl[0]:br[0]] = contour_img[tl[1]:br[1], tl[0]:br[0]]
                box_points = cv.findNonZero(endpnt_box)

                vx, vy, x, y = cv.fitLine(box_points, cv.DIST_L2, 0, 0.01, 0.01).flatten()
                new_endpnt = tuple(map(int, intersect_point_line(endpnt, (x, y), (0, -x*vy/vx+y))))

                distance = 20
                point_a = (int(new_endpnt[0] + distance * vx), int(new_endpnt[1] + distance * vy))
                point_b = (int(new_endpnt[0] - distance * vx), int(new_endpnt[1] - distance * vy))

                avg_pnt = np.mean(box_points, axis=0)[0].astype(np.uint64)
                points = np.array([point_a, point_b])
                point = tuple(points[np.argmax([np.linalg.norm(avg_pnt - pnt) for pnt in points])])
                vx, vy = [-vx, -vy] if point == point_b else [vx, vy]

                cv.circle(debug_img, new_endpnt, 2, (0, 255,))
                endpnt_line.append([new_endpnt, point, vx, vy])

    have_connection = np.array([False] * len(endpnt_line))
    for i, curr_endpnt_line in enumerate(endpnt_line):
        for j, comp_contour_lines in enumerate(endpnt_line[i+1:]):
            if connectable(curr_endpnt_line, comp_contour_lines):
                have_connection[i] = True
                have_connection[i+j+1] = True
                cv.line(debug_img, curr_endpnt_line[0], comp_contour_lines[0], (255, 0, 255))
        if not have_connection[i]:
            cv.line(debug_img, curr_endpnt_line[0], curr_endpnt_line[1], (0, 0, 255))

    cv.imshow('Image', debug_img)
    cv.waitKey(1)

def update_wall_map(map):
    img = np.reshape(map.data, (map.info.height, map.info.width)).astype(np.uint8)
    img = np.flipud(np.rot90(img, 3))

    show_fake_walls(img)

def listener():
    rospy.init_node('map_closer', anonymous=True)
    rospy.Subscriber("/basic_mapper/costmap/costmap", OccupancyGrid, update_wall_map)
    rospy.spin()

if __name__ == '__main__':
    listener()