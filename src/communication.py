#!/usr/bin/env python3

import pygame, rospy, math
from pygame.locals import *
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import HomePosition
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import ExtendedState
from mavros_msgs.msg import Altitude

rospy.init_node("polygon")
polygon_publisher = rospy.Publisher("/polygon", Polygon, queue_size=10)

current_state = State()
home_position = HomePosition()
extended_state = ExtendedState()


num_drones = 5  
min_height = 5.0  
max_height = 10.0  

height_interval = (max_height - min_height) / num_drones

def state_cb(msg):
    global current_state
    current_state = msg

def home_cb(msg):
    global home_position
    home_position = msg

def ext_state_cb(msg):
    global extended_state
    extended_state = msg

def set_mode(custom_mode):
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        response = set_mode(0, custom_mode)
        return response.mode_sent
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def arm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        response = arm(True)
        return response.success
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def disarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        response = arm(False)
        return response.success
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def move_to_position(x, y, z):
    global current_state

    if not current_state.armed:
        print("Drone is not armed. Arming...")
        arm()

    if current_state.mode != "OFFBOARD":
        print("Setting mode to OFFBOARD...")
        set_mode("OFFBOARD")

    target_pose = PositionTarget()
    target_pose.header.stamp = rospy.Time.now()
    target_pose.header.frame_id = "map"

    target_pose.position.x = x/10
    target_pose.position.y = y/10
    target_pose.position.z = z

    local_pos_pub.publish(target_pose)

def distance(point_1, point_2):
    return math.sqrt((point_1[0]-point_2[0])**2+(point_1[1]-point_2[1])**2)

background_color = (0, 0, 0)
line_color = (255, 255, 255)
line_width = 5
WIDTH, HEIGHT = 600, 600
frequency = 10
points = []

pygame.init()
logo = pygame.image.load("assets/images/ariitk.jpg")
screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
pygame.display.set_caption("Polygon")
pygame.display.set_icon(logo)

print("Left Click to Add a Point")
print("Right Click to Delete the Nearest Point")
print("Press Backspace on Keyboard to delete the Lastest Added Point")
print("Press 'C' on Keyboard to delete all the points")
print("Points Published on /polygon with message type as geometry_msgs/Polygon")

running = True
rate = rospy.Rate(frequency)

rospy.Subscriber("/mavros/state", State, state_cb)
rospy.Subscriber("/mavros/home_position/home", HomePosition, home_cb)
rospy.Subscriber("/mavros/extended_state", ExtendedState, ext_state_cb)
local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PositionTarget, queue_size=10)

while running and not rospy.is_shutdown():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            break
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_BACKSPACE:
                if len(points) > 0:
                    points.pop()
            if event.key == pygame.K_c:
                points.clear()
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                points.append(pygame.mouse.get_pos())
                if len(points) <= num_drones:
                    x = points[-1][0]
                    y = points[-1][1]
                    z = min_height + (len(points) - 1) * height_interval
                    move_to_position(x, y, z)
            if event.button == 3:
                mouse_position = pygame.mouse.get_pos()
                point_index = -1
                width, height = screen.get_size()
                min_distance = width + height
                for index, point in enumerate(points):
                    current_distance = distance(point, mouse_position)
                    if current_distance < min_distance:
                        min_distance = current_distance
                        point_index = index
                if point_index != -1:
                    points.pop(point_index)

    screen.fill(background_color)
    ros_publisher_points = Polygon()
    for draw_point_index, point in enumerate(points):
        ros_publisher_point = Point32(point[0], point[1], 0)
        ros_publisher_points.points.append(ros_publisher_point)
        pygame.draw.line(screen, line_color, point, points[(draw_point_index+1) % len(points)], line_width)
    polygon_publisher.publish(ros_publisher_points)
    pygame.display.update()

    rate.sleep()

pygame.quit()
