#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 10 11:28:32 2024

@author: owlnuc13
"""

import socket
import json
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys

# Define global variables
global_width = 0
global_length = 0
global_height = 0
global_pallet_height = 0
global_pallet_width = 0
global_clearance = 0

def store_data(data):
    global global_width, global_length, global_height, global_pallet_height, global_pallet_width, global_clearance
    # Store the six integers in the global variables
    global_width, global_length, global_height,_, global_pallet_height, global_pallet_width, global_clearance = data



def connectETController(ip, port=8055):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((ip, port))
        return (True, sock)
    except Exception as e:
        sock.close()
        return (False, sock)  # Return the socket even in case of failure

def disconnectETController(sock):
    if sock:
        sock.close()
        sock = None
    else:
        sock = None

def sendCMD(sock, cmd, params=None, id=1):
    if not params:
        params = []
    else:
        params = json.dumps(params)
    sendStr = "{{\"method\":\"{0}\",\"params\":{1},\"jsonrpc\":\"2.0\",\"id\":{2}}}".format(cmd, params, id) + "\n"
    try:
        sock.sendall(bytes(sendStr, "utf-8"))
        ret = sock.recv(1024)
        jdata = json.loads(str(ret, "utf-8"))
        if "result" in jdata.keys():
            return (True, json.loads(jdata["result"]), jdata["id"])
        elif "error" in jdata.keys():
            return (False, jdata["error"], jdata["id"])
        else:
            return (False, None, None)
    except Exception as e:
        return (False, None, None)

def calculate_inverse_kinematics(robot_ip, target_pose):
    P000 = [0, -90, 90, -90, 90, 0]

    conSuc, sock = connectETController(robot_ip)

    if conSuc:
        try:
            suc, result, id = sendCMD(sock, "inverseKinematic", {"targetPose": target_pose, "referencePos": P000})
            if suc:
                return result
            else:
                print("Inverse kinematics failed for this pose:", result)
                return None

        except Exception as e:
            print("Error:", e)
        finally:
            disconnectETController(sock)
    else:
        print("Connection to the robot failed.")
        return None

def movel(robot_ip, points):
    conSuc, sock = connectETController(robot_ip)

    if not conSuc:
        return None

    try:
        suc, result, id = sendCMD(sock, "set_servo_status", {"status": 1})
        angle_point = calculate_inverse_kinematics(robot_ip, points)

        print("Moving to point linearly:", angle_point)

        if angle_point is None:
            print("Error: Target position for linear motion is invalid.")
            return

        suc, result, id = sendCMD(sock, "moveByLine", {
            "targetPos": angle_point,
            "speed_type": 0,
            "speed": 200,
            "cond_type": 0,
            "cond_num": 7,
            "cond_value": 1})

        if not suc:
            print("Error in moveByLine:", result)
            return

        while True:
            suc, result, id = sendCMD(sock, "getRobotState")
            if result == 0:
                break

    except Exception as e:
        print("Error in moveRobotToPoint:", e)
    finally:
        disconnectETController(sock)

def movec(robot_ip, points, nexpoints):
    conSuc, sock = connectETController(robot_ip)

    if not conSuc:
        return None

    try:
        suc, result, id = sendCMD(sock, "set_servo_status", {"status": 1})
        P000 = calculate_inverse_kinematics(robot_ip, points)
        P001 = calculate_inverse_kinematics(robot_ip, nexpoints)

        print("Moving to point angular:", P000)
        print("Target point angular:", P001)

        suc, result, id = sendCMD(sock, "moveByArc", {
            "midPos": P000,
            "targetPos": P001,
            "speed_type": 0,
            "speed": 150,
            "cond_type": 0,
            "cond_num": 7,
            "cond_value": 1
        })

        if not suc:
            print("Error in moveByArc:", result)
            return

        while True:
            suc, result, id = sendCMD(sock, "getRobotState")
            if result == 0:
                break

    except Exception as e:
        print("Error in moveRobotToPoint:", e)
    finally:
        disconnectETController(sock)

def getMasterPoint(robot_ip):
    conSuc, sock = connectETController(robot_ip)
    num_layers = int(input("Enter the number of layers: "))
    input("Press enter after reaching desired master location and Switch robot to Remote mode")

    if conSuc:
        suc, result, id = sendCMD(sock, "get_tcp_pose", {"coordinate_num": 2, "tool_num": 4})
        print(result)

    input("Press enter after reaching desired Transfer location ")
    if conSuc:
        suc, transfer, id = sendCMD(sock, "get_tcp_pose", {"coordinate_num": 2, "tool_num": 4})
        print(transfer)
    input("Press enter after reaching desired pickup location ")
    if conSuc:
        suc, pickup, id = sendCMD(sock, "get_tcp_pose", {"coordinate_num": 2, "tool_num": 4})
        suc, resultt, id = sendCMD(sock, "setSpeed", {"value": 90})
        print(pickup)

    return pickup, transfer, result

def offsetPoses(master_point, target_poses):
    offset_poses = []
    for pose in target_poses:
        offset_pose = [master_point[i] + pose[i] if i < 2 else master_point[i] for i in range(6)]
        if pose[2] == 1:
            offset_pose[5] += 1.5708  # Add 1.5708 to the sixth value
        offset_poses.append(offset_pose)
        print(offset_pose)
    return offset_poses

def turn_on_digital_io(robot_ip, io_number):
    conSuc, sock = connectETController(robot_ip)
    
    if conSuc:
        try:
            suc, result, id = sendCMD(sock, "setOutput", {"addr": io_number, "status": 1})
            if suc:
                print("Digital I/O {} turned on.".format(io_number))
                return True
            else:
                print("Failed to turn on digital I/O {}:".format(io_number), result)
                return False
        
        except Exception as e:
            print("Error:", e)
            return False
        finally:
            disconnectETController(sock)
    else:
        print("Connection to the robot failed.")
        return False

def turn_off_digital_io(robot_ip, io_number):
    conSuc, sock = connectETController(robot_ip)
    
    if conSuc:
        try:
            suc, result, id = sendCMD(sock, "setOutput", {"addr": io_number, "status": 0})
            if suc:
                print("Digital I/O {} turned on.".format(io_number))
                return True
            else:
                print("Failed to turn on digital I/O {}:".format(io_number), result)
                return False
        
        except Exception as e:
            print("Error:", e)
            return False
        finally:
            disconnectETController(sock)
    else:
        print("Connection to the robot failed.")
        return False

def can_place_box(x, y, box_width, box_length, pallet_size, clearance, placed_items, overhang):
    if overhang:
        if (x + box_width + clearance > pallet_size[0] + overhang[0] or 
            y + box_length + clearance > pallet_size[1] + overhang[1]):
            return False
    else:
        if (x + box_width + clearance > pallet_size[0] or 
            y + box_length + clearance > pallet_size[1]):
            return False
    return not any(
        (x - clearance < px + pw and x + box_width + clearance > px and 
         y - clearance < py + ph and y + box_length + clearance > py)
        for px, py, pw, ph, _ in placed_items
    )

def find_next_position(pallet_size, box_dim, clearance, placed_items, overhang):
    for y in range(pallet_size[1] - box_dim[1] + 1):
        for x in range(pallet_size[0] - box_dim[0] + 1):
            if can_place_box(x, y, box_dim[0], box_dim[1], pallet_size, clearance, placed_items, overhang):
                return (x, y)
    return None

def optimize_box_placement(box_dim, pallet_size, clearance, overhang):
    placed_items = []
    # Calculate total area covered by placed items for skipping occupied areas
    total_covered_area = sum(item[2] * item[3] for item in placed_items)
    
    # Bounding box heuristic
    if (box_dim[0] > pallet_size[0] or box_dim[1] > pallet_size[1]):
        print("Box is larger than the pallet. Cannot fit.")
        return placed_items
    
    # Start from the top-left corner of the pallet
    y = 0
    while y < pallet_size[1] - box_dim[1] + 1:
        x = 0
        while x < pallet_size[0] - box_dim[0] + 1:
            # Skip if this position overlaps with already placed items
            if any(
                (x - clearance < px + pw and x + box_dim[0] + clearance > px and 
                 y - clearance < py + ph and y + box_dim[1] + clearance > py)
                for px, py, pw, ph, _ in placed_items
            ):
                x += 1
                continue
            
            # Check if this position can accommodate the box
            if overhang:
                if (x + box_dim[0] + clearance > pallet_size[0] + overhang[0] or 
                    y + box_dim[1] + clearance > pallet_size[1] + overhang[1]):
                    x += 1
                    continue
            else:
                if (x + box_dim[0] + clearance > pallet_size[0] or 
                    y + box_dim[1] + clearance > pallet_size[1]):
                    x += 1
                    continue
            
            # Found a valid position for the box
            placed_items.append((x, y, box_dim[0], box_dim[1], 0))
            total_covered_area += box_dim[0] * box_dim[1]
            x += 1
        
        # Move to the next row
        y += 1

    return placed_items


def get_center_coordinates(placed_items):
    center_coordinates = []
    for item in placed_items:
        center_x = item[0] + item[2] / 2
        center_y = item[1] + item[3] / 2
        center_coordinates.append((center_x, center_y, item[4]))
    return center_coordinates

def plot_pallet(pallet_size, items, clearance):
    fig, ax = plt.subplots()
    pallet = patches.Rectangle((0, 0), pallet_size[0], pallet_size[1], edgecolor='black', facecolor='none')
    ax.add_patch(pallet)

    for item in items:
        rect = patches.Rectangle((item[0], item[1]), item[2], item[3], edgecolor='blue', facecolor='lightblue')
        ax.add_patch(rect)

    for item in items:
        clearance_rect = patches.Rectangle((item[0] - clearance, item[1] - clearance), 
                                            item[2] + 2 * clearance, item[3] + 2 * clearance, 
                                            edgecolor='red', facecolor='none')
        ax.add_patch(clearance_rect)

    center_coordinates = get_center_coordinates(items)
    for center in center_coordinates:
        plt.scatter(center[0], center[1], color='green', marker='x')

    plt.xlim(0, pallet_size[0])
    plt.ylim(0, pallet_size[1])
    plt.gca().set_aspect('equal', adjustable='box')
    
    # Turn on interactive mode
    plt.ion()
    plt.show()
    plt.pause(0.001)  # Pause for a short time to update the plot


if __name__ == "__main__":
    robot_ip = "192.168.1.200"

    data = list(map(int, sys.argv[1:]))

    # Store the data in global variables
    store_data(data)

    # User input for box dimensions
    box_width = global_width
    box_length = global_length
    box_height = global_height

    # User input for pallet size (width, height)
    pallet_width = global_pallet_width
    pallet_height = global_pallet_height

    # User input for clearance
    clearance = global_clearance
    # User input for overhang
    overhang_option = 'n'
    if overhang_option.lower() == 'y':
        overhang_length = int(input("Enter the overhang in MM: "))
        overhang_width = overhang_length
        overhang = (overhang_length, overhang_width)
    else:
        overhang = None

    # Adjust pallet size with overhang
    if overhang:
        pallet_width += overhang[0]
        pallet_height += overhang[1]
    # User input for number of layers
    num_layers = int(input("Enter the number of layers: "))
    
    # Optimize box placement
    placed_items = optimize_box_placement((box_width, box_length), (pallet_width, pallet_height), clearance, overhang)

    # Plot the pallet with items and center coordinates
    plot_pallet((pallet_width, pallet_height), placed_items, clearance)

    # Get the master point and pickup point from the user
    pickup_point, transfer_point, master_point = getMasterPoint(robot_ip)
    
    # Output the number of boxes that could be placed
    print(f"Number of boxes that could be placed: {num_layers * len(placed_items)}")

    # Calculate offset poses based on the master point
    offset_height = [0, 0, box_height, 0, 0, 0]
    clearence = [0, 0, 10, 0, 0, 0]  # height clearance would be from how much above box should be released
    offset_poses = offsetPoses(master_point, placed_items)
    con_success, sock = connectETController(robot_ip)

    if con_success:
        try:
            for num_layer in range(1, num_layers + 1):  # Start from 1 and go until num_layers
                for pose in offset_poses:
                    place = [x + num_layer * y + c for x, y, c in zip(pose, offset_height, clearence)]
                    pre_place = [x + y for x, y in zip(place, offset_height)]
                    n_pickup_point = [x + y  for x, y in zip(pickup_point, offset_height)]
                    n_pre_pickup = [x + 2*y for x, y in zip(pickup_point, offset_height)]

                    movel(robot_ip, n_pre_pickup)
                    movel(robot_ip, n_pickup_point)
                    turn_on_digital_io(robot_ip, 2)
                    movel(robot_ip, n_pre_pickup)
                    movec(robot_ip, transfer_point, pre_place)
                    movel(robot_ip, place)
                    turn_off_digital_io(robot_ip, 2)
                    movel(robot_ip, pre_place)
                    movec(robot_ip, transfer_point, n_pre_pickup)

            print("All poses reached successfully.")

        except Exception as e:
            print("Error:", e)
        finally:
            disconnectETController(sock)
    else:
        print("Connection to the robot failed.")

