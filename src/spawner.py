import subprocess 
 
def marker_position(): 
    pass 
 
def launch_simulation(pick_tray_coords, place_tray_coords): 
    # Unpack the coordinates 
    pick_tray_x, pick_tray_y, pick_tray_z = pick_tray_coords 
    place_tray_x, place_tray_y, place_tray_z = place_tray_coords 
 
    # Construct the command 
    command = [ 
        "roslaunch", "panda_moveit_config", "demo_robot.launch", 
        "pick_tray_x:={}".format(pick_tray_x), 
        "pick_tray_y:={}".format(pick_tray_y), 
        "pick_tray_z:={}".format(pick_tray_z), 
        "place_tray_x:={}".format(place_tray_x), 
        "place_tray_y:={}".format(place_tray_y), 
        "place_tray_z:={}".format(place_tray_z) 
    ] 
 
    # Execute the command 
    subprocess.run(command) 
 
 
# Example usage 
pick_tray_coords = (0.5, -0.25, 0.4) 
place_tray_coords = (0.5, 0.25, 0.4) 
 
launch_simulation(pick_tray_coords, place_tray_coords) 
 
#0.508578 -y -0.215704 -z 0.419773