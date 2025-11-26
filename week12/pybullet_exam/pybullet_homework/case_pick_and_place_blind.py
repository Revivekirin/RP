import pybullet as p
import pybullet_data
import time
import numpy as np

def move_joints(robot, joint_indices, target_positions, steps=240, sleep=1/240):
    """Move robot joints smoothly using linear interpolation"""
    zeroPositions = np.array([p.getJointState(robot, j)[0] for j in joint_indices])
    targetPositions = np.array(target_positions)
    
    for step in range(steps):
        alpha = step / steps
        desiredPositions = alpha * targetPositions + (1-alpha) * zeroPositions
        
        p.setJointMotorControlArray(
            bodyIndex=robot,
            jointIndices=joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=desiredPositions,
        )
        p.stepSimulation()
        time.sleep(sleep)

def control_gripper(robot, gripper_joint_indices, target_opening, steps=120, sleep=1/240):
    """Control gripper opening/closing"""
    for _ in range(steps):
        p.setJointMotorControlArray(
            bodyIndex=robot,
            jointIndices=gripper_joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=[target_opening]*len(gripper_joint_indices),
            forces=[2800, 2800]
        )
        p.stepSimulation()
        time.sleep(sleep)

def get_object_height(obj_id):
    """Get the height of an object from its AABB"""
    aabb_min, aabb_max = p.getAABB(obj_id)
    return aabb_max[2] - aabb_min[2]

def get_object_center_of_mass(obj_id):
    """Get the center of mass height offset from base"""
    aabb_min, aabb_max = p.getAABB(obj_id)
    obj_pos, _ = p.getBasePositionAndOrientation(obj_id)
    # Center of mass height relative to object base
    com_height = (aabb_max[2] + aabb_min[2]) / 2 - obj_pos[2]
    return com_height

def pick_and_place(robot, arm_joints, gripper_joints, ee_link, obj_id, target_pos, 
                   grasp_height_offset=0.0, place_rotation=0.0, case_height=0.64, 
                   grasp_rotation=0.0, is_triangle=False):
    """Pick up an object and place it at target position"""
    # Get object position and height
    obj_pos, obj_orn = p.getBasePositionAndOrientation(obj_id)
    obj_height = get_object_height(obj_id)
    com_offset = get_object_center_of_mass(obj_id)
    
    # Calculate optimal grasp height (at center of mass for stability)
    optimal_grasp_height = com_offset if grasp_height_offset == 0.0 else grasp_height_offset
    
    # Open gripper wider for triangle
    gripper_opening = 0.05 if is_triangle else 0.04
    control_gripper(robot, gripper_joints, gripper_opening, steps=60)
    
    # Move above object
    pre_grasp_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + obj_height + 0.1]
    # Apply grasp rotation to approach from correct angle
    target_orn = p.getQuaternionFromEuler([np.pi, 0, grasp_rotation])
    ik_solution = p.calculateInverseKinematics(robot, ee_link, pre_grasp_pos, target_orn, maxNumIterations=200)
    move_joints(robot, arm_joints, ik_solution[:7], steps=200)
    
    # Move down to grasp at center of mass
    grasp_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + optimal_grasp_height]
    ik_solution = p.calculateInverseKinematics(robot, ee_link, grasp_pos, target_orn, maxNumIterations=200)
    move_joints(robot, arm_joints, ik_solution[:7], steps=250)
    
    # Close gripper with more steps for secure grip
    close_steps = 150 if is_triangle else 120
    control_gripper(robot, gripper_joints, 0.0, steps=close_steps)
    
    # Wait for stable grip (longer for triangle)
    wait_frames = 50 if is_triangle else 30
    for _ in range(wait_frames):
        p.stepSimulation()
        time.sleep(1/240)
    
    # Lift object slowly to prevent swinging
    lift_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + 0.25]
    ik_solution = p.calculateInverseKinematics(robot, ee_link, lift_pos, target_orn, maxNumIterations=200)
    lift_steps = 350 if is_triangle else 300
    move_joints(robot, arm_joints, ik_solution[:7], steps=lift_steps, sleep=1/200)
    
    # Move to above target position
    pre_place_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.2]
    place_orn = p.getQuaternionFromEuler([np.pi, 0, place_rotation])
    ik_solution = p.calculateInverseKinematics(robot, ee_link, pre_place_pos, place_orn, maxNumIterations=200)
    move_joints(robot, arm_joints, ik_solution[:7], steps=300)
    
    # Calculate proper place height: case_height + half of object height
    # This ensures the bottom of the object is at case_height
    proper_place_height = case_height + obj_height / 2
    
    # Move down to place
    place_pos = [target_pos[0], target_pos[1], proper_place_height + 0.01]
    ik_solution = p.calculateInverseKinematics(robot, ee_link, place_pos, place_orn, maxNumIterations=200)
    move_joints(robot, arm_joints, ik_solution[:7], steps=400)
    
    # Fine adjustment - go slightly lower to ensure proper placement
    deep_place_pos = [target_pos[0], target_pos[1], proper_place_height - 0.01]
    ik_solution = p.calculateInverseKinematics(robot, ee_link, deep_place_pos, place_orn, maxNumIterations=200)
    move_joints(robot, arm_joints, ik_solution[:7], steps=200, sleep=1/120)
    
    # Open gripper to release slowly
    control_gripper(robot, gripper_joints, gripper_opening, steps=150)
    
    # Wait for object to settle
    for _ in range(20):
        p.stepSimulation()
        time.sleep(1/240)
    
    # Move up after placing
    post_place_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.2]
    ik_solution = p.calculateInverseKinematics(robot, ee_link, post_place_pos, place_orn, maxNumIterations=200)
    move_joints(robot, arm_joints, ik_solution[:7], steps=200)

# Setup PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

p.loadURDF("plane.urdf")
table = p.loadURDF("table/table.urdf", [0.5, 0, 0], useFixedBase=True)
robot = p.loadURDF("franka_panda/panda.urdf", basePosition=[0.0, 0, 0.66], useFixedBase=True)

# Load blocks at initial positions
box_6 = p.loadURDF("models/box6.xacro", basePosition=[0.3, -0.1, 0.66])
box_5 = p.loadURDF("models/box5.xacro", basePosition=[0.3, 0.0, 0.66])
box_4 = p.loadURDF("models/box4.xacro", basePosition=[0.3, 0.1, 0.66])

box_3 = p.loadURDF("models/box3.xacro", basePosition=[0.4, -0.1, 0.66])
cylinder_0 = p.loadURDF("models/cylinder1.xacro", basePosition=[0.4, 0.0, 0.66])
box_2 = p.loadURDF("models/box2.xacro", basePosition=[0.4, 0.1, 0.66])

box_0 = p.loadURDF("models/box.xacro", basePosition=[0.5, 0.0, 0.66])
triangle = p.loadURDF("models/triangle.xacro", basePosition=[0.5, 0.1, 0.66])

# FIXED - useFixedBase=True
case_collision = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName="models/case.obj", 
    flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
    meshScale=[1, 1, 1]
)

case_visual = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName="models/case.obj",
    meshScale=[1, 1, 1]
)

case = p.createMultiBody(
    baseMass=0,  # Set mass to 0 for fixed object
    baseCollisionShapeIndex=case_collision,
    baseVisualShapeIndex=case_visual,
    basePosition=[0.0, 0.3, 0.64],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 3.14159])
)

# Set colors
p.changeVisualShape(box_6, -1, rgbaColor=[1, 0, 1, 1])
p.changeVisualShape(box_5, -1, rgbaColor=[1, 0, 0, 1])
p.changeVisualShape(box_4, -1, rgbaColor=[1, 0, 0, 1])
p.changeVisualShape(box_3, -1, rgbaColor=[1, 0, 0, 1])
p.changeVisualShape(cylinder_0, -1, rgbaColor=[0, 0, 1, 1])
p.changeVisualShape(box_2, -1, rgbaColor=[1, 0, 0, 1])
p.changeVisualShape(box_0, -1, rgbaColor=[1, 0, 0, 1])
p.changeVisualShape(triangle, -1, rgbaColor=[1, 0, 1, 1])

p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0.3, 0, 0.5])

# Robot configuration
arm_joint_indices = list(range(7))
gripper_joint_indices = [9, 10]
ee_link_index = 11

# Home pose
home_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
move_joints(robot, arm_joint_indices, home_pose, steps=100)

# Case height
CASE_HEIGHT = 0.64

# Define pick and place tasks
# Object mapping: (object_id, initial_pos, target_pos, grasp_height_offset, place_rotation, grasp_rotation, is_triangle)
tasks = [
    (box_4, [0.3, 0.1, 0.66], [-0.15, 0.35, 0.70], 0.0, 0.0, 0.0, False),           # 1: box4 (grasp higher)
    (box_5, [0.3, 0.0, 0.66], [-0.15, 0.25, 0.70], 0.0, 0.0, 0.0, False),              # 2: box5
    (box_6, [0.3, -0.1, 0.66], [0.15, 0.25, 0.70], 0.0, np.pi/2, 0.0, False),         # 3: box6 (90도 회전)
    (box_2, [0.4, 0.1, 0.66], [0.05, 0.35, 0.70], 0.0, 0.0, 0.0, False),            # 4: box2 (grasp hig> -0.03, y:0.25->0.27)her)
    (cylinder_0, [0.4, 0.0, 0.66], [-0.03, 0.24, 0.70], 0.0, 0.0, 0.0, False),        # 5: cylinder (x: -0.05 -
    (box_3, [0.4, -0.1, 0.66], [-0.05, 0.35, 0.70], 0.0, 0.0, 0.0, False),            # 6: box3
    (box_0, [0.5, 0.0, 0.66], [0.15, 0.35, 0.70], 0.0, 0.0, 0.0, False),              # 7: box
    (triangle, [0.5, 0.1, 0.66], [0.05, 0.25, 0.70], 0.02, -np.pi/12, np.pi/6, True),       # 8: triangle (grasp_rotation=np.p/6, place_rotation=-np.pi/12)
]

# Execute pick and place for each object
for i, (obj_id, initial_pos, target_pos, height_offset, place_rotation, grasp_rotation, is_triangle) in enumerate(tasks):
    print(f"Processing object {i+1}/{len(tasks)}...")
    pick_and_place(robot, arm_joint_indices, gripper_joint_indices, ee_link_index, 
                   obj_id, target_pos, height_offset, place_rotation, CASE_HEIGHT, 
                   grasp_rotation, is_triangle)
    
    # Return to home pose between tasks
    move_joints(robot, arm_joint_indices, home_pose, steps=200)

print("All objects placed successfully!")

# Final simulation steps
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1/60)

p.disconnect()
