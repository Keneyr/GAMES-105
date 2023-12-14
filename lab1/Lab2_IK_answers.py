import numpy as np
from scipy.spatial.transform import Rotation as R
import torch

# get the rotation as quterninon form from aligning vector v_1 to v_2
def get_rotation_scipy(v_1, v_2):
    r, _, = R.align_vectors(np.atleast_2d(v_2), np.atleast_2d(v_1))
    return r

def inv_safe(data):
    matrix = None
    # return R.from_quat(data).inv()
    if np.allclose(data, [0, 0, 0, 0]):
        matrix = np.eye(3) 
    else:
        matrix = np.linalg.inv(R.from_quat(data).as_matrix())
    return R.from_matrix(matrix).as_quat()
    
def from_quat_safe(data):
    # return R.from_quat(data)
    matrix = None
    if np.allclose(data, [0, 0, 0, 0]):
        matrix = np.eye(3)
    else:
        matrix = R.from_quat(data).as_matrix()
    return R.from_matrix(matrix).as_quat()

def part1_inverse_kinematics_Jacobian_DLS(path,target_pose,chain_positions,chain_orientations,chain_relative_rotations,chain_relative_offsets): 
    """
        Jacobian Damped_Least_Squares Method: Levenberg-Marquardt algorithm
        Input: 
            path: IK chain joint id, from static root joint to end effectr joint
            target_pose: the tartget position for the end effector joint
            chain_positions: the positions(under absoluate coordinate system) for the joints in this IK chain
            chain_orientations: the orientation(under absoluate coordinate system) for joints in this chain
            chain_relative_rotations: R_1 = Q_0^{-1}Q_1, local rotation transformation between two orientation coordinate systems
            chain_relative_offsets: relative offsets, the same with bvh file
        Output:
            pose after Ik solver
            joint_positions: 
            joint_orientations: 
    """

    """
    theta = theta - alpha * J^T(JJ^T + lambda I )^{-1} (current_position - target_position)
    """
    pass

def part1_inverse_kinematics_Jacobian_PseudoInverse(path,target_pose,chain_positions,chain_orientations,chain_relative_rotations,chain_relative_offsets):
    """
        Jacobian PseudoInverse Method: Moore-Penrose PseudoInverse
        Input: 
            path: IK chain joint id, from static root joint to end effectr joint
            target_pose: the tartget position for the end effector joint
            chain_positions: the positions(under absoluate coordinate system) for the joints in this IK chain
            chain_orientations: the orientation(under absoluate coordinate system) for joints in this chain
            chain_relative_rotations: R_1 = Q_0^{-1}Q_1, local rotation transformation between two orientation coordinate systems
            chain_relative_offsets: relative offsets, the same with bvh file
        Output:
            pose after Ik solver
            joint_positions: 
            joint_orientations: 
    """
    # the error threshold is 0.01
    DISTANCE_THRESHOLD = 0.0001
    # the max iterative steps
    STEPS_THRESHOLD = 20
    steps = 0
    learning_rate = 0.1
    
    # change numpy to pytorch data structure
    rotation_chain_tensor = torch.tensor(chain_relative_rotations, requires_grad = True, dtype = torch.float32)
    offset_chain_tensor = torch.tensor(chain_relative_offsets, requires_grad = False, dtype = torch.float32)
    target_position = torch.tensor(target_pose, requires_grad=False, dtype=torch.float32)
    
    """
    F(theta) = 1/2 ||f(theta) - target_position||^2
    Jacobian = gradient_F
    theta = theta - learning_rate * J^T (JJ^T)^{-1} * (current_position - target_position)

    !!!!theta means the relative_local_rotation for each joint, we can use Quaternion form, f(theta) is the Forward Kinematics Function !!!

    you can change a vector v to pure quaternion [0, v]
    """
    while steps < STEPS_THRESHOLD:
        pass
    return chain_positions, chain_orientations, chain_relative_rotations

def part1_inverse_kinematics_CCD(path,target_pose,chain_positions,chain_orientations,chain_relative_rotations,chain_relative_offsets):
    """
        CCD Solver
        Input: 
            path: IK chain joint id, from static root joint to end effectr joint
            target_pose: the tartget position for the end effector joint
            chain_positions: the positions(under absoluate coordinate system) for the joints in this IK chain
            chain_orientations: the orientation(under absoluate coordinate system) for joints in this chain
            chain_relative_rotations: R_1 = Q_0^{-1}Q_1, local rotation transformation between two orientation coordinate systems
            chain_relative_offsets: relative offsets, the same with bvh file
        Output:
            pose after Ik solver
            joint_positions: 
            joint_orientations: 
    """
    # the error threshold is 0.01
    DISTANCE_THRESHOLD = 0.0001
    # the max iterative steps
    STEPS_THRESHOLD = 20
    steps = 0
    # calculating Euclidean distance from end effector to target_pose
    distance = np.linalg.norm(chain_positions[-1] - target_pose)
    # CCD Iterative
    while distance > DISTANCE_THRESHOLD and steps < STEPS_THRESHOLD:
        # from outmost to inner, update the orientation paramter, excluding the end effector joint
        for idx in range(len(path)-2, 0, -1): 
            # align the vector from current joint to end_effector, to the vector from current joint to target_pose, toEnd to toTarget
            # r = get_rotation_scipy(chain_positions[-1] - chain_positions[idx], target_pose - chain_positions[idx])
            toEnd = chain_positions[-1] - chain_positions[idx]
            toTarget = target_pose - chain_positions[idx]
            rotate_axis = np.cross(toEnd, toTarget)
            rotate_axis = rotate_axis / np.linalg.norm(rotate_axis)
            cos = min(np.dot(toEnd, toTarget) / (np.linalg.norm(toEnd) * np.linalg.norm(toTarget)), 1.0)
            theta = np.arccos(cos)
            # It's already optimal 
            if theta < 0.0001:
                continue
            deltaRotation = R.from_rotvec(theta * rotate_axis)
            # Attention：the order in dot，deltaRotation * orientation, since deltaRotation is calculated under global coordinate system
            chain_orientations[idx] = (deltaRotation * R.from_quat(chain_orientations[idx])).as_quat()
            # print(chain_orientations[idx])
            # 下面这个写法也可以
            # ori_matrix = np.dot(deltaRotation.as_matrix(), R.from_quat(chain_orientations[idx]).as_matrix())
            # print(R.from_matrix(ori_matrix).as_quat())
            
            # update the rotation transformation relative to parent joint
            #chain_relative_rotations[idx] = (R.from_quat(chain_orientations[idx-1]).inv() * R.from_quat(chain_orientations[idx])).as_quat()
            chain_relative_rotations[idx] = (R.from_quat(inv_safe(chain_orientations[idx-1])) * R.from_quat(chain_orientations[idx])).as_quat()
            # articulate(update) the child joints of cur joint global positions and orientations by chain structure
            for child_idx in range(idx+1, len(path)):
                # update global position, the same with FK, p_1 = p_0 + Q_0 * L_0
                chain_positions[child_idx] = chain_positions[child_idx-1] + np.dot(R.from_quat(chain_orientations[child_idx-1]).as_matrix(), chain_relative_offsets[child_idx])
                # update global orientation, the same with FK, Q_1 = Q_0 * R_1, in this case, it's equal to r.as_quat()() * chain_orientations[iidx]
                # chain_orientations[child_idx] = (R.from_quat(chain_orientations[child_idx-1]) * R.from_quat(chain_relative_rotations[child_idx])).as_quat()
                # 下面这个写法也可以
                chain_orientations[child_idx] = (deltaRotation * R.from_quat(chain_orientations[child_idx])).as_quat()
        steps = steps+1
        distance = np.linalg.norm(chain_positions[-1] - target_pose)
        print("distance : " + str(distance))
    return chain_positions, chain_orientations, chain_relative_rotations

def part1_inverse_kinematics_FABRIK(path,target_pose,chain_positions,chain_orientations,chain_relative_rotations,chain_relative_offsets):
    pass

def get_joints_relative_rotation(joint_names, joint_parents, joint_orientations):
        joints_relative_rotations = np.empty(joint_orientations.shape)
        for i in range(len(joint_names)):
            # default: RootJoint usually locates in the origin in global coordinate system
            if joint_parents[i] == -1:
                joints_relative_rotations[i] = R.from_euler('XYZ', [0.,0.,0.]).as_quat()
            else:
                # Q_1 = Q_0 * R_1, thus R_1 = Q_0^{-1}Q_1
                #joints_relative_rotations[i] = (R.from_quat(joint_orientations[joint_parents[i]]).inv() * R.from_quat(joint_orientations[i])).as_quat()
                joints_relative_rotations[i] = (R.from_quat(inv_safe(joint_orientations[joint_parents[i]])) * R.from_quat(from_quat_safe(joint_orientations[i]))).as_quat()
        return joints_relative_rotations

def get_joints_offsets(joint_names, joint_parents, joint_positions, joint_initial_position):
        joint_relative_offsets = np.empty(joint_positions.shape)
        for i in range(len(joint_names)):
            if joint_parents[i] == -1:
                joint_relative_offsets[i] = np.array([0.,0.,0.])
            else:
                joint_relative_offsets[i] = joint_initial_position[i] - joint_initial_position[joint_parents[i]]
        return joint_relative_offsets

def part1_inverse_kinematics(meta_data, joint_positions, joint_orientations, target_pose):
    """
    完成函数，计算逆运动学
    输入: 
        meta_data: 为了方便，将一些固定信息进行了打包，见上面的meta_data类
        joint_positions: 当前的关节位置，是一个numpy数组，shape为(M, 3)，M为关节数
        joint_orientations: 当前的关节朝向，是一个numpy数组，shape为(M, 4)，M为关节数
        target_pose: 目标位置，是一个numpy数组，shape为(3,)
    输出:
        经过IK后的姿态
        joint_positions: 计算得到的关节位置，是一个numpy数组，shape为(M, 3)，M为关节数
        joint_orientations: 计算得到的关节朝向，是一个numpy数组，shape为(M, 4)，M为关节数
    """
    # the elements order in list path is from inner to outmost, from the static joint to the end effector joint
    path, path_name, path1, path2 = meta_data.get_path_from_root_to_end()
    
    """
    step1: init bone data
    """
    joint_names = meta_data.joint_name
    joint_parents = meta_data.joint_parent
    joint_initial_position = meta_data.joint_initial_position
    # calculate the relative rotation matrix between joints orientation coordinate system, and relative offsets of each joints
    joint_relative_offsets = get_joints_offsets(joint_names, joint_parents, joint_positions, joint_initial_position)
    joint_relative_rotations = get_joints_relative_rotation(joint_names, joint_parents, joint_orientations)
    
    """
    step2: init chain
    """
    chain_positions = np.empty((len(path), 3))
    chain_orientations = np.empty((len(path), 4))
    # in this IK chain, joint orientation relative to it's parent(parent in IK chain) joint orientaion Q_ik_parent * rr_ik = Q_ik_joint
    chain_relative_rotations = np.empty((len(path), 4))
    chain_relative_offsets = np.empty((len(path), 3))

    # for static joint, it's position and orientation should not change
    chain_positions[0] = joint_positions[path[0]]
    chain_orientations[0] = joint_orientations[path[0]]
    # just suppose this static joint is located at the origin(0,0,0) for this IK chain
    chain_relative_offsets[0] = np.array([0.0, 0.0, 0.0])
    chain_relative_rotations[0] = joint_relative_rotations[path[0]]
    for i in range(1, len(path)):
        joint_id = path[i]
        chain_positions[i] = joint_positions[joint_id]
        chain_orientations[i] = joint_orientations[joint_id]
        if joint_id in path2:
            chain_relative_offsets[i] = -joint_relative_offsets[path[i-1]]
            # in this chain, for lKnee, it's rotation trasformation relative to lAnkle, is the inverse of rotation transformation lAnkle relative to lKnee
            chain_relative_rotations[i] = (R.from_quat(joint_relative_rotations[path[i-1]]).inv()).as_quat()
            #chain_relative_rotations[i] = inv_safe(joint_relative_rotations[path[i-1]])
        else:
            chain_relative_offsets[i] = joint_relative_offsets[joint_id]
            chain_relative_rotations[i] = joint_relative_rotations[joint_id]
    
    # debug: save the init chain to init_chain.txt file
    np.savetxt("results/debug/init_chain_positions.txt", chain_positions)
    np.savetxt("results/debug/init_chain_orientations.txt", chain_orientations)
    np.savetxt("results/debug/init_chain_relative_rotations.txt", chain_relative_rotations)
    np.savetxt("results/debug/init_chain_relative_offsets.txt", chain_relative_offsets)
    
    """
    step3: Solver
    """
    # calculate the final chain pose after IK solver
    #chain_positions, chain_orientations, chain_relative_rotations = part1_inverse_kinematics_CCD(path,target_pose,chain_positions,chain_orientations,chain_relative_rotations,chain_relative_offsets)
    chain_positions, chain_orientations, chain_relative_rotations = part1_inverse_kinematics_Jacobian_PseudoInverse(path,target_pose,chain_positions,chain_orientations,chain_relative_rotations,chain_relative_offsets)
    #chain_positions, chain_orientations, chain_relative_rotations = part1_inverse_kinematics_Jacobian_DLS(path,target_pose,chain_positions,chain_orientations,chain_relative_rotations,chain_relative_offsets)
    chain_positions, chain_orientations, chain_relative_rotations = part1_inverse_kinematics_FABRIK(path,target_pose,chain_positions,chain_orientations,chain_relative_rotations,chain_relative_offsets)
    
    """
    step4: update bone data, index of i-1, and i+1 is becuase this framework...
    """
    # Copy the data from the IK chain to the actual bones
    joint_positions[path[0]] = chain_positions[0]
    joint_orientations[path[0]] = chain_orientations[0]
    joint_relative_rotations[path[0]] = chain_relative_rotations[0]
    for i in range(1, len(path)):
        joint_id = path[i]
        joint_positions[joint_id] = chain_positions[i]
        if joint_id in path2:
            joint_orientations[joint_id] = chain_orientations[i-1]
            joint_relative_rotations[joint_id] = (R.from_quat(chain_orientations[i+1]).inv()).as_quat()
            #joint_relative_rotations[joint_id] = inv_safe(chain_orientations[i+1])
        else:
            joint_orientations[joint_id] = chain_orientations[i]   
            joint_relative_rotations[joint_id] = chain_relative_rotations[i]
    """
    step5: FK Calculation
    """
    for i in range(len(joint_positions)):
        # Root Joint
        if joint_parents[i] == -1:
            continue
        # reduce calculation
        if i in path:
            continue
        # normal joints
        else:
            # p_1 = p_0 + Q_0 * L_0
            joint_positions[i] = joint_positions[joint_parents[i]] + np.dot(R.from_quat(joint_orientations[joint_parents[i]]).as_matrix(), joint_relative_offsets[i])
            # Q_1 = Q_0 * R_1
            joint_orientations[i] = (R.from_quat(joint_orientations[joint_parents[i]]) * R.from_quat(joint_relative_rotations[i])).as_quat()
    return joint_positions, joint_orientations

def part2_inverse_kinematics(meta_data, joint_positions, joint_orientations, relative_x, relative_z, target_height):
    """
    输入lWrist相对于RootJoint前进方向的xz偏移，以及目标高度，IK以外的部分与bvh一致
    """
    # calculate the new effector target_pose
    target_pose = joint_positions[0] + np.array([relative_x, target_height - joint_positions[0][1], relative_z])
    joint_positions, joint_orientations = part1_inverse_kinematics(meta_data, joint_positions, joint_orientations, target_pose)
    
    return joint_positions, joint_orientations

def bonus_inverse_kinematics(meta_data, meta_data_r, joint_positions, joint_orientations, left_target_pose, right_target_pose):
    """
    输入左手和右手的目标位置，固定左脚，完成函数，计算逆运动学
    """
    #joint_positions, joint_orientations = part1_inverse_kinematics(meta_data, joint_positions, joint_orientations, left_target_pose)
    #joint_positions, joint_orientations = part1_inverse_kinematics(meta_data_r, joint_positions, joint_orientations, right_target_pose)
    return joint_positions, joint_orientations