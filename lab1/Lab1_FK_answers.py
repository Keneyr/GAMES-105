import numpy as np
from scipy.spatial.transform import Rotation as R

def load_motion_data(bvh_file_path):
    """part2 辅助函数，读取bvh文件"""
    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            if lines[i].startswith('Frame Time'):
                break
        motion_data = []
        for line in lines[i+1:]:
            data = [float(x) for x in line.split()]
            if len(data) == 0:
                break
            motion_data.append(np.array(data).reshape(1,-1))
        motion_data = np.concatenate(motion_data, axis=0)
    return motion_data



def part1_calculate_T_pose(bvh_file_path):
    """请填写以下内容
    输入： bvh 文件路径
    输出:
        joint_name: List[str]，字符串列表，包含着所有关节的名字
        joint_parent: List[int]，整数列表，包含着所有关节的父关节的索引,根节点的父关节索引为-1
        joint_offset: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的偏移量

    Tips:
        joint_name顺序应该和bvh一致
    """
    joint_names = []
    joint_parents = []
    joint_offsets = []
    # use list to simulate stack
    stk = []
    # cur parent joint name
    parent_name = None
    # cur joint name
    joint_name = None
    with open(bvh_file_path) as f:
        for line in f:
            if line.startswith("ROOT"):
                parent_name = line.split()[1]
                stk.append(parent_name)
                joint_names.append(parent_name)
                joint_parents.append(-1)
            elif ("JOINT" in line) or ("End Site" in line):
                if ("JOINT" in line):
                    joint_name = line.split()[1]
                elif ("End Site" in line):
                    joint_name = parent_name + "_end"
                stk.append(joint_name)
                joint_names.append(joint_name)
                joint_parents.append(joint_names.index(parent_name))
                parent_name = joint_name
            elif ("OFFSET" in line):
                data = [float(x) for x in line.split()[1:]]
                if joint_offsets == []:
                    joint_offsets = np.array(data)
                else:
                    joint_offsets = np.vstack((joint_offsets, np.array(data)))
            elif ("}" in line):
                # pop stack, change cur parent joint name
                stk.pop()
                # hierarchy end
                if stk == []:
                    break
                parent_name = stk[-1] #the last item of the list
    # debug, write the motion_data to file hierachy.txt
    np.savetxt("results/debug/hierachy_offset.txt", joint_offsets)
    with open("results/debug/hierachy_names.txt", 'w') as fp:
        fp.write('\n'.join(joint_names))
    with open("results/debug/hierachy_parents.txt", 'w') as fp:
        fp.write('\n'.join(str(v) for v in joint_parents))
    return joint_names, joint_parents, joint_offsets


def part2_forward_kinematics(joint_name, joint_parent, joint_offset, motion_data, frame_id):
    """请填写以下内容
    输入: part1 获得的关节名字，父节点列表，偏移量列表
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数
        frame_id: int，需要返回的帧的索引
    输出:
        joint_positions: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的全局位置
        joint_orientations: np.ndarray，形状为(M, 4)的numpy数组，包含着所有关节的全局旋转(四元数)
    Tips:
        1. joint_orientations的四元数顺序为(x, y, z, w)
        2. from_euler时注意使用大写的XYZ
    """
    # quaterninon: (cos 1/2 theta, sin 1/2 theta v), around v axis (i,j,k) rotate theta
    # initialize
    joint_positions = np.zeros((len(joint_name), 3))
    joint_orientations = np.zeros((len(joint_name), 4))
    # j is the iterative indx for motion data, since end effectors don't have any motion data, but occupy indexs in joint_name, joint_offset and joint_parent
    j = 0
    # iterate all the joints, update their pos and ori in order
    for i in range(len(joint_name)):
        parent_idx = joint_parent[i]
        # root has 6 channels: translation and rotation
        if i == 0:
            joint_positions[i] = motion_data[frame_id, 0:3]
            joint_orientations[i] = R.from_euler('XYZ', motion_data[frame_id, 3:6], degrees=True).as_quat()
        # end sites don't have channels, only need to calculate it's new position
        elif "_end" in joint_name[i]:
            # new global end effector postiion: x = p_E + Q_Ex_0
            parent_orientation = R.from_quat(joint_orientations[parent_idx]).as_matrix()
            joint_positions[i] = joint_positions[parent_idx] + np.dot(parent_orientation, joint_offset[i])
            # end effectors dont's need orientation, so the joint_orientation element in this idx is zero
            j += 1
        else:
            # new global joint position: p_i+1 = p_i + Q_iL_i
            parent_orientation = R.from_quat(joint_orientations[parent_idx]).as_matrix()
            joint_positions[i] = joint_positions[parent_idx] + np.dot(parent_orientation, joint_offset[i])
            # new global joint orientation: Q_i = Q_i-1R_i
            joint_rotation = R.from_euler('XYZ', motion_data[frame_id, (i+1-j)*3 : (i+1-j)*3+3], degrees=True).as_matrix()
            joint_orientation = np.dot(parent_orientation, joint_rotation)
            joint_orientations[i] = R.from_matrix(joint_orientation).as_quat()
    return joint_positions, joint_orientations


def part3_retarget_func(T_pose_bvh_path, A_pose_bvh_path):
    """
    将 A-pose的bvh重定向到T-pose上
    输入: 两个bvh文件的路径
    输出: 
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数。retarget后的运动数据
    Tips:
        两个bvh的joint name顺序可能不一致哦(
        as_euler时也需要大写的XYZ
    """
    # load motion data under A-pose
    A_motion_data = load_motion_data(A_pose_bvh_path)
    # get the frame num of this A-pose motion data
    frame_num = A_motion_data.shape[0]
    # since this function is mapping A-pose motion to T-pose motion, so this motion_data should have T-pose joints order
    motion_data = np.zeros((frame_num, A_motion_data.shape[1]))
    # get skeleton hierarchy under A-pose and T-pose 
    A_joint_names, A_joint_parents, A_joint_offsets = part1_calculate_T_pose(A_pose_bvh_path)
    T_joint_names, _, T_joint_offsets= part1_calculate_T_pose(T_pose_bvh_path)
    # use hash to record the joint name idx mapping from A-pose to T-pose, suppose the joint names and parent-child relation is the same
    assert len(A_joint_names) == len(T_joint_names)
    A2TMapping = {}
    for i in range(len(A_joint_names)):
        j = T_joint_names.index(A_joint_names[i])
        #debug
        print("A joint idx : " + str(i) + ", joint name: " + A_joint_names[i] + ", T joint idx : " + str(j))
        A2TMapping[i] = j
    # calculate the joint pos vecotr under reference pose in local coordinate system
    Q_A2T = np.zeros((len(A_joint_names), 3))
    for i in range(len(A_joint_names)):
        A_joint_offset = A_joint_offsets[i]
        T_joint_offset = T_joint_offsets[A2TMapping[i]]
        # debug
        print(str(i) + "-->" + str(A2TMapping[i]) + ":" + A_joint_names[i])
        print(A_joint_offset)
        print(T_joint_offset)
        # given two 3d vectors A and B, calculate the rotation matrix from A to B
        r, _, = R.align_vectors(np.atleast_2d(T_joint_offset), np.atleast_2d(A_joint_offset))
        if i != 0:
            Q_A2T[A_joint_parents[i]] = r.as_euler('XYZ', degrees=True)
    # debug, write the motion_data to file T_pose_run.txt
    np.savetxt("results/debug/Q_A2T.txt", Q_A2T)
    # presum, record the end effector joints num before this joint in T_pose
    preSum = [0 for i in range(len(T_joint_names))]
    for idx, joint_name in enumerate(T_joint_names):
        if idx != 0:
            preSum[idx] = preSum[idx-1]
        if "_end" in joint_name:
            preSum[idx] = preSum[idx-1] + 1
    # iterate each frame of A-pose motion data, transform them into T_pose motion data
    for frame_idx in range(frame_num):
        j = 0
        for joint_idx in range(len(A_joint_names)):
            if joint_idx == 0:
                # root translation
                motion_data[frame_idx, A2TMapping[joint_idx] : A2TMapping[joint_idx] + 3] = A_motion_data[frame_idx, 0:3]
                # root rotation, no need to convert translation between A pose and T pose
                # R_pi^B = R_pi^A (Q_pi_A2B)^T
                R_rootA = R.from_euler('XYZ', A_motion_data[frame_idx, 3:6], degrees=True).as_matrix()
                R_rootB = np.dot(R_rootA, np.transpose(R.from_euler('XYZ', Q_A2T[0], degrees=True).as_matrix()))
                motion_data[frame_idx, A2TMapping[joint_idx] + 3 : A2TMapping[joint_idx] + 6] = R.from_matrix(R_rootB).as_euler('XYZ', degrees=True)
            elif "_end" in A_joint_names[joint_idx]:
                # end effector joints, don't have motion data
                j += 1
            else:
                # normal joints, only have rotation
                T_joint_idx = A2TMapping[joint_idx]
                A_Rotation = A_motion_data[frame_idx, (joint_idx+1-j)*3:(joint_idx+1-j)*3+3]
                # R_i^B = Q_pi^A2B R_i^A (Q_i_A2B)^T
                R_A = R.from_euler('XYZ', A_Rotation, degrees=True).as_matrix()
                Q_pi_A2T = R.from_euler('XYZ', Q_A2T[A_joint_parents[joint_idx]], degrees=True).as_matrix()
                Q_i_A2T = R.from_euler('XYZ', Q_A2T[joint_idx], degrees=True).as_matrix()
                R_B =  np.matmul(np.matmul(Q_pi_A2T, R_A), np.transpose(Q_i_A2T))
                #R_B =  np.matmul(R_A, np.transpose(Q_i_A2T))
                motion_data[frame_idx, (T_joint_idx+1-preSum[T_joint_idx])*3 : (T_joint_idx+1-preSum[T_joint_idx])*3 + 3] = R.from_matrix(R_B).as_euler('XYZ', degrees=True)
    # debug, write the motion_data to file T_pose_run.txt
    np.savetxt("results/debug/T_pose_run.txt", motion_data)
    return motion_data
