import numpy as np
from scipy.spatial.transform import Rotation as R
from bvh_loader import BVHMotion
from physics_warpper import PhysicsInfo


def part1_cal_torque(pose, physics_info: PhysicsInfo, **kargs):
    '''
    输入： pose： (20,4)的numpy数组，表示每个关节的目标旋转(相对于父关节的)
           physics_info: PhysicsInfo类，包含了当前的物理信息，参见physics_warpper.py
           **kargs：指定参数，可能包含kp,kd
    输出： global_torque: (20,3)的numpy数组，表示每个关节的全局坐标下的目标力矩，根节点力矩会被后续代码无视
    '''
    # ------一些提示代码，你可以随意修改------------#
    kp = kargs.get('kp', 50000) # 需要自行调整kp和kd！ 而且也可以是一个数组，指定每个关节的kp和kd
    kd = kargs.get('kd', 20) 
    parent_index = physics_info.parent_index
    joint_name = physics_info.joint_name
    joint_orientation = physics_info.get_joint_orientation()
    # get each link current angular velocity ω
    joint_avel = physics_info.get_body_angular_velocity()
    global_torque = np.zeros((20,3))
    """
    torque = k_p * (q_target - q) - k_d * q_velocity
    在每个关节的父亲坐标系下计算pd力矩再转换为全局坐标下的。并且为了避免过大的力矩，建议对其进行一定的clip
    物理中的关节一般没有朝向定义，此处朝向/角速度都是用子body的朝向/角速度代替
    """
    for i in range(pose.shape[0]):
        joint_target_local_rot = pose[i]
        # Q_0 * R_1 = Q_1, R_1 = Q_0^T Q_1
        if parent_index[i] == -1:
            joint_cur_local_rot = joint_orientation[i]
        else:
            # the conjugate of a quaternion corresponds to the transpose of the matrix 
            rot = R.from_quat(joint_orientation[parent_index[i]]).inv().__mul__(R.from_quat(joint_orientation[i]))
            joint_cur_local_rot = rot.as_quat()
        
        """
        quaternion: (w, i, j, k)
        q_difference * q_cur = q_target, q_diff = q_target * q_cur.inv()
        q_difference will rotate from our current rotation to desired rotation
        """
        diff_rot =  R.from_quat(joint_target_local_rot).__mul__(R.from_quat(joint_cur_local_rot).inv())
        delta_rot = diff_rot.as_quat()
        """
        delta_rot can be the-long-rotation-around-the-sphere eg. 350 degrees
        We want the equivalant short rotation eg. -10 degrees
        Check if rotation is greater than 190 degees == q.w is negative
        """
        """
        if delta_rot[0] < 0:
            #Convert the quaterion to eqivalent "short way around" quaterion
            delta_rot[1] = -delta_rot[1]
            delta_rot[2] = -delta_rot[2]
            delta_rot[3] = -delta_rot[3]
            delta_rot[0] = -delta_rot[0]
        """
        
        # from quaternion to torque vector 
        s = R.from_quat(delta_rot).as_rotvec()
        local_torque = kp * s - kd * joint_avel[i]
        
        # change the local torque to global torque(under global coordinate system)
        if parent_index[i] == -1:
            global_torque[i] = local_torque
        else:
            global_torque[i] = R.from_quat(joint_orientation[parent_index[i]]).apply(local_torque)
        
        # normalization
        #global_torque[i] /= np.linalg.norm(global_torque[i])
        
        # clip
        if np.linalg.norm(global_torque[i]) > 1000:
            global_torque[i] /= 50
        
    return global_torque

def part2_cal_float_base_torque(target_position, pose, physics_info, **kargs):
    '''
    输入： target_position: (3,)的numpy数组，表示根节点的目标位置，其余同上
    输出： global_root_force: (3,)的numpy数组，表示根节点的全局坐标下的辅助力
          global_torque: 同上
    注意：
        1. 你需要自己计算kp和kd，并且可以通过kargs调整part1中的kp和kd
        2. global_torque[0]在track静止姿态时会被无视，但是track走路时会被加到根节点上，不然无法保持根节点朝向
    '''
    global_torque = part1_cal_torque(pose, physics_info)
    kp = kargs.get('root_kp', 3000) # 需要自行调整root的kp和kd！
    kd = kargs.get('root_kd', 200)
    root_position, root_velocity = physics_info.get_root_pos_and_vel()
    global_root_force = np.zeros((3,))

    """
    track rootjoint position in bvh motion file, calculate the force, 
    attention!!!! it's force not torque
    f = kp * (pos_target - pos_current) - kd * velocity_current
    """
    global_root_force = kp * (target_position - root_position) - kd * root_velocity

    return global_root_force, global_torque

def part3_cal_static_standing_torque(self, bvh: BVHMotion, physics_info, **kargs):
    '''
    输入： bvh: BVHMotion类，包含了当前的动作信息，参见bvh_loader.py
    其余同上
    Tips: 
        只track第0帧就能保持站立了
        为了保持平衡可以把目标的根节点位置适当前移，比如把根节点位置和左右脚的中点加权平均
        为了仿真稳定最好不要在Toe关节上加额外力矩
    '''
    # 第0帧，RootJoint在bvh中的位置
    tar_pos = bvh.joint_position[0][0]
    # 第0帧，角色在bvh中的相对于父关节的旋转
    pose = bvh.joint_rotation[0]
    joint_name = physics_info.joint_name
    joint_parent = physics_info.parent_index
    # 物理角色的所有关节的位置
    joint_positions = physics_info.get_joint_translation()
    # RootJoint适当前移
    tar_pos = tar_pos * 0.8 + joint_positions[9] * 0.1 + joint_positions[10] * 0.1
    torque = np.zeros((20,3))

    """
    calculate the force onto RootJoint, and the torque for tracking frame0 in bvh
    """
    global_root_force, torque = part2_cal_float_base_torque(tar_pos, pose, physics_info)

    # draw the RootJoint Target position
    # self.viewer.create_marker2(tar_pos, [1, 0, 0, 1])
    # self.viewer.create_arrow2(tar_pos, global_root_force, [0, 0, 1, 1])
    """
    compute necessary joint torques using Jacobian Transpose Control to repalce the effect of this force 
    (on COM --- RootJoint), only consider lower body joints, from Ankle to Hip
    the power is similar : COM_force * COM_velocity = joint_torque * joint_angular_velocity 
    -----> COM_velocity = Jocabian * joint_angular_velocity
    -----> joint_torque = COM_force * Jacobian
    -----> torque_i = (x - p_i) x f
    """
    #substrings = ["Hip", "Knee", "Ankle"]
    substrings = ["Knee"]
    for i in range(len(joint_name)):
        parent_idx = joint_parent[i]
        for substring in substrings:
            if substring in joint_name[i]:
                torque[parent_idx] += np.cross(joint_positions[0] - joint_positions[i], global_root_force)
                #self.viewer.create_marker2(tar_pos, [1, 0, 0, 1])
                break
    return torque

