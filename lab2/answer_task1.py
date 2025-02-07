import numpy as np
import copy
import math
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.spatial import geometric_slerp
# ------------- lab1里的代码 -------------#
def load_meta_data(bvh_path):
    with open(bvh_path, 'r') as f:
        channels = []
        joints = []
        joint_parents = []
        joint_offsets = []
        end_sites = []

        parent_stack = [None]
        for line in f:
            if 'ROOT' in line or 'JOINT' in line:
                joints.append(line.split()[-1])
                joint_parents.append(parent_stack[-1])
                channels.append('')
                joint_offsets.append([0, 0, 0])

            elif 'End Site' in line:
                end_sites.append(len(joints))
                joints.append(parent_stack[-1] + '_end')
                joint_parents.append(parent_stack[-1])
                channels.append('')
                joint_offsets.append([0, 0, 0])

            elif '{' in line:
                parent_stack.append(joints[-1])

            elif '}' in line:
                parent_stack.pop()

            elif 'OFFSET' in line:
                joint_offsets[-1] = np.array([float(x) for x in line.split()[-3:]]).reshape(1,3)

            elif 'CHANNELS' in line:
                trans_order = []
                rot_order = []
                for token in line.split():
                    if 'position' in token:
                        trans_order.append(token[0])

                    if 'rotation' in token:
                        rot_order.append(token[0])

                channels[-1] = ''.join(trans_order)+ ''.join(rot_order)

            elif 'Frame Time:' in line:
                break
        
    joint_parents = [-1]+ [joints.index(i) for i in joint_parents[1:]]
    channels = [len(i) for i in channels]
    return joints, joint_parents, channels, joint_offsets

def load_motion_data(bvh_path):
    with open(bvh_path, 'r') as f:
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

def align_vector(vec, target):
    rotate_axis = np.cross(vec, target)
    rotate_axis = rotate_axis / np.linalg.norm(rotate_axis)
    cos = min(np.dot(vec, target) / (np.linalg.norm(vec) * np.linalg.norm(target)), 1.0)
    theta = np.arccos(cos)
    if theta < 0.1:
        return R.identity()
    # A rotation vector is a 3 dimensional vector which is co-directional to the axis of rotation and whose norm gives the angle of rotation
    deltaRotation = R.from_rotvec(theta * rotate_axis)
    return deltaRotation


# ------------- 实现一个简易的BVH对象，进行数据处理 -------------#

'''
注释里统一N表示帧数，M表示关节数
position, rotation表示局部平移和旋转
translation, orientation表示全局平移和旋转
'''

class BVHMotion():
    def __init__(self, bvh_file_name = None) -> None:
        
        # 一些 meta data
        self.joint_name = []
        self.joint_channel = []
        self.joint_parent = []
        
        # 一些local数据, 对应bvh里的channel, XYZposition和 XYZrotation
        #! 这里我们把没有XYZ position的joint的position设置为offset, 从而进行统一
        self.joint_position = None # (N,M,3) 的ndarray, 局部平移
        self.joint_rotation = None # (N,M,4)的ndarray, 用四元数表示的局部旋转
        
        if bvh_file_name is not None:
            self.load_motion(bvh_file_name)
        pass
    
    #------------------- 一些辅助函数 ------------------- #
    def load_motion(self, bvh_file_path):
        '''
            读取bvh文件，初始化元数据和局部数据
        '''
        self.joint_name, self.joint_parent, self.joint_channel, joint_offset = \
            load_meta_data(bvh_file_path)
        
        motion_data = load_motion_data(bvh_file_path)

        # 把motion_data里的数据分配到joint_position和joint_rotation里
        self.joint_position = np.zeros((motion_data.shape[0], len(self.joint_name), 3))
        self.joint_rotation = np.zeros((motion_data.shape[0], len(self.joint_name), 4))
        self.joint_rotation[:,:,3] = 1.0 # 四元数的w分量默认为1
        
        cur_channel = 0
        for i in range(len(self.joint_name)):
            if self.joint_channel[i] == 0:
                self.joint_position[:,i,:] = joint_offset[i].reshape(1,3)
                continue   
            elif self.joint_channel[i] == 3:
                self.joint_position[:,i,:] = joint_offset[i].reshape(1,3)
                rotation = motion_data[:, cur_channel:cur_channel+3]
            elif self.joint_channel[i] == 6:
                self.joint_position[:, i, :] = motion_data[:, cur_channel:cur_channel+3]
                rotation = motion_data[:, cur_channel+3:cur_channel+6]
            self.joint_rotation[:, i, :] = R.from_euler('XYZ', rotation,degrees=True).as_quat()
            cur_channel += self.joint_channel[i]
        
        return

    def batch_forward_kinematics(self, joint_position = None, joint_rotation = None):
        '''
        利用自身的metadata进行批量前向运动学
        joint_position: (N,M,3)的ndarray, 局部平移
        joint_rotation: (N,M,4)的ndarray, 用四元数表示的局部旋转
        '''
        if joint_position is None:
            joint_position = self.joint_position
        if joint_rotation is None:
            joint_rotation = self.joint_rotation
        
        joint_translation = np.zeros_like(joint_position)
        joint_orientation = np.zeros_like(joint_rotation)
        joint_orientation[:,:,3] = 1.0 # 四元数的w分量默认为1
        
        # 一个小hack是root joint的parent是-1, 对应最后一个关节
        # 计算根节点时最后一个关节还未被计算，刚好是0偏移和单位朝向
        
        for i in range(len(self.joint_name)):
            pi = self.joint_parent[i]
            parent_orientation = R.from_quat(joint_orientation[:,pi,:]) 
            joint_translation[:, i, :] = joint_translation[:, pi, :] + \
                parent_orientation.apply(joint_position[:, i, :])
            joint_orientation[:, i, :] = (parent_orientation * R.from_quat(joint_rotation[:, i, :])).as_quat()
        return joint_translation, joint_orientation
    
    
    def adjust_joint_name(self, target_joint_name):
        '''
        调整关节顺序为target_joint_name
        '''
        idx = [self.joint_name.index(joint_name) for joint_name in target_joint_name]
        idx_inv = [target_joint_name.index(joint_name) for joint_name in self.joint_name]
        self.joint_name = [self.joint_name[i] for i in idx]
        self.joint_parent = [idx_inv[self.joint_parent[i]] for i in idx]
        self.joint_parent[0] = -1
        self.joint_channel = [self.joint_channel[i] for i in idx]
        self.joint_position = self.joint_position[:,idx,:]
        self.joint_rotation = self.joint_rotation[:,idx,:]
        pass
    
    def raw_copy(self):
        '''
        返回一个拷贝
        '''
        return copy.deepcopy(self)
    
    @property
    def motion_length(self):
        return self.joint_position.shape[0]
    
    
    def sub_sequence(self, start, end):
        '''
        返回一个子序列
        start: 开始帧
        end: 结束帧
        '''
        res = self.raw_copy()
        res.joint_position = res.joint_position[start:end,:,:]
        res.joint_rotation = res.joint_rotation[start:end,:,:]
        return res
    
    def append(self, other):
        '''
        在末尾添加另一个动作
        '''
        other = other.raw_copy()
        other.adjust_joint_name(self.joint_name)
        self.joint_position = np.concatenate((self.joint_position, other.joint_position), axis=0)
        self.joint_rotation = np.concatenate((self.joint_rotation, other.joint_rotation), axis=0)
        pass
    
    #--------------------- 你的任务 -------------------- #
    def decompose_rotation_with_yaxis(self, rotation):
        '''
        输入: rotation 形状为(4,)的ndarray, 四元数旋转
        输出: Ry, Rxz，分别为绕y轴的旋转和转轴在xz平面的旋转，并满足R = Ry * Rxz
        '''
        Ry = np.zeros_like(rotation)
        Rxz = np.zeros_like(rotation)
        
        # TODO:
        # get RootJoint orientation
        rotation_matrix = R.from_quat(rotation).as_matrix()
        # get RootJoint y_axis
        RootJoint_y_axis = rotation_matrix[:, 1]
        # align RootJoint_y_axis to global_y_axis to get rotation delta_rotation
        #r, _, = R.align_vectors(np.atleast_2d(np.array([0,1,0])), np.atleast_2d(RootJoint_y_axis))
        r = align_vector(RootJoint_y_axis, np.array([0,1,0]))
        delta_rotation = r.as_matrix()
        # the rotation around Y aixs is R_y
        R_y = np.dot(delta_rotation, rotation_matrix)
        Ry = R.from_matrix(R_y).as_quat()
        # the rotation around a axis from xz-plane
        Rxz = R.from_matrix(np.transpose(R_y) * rotation_matrix).as_quat()
        return Ry, Rxz
    
    # part 1
    def translation_and_rotation(self, frame_num, target_translation_xz, target_facing_direction_xz):
        '''
        计算出新的joint_position和joint_rotation
        使第frame_num帧的根节点平移为target_translation_xz, 水平面朝向为target_facing_direction_xz
        frame_num: int
        target_translation_xz: (2,)的ndarray
        target_faceing_direction_xz: (2,)的ndarray，表示水平朝向。你可以理解为原本的z轴被旋转到这个方向。
        Tips:
            主要是调整root节点的joint_position和joint_rotation
            frame_num可能是负数，遵循python的索引规则
            你需要完成并使用decompose_rotation_with_yaxis
            输入的target_facing_direction_xz的norm不一定是1
        '''
        
        res = self.raw_copy() # 拷贝一份，不要修改原始数据
        
        # for RootJoint, calcuate the translation offset at frame_num, and apply this offset to all the frames 
        #offset = target_translation_xz - res.joint_position[frame_num, 0, [0,2]]
        #res.joint_position[:, 0, [0,2]] += offset
        
        # TODO:
        # Facing Frame Orientation
        Ry, _, = res.decompose_rotation_with_yaxis(res.joint_rotation[frame_num, 0])
        face_time_original_orientation = R.from_quat(Ry).as_matrix()
        #r, _, = R.align_vectors(np.atleast_2d(np.array([target_facing_direction_xz[0],0,target_facing_direction_xz[1]])), np.atleast_2d(np.array([0,0,1])))
        r = align_vector(np.array([0,0,1]), np.array([target_facing_direction_xz[0],0,target_facing_direction_xz[1]]))
        face_time_target_orientation = r.as_matrix()
        # now our task becomes transform bvh character target direction to given target direction
        transformation_matrix = np.dot(face_time_target_orientation, np.transpose(face_time_original_orientation))
        frames = res.joint_rotation.shape[0]
        # 注意这里也是深拷贝
        original_rootjoint_position = copy.deepcopy(res.joint_position[frame_num, 0])
        target_rootjoint_position = np.array([target_translation_xz[0], 0 ,target_translation_xz[1]])
        
        for i in range(0, frames):
            # update RootJoint Orientation
            root_joint_orientation = res.joint_rotation[i, 0]
            new_rotation_matrix = np.dot(transformation_matrix, R.from_quat(root_joint_orientation).as_matrix())
            res.joint_rotation[i, 0] = R.from_matrix(new_rotation_matrix).as_quat()
            # update RootJoint Translation
            root_origin_offset = res.joint_position[i, 0] - original_rootjoint_position
            new_translation =  target_rootjoint_position + np.dot(transformation_matrix, root_origin_offset)
            res.joint_position[i, 0, [0,2]] = new_translation[[0,2]]
        
        return res

def rot_slerp(rot_a, rot_b, alpha):
    """
    rot_a: M x 4
    rot_b: M x 4
    w: scalar
    """
    #ret_rot = geometric_slerp(bvh_motion.joint_rotation[integer_lower_frame_idx, i], bvh_motion.joint_rotation[integer_upper_frame_idx, i], alpha)
    ret_rot = np.empty_like(rot_a)
    for joint_idx in range(len(rot_a)):
        key_rots = R.from_quat([rot_a[joint_idx], rot_b[joint_idx]])
        slerp = Slerp([0, 1], key_rots)
        ret_rot[joint_idx] = slerp([alpha]).as_quat()
    return ret_rot

def frame_linear_interpolate(actual_frame, bvh_motion):
    """
    sample the walk_bvh and run_bvh at specified frame(time)
    """
    integer_lower_frame_idx = math.floor(actual_frame)
    alpha = actual_frame - integer_lower_frame_idx
    integer_upper_frame_idx = (integer_lower_frame_idx + 1) % (bvh_motion.motion_length)
    # slerp quaterninon
    ret_rot = rot_slerp(bvh_motion.joint_rotation[integer_lower_frame_idx], bvh_motion.joint_rotation[integer_upper_frame_idx], alpha)
    # slerp position
    ret_pos = (1 - alpha) * bvh_motion.joint_position[integer_lower_frame_idx] + alpha * bvh_motion.joint_position[integer_upper_frame_idx]

    return ret_rot, ret_pos
# part2
def blend_two_motions(bvh_motion1, bvh_motion2, alpha):
    '''
    blend两个bvh动作
    假设两个动作的帧数分别为n1, n2
    alpha: 0~1之间的浮点数组，形状为(n3,)
    返回的动作应该有n3帧，第i帧由(1-alpha[i]) * bvh_motion1[j] + alpha[i] * bvh_motion2[k]得到
    i均匀地遍历0~n3-1的同时，j和k应该均匀地遍历0~n1-1和0~n2-1
    '''
    
    res = bvh_motion1.raw_copy()
    res.joint_position = np.zeros((len(alpha), res.joint_position.shape[1], res.joint_position.shape[2]))
    res.joint_rotation = np.zeros((len(alpha), res.joint_rotation.shape[1], res.joint_rotation.shape[2]))
    res.joint_rotation[...,3] = 1.0

    # TODO: 你的代码
    n = len(alpha)
    # from frame_1 to frame_n
    for cur_frame_idx in range(n):
        # motion_1 frame nums: n_1, motion_2 frame nums: n_2
        n_1 = bvh_motion1.motion_length
        n_2 = bvh_motion2.motion_length

        # corresponding frame_idx in motion_1, motion_2, attention, this frame_idx is float
        frame_idx_1 = cur_frame_idx * (n_1 / n)
        frame_idx_2 = cur_frame_idx * (n_2 / n)

        # find the two sides integer frame for frame_idx_1 and linear interpolate
        rot_1, pos_1 = frame_linear_interpolate(frame_idx_1, bvh_motion1)
        # find the two sides integer frame  for frame_idx_2 and linear interpolate
        rot_2, pos_2 = frame_linear_interpolate(frame_idx_2, bvh_motion2)

        # blend motion
        res.joint_rotation[cur_frame_idx, ...] = rot_slerp(rot_1, rot_2, alpha[cur_frame_idx])
        res.joint_position[cur_frame_idx, ...] = (1 - alpha[cur_frame_idx]) * pos_1 + alpha[cur_frame_idx] * pos_2
    return res

# part3
def build_loop_motion(bvh_motion):
    '''
    将bvh动作变为循环动作
    由于比较复杂,作为福利,不用自己实现
    (当然你也可以自己实现试一下)
    推荐阅读 https://theorangeduck.com/
    Creating Looping Animations from Motion Capture
    '''
    res = bvh_motion.raw_copy()
    
    from smooth_utils import build_loop_motion
    return build_loop_motion(res)

def find_nearest_pose(target_pose, motion_poses):
    """
    find the most similar pose in motion_pose for target_pose, only consider joint_rotation
    target_pose: M x 4
    motion_pose: N x M x 4
    """
    def pose_distance(pose_a, pose_b):
        res = 0
        for i in range(len(pose_a)):
            # the distance between two joints
            res += np.linalg.norm(pose_a[i] - pose_b[i])
        return res
    
    min_distance = float("inf")
    res_frame = 0  
    for i in range(len(motion_poses)):
        cur_pose = motion_poses[i]
        # calculate the similarity between target_pose and cur_pose
        distance = pose_distance(cur_pose, target_pose)
        if distance < min_distance:
            min_distance = distance
            res_frame = i
    return res_frame        

# part4
def concatenate_two_motions(bvh_motion1, bvh_motion2, mix_frame1, mix_time):
    '''
    将两个bvh动作平滑地连接起来，mix_time表示用于混合的帧数
    混合开始时间是第一个动作的第mix_frame1帧
    虽然某些混合方法可能不需要mix_time，但是为了保证接口一致，我们还是保留这个参数
    Tips:
        你可能需要用到BVHMotion.sub_sequence 和 BVHMotion.append
    '''
    res = bvh_motion1.raw_copy()
    
    # TODO: 你的代码
    # 下面这种直接拼肯定是不行的(
    #res.joint_position = np.concatenate([res.joint_position[:mix_frame1], bvh_motion2.joint_position], axis=0)
    #res.joint_rotation = np.concatenate([res.joint_rotation[:mix_frame1], bvh_motion2.joint_rotation], axis=0)
    
    # change bvh_motion2 to loop animation, since mix_time may be to large for bvh_motion2
    bvh_motion2 = build_loop_motion(bvh_motion2)
    motion = bvh_motion2
    pos = motion.joint_position[-1, 0, [0,2]]
    rot = motion.joint_rotation[-1,0]
    facing_axis = R.from_quat(rot).apply(np.array([0,0,1])).flatten()[[0,2]]
    new_motion = motion.translation_and_rotation(0, pos, facing_axis)

    # for bvh_motion1, it's going to blend motion at mix_frame1, but which frame for bvh_motion2?
    # we need to find the most similar pose in bvh_motion2 for mix_frame1 at bvh_motion1
    target_pose = bvh_motion1.joint_rotation[mix_frame1]
    mix_frame2 = find_nearest_pose(target_pose, bvh_motion2.joint_rotation)

    # in case the mix_frame2 + mix_time > bvh_motion2.motion_length
    bvh_motion2.append(new_motion)

    # start blending
    # let bvh_motion2's face time coordinate system at mix_frame2 the same with bvh1 at mix_frame1
    target_face_time_pos = bvh_motion1.joint_position[mix_frame1, 0, [0,2]]
    target_root_ori = bvh_motion1.joint_rotation[mix_frame1, 0]
    target_face_direction_xz = R.from_quat(target_root_ori).apply(np.array([0,0,1])).flatten()[[0,2]]
    bvh_motion2 = bvh_motion2.translation_and_rotation(mix_frame2, target_face_time_pos, target_face_direction_xz)

    cur_frame1 = mix_frame1
    cur_frame2 = mix_frame2
    for i in range(mix_time):
        cur_frame1 += 1
        cur_frame2 += 1
        # interpoloate between bvh_motion1[mix_frame1] and bvh_motion2[mix_frame2]
        # (1-alpha) * bvh_motion1[mix_frame1] + alpha * bvh_motion2[mix_frame2], alpha: from 0 to 1
        alpha = i / (mix_time - 1)
        pos_a = bvh_motion1.joint_position[cur_frame1]
        pos_b = bvh_motion2.joint_position[cur_frame2]
        rot_a = bvh_motion1.joint_rotation[cur_frame1]
        rot_b = bvh_motion2.joint_rotation[cur_frame2]
        # blend motion
        res.joint_rotation[cur_frame1, ...] = rot_slerp(rot_a, rot_b, alpha)
        res.joint_position[cur_frame1, ...] = (1 - alpha) * pos_a + alpha * pos_b
    
    # let the motion after [cur_frame1 + mix_time, inf] is bvh_motion2
    res.joint_position = np.concatenate([res.joint_position[:cur_frame1], bvh_motion2.joint_position[cur_frame2:]], axis=0)
    res.joint_rotation = np.concatenate([res.joint_rotation[:cur_frame1], bvh_motion2.joint_rotation[cur_frame2:]], axis=0)
    return res

