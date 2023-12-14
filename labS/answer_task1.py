from bvh_utils import *
#import cupy as cp
#import cupyx
#from numba import jit,njit


#---------------你的代码------------------#

def LBS_1(joint_translation, joint_orientation, T_pose_joint_translation, T_pose_vertex_translation, skinning_idx, skinning_weight):
    """
    Method1:
        we can first calculate the local coordinates of vertice under joint coordinate system at T-pose, 
        which is r_ij = v_i - o_j
        then use this formula: x_i' = w_ij * (Q_j' * r_ij + o_j')
    """
    # cpu fps : 3
    vertex_translation = np.zeros(T_pose_vertex_translation.shape)
    vertex_num = skinning_idx.shape[0]
    for vertex_idx in range(vertex_num):
        # the four joints influence this vertex
        influence_joints = skinning_idx[vertex_idx]
        joints_weights = skinning_weight[vertex_idx]
        for joints_idx in range(len(influence_joints)):
            joint_id = influence_joints[joints_idx]
            weight = joints_weights[joints_idx]
            if weight == 0:
                continue
            Q_j = R.from_quat(joint_orientation[joint_id]).as_matrix()
            r_ij = T_pose_vertex_translation[vertex_idx] - T_pose_joint_translation[joint_id]
            o_j = joint_translation[joint_id]
            vertex_translation[vertex_idx] += weight * (np.dot(Q_j, r_ij) + o_j)
    return vertex_translation

#@njit
def LBS_2(joint_translation, joint_orientation, T_pose_joint_translation, T_pose_vertex_translation, skinning_idx, skinning_weight):
    """
    Method2:
        we can first calcuate the rotation and translation matrix of each joint,
        which is R_j = Q_j'Q_j^T, t_j = p_j' - Q_j'Q_j^Tp_j
        then use this formula: x_i' = (w_ij * R_j) * x_i + w_ij * t_j 
    """
    vertex_translation = np.zeros(T_pose_vertex_translation.shape)
    t = np.zeros(joint_translation.shape)
    for joint_id in range(len(joint_translation)):
        # 注意这里的平移的计算， 并不是当前帧的 joint 位置 - Tpose下的joint位置啊！！！！
        Q_j = R.from_quat(joint_orientation[joint_id]).as_matrix()
        t[joint_id] = joint_translation[joint_id] - np.dot(Q_j, T_pose_joint_translation[joint_id])
        # rotation for joints are their orientation
    for vertex_idx in range(len(skinning_idx)):
        joints = skinning_idx[vertex_idx]
        joints_weight = skinning_weight[vertex_idx]
        
        blend_rotation = np.zeros((3,3))
        blend_t = np.zeros(3)
        # the four joints influence this vertex
        for joint_idx in range(len(joints)):
            joint_id = joints[joint_idx]
            joint_weight = joints_weight[joint_idx]
            if joint_weight == 0:
                continue
            blend_rotation += (R.from_quat(joint_orientation[joint_id]).as_matrix()) * joint_weight
            blend_t += joint_weight * t[joint_id]
        vertex_translation[vertex_idx] = np.dot(blend_rotation, T_pose_vertex_translation[vertex_idx]) + blend_t
    return vertex_translation

# cpu fps : 9    
def LBS_2_Optimization(joint_translation, joint_orientation, T_pose_joint_translation, T_pose_vertex_translation, skinning_idx, skinning_weight):
    """
    Optimization: loop is expensive, change the iteration to matrix or vector operation
    this formula: x_i' = (w_ij * R_j) * x_i + w_ij * t_j , R_j = Q_j'Q_j^T, t_j = p_j' - Q_j'Q_j^Tp_j
    (N,3) = (N, 3, 3) * (N, 3) + (1, ) * (N, 3)
    """
    # (M, 3)
    Q = R.from_quat(joint_orientation).as_matrix() #(65, 3, 3)
    pose_joint_translation = T_pose_joint_translation.reshape(T_pose_joint_translation.shape[0], 1, 3) #(65, 1, 3)
    tmp_trans = np.matmul(pose_joint_translation, Q.transpose(0,2,1)) #注意顺序
    tmp_trans = tmp_trans.reshape(joint_translation.shape[0], 3)
    t = joint_translation - tmp_trans #(65,3)
    
    # (N, 3) = (N, 1, 4) * (N, 4, 3)
    skinning_weight = skinning_weight.reshape(skinning_weight.shape[0], 1, 4) #(16340, 1, 4)
    tranlation = [t[fancy_idx, :] for fancy_idx in skinning_idx] #16340个（4，3）
    tranlation = np.array(tranlation) # (16340, 4, 3)
    tmp_blend_trans = np.matmul(skinning_weight, tranlation) #(16340, 1, 3)
    blending_translation = tmp_blend_trans.reshape(skinning_weight.shape[0], 3) #(16340, 3)

    # (N, 3, 3) = ((N个顶点, 4个权重) * (N个顶点, 4个关节点, 3, 3))
    rotation = [joint_orientation[fancy_idx, :] for fancy_idx in skinning_idx] #16340 个(4,4), still quaternion
    rotation = np.array(rotation) #(16340, 4, 4)
    """
    blending quaternion: bad effect

    tmp_rotation = np.matmul(skinning_weight, rotation) #(16340, 1, 4)
    blending_rotation = tmp_rotation.reshape(skinning_weight.shape[0], 4) #(16340, 4)
    blending_rotation = R.from_quat(blending_rotation).as_matrix() #(16340, 3, 3)
    """
    rotation_list = [R.from_quat(joints_4_rot).as_matrix() for joints_4_rot in rotation] #16340 个 (4, 3 ,3)
    rotation_matrix = np.array(rotation_list) #(16340, 4, 3 ,3)
    # cost-expensive
    blending_rotation = np.einsum('ijk,ikcd->icd', skinning_weight, rotation_matrix) #(16340, 1, 4) * (16340, 4, 3 ,3) = (16340, 3, 3)
    #blending_rotation = tmp_blend_rotation.reshape(skinning_weight.shape[0], 3, 3) #(16340, 3, 3)
    # 注意顺序
    tmp_rotation_translation = np.matmul(T_pose_vertex_translation.reshape(T_pose_vertex_translation.shape[0], 1, 3), blending_rotation.transpose(0,2,1)) #(16340, 1, 3)
    blending_rotation_translation = tmp_rotation_translation.reshape(skinning_weight.shape[0], 3) #(16340, 3)
    vertex_translation =  blending_rotation_translation + blending_translation
    
    return vertex_translation

# translation 和 orientation 都是全局的
def skinning(joint_translation, joint_orientation, T_pose_joint_translation, T_pose_vertex_translation, skinning_idx, skinning_weight):
    """
    skinning函数，给出一桢骨骼的位姿，计算蒙皮顶点的位置
    假设M个关节，N个蒙皮顶点，每个顶点受到最多4个关节影响
    输入：
        joint_translation: (M,3)的ndarray, 目标关节的位置
        joint_orientation: (M,4)的ndarray, 目标关节的旋转，用四元数表示
        T_pose_joint_translation: (M,3)的ndarray, T pose下关节的位置
        T_pose_vertex_translation: (N,3)的ndarray, T pose下蒙皮顶点的位置
        skinning_idx: (N,4)的ndarray, 每个顶点受到哪些关节的影响（假设最多受4个关节影响）
        skinning_weight: (N,4)的ndarray, 每个顶点受到对应关节影响的权重
    输出：
        vertex_translation: (N,3)的ndarray, 蒙皮顶点的位置
    """
    #vertex_translation = T_pose_vertex_translation.copy()
    #vertex_translation = np.zeros(T_pose_vertex_translation.shape)
    
    #---------------你的代码------------------#
    
    #vertex_translation = LBS_1(joint_translation, joint_orientation, T_pose_joint_translation, T_pose_vertex_translation, skinning_idx, skinning_weight)

    #vertex_translation = LBS_2(joint_translation, joint_orientation, T_pose_joint_translation, T_pose_vertex_translation, skinning_idx, skinning_weight)

    vertex_translation = LBS_2_Optimization(joint_translation, joint_orientation, T_pose_joint_translation, T_pose_vertex_translation, skinning_idx, skinning_weight)
    
    return vertex_translation