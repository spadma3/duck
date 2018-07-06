#!/usr/bin/env python
# Utility functions for coordinate transformations

import tf
import tf.transformations as tr
import numpy as np


##### COORDINATE TRANSFORMATION UTILITY FUNCTIONS #############################
# creates a 4x4 translation matrix from a 1x3 translation vector and
# a 1x4 quaternion vector

'''
# Josefine Version of getting tf matrix
def create_tf_matrix(trans, rot):
    # numpy arrays to 4x4 transform matrix
    trans_mat = tr.translation_matrix(trans)
    rot_mat = tr.quaternion_matrix(rot)
    transformation_matrix = np.dot(trans_mat, rot_mat)
    return transformation_matrix
'''

# Eric Version of getting tf matrix
# using tf.transformation.compose_matrix
# See here for more information http://docs.ros.org/jade/api/tf/html/python/transformations.html
def create_tf_matrix(trans, rot):
    # numpy arrays to 4x4 transform matrix
    transformation_matrix = tr.compose_matrix(angle = tr.euler_from_quaternion(rot), translate = trans)
    return transformation_matrix


# #euler and translation to Homography
# #para is x,y,z,roll,pitch,yaw
# def create_tf_matrix(trans, rot):
#     para = np.zeros(6)
#     para[3], para[4], para[5] = tr.euler_from_quaternion(rot)
#
#     Rx = np.array([[1,0,0],
#                    [0,np.cos(para[3]),-np.sin(para[3])],
#                    [0,np.sin(para[3]), np.cos(para[3])]])
#     Ry = np.array([[ np.cos(para[4]),0,np.sin(para[4])],
#                    [0,1,0],
#                    [-np.sin(para[4]),0,np.cos(para[4])]])
#     Rz = np.array([[np.cos(para[5]),-np.sin(para[5]),0],
#                    [np.sin(para[5]), np.cos(para[5]),0],
#                    [0,0,1]])
#     R = np.dot(np.dot(Rz,Ry),Rx)
#     t = np.array([[trans[0]],[trans[1]],[trans[2]]])
#     return np.concatenate((np.concatenate((R,t), axis=1),np.array([[0,0,0,1]])), axis=0)


def rot_trans_from_matrix(mat):
    trans = tf.transformations.translation_from_matrix(mat)
    rot = tf.transformations.quaternion_from_matrix(mat)
    return trans, rot

# This function projects the 3D position onto the 2D plane
def project_position_to_2D_plane(mat_bot_abs):

    trans_bot_abs, rot_bot_abs = rot_trans_from_matrix(mat_bot_abs)

    x = trans_bot_abs[0]
    y = trans_bot_abs[1]
    # returns theta in radians
    alpha,beta,theta = tr.euler_from_matrix(mat_bot_abs, 'rxyz')

    return x,y,theta


# computes a coordinate transformation through concationation
# of translation matrices
# INPUT:    translation matrix of A with respect to B
#           translation matrix of B with respect to C
# OUTPUT:   translation matrix of A with respect to C
def absolute_from_relative_position(mat_A_B, mat_B_C):
    # https://answers.ros.org/question/215656/how-to-transform-a-pose/

    # TODO: verify order of arguments
    # mat_bot_abs = tr.concatenate_matrices(mat_tag_abs, mat_bot_tag)
    # mat_A_C = tr.concatenate_matrices(mat_A_B, mat_B_C)
    mat_A_C = np.dot(mat_A_B, mat_B_C)
    return mat_A_C



# takes a PoseStamped.pose and returns translation and orientations as arrays
def get_trans_rot_from_pose(pose):
    # trans: xyz translation
    # rot: xyzw quaternion
    # mat: 4x4 transformation matrix
    position = pose.position
    orientation = pose.orientation
    trans = [position.x, position.y, position.z]
    rot = [orientation.x, orientation.y, orientation.z, orientation.w]
    return trans, rot

# takes a PoseStamped.pose and returns a 4x4 translation matrix
def get_matrix_from_pose(pose):
    # mat: 4x4 transformation matrix
    position = pose.position
    orientation = pose.orientation
    trans = [position.x, position.y, position.z]
    rot = [orientation.x, orientation.y, orientation.z, orientation.w]
    mat = create_tf_matrix(trans, rot)
    return mat
