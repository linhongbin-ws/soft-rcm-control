#!/usr/bin/env python

import numpy as np
from kinematics import *

PI = np.pi
PI_2 = np.pi/2

def compute_FK(joint_pos,RCM_flag,lambda_rcm):
    j = [0, 0, 0, 0, 0, 0, 0]
    for i in range(len(joint_pos)):
        j[i] = joint_pos[i]

    C = 0.34
    D = 0.4
    E = 0.4
    F = 0.126
    L_Shaft = 0.1
    L_EndEffector= 0.3

    # iiwa7 DH Params
    link1 = DH(alpha=0, a=0, theta=j[0], d=C, offset=0, joint_type='R', convention='MODIFIED')
    link2 = DH(alpha=PI_2, a=0, theta=j[1], d=0, offset=0, joint_type='R', convention='MODIFIED')
    link3 = DH(alpha=-PI_2, a=0, theta=j[2], d=D, offset=PI, joint_type='R', convention='MODIFIED')
    link4 = DH(alpha=PI_2, a=0, theta=j[3], d=0, offset=0, joint_type='R', convention='MODIFIED')
    link5 = DH(alpha=-PI_2, a=0, theta=j[4], d=E, offset=PI, joint_type='R', convention='MODIFIED')
    link6 = DH(alpha=PI_2, a=0, theta=j[5], d=0, offset=0, joint_type='R', convention='MODIFIED')
    link7 = DH(alpha=-PI_2, a=0, theta=j[6], d=F+L_Shaft, offset=0, joint_type='R', convention='MODIFIED')
    TOP = DH(alpha=PI_2, a=0, theta=0, d=0, offset=0, joint_type='R', convention='MODIFIED')
    TIP = DH(alpha=PI_2, a=0, theta=0, d=L_EndEffector, offset=0, joint_type='R', convention='MODIFIED')
    
    T_1_0 = link1.get_trans()
    T_2_1 = link2.get_trans()
    T_3_2 = link3.get_trans()
    T_4_3 = link4.get_trans()
    T_5_4 = link5.get_trans()
    T_6_5 = link6.get_trans()
    T_7_6 = link7.get_trans()
    T_TOP_7 = TOP.get_trans()
    T_TIP_7 = TIP.get_trans()

    # print(T_1_0)
    # print(T_2_1)
    T_2_0 = np.matmul(T_1_0, T_2_1)
    T_3_0 = np.matmul(T_2_0, T_3_2)
    T_4_0 = np.matmul(T_3_0, T_4_3)
    T_5_0 = np.matmul(T_4_0, T_5_4)
    T_6_0 = np.matmul(T_5_0, T_6_5)
    T_7_0 = np.matmul(T_6_0, T_7_6)
    T_TOP_0 = np.matmul(T_7_0, T_TOP_7)
    T_TIP_0 = np.matmul(T_7_0, T_TIP_7)
  
    # get jacobiam matrix - all revolute joints
    p_vec_TOP = np.hstack((T_1_0[0:3,3],T_2_0[0:3,3],T_3_0[0:3,3],T_4_0[0:3,3],T_5_0[0:3,3],T_6_0[0:3,3],T_7_0[0:3,3],T_TOP_0[0:3,3]))
    z_vec_TOP = np.hstack((T_1_0[0:3,2],T_2_0[0:3,2],T_3_0[0:3,2],T_4_0[0:3,2],T_5_0[0:3,2],T_6_0[0:3,2],T_7_0[0:3,2],T_TOP_0[0:3,2]))
    p_vec_TIP = np.hstack((T_1_0[0:3,3],T_2_0[0:3,3],T_3_0[0:3,3],T_4_0[0:3,3],T_5_0[0:3,3],T_6_0[0:3,3],T_7_0[0:3,3],T_TIP_0[0:3,3]))
    z_vec_TIP = np.hstack((T_1_0[0:3,2],T_2_0[0:3,2],T_3_0[0:3,2],T_4_0[0:3,2],T_5_0[0:3,2],T_6_0[0:3,2],T_7_0[0:3,2],T_TIP_0[0:3,2]))
    jaco_mat_TOP = np.zeros((6,len(joint_pos)))
    jaco_mat_TIP = np.zeros((6,len(joint_pos)))
    for i in range(len(joint_pos)):
        temp1 = z_vec_TOP[:,i]
        temp2 = p_vec_TOP[:,-1] - p_vec_TOP[:,i]
        temp3 = np.cross(temp1.T,temp2.T)
        temp4 = np.hstack((temp3,temp1.T))
        jaco_mat_TOP[:,i] = temp4
        temp1 = z_vec_TIP[:,i]
        temp2 = p_vec_TIP[:,-1] - p_vec_TIP[:,i]
        temp3 = np.cross(temp1.T,temp2.T)
        temp4 = np.hstack((temp3,temp1.T))
        jaco_mat_TIP[:,i] = temp4

    if RCM_flag == 1:
        RCM = DH(alpha=PI_2, a=0, theta=0, d=L_EndEffector*lambda_rcm, offset=0, joint_type='R', convention='MODIFIED')
        T_RCM_7 = RCM.get_trans()
        T_RCM_0 = np.matmul(T_7_0, T_RCM_7)
        p_vec_RCM = np.hstack((T_1_0[0:3,3],T_2_0[0:3,3],T_3_0[0:3,3],T_4_0[0:3,3],T_5_0[0:3,3],T_6_0[0:3,3],T_7_0[0:3,3],T_RCM_0[0:3,3]))
        z_vec_RCM = np.hstack((T_1_0[0:3,2],T_2_0[0:3,2],T_3_0[0:3,2],T_4_0[0:3,2],T_5_0[0:3,2],T_6_0[0:3,2],T_7_0[0:3,2],T_RCM_0[0:3,2]))
        jaco_mat_RCM = np.zeros((6,len(joint_pos)))
        for i in range(len(joint_pos)):
            temp1 = z_vec_RCM[:,i]
            temp2 = p_vec_RCM[:,-1] - p_vec_RCM[:,i]
            temp3 = np.cross(temp1.T,temp2.T)
            temp4 = np.hstack((temp3,temp1.T))
            jaco_mat_RCM[:,i] = temp4
        return (T_TOP_0,jaco_mat_TOP,T_TIP_0,jaco_mat_TIP,T_RCM_0,jaco_mat_RCM)


    return (T_TOP_0,jaco_mat_TOP,T_TIP_0,jaco_mat_TIP)

def set_rcm(x,y,z,rx,ry,rz):
    T_RCM = np.eye(4)
    P_RCM = np.array([x,y,z])
    R_x = np.array([[1,0,0],[0,np.cos(rx),-np.sin(rx)],[0,np.sin(rx),np.cos(rx)]])
    R_y = np.array([[np.cos(ry),0,np.sin(ry)],[0,1,0],[-np.sin(ry),0,np.cos(ry)]])
    R_z = np.array([[np.cos(rz),-np.sin(rz),0],[np.sin(rz),np.cos(rz),0],[0,0,1]])
    temp = np.matmul(R_x,R_y)
    R_RCM = np.matmul(temp,R_z)
    T_RCM[0:3,0:3] = R_RCM
    T_RCM[0:3,3] = P_RCM

    return T_RCM

def jaco_constrain_mat(T_RCM,lambda_rcm,H1,H2):
    i_vec = T_RCM[0:3,0]
    m_vec = T_RCM[0:3,1]
    # print(i_vec)
    # print(m_vec)
    I_M = np.vstack((i_vec,m_vec))
    # print(I_M)
    temp = lambda_rcm*H1+(1-lambda_rcm)*H2
    # print(temp)
    H = np.matmul(I_M,temp)
    
    return H

def jaco_hat(J,H):
    _n_jnts = J.shape[1]
    # print('n_jnts',_n_jnts)
    H_pinv = np.linalg.pinv(H)
    temp = np.eye(_n_jnts)-np.matmul(H_pinv,H)
    J_hat = np.matmul(J,temp)

    return J_hat


def RCM_ctrl(H_pinv,K1,i_vec,m_vec,err_rcm,J_hat_pinv,v_tip,K2,err_tip,J_tip):
    H_K = np.matmul(H_pinv,K1)
    # print('H_K:',H_K.shape)
    I_M = np.vstack((i_vec.T,m_vec.T))
    # print('I_M',I_M.shape)
    I_M_err = np.matmul(I_M,err_rcm)
    # print('I_M_err',I_M_err.shape)
    H_K_I_M_err = np.matmul(H_K,I_M_err)
    # print('H_K_I_M_err',H_K_I_M_err.shape)
    J_H_K_I_M_err = np.matmul(J_tip,H_K_I_M_err)
    # print('J_H_K_I_M_err',J_H_K_I_M_err.shape)
    K_err = np.matmul(K2,err_tip)
    # print('K_err',K_err.shape)
    temp = v_tip+K_err-J_H_K_I_M_err
    # print('temp',temp.shape)
    temp1 = np.matmul(J_hat_pinv,temp)
    # print('temp1',temp1.shape)
    q_dot = H_K_I_M_err+temp1

    return q_dot

def computeError(T_dsr,T_act):
    P_dsr = T_dsr[0:3,3]
    R_dsr = T_dsr[0:3,0:3]
    P_act = T_act[0:3,3]
    R_act = T_act[0:3,0:3]
    
    Re = np.matmul(R_dsr,R_act.T)
    K = np.array([Re[2,1]-Re[1,2], Re[0,2]-Re[2,0], Re[1,0]-Re[0,1]])
      
    P_err = P_dsr-P_act
    R_err = 0.5*K.T
    x_err = np.vstack((P_err,R_err))

    return x_err

if __name__ == "__main__":
    x = 0.5
    y = 0
    z = 0.1
    rx = 0
    ry = PI
    rz = 0
    T_RCM = set_rcm(x,y,z,rx,ry,rz)
    print('T_RCM',T_RCM)
    Q_RCM = np.array([0.,-0.7013,0.,-2.3376,0.,1.5386,1.7157])
    # -0.2706   -0.7648   -0.2044   -2.0910   -1.2457    1.6141    1.8954   -0.7160
    q_cur = Q_RCM
    lambda_rcm = 0.5
    (T_TOP_0,jaco_mat_TOP,T_TIP_0,jaco_mat_TIP,T_RCM_0,jaco_mat_RCM) = compute_FK(q_cur,1,lambda_rcm)
    
    print('T_TOP_0',T_TOP_0)
    print('jaco_mat_TOP',jaco_mat_TOP)
    print('T_TIP_0',T_TIP_0)
    print('jaco_mat_TIP',jaco_mat_TIP)
    print('T_RCM_0',T_RCM_0)
    print('jaco_mat_RCM',jaco_mat_RCM)
    H = jaco_constrain_mat(T_RCM,lambda_rcm,jaco_mat_TOP[0:3,:],jaco_mat_TIP[0:3,:])
    H_pinv = np.linalg.pinv(H)
    # print(H_pinv)
    j_hat = jaco_hat(jaco_mat_TIP,H)
    # print(j_hat)
