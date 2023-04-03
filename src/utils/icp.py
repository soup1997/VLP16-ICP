#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
from sklearn.neighbors import NearestNeighbors

def homogenous_2d(H): 
    sin_pitch = -H[2, 0]
    pitch = np.arcsin(sin_pitch)
    sin_yaw = np.arcsin(H[1, 0] / np.cos(pitch))
    yaw = np.arcsin(sin_yaw)
    
    translation = H[:2, 3]
    tx, ty = translation[0], translation[1] 
    
    
    H2d = np.array([[np.cos(yaw), -np.sin(yaw), tx],
                          [np.sin(yaw), np.cos(yaw), ty],
                          [0, 0, 1]], dtype=np.float32)

    return H2d


def best_fit_transform(src, dst):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      source: Nxm numpy array of corresponding points
      target: Nxm numpy array of corresponding points
    Output:
      H: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''

    assert src.shape == dst.shape
    # get number of dimensions
    m = src.shape[1]

    # translate points to their centroids
    centroid_src = np.mean(src, axis=0)
    centroid_dst = np.mean(dst, axis=0)

    psrc = src - centroid_src
    pdst = dst - centroid_dst

    # compute cross covariance matrix
    W = np.dot(psrc.T, pdst)

    # singular value decomposition
    U, S, Vt = np.linalg.svd(W)

    # compute roation matrix
    R = np.dot(Vt.T, U.T)
    
    
    # 반사된 경우 행렬 계산
    if np.linalg.det(R) < 0:
        Vt[m-1, :] *= -1
        R = np.dot(Vt.T, U.T)
        
    # compute translation matrix
    t = centroid_dst.T - np.dot(R, centroid_src.T)

    # homogeneous transformation matrix
    H = np.identity(m+1)
    H[:m, :m] = R
    H[:m, m] = t

    return H


def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''
    
    neighbors = NearestNeighbors(n_neighbors=1, algorithm='kd_tree') # 1 clusters
    neighbors.fit(dst)

    distances, indices = neighbors.kneighbors(
        src, return_distance=True)
    
    return distances.flatten(), indices.flatten()


def icp(A, B, init_pose=None, max_iterations=100, tolerance=0.001):
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxm numpy array of source mD points
        B: Nxm numpy array of destination mD point
        init_pose: (m+1)x(m+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        H: final homogeneous transformation that maps A on to B
    '''

    # get number of dimensions

    # A shape: (23679, 3) 
    # B shape: (23556, 3)
    m = A.shape[1]
    
    # make points homogenous, copy the to maintain the originals
    src  = np.ones((m+1, A.shape[0])) # 4 X (number of points A)
    dst = np.ones((m+1, B.shape[0])) # 4 X (number of points A)
    
    src[:m, :] = np.copy(A.T[:m, :])
    dst[:m, :] = np.copy(B.T[:m, :])
    
    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)
    
    prev_error = 0
    
    for _ in range(max_iterations):
        # 소스와 목적 점군 간에 가장 근처 이웃점 탐색, 계산량이 많음
        distances, indices = nearest_neighbor(src[:m, :].T, dst[:m, :].T)
        
        # 소스 점군에서 대상 점군으로 정합 시 필요한 변환행렬 계산
        H = best_fit_transform(src[:m, :].T, dst[:m, indices].T)
        
        # update the current source
        src = np.dot(H, src)
        
        # check error
        mean_error = np.mean(distances)
        
        error = prev_error - mean_error
        
        if np.abs(error) < tolerance:
            break
        
        prev_error = mean_error
        
        
    # calculate the final transformation
    H =  best_fit_transform(A[:, :m], src[:m,:].T)
    H2d = homogenous_2d(H)
    print('Loss: {0:.3f}'.format(prev_error - mean_error))
    
    return H, H2d