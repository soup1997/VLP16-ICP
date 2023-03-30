#!/usr/bin/env python3

import numpy as np
from sklearn.neighbors import NearesetNeighbors


def best_fit_transform(src, dst):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      source: Nxm numpy array of corresponding points
      target: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''
    
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
    
    # compute translation matrix
    t = centroid_dst.T - np.dot(R, centroid_src.T)
    
    # homogeneous transformation matrix
    H = np.identity(m+1)
    H[:m, :m] = R
    H[:m , m] = t
    
    return H, R, t

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
    pass

def icp(src, dst, init_pose=None, max_iterations=100, tolerance=0.001):
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxm numpy array of source mD points
        B: Nxm numpy array of destination mD point
        init_pose: (m+1)x(m+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
    '''
    pass
    
    
    