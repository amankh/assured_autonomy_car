from sklearn import gaussian_process
from sklearn.gaussian_process.kernels import Matern, WhiteKernel, ConstantKernel
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import cholesky, det, lstsq
from scipy.optimize import minimize
from numpy.linalg import inv
import math

def kernel(X1, X2, l=1.0, sigma_f=1.0):
    '''
    Isotropic squared exponential kernel. Computes 
    a covariance matrix from points in X1 and X2.
        
    Args:
        X1: Array of m points (m x d).
        X2: Array of n points (n x d).

    Returns:
        Covariance matrix (m x n).
    '''
    sqdist = np.sum(X1**2, 1).reshape(-1, 1) + np.sum(X2**2, 1) - 2 * np.dot(X1, X2.T)
    return sigma_f**2 * np.exp(-0.5 / l**2 * sqdist)

def kernel_gaussian(X1, X2, l=1.0):
    '''
    Isotropic squared exponential kernel. Computes 
    a covariance matrix from points in X1 and X2.
        
    Args:
        X1: Array of m points (m x d).
        X2: Array of n points (n x d).

    Returns:
        Covariance matrix (m x n).
    '''
    sqdist = np.sum(X1**2, 1).reshape(-1, 1) + np.sum(X2**2, 1) - 2 * np.dot(X1, X2.T)
    return np.exp(-0.5 / l**2 * sqdist)
def nll_fn_gaussian_kernel(X_train, Y_train, naive=True):
    '''
    Returns a function that computes the negative marginal log-
    likelihood for training data X_train and Y_train and given 
    noise level.
    
    Args:
        X_train: training locations (m x d).
        Y_train: training targets (m x 1).
        noise: known noise level of Y_train.
        naive: if True use a naive implementation of Eq. (7), if 
               False use a numerically more stable implementation. 
        
    Returns:
        Minimization objective.
    '''
    def nll_naive(theta):
        # Naive implementation of Eq. 2.29. Works well for the examples 
        # in this article but is numerically less stable compared to 
        # the implementation in nll_stable below.
        K = kernel_gaussian(X_train, X_train, l=theta[0])
        return 0.5 * np.log(det(K)) + \
               0.5 * Y_train.T.dot(inv(K).dot(Y_train)) + \
               0.5 * len(X_train) * np.log(2*np.pi)

    def nll_stable(theta):
        # Numerically more stable implementation of Eq. (7) as described
        # in http://www.gaussianprocess.org/gpml/chapters/RW2.pdf, Section
        # 2.2, Algorithm 2.1.
        K = kernel_gaussian(X_train, X_train, l=theta[0])
        L = cholesky(K)
        return np.sum(np.log(np.diagonal(L))) + \
               0.5 * Y_train.T.dot(lstsq(L.T, lstsq(L, Y_train)[0])[0]) + \
               0.5 * len(X_train) * np.log(2*np.pi)
    
    if naive:
        return nll_naive
    else:
        return nll_stable

def posterior_predictive_noise(X_s, X_train, Y_train, l=1.0, sigma_f=1.0, sigma_y=1e-8):
    '''  
    Computes the suffifient statistics of the GP posterior predictive distribution 
    from m training data X_train and Y_train and n new inputs X_s.
    
    Args:
        X_s: New input locations (n x d).
        X_train: Training locations (m x d).
        Y_train: Training targets (m x 1).
        l: Kernel length parameter.
        sigma_f: Kernel vertical variation parameter.
        sigma_y: Noise parameter.
    
    Returns:
        Posterior mean vector (n x d) and covariance matrix (n x n).
    '''
    K = kernel(X_train, X_train, l, sigma_f) + sigma_y**2 * np.eye(len(X_train))
    K_s = kernel(X_train, X_s, l, sigma_f)
    K_ss = kernel(X_s, X_s, l, sigma_f) + 1e-8 * np.eye(len(X_s))
    K_inv = inv(K)
    
    # Equation (4)
    mu_s = K_s.T.dot(K_inv).dot(Y_train)

    # Equation (5)
    cov_s = K_ss - K_s.T.dot(K_inv).dot(K_s)
    
    return mu_s, cov_s
def posterior_predictive_noise_free(K_inv, X_s, X_train, Y_train, l):
    '''  
    Computes the suffifient statistics of the GP posterior predictive distribution 
    from m training data X_train and Y_train and n new inputs X_s.
    
    Args:
        X_s: New input locations (n x d).
        X_train: Training locations (m x d).
        Y_train: Training targets (m x 1).
        l: Kernel length parameter.
        sigma_f: Kernel vertical variation parameter.
    
    Returns:
        Posterior mean vector (n x d) and covariance matrix (n x n).
    '''
    #K = kernel_gaussian(X_train, X_train, l)
    #K = K + 0.1**2 * np.eye(len(X_train))
    K_s = kernel_gaussian(X_train, X_s, l)
    K_ss = kernel_gaussian(X_s, X_s, l) #+ 1e-8 * np.eye(len(X_s))#non zero?
    #K_inv = inv(K)
    
    # Equation (4)
    mu_s = K_s.T.dot(K_inv).dot(Y_train)

    # Equation (5)
    cov_s = K_ss - K_s.T.dot(K_inv).dot(K_s)
    
    return mu_s, cov_s
def compute_l(X_train, u, S, diag):
    #global lag
    #X_train: N*d
    #u: 1*d
    #l: N*1
    data_size = X_train.shape[0]
    dim = X_train.shape[1]
    l = np.zeros((data_size, 1))
    I = np.eye(dim)
    #print('diag', diag.shape, S.shape, I.shape)
    for i in range(data_size):
        temp = np.linalg.det(inv(diag).dot(S)+I)
        if temp > 0:
            l[i, 0] = 1/math.sqrt(temp) * math.exp(-0.5 * (u-X_train[i,:]).dot(inv(S+diag)).dot((u-X_train[i,:]).T))
    return l
def compute_beta(K_inv, Y_train):
    #beta, N*1
    return K_inv.dot(Y_train)#N*1
def compute_mean(beta, l):
    return beta.T.dot(l)[0][0]
def compute_variance(u, K_inv, X_train, beta, l, length, sigma_det, diag):
    #u 1*d
    #beta N*1
    #l N*1
    N = X_train.shape[0]
    dim = X_train.shape[1]
    I = np.eye(dim)
    k = np.zeros((N,1))

    for i in range(N):
        tsts = X_train.T[:, i].reshape(dim, 1)
        #tsts = tsts.reshape(dim, 1)
        #print('err', u.shape, tsts)
        k[i, 0] = kernel_gaussian(u, X_train.T[:, i].reshape(1, dim), length)[0][0]
    #print(u, X_train.T[:, -1].reshape(1, dim))
    #print('small k', k)
    L = np.zeros((N, N))
    for i in range(N):
        for j in range(N):
            xd = 0.5*(X_train[i, :]+X_train[j, :])#1*d
            temp = np.linalg.det(2 * inv(diag).dot(S) + I)
            if temp > 0:
                L[i,j] = k[i, 0]*k[j, 0]*1/math.sqrt(temp) * math.exp(2 * (u-xd).dot(inv(diag)).dot(diag_inv(2*inv(diag)+diag_inv(S))).dot(inv(diag)).dot((u-xd).T))
    v = sigma_det + np.trace(K_inv.dot(k.dot(k.T) - L)) + np.trace(beta.dot(beta.T).dot(L - l.dot(l.T)))
    #print('trace1', np.trace(K_inv.dot(k.dot(k.T) - L)))
    #print(beta)
    #print('trace2', np.trace(beta.dot(beta.T).dot(L - l.dot(l.T))))
    #print('K-inv', K_inv)
    return v
def diag_inv(M):
    for i in range(M.shape[0]):
        if M[i][i] == 0:
            continue
        else:
           M[i][i] = 1.0/M[i][i]
    return M 
def compute_c(diag, S, X_train, u):
    #diag dim*dim
    #S dim*dim
    #X_train N*dim
    #u 1*dim
    C = diag_inv(diag_inv(diag)+diag_inv(S))
    N = X_train.shape[0]
    dim = X_train.shape[1]
    CC = np.zeros((N, N))
    u_matrix = np.zeros((dim, N))
    for i in range(N):
        #print(u_matrix[:, i].shape, u[0,:].shape)
        u_matrix[:, i] = u[0,:]
    return C.dot(inv(diag).dot(X_train.T)+diag_inv(S).dot(u_matrix))#dim*N
