#!/usr/bin/env python

from qpsolvers import solve_qp
import numpy as np
import warnings
from qpsolvers import dense_solvers, sparse_solvers, available_solvers
import numpy.matlib
from scipy.linalg import block_diag
from getMPCmatrices import createMPCmatrices, bdiag
import matplotlib.pyplot as plt 
from scipy.sparse import csc_matrix
import timeit


class QP_MPC:
    # Horizon time steps
    N = 0
    # Linear matrices dimensions
    n = 0; m = 0;   p = 0
    # Linear matrices
    Ab = np.array([[]])
    Bb = np.array([[]])
    Cb = np.array([[]])
    # Quadratic cost terms
    Qb = np.array([[]])
    Rb = np.array([[]])
    # Linear cost terms 
    ulin = np.array([[]])
    qlin = np.array([[]])
    # Bounds on the states
    Aineq = np.array([[]])
    bineq = np.array([[]])
    # x0 = np.array([])
    # QP cost terms
    H = np.array([[]]) 
    f = np.array([[]])
    # Controller parameters
    M1 = np.array([[]]) 
    M2 = np.array([[]]) 

    U_curr = np.array([])


    def __init__(self, A, B, C, d, Q, R, QN, N, Ulb, Uub, Xlb, Xub, x0, yr, ulin, qlin):
        # solves MPC problem in dense formulation and returns the predicted
        # control sequence U, the predicted state sequence X as well as the
        # optimal value optval.

        # If you want to use MPC in closed loop with the same data (A,B,C,...),
        # then use getMPC.m instead (it's faster since all fixed data is precomputed and
        # the optimization problem is warm started).

        # Dynamics:
        # x^+ = A*x + B*u + d
        # y   = C*x
        # Cost:
        # J = (y_N - yr_N)'*Q_N*(y_N - yr_N) + sum_{i=0:N-1} [ (y_i - yr_i)'*Q*(y_i - yr_i) + u_i'*R*u_i + ulin_i'u + qlin'*y ]

        # INPUTS:

        # Xlb, Xub:  n x 1 or n x N matrix
        # Ulb, Uub: m x 1 vector or m x N matrix
        # ** If elements of Xlb, Xub, Ulb, Uub not present, set them to NaN or to +-Inf **

        # yr - n_outputs x 1 vector or n_outputs x N matrix

        # ulin - linear term in the cost
        # qlin - linear term in the cost

        # solver = 'qpoases'; %or quadprog
        start = timeit.timeit()
        self.N = N
        self.n = np.size(A, 0) # Number of states
        self.m = np.size(B, 1) # Numbet of control inputs
        self.p = np.size(C, 0) # Number of outputs


        if (len(Xub) or len(Xlb)):
            # if (np.size(Xub,1)  == 1 or np.size(Xlb,1)  == 1):
                # change it later
                # if( length(Xub) is not n or length(Xlb) is not n):
                #     error('The dimension of Xub or Xlb seems to be wrong')
            self.Xlb = np.matlib.repmat(Xlb, 1, N)
            self.Xub = np.matlib.repmat(Xub, 1, N)
        else:
            self.Xlb = np.array([])
            self.Xub = np.array([])


        if (len(Uub) or len(Ulb)): 
            # if (np.size(Uub,1)  == 1 or np.size(Ulb,1)  == 1):
                # if( numel(Uub) ~= m or numel(Ulb) ~= m):
                #     error('The dimension of Xub or Xlb seems to be wrong')
            self.Ulb = np.matlib.repmat(Ulb, 1, N)
            self.Uub = np.matlib.repmat(Uub, 1, N)
        else:
            self.Ulb = np.array([])
            self.Uub = np.array([])

        # make sure the input is valid
        # Xub[Xub ==  'Inf'] = np.nan
        # Xlb[Xlb == -'Inf'] = -np.nan

        # Not sure what this does - LW
        # Affine term in the dynamics - handled by state inflation
        # A = [A np.eye(n,n) ; np.zeros((n,n)) np.eye((n,n))]
        # B = [B ; np.zeros((n,m))]
        # C = [C np.zeros((p,n))]

        
        # Output reference
        # yr = np.ones((p * (N), 1))
        # else if(size(yr,1) == 1)
        #         yr = np.matlib.repmat(yr,N,1)
        #     else
        #         yr = yr(:)
       

        # Linear term in the cost
        self.ulin = np.zeros((self.m * (N), 1))

            # if(numel(ulin) == m)
            #     if (size(ulin,2) > size(ulin,1))
            #         ulin = ulin.T
                
            #     ulin = repmat(ulin,N,1);
            
        self.qlin = np.zeros((self.p * (self.N),1))

            # warning('Functionality of qlin not tested properly!')
            # if(numel(qlin) == p)
            #     qlin = repmat(qlin(:),N,1);
            # elseif(numel(qlin) == N*p)
            #     qlin = qlin(:);
            # else
            #     error('Wrong size of qlin')


        # Get A, B and C matrices for the whole horizon
        [self.Ab, self.Bb] = createMPCmatrices(A, B, self.N, include_x0=0)
        self.Cb = bdiag(C, self.N)

        # Get Qb, and Rb matrices
        self.Rb = bdiag(R, self.N)        
        self.Qb = np.zeros((self.p * (self.N), self.p * (self.N)))
        self.Qb[0:self.p*(self.N-1), 0:self.p*(self.N-1)] = bdiag(Q, self.N-1)
        self.Qb[-self.p-1:-1, -self.p-1:-1] = QN


        # Aineq = [Aineq; Bb]
        self.Aineq = self.Bb

        # bineq = [bineq; Xub(:) - np.matmul(Ab, x0)]
        self.bineq = self.Xub[:].T - np.matmul(self.Ab, x0)


        # Aineq = [Aineq; -Bb]
        self.Aineq = np.append(self.Aineq, -self.Bb, axis=0)
        # bineq = [bineq; -Xlb(:) + np.matmul(Ab, x0)]
        self.bineq = np.append(self.bineq, -self.Xlb[:].T + np.matmul(self.Ab, x0), axis=0)

        # Not sure what this does
        # Aineq = Aineq(~isnan(bineq), :)
        # bineq = bineq(~isnan(bineq), :)

        # # convert all the arrays to matrix form to ease the multiplication
        self.Ab = np.asmatrix(self.Ab)
        self.Bb = np.asmatrix(self.Bb) 
        self.Cb = np.asmatrix(self.Cb)
        self.Qb = np.asmatrix(self.Qb)
        self.Rb = np.asmatrix(self.Rb)
        self.x0 = np.asmatrix(x0)



        self.H = 2*(self.Bb.T*self.Cb.T*self.Qb*self.Cb*self.Bb + self.Rb)
        # self.f = (2*x0.T*self.Ab.T*self.Cb.T*self.Qb*self.Cb*self.Bb - 2*yr.T*self.Qb*self.Cb*self.Bb).T + self.ulin + self.Bb.T*(self.Cb.T*self.qlin)
        self.f = (2*x0.T*self.Ab.T*self.Cb.T*self.Qb*self.Cb*self.Bb).T + self.ulin + self.Bb.T*(self.Cb.T*self.qlin)
        # convert to an numpy array
        self.f = np.array(self.f)

        # build the controller
        self.M1 =  2 * (self.Bb.T*(self.Cb.T*self.Qb*self.Cb))*self.Ab
        self.M2 = -2 * ((self.Qb*self.Cb)*self.Bb).T

        end = timeit.timeit()
        print (end - start)

        # r_init = solve_qp(np.array((self.H + self.H.T)/2), self.f.flatten(), np.array(self.Aineq), self.bineq.flatten(), solver=available_solvers[2])

        # # optval = optval + x0.T*Q*x0;
        # if (res == None):
        #     optval = Inf;
        # res = res[1:m*N];

    # update the A matric based on the changing stiffness.
    def update_Ab(self, A):
        return True
    
    # takes in current x and y_ref and returns the optimal output.
    def mpc_rtn(self, x00, yrr, plot_=False):

        # x00, yrr, QP, N, Ab, Xlb, Xub, M1, M2, ulin, d, Ulb, Uub, p, C, Q)
        # repeating the reference

        start = timeit.timeit()

        Yrr = np.matlib.repmat(yrr, self.N, 1)

        # Linear part of the constratints
        # Can be significantly sped up by selecting by carrying out the
        # multiplication Ab*x0 only for those rows of Ab corresponding to non-nan
        # entries of Xub or Xlb
        
        # bineq = [];
        # if (~isempty(Xub))
        #     bineq = [bineq; Xub - Ab*x0];
        # end
        # if (~isempty(Xlb))
        #     bineq = [bineq; -Xlb + Ab*x0];
        # end

        # bineq = bineq(~isnan(bineq),:);

        # Linear part of the constraints
        # Linear part of the objective function

        f = np.asmatrix(self.M1) * np.asmatrix(x00) + np.asmatrix(self.M2) * Yrr # + self.ulin;

        r_curr = solve_qp(np.array((self.H + self.H.T)/2), f, np.array(self.Aineq), self.bineq.flatten(), solver=available_solvers[2])

        self.X = self.Ab*np.asmatrix(x00) + self.Bb*np.matrix(r_curr).T
        self.U_curr = r_curr.reshape(self.m, self.N);
        self.X = self.X.reshape(self.n, -1, order='F')

        end = timeit.timeit()
        print (end - start)


        if plot_ is True:
            plot(np.array(self.U_curr), np.array(self.X))

        # u = u[1:numel(ulin)/N]
        # y = C*x0
        # optval = optval + y.T * Q * y.T*Q*y

        return self.U_curr, self.X

def main():

    # define all the matrices.
    # A   = np.array([[1,0.001],[0,1]])
    # B   = np.array([[0,1],[1,0]])
    # C   = np.eye(2)
    # d   = []
    # Q   = np.array([[1,0],[0,1]])
    # R   = np.array([[0.1,0],[0,0.9]])
    # QN  = Q
    # Ulb = np.array([-5.0])
    # Uub = np.array([5.0])
    # Xlb = np.array([-1.0,-1.0])
    # Xub = np.array([1.0, 1.0])
    # x0  = np.array([[0.5,0.0]]).T

    #-------------------------------------------------------------------------------------------------------------------#
    A = np.zeros((16,16))
    K = 250
    deltaT = 0.01

    A[0,:] = np.array([1,0,0,0,0,0,0,0,0,0,0,K*deltaT, 0, 0, -K*deltaT,0])
    A[1:11, 2:12] = np.eye(10)
    A[-1-4,-1-4] = 1
    A[-1-4,-1-3] = deltaT
    A[-1-3, 1:11] = np.array([-3.2554, 3.3643,-0.5260,-0.8991, 0.3114,-0.2502, 0.1744, 0.4726, -0.5620,  0.1701])

    A[-1-2,:] = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,1,deltaT,0.001])
    A[-1-1,:] = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,deltaT])

    C   = np.array([[1],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]]).T
    B   = np.array([[0], [0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[1]])
    d   = []
    Q   = np.array([[1]])
    R   = np.array([[6.9]])
    QN  = Q

    # m = 1
    Ulb = np.array([[-4.0]])
    Uub = np.array([[4.0]])

    # n = 16
    Xlb = -1 * 5 * np.ones((1,16))  
    Xub =  1 * 5 * np.ones((1,16))  


    x0  = np.array([[0.0],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.]])
    #-------------------------------------------------------------------------------------------------------------------#


    yr   = np.array([[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]])
    ulin = []
    qlin = []
    N    = 100

    # print(quadprog_solve_qp(P, q, G, h))
    QP = QP_MPC(A, B, C, d, Q, R, QN, N, Ulb, Uub, Xlb, Xub, x0, yr, ulin, qlin)
    QP.mpc_rtn(x0, np.array([[-3]]))
    # QP.mpc_rtn(x0, np.array([[-3],[1]]))

    # U, X = solveMPCprob(A, B, C, d, Q, R, QN, N, Ulb, Uub, Xlb, Xub, x0, yr, ulin, qlin)
    # plot(np.array(U), np.array(X))

def plot(U, X):

    plt.figure()
    ts = U.shape[1]
    m  = U.shape[0]
    n  = X.shape[0]
    t  = np.linspace(0, ts, num=ts)

    # plot the states
    for i in range(0, n):
        plt.subplot(n,1,i+1)
        plt.plot(t, X[i,:], '-', lw=2)
        plt.title('States')
        plt.xlabel('time (s)')
        plt.ylabel('State Units ()')

    # plot the input
    plt.figure()
    for i in range(0, m):
        plt.subplot(m,1,i+1)
        plt.plot(t, U[i,:], '-', lw=2)
        plt.title('Control Input')
        plt.xlabel('time (s)')
        plt.ylabel('Input Units ()')
    # plt.legend(['u_1','u_2'])

    plt.tight_layout()
    plt.show()


if __name__== "__main__":
    # instantialte the class
    main()