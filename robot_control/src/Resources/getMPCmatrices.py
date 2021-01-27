
# Author: Lasitha Wijayarathne

import numpy as np
from scipy.linalg import block_diag


# Lifted matrices for MPC on horizon N
def createMPCmatrices(A, B, N, include_x0=1):
    
    n = np.size(A,0)
    Ab = np.zeros(((N+1)*n, n))
    Ab[0:n,:] = np.eye(n,n)

    for i in range(2, N+2): 
        Ab[(i-1)*n:i*n,:] = np.matmul(Ab[(i-2)*n:(i-1)*n,:], A)

    if(include_x0 == 0):
        Ab = Ab[n::, :]
    
    m = np.size(B,1)
    Bb = np.zeros(((N+1)*n, N*m));
    for i in range(2,N+2):
        Bb[(i-1)*n:i*n,:] = np.matmul(A, Bb[(i-2)*n:(i-1)*n,:])
        Bb[(i-1)*n:n*i,(i-2)*m:m*(i-1)] = B

    if(include_x0 == 0):
        Bb = Bb[n::, :]

    return Ab, Bb

# diagnolozes blocks repetitavily on the diagnal
def bdiag(A, N):
    X = A
    for i in range(0, N-1):
        X = block_diag(X, A)

    return X

def main():
    A = np.array([[2,3],[2,1]])
    B = np.array([[1],[2]])
    Ab, Bb = createMPCmatrices(A, B, 2, 0)
    print(Ab)
    print(Bb)

# if __name__== "__main__":
#     # main()
#     return 0