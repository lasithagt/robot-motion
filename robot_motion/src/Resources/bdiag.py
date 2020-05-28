import numpy as np
from scipy.linalg import block_diag

def bdiag(A, N):
	X = A
	for i in range(0, N-1):
	    X = block_diag(X, A)

	return X

def main():
    A = np.array([[2,3],[2,1]])
    An = bdiag(A, 2)
    print(An)

if __name__== "__main__":
    main()