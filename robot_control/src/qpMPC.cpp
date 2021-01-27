/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2015 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file examples/example1.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.1
 *	\date 2007-2015
 *
 *	Very simple example for testing qpOASES using the QProblem class.
 */

#include <qpOASES.hpp>

void solve_MPC(Eigen::MatrixXd &A, Eigen::MatrixXd &B, Eigen::VectorXd &Xlb, Eigen::VectorXd &Xub, Eigen::VectorXd &Ulb, Eigen::VectorXd &Uub, int N) {

    int n = A.cols();    // Number of states
    int m = B.cols();    // Numbet of control inputs

    C = MatrixXd::Identity(n, n);

    p = C.cols(); // Number of outputs

    if (Xub.size() || Xlb.size()) {
        // if (np.size(Xub,1)  == 1 or np.size(Xlb,1)  == 1):
            // change it later
            // if( length(Xub) is not n or length(Xlb) is not n):
            //     error('The dimension of Xub or Xlb seems to be wrong')
        Xlb = Xlb.replicate(N,1).eval();
        Xub = Xub.replicate(N,1).eval();
    } else {
        // Xlb = 
        // Xub = 
    }


    if (Uub.size() || Ulb.size()) {
        // if (np.size(Uub,1)  == 1 or np.size(Ulb,1)  == 1):
            // if( numel(Uub) ~= m or numel(Ulb) ~= m):
            //     error('The dimension of Xub or Xlb seems to be wrong')
        Ulb = Ulb.replicate(N,1).eval();
        Uub = Uub.replicate(N,1).eval();
    } else {
        // Ulb = np.array([])
        // Uub = np.array([])
    }

    // make sure the input is valid
    // Xub[Xub ==  'Inf'] = np.nan
    // Xlb[Xlb == -'Inf'] = -np.nan

    // Not sure what this does
    // Affine term in the dynamics - handled by state inflation
    // A = [A np.eye(n,n) ; np.zeros((n,n)) np.eye((n,n))]
    // B = [B ; np.zeros((n,m))]
    // C = [C np.zeros((p,n))]

    
    n = np.size(A,1);
    // x0 = [x0;d];

    // Output reference
    yr = np.zeros((p * (N), 1))
    # else if(size(yr,1) == 1)
    #         yr = np.matlib.repmat(yr,N,1)
    #     else
    #         yr = yr(:)
   

    # Linear term in the cost
    ulin = np.zeros((m * (N), 1))

        # if(numel(ulin) == m)
        #     if (size(ulin,2) > size(ulin,1))
        #         ulin = ulin.T
            
        #     ulin = repmat(ulin,N,1);
        
    qlin = np.zeros((p * (N),1))

        # warning('Functionality of qlin not tested properly!')
        # if(numel(qlin) == p)
        #     qlin = repmat(qlin(:),N,1);
        # elseif(numel(qlin) == N*p)
        #     qlin = qlin(:);
        # else
        #     error('Wrong size of qlin')

    # Get A and B matrices for the whole horizon

    [Ab, Bb] = createMPCmatrices(A, B, N, include_x0=0)

    Qb = np.zeros((p * (N), p * (N)))

    Qb[0:p*(N-1), 0:p*(N-1)] = bdiag(Q, N-1)

    Qb[-p-1:-1, -p-1:-1] = QN

    
    Cb = bdiag(C, N)
    Rb = bdiag(R, N)

    # % Bounds on the states
    Aineq = np.array([[]])
    bineq = np.array([[]])

    # Aineq = [Aineq; Bb]
    Aineq = Bb
    # bineq = [bineq; Xub(:) - np.matmul(Ab, x0)]
    bineq = Xub[:].T - np.matmul(Ab, x0)


    # Aineq = [Aineq; -Bb]
    Aineq = np.append(Aineq, -Bb, axis=0)
    # bineq = [bineq; -Xlb(:) + np.matmul(Ab, x0)]
    bineq = np.append(bineq, -Xlb[:].T + np.matmul(Ab, x0), axis=0)

    # Not sure what this does
    # Aineq = Aineq(~isnan(bineq), :)
    # bineq = bineq(~isnan(bineq), :)

    # # convert all the arrays to matrix form to ease the multiplication
    Ab = np.asmatrix(Ab)
    Bb = np.asmatrix(Bb)
    Cb = np.asmatrix(Cb)
    Qb = np.asmatrix(Qb)
    Rb = np.asmatrix(Rb)
    yr = np.asmatrix(yr)
    x0 = np.asmatrix(x0)


    H = 2*(Bb.T*Cb.T*Qb*Cb*Bb + Rb)
    f = (2*x0.T*Ab.T*Cb.T*Qb*Cb*Bb - 2*yr.T*Qb*Cb*Bb).T + ulin + Bb.T*(Cb.T*qlin)
    f = np.array(f)

    # print(H.shape)
    # print(np.array(f).flatten().shape)
    # print(Aineq.shape)
    # print(bineq.flatten().shape)


    res = solve_qp(np.array((H + H.T)/2), f.flatten(), np.array(Aineq), bineq.flatten(), solver=available_solvers[0])
    # # optval = optval + x0.T*Q*x0;
    # if (res == None):
    #     optval = Inf;

    # res = res[1:m*N];

    X = Ab*x0 + Bb*np.matrix(res).T

    U = res.reshape(m, N);
    X = X.reshape(n, -1, order='F')

    return U, X

}


/** Example for qpOASES main function using the QProblem class. */
int main( )
{
	USING_NAMESPACE_QPOASES

	/* Setup data of first QP. */
	real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t A[1*2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 };

	/* Setup data of second QP. */
	real_t g_new[2] = { 1.0, 1.5 };
	real_t lb_new[2] = { 0.0, -1.0 };
	real_t ub_new[2] = { 5.0, -0.5 };
	real_t lbA_new[1] = { -2.0 };
	real_t ubA_new[1] = { 1.0 };


	/* Setting up QProblem object. */
	QProblem example( 2,1 );

	Options options;
	example.setOptions( options );

	/* Solve first QP. */
	int nWSR = 10;
	example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

	/* Get and print solution of first QP. */
	real_t xOpt[2];
	real_t yOpt[2+1];
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
			xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );
	
	/* Solve second QP. */
	nWSR = 10;
	example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );

	/* Get and print solution of second QP. */
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
			xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );

	example.printOptions();
	/*example.printProperties();*/

	return 0;
}


/*
 *	end of file
 */
