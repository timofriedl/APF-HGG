/*
 * AD tool to FORCESPRO Template - missing information to be filled in by createADTool.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2023. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "include/FORCESNLPsolverDynObstacle1.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif

#include "FORCESNLPsolverDynObstacle1_model.h"



/* copies data from sparse matrix into a dense one */
static void FORCESNLPsolverDynObstacle1_sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, FORCESNLPsolverDynObstacle1_callback_float *data, FORCESNLPsolverDynObstacle1_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((FORCESNLPsolverDynObstacle1_float) data[j]);
        }
    }
}




/* AD tool to FORCESPRO interface */
extern solver_int32_default FORCESNLPsolverDynObstacle1_adtool2forces(FORCESNLPsolverDynObstacle1_float *x,        /* primal vars                                         */
                                 FORCESNLPsolverDynObstacle1_float *y,        /* eq. constraint multiplers                           */
                                 FORCESNLPsolverDynObstacle1_float *l,        /* ineq. constraint multipliers                        */
                                 FORCESNLPsolverDynObstacle1_float *p,        /* parameters                                          */
                                 FORCESNLPsolverDynObstacle1_float *f,        /* objective function (scalar)                         */
                                 FORCESNLPsolverDynObstacle1_float *nabla_f,  /* gradient of objective function                      */
                                 FORCESNLPsolverDynObstacle1_float *c,        /* dynamics                                            */
                                 FORCESNLPsolverDynObstacle1_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 FORCESNLPsolverDynObstacle1_float *h,        /* inequality constraints                              */
                                 FORCESNLPsolverDynObstacle1_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 FORCESNLPsolverDynObstacle1_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* AD tool input and output arrays */
    const FORCESNLPsolverDynObstacle1_callback_float *in[4];
    FORCESNLPsolverDynObstacle1_callback_float *out[7];
    

    /* Allocate working arrays for AD tool */
    
    FORCESNLPsolverDynObstacle1_callback_float w[19];
	
    /* temporary storage for AD tool sparse output */
    FORCESNLPsolverDynObstacle1_callback_float this_f = (FORCESNLPsolverDynObstacle1_callback_float) 0.0;
    FORCESNLPsolverDynObstacle1_float nabla_f_sparse[7];
    FORCESNLPsolverDynObstacle1_float h_sparse[3];
    FORCESNLPsolverDynObstacle1_float nabla_h_sparse[8];
    FORCESNLPsolverDynObstacle1_float c_sparse[1];
    FORCESNLPsolverDynObstacle1_float nabla_c_sparse[1];
            
    
    /* pointers to row and column info for 
     * column compressed format used by AD tool */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for AD tool */
    in[0] = x;
    in[1] = p;
    in[2] = l;
    in[3] = y;

	if ((0 <= stage && stage <= 6))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		FORCESNLPsolverDynObstacle1_objective_0(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = FORCESNLPsolverDynObstacle1_objective_0_sparsity_out(1)[0];
			ncol = FORCESNLPsolverDynObstacle1_objective_0_sparsity_out(1)[1];
			colind = FORCESNLPsolverDynObstacle1_objective_0_sparsity_out(1) + 2;
			row = FORCESNLPsolverDynObstacle1_objective_0_sparsity_out(1) + 2 + (ncol + 1);
			FORCESNLPsolverDynObstacle1_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		FORCESNLPsolverDynObstacle1_rkfour_0(x, p, c, nabla_c, FORCESNLPsolverDynObstacle1_cdyn_0rd_0, FORCESNLPsolverDynObstacle1_cdyn_0, threadID);
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		FORCESNLPsolverDynObstacle1_inequalities_0(in, out, NULL, w, 0);
		if( h )
		{
			nrow = FORCESNLPsolverDynObstacle1_inequalities_0_sparsity_out(0)[0];
			ncol = FORCESNLPsolverDynObstacle1_inequalities_0_sparsity_out(0)[1];
			colind = FORCESNLPsolverDynObstacle1_inequalities_0_sparsity_out(0) + 2;
			row = FORCESNLPsolverDynObstacle1_inequalities_0_sparsity_out(0) + 2 + (ncol + 1);
			FORCESNLPsolverDynObstacle1_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = FORCESNLPsolverDynObstacle1_inequalities_0_sparsity_out(1)[0];
			ncol = FORCESNLPsolverDynObstacle1_inequalities_0_sparsity_out(1)[1];
			colind = FORCESNLPsolverDynObstacle1_inequalities_0_sparsity_out(1) + 2;
			row = FORCESNLPsolverDynObstacle1_inequalities_0_sparsity_out(1) + 2 + (ncol + 1);
			FORCESNLPsolverDynObstacle1_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
	if ((7 == stage))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		FORCESNLPsolverDynObstacle1_objective_1(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = FORCESNLPsolverDynObstacle1_objective_1_sparsity_out(1)[0];
			ncol = FORCESNLPsolverDynObstacle1_objective_1_sparsity_out(1)[1];
			colind = FORCESNLPsolverDynObstacle1_objective_1_sparsity_out(1) + 2;
			row = FORCESNLPsolverDynObstacle1_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			FORCESNLPsolverDynObstacle1_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		FORCESNLPsolverDynObstacle1_inequalities_1(in, out, NULL, w, 0);
		if( h )
		{
			nrow = FORCESNLPsolverDynObstacle1_inequalities_1_sparsity_out(0)[0];
			ncol = FORCESNLPsolverDynObstacle1_inequalities_1_sparsity_out(0)[1];
			colind = FORCESNLPsolverDynObstacle1_inequalities_1_sparsity_out(0) + 2;
			row = FORCESNLPsolverDynObstacle1_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			FORCESNLPsolverDynObstacle1_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = FORCESNLPsolverDynObstacle1_inequalities_1_sparsity_out(1)[0];
			ncol = FORCESNLPsolverDynObstacle1_inequalities_1_sparsity_out(1)[1];
			colind = FORCESNLPsolverDynObstacle1_inequalities_1_sparsity_out(1) + 2;
			row = FORCESNLPsolverDynObstacle1_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
			FORCESNLPsolverDynObstacle1_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((FORCESNLPsolverDynObstacle1_float) this_f);
    }

    return 0;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
