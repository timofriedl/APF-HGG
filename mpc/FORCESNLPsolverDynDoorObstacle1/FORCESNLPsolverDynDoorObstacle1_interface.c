/*
 * AD tool to FORCESPRO Template - missing information to be filled in by createADTool.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2023. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "include/FORCESNLPsolverDynDoorObstacle1.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif

#include "FORCESNLPsolverDynDoorObstacle1_model.h"



/* copies data from sparse matrix into a dense one */
static void FORCESNLPsolverDynDoorObstacle1_sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, FORCESNLPsolverDynDoorObstacle1_callback_float *data, FORCESNLPsolverDynDoorObstacle1_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((FORCESNLPsolverDynDoorObstacle1_float) data[j]);
        }
    }
}




/* AD tool to FORCESPRO interface */
extern solver_int32_default FORCESNLPsolverDynDoorObstacle1_adtool2forces(FORCESNLPsolverDynDoorObstacle1_float *x,        /* primal vars                                         */
                                 FORCESNLPsolverDynDoorObstacle1_float *y,        /* eq. constraint multiplers                           */
                                 FORCESNLPsolverDynDoorObstacle1_float *l,        /* ineq. constraint multipliers                        */
                                 FORCESNLPsolverDynDoorObstacle1_float *p,        /* parameters                                          */
                                 FORCESNLPsolverDynDoorObstacle1_float *f,        /* objective function (scalar)                         */
                                 FORCESNLPsolverDynDoorObstacle1_float *nabla_f,  /* gradient of objective function                      */
                                 FORCESNLPsolverDynDoorObstacle1_float *c,        /* dynamics                                            */
                                 FORCESNLPsolverDynDoorObstacle1_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 FORCESNLPsolverDynDoorObstacle1_float *h,        /* inequality constraints                              */
                                 FORCESNLPsolverDynDoorObstacle1_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 FORCESNLPsolverDynDoorObstacle1_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* AD tool input and output arrays */
    const FORCESNLPsolverDynDoorObstacle1_callback_float *in[4];
    FORCESNLPsolverDynDoorObstacle1_callback_float *out[7];
    

    /* Allocate working arrays for AD tool */
    
    FORCESNLPsolverDynDoorObstacle1_callback_float w[64];
	
    /* temporary storage for AD tool sparse output */
    FORCESNLPsolverDynDoorObstacle1_callback_float this_f = (FORCESNLPsolverDynDoorObstacle1_callback_float) 0.0;
    FORCESNLPsolverDynDoorObstacle1_float nabla_f_sparse[10];
    FORCESNLPsolverDynDoorObstacle1_float h_sparse[3];
    FORCESNLPsolverDynDoorObstacle1_float nabla_h_sparse[12];
    FORCESNLPsolverDynDoorObstacle1_float c_sparse[1];
    FORCESNLPsolverDynDoorObstacle1_float nabla_c_sparse[1];
            
    
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
		FORCESNLPsolverDynDoorObstacle1_objective_0(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = FORCESNLPsolverDynDoorObstacle1_objective_0_sparsity_out(1)[0];
			ncol = FORCESNLPsolverDynDoorObstacle1_objective_0_sparsity_out(1)[1];
			colind = FORCESNLPsolverDynDoorObstacle1_objective_0_sparsity_out(1) + 2;
			row = FORCESNLPsolverDynDoorObstacle1_objective_0_sparsity_out(1) + 2 + (ncol + 1);
			FORCESNLPsolverDynDoorObstacle1_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		FORCESNLPsolverDynDoorObstacle1_rkfour_0(x, p, c, nabla_c, FORCESNLPsolverDynDoorObstacle1_cdyn_0rd_0, FORCESNLPsolverDynDoorObstacle1_cdyn_0, threadID);
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		FORCESNLPsolverDynDoorObstacle1_inequalities_0(in, out, NULL, w, 0);
		if( h )
		{
			nrow = FORCESNLPsolverDynDoorObstacle1_inequalities_0_sparsity_out(0)[0];
			ncol = FORCESNLPsolverDynDoorObstacle1_inequalities_0_sparsity_out(0)[1];
			colind = FORCESNLPsolverDynDoorObstacle1_inequalities_0_sparsity_out(0) + 2;
			row = FORCESNLPsolverDynDoorObstacle1_inequalities_0_sparsity_out(0) + 2 + (ncol + 1);
			FORCESNLPsolverDynDoorObstacle1_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = FORCESNLPsolverDynDoorObstacle1_inequalities_0_sparsity_out(1)[0];
			ncol = FORCESNLPsolverDynDoorObstacle1_inequalities_0_sparsity_out(1)[1];
			colind = FORCESNLPsolverDynDoorObstacle1_inequalities_0_sparsity_out(1) + 2;
			row = FORCESNLPsolverDynDoorObstacle1_inequalities_0_sparsity_out(1) + 2 + (ncol + 1);
			FORCESNLPsolverDynDoorObstacle1_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
	if ((7 == stage))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		FORCESNLPsolverDynDoorObstacle1_objective_1(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = FORCESNLPsolverDynDoorObstacle1_objective_1_sparsity_out(1)[0];
			ncol = FORCESNLPsolverDynDoorObstacle1_objective_1_sparsity_out(1)[1];
			colind = FORCESNLPsolverDynDoorObstacle1_objective_1_sparsity_out(1) + 2;
			row = FORCESNLPsolverDynDoorObstacle1_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			FORCESNLPsolverDynDoorObstacle1_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		FORCESNLPsolverDynDoorObstacle1_inequalities_1(in, out, NULL, w, 0);
		if( h )
		{
			nrow = FORCESNLPsolverDynDoorObstacle1_inequalities_1_sparsity_out(0)[0];
			ncol = FORCESNLPsolverDynDoorObstacle1_inequalities_1_sparsity_out(0)[1];
			colind = FORCESNLPsolverDynDoorObstacle1_inequalities_1_sparsity_out(0) + 2;
			row = FORCESNLPsolverDynDoorObstacle1_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			FORCESNLPsolverDynDoorObstacle1_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = FORCESNLPsolverDynDoorObstacle1_inequalities_1_sparsity_out(1)[0];
			ncol = FORCESNLPsolverDynDoorObstacle1_inequalities_1_sparsity_out(1)[1];
			colind = FORCESNLPsolverDynDoorObstacle1_inequalities_1_sparsity_out(1) + 2;
			row = FORCESNLPsolverDynDoorObstacle1_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
			FORCESNLPsolverDynDoorObstacle1_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((FORCESNLPsolverDynDoorObstacle1_float) this_f);
    }

    return 0;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
