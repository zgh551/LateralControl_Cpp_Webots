/*
 * linear_quadratic_regulator.h.h
 *
 *  Created on: January 2 2020
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: linear_quadratic_regulator.h        COPYRIGHT (c) Motovis 2020 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the vector property             					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 2 2019      Initial Version                  */
/*****************************************************************************/
#ifndef LINEARQUADRATICREGULATOR_H
#define LINEARQUADRATICREGULATOR_H

/**
 * @brief Eigen矩阵库
 */
#include "Eigen/Dense"

namespace math
{
    /**
     * @brief Solver for discrete-time linear quadratic problem.
     * @param A The system dynamic matrix
     * @param B The control matrix
     * @param Q The cost matrix for system state
     * @param R The cost matrix for control output
     * @param tolerance The numerical tolerance for solving Discrete
     *        Algebraic Riccati equation (DARE)
     * @param max_num_iteration The maximum iterations for solving ARE
     * @param ptr_K The feedback control matrix (pointer)
     */
    void SolveLQR(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                  const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                  const double tolerance, const uint16_t max_num_iteration,
                  Eigen::MatrixXd *ptr_k);
}

#endif // LINEARQUADRATICREGULATOR_H
