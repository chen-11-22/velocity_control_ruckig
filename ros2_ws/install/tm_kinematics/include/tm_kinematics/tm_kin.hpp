/*********************************************************************
 * tm_kin.h
 *
 * Copyright 2016 Copyright 2016 Techman Robot Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************
 * 
 * Author: Yun-Hsuan Tsai
 */

#ifndef TM_KIN_H
#define TM_KIN_H
#include <array>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

namespace tm_kinematics {
  
  void forward(const double* q, double* T);

  void forward_tool(const std::array<double, 6> q, Eigen::Matrix4d &T, double EElen);
  
  int inverse(const double* T, double* q_sols, const double* q_ref);
  
  int inverse(const double* T, double* q_sols, double q6_des = 0.0);

}


/*********************************************************************
 *
 * Copyright (c) 2017, ISCI / National Chiao Tung University (NCTU)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************
 * 
 * Author: Howard Chen
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <math.h>
#include <vector>

#include <eigen3/Eigen/Dense>

#define D1 0.1451
#define D2 0
#define D3 0
#define D4 -0.1222
#define D5 0.106
#define D6 0.1144

#define A1 0
#define A2 0.4290
#define A3 0.4115
#define A4 0
#define A5 0
#define A6 0


#define ALPHA1 -90
#define ALPHA2 0
#define ALPHA3 0 
#define ALPHA4 90
#define ALPHA5 90
#define ALPHA6 0

#define PI 3.141592654
#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951

namespace tm_jacobian {

  	Eigen::Matrix<float, 6, 6> Forward_Jacobian(Eigen::Matrix<float, 6,1> q);
	
	Eigen::Matrix<float, 6, 6> Inverse_Jacobian(Eigen::Matrix<float, 6,1> q);

  	Eigen::Matrix<double, 6, 6> Forward_Jacobian_d(Eigen::Matrix<float, 6,1> q);

	Eigen::Matrix<double, 6, 6> Forward_Jacobian_d(Eigen::Matrix<double, 6,1> q);
	
	Eigen::Matrix<double, 6, 6> Inverse_Jacobian_d(Eigen::Matrix<float, 6,1> q);

	Eigen::Matrix<double, 6, 6> Forward_Jacobian_gripper(Eigen::Matrix<float, 6,1> q);

	Eigen::Matrix<double, 6, 6> Forward_Jacobian_gripper(Eigen::Matrix<float, 6,1> q, double D7);

	Eigen::Matrix<double, 6, 6> Forward_Jacobian_tool(Eigen::Matrix<double, 6,1> q, double D7);

	Eigen::Matrix<double, 3, 2> Forward_Linear_Jacobian_3(Eigen::Matrix<float, 6,1> q);

	void Forward_Kinematics_3(const double* q, double* T);

	void Forward_Kinematics_3(const double* q, double* T, double y_offset);

	void Forward_Kinematics_4(const double* q, double* T);

	void Forward_Kinematics_gripper(const double* q, double* T, double Length);
	
	void printMatrix(Eigen::MatrixXf InputMatrix);
	
	void printMatrixd(Eigen::MatrixXd InputMatrix);
	
	void printMatrix(double *InputMatrix, short, int);

	void printVector(Eigen::VectorXd InputVector);

	void printVector(std::vector<double> vec);

	void Matrix2DoubleArray_d(Eigen::MatrixXd InputMatrix, double *T);
	
	void Matrix2DoubleArray(Eigen::MatrixXf InputMatrix, double *T);

	void Matrix2DoubleVector(Eigen::MatrixXf InputMatrix, std::vector<double> &vec);

	void Matrix2DoubleVector_d(Eigen::MatrixXd InputMatrix, std::vector<double> &vec);

	bool CheckJointLimit(double *q);

	bool CheckVelocityLimit(std::vector<double> qd);

	bool GetQfromInverseKinematics( std::vector<double> CartesianPosition, double *q_inv);

	bool GetQdfromInverseJacobian(std::vector<double> CurrentPosition,std::vector<double> EFF_Velocity, std::vector<double>& qd);
}

#endif //TM_KIN_H
