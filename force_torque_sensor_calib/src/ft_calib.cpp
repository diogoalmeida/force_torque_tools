/*
 *  ft_calib.cpp
 *
 *
 *  Created on: Sep 26, 2012
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2012, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <force_torque_sensor_calib/ft_calib.h>
#include <eigen3/Eigen/Dense>
#include <kdl/frameacc.hpp>
#include <kdl/frames.hpp>

namespace Calibration{

FTCalib::FTCalib()
{
	m_num_meas = 0;
  Sigma = Eigen::Matrix<double, 6, 6>::Identity();
  Lambda = 0.1*Eigen::Matrix<double, 6, 6>::Identity();
  phi = Eigen::Matrix<double, 4, 1>::Zero();
}

FTCalib::~FTCalib(){

}

void FTCalib::addMeasurement(const geometry_msgs::Vector3Stamped &gravity,
		const geometry_msgs::WrenchStamped &ft_raw)
{
	if(gravity.header.frame_id != ft_raw.header.frame_id)
	{
		ROS_ERROR("Gravity vector and ft raw expressed in different frames (%s, %s)!",
				gravity.header.frame_id.c_str(), ft_raw.header.frame_id.c_str());
		return;
	}

	m_num_meas++;

  Eigen::MatrixXd h = getMeasurementMatrix(gravity);
  Eigen::Matrix<double, 6, 6> K, I = Eigen::Matrix<double, 6, 6>::Identity();
	Eigen::VectorXd z = Eigen::Matrix<double, 6, 1>::Zero();
	z(0) = ft_raw.wrench.force.x;
	z(1) = ft_raw.wrench.force.y;
	z(2) = ft_raw.wrench.force.z;

	z(3) = ft_raw.wrench.torque.x;
	z(4) = ft_raw.wrench.torque.y;
	z(5) = ft_raw.wrench.torque.z;
  
  K = Sigma*h.transpose()*(h*Sigma*h.transpose() + Lambda).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(I);
  Sigma = (I - K*h)*Sigma;
  phi = phi + K*(z - h*phi);
}


// Least squares to estimate the FT sensor parameters
Eigen::VectorXd FTCalib::getCalib()
{
	return phi;
}


Eigen::MatrixXd FTCalib::getMeasurementMatrix(const geometry_msgs::Vector3Stamped &gravity_geo)
{
  Eigen::Vector3d gravity;
	
	Eigen::MatrixXd H;
	H = Eigen::Matrix<double, 6, 4>::Zero();

  gravity << gravity_geo.vector.x, gravity_geo.vector.y, gravity_geo.vector.z;
  
  H.block<3,1>(0,0) = -gravity;
  H.block<3,3>(3,1) << 0, -gravity[2], gravity[1],
                       gravity[2], 0, -gravity[0],
                       -gravity[1], gravity[0], 0;
                       
  std::cout << "Measurement Matrix" << std::endl;
  std::cout << H << std::endl << std::endl;                     
  
	return H;
}

}
