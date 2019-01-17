
#include <math.h>
#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;
using namespace std;



Matrix4d calc_RT(MatrixXd gps, MatrixXd orb, int MAX_ITERS);

Matrix4d calc_RT(MatrixXd gps, MatrixXd orb, int MAX_ITERS)
{


	int num_points = gps.cols();

	std::cout << "Number of points: " << num_points << std::endl;
	Vector3d mu_lidar, mu_camera;

	mu_lidar << 0.0, 0.0, 0.0;
	mu_camera << 0.0, 0.0, 0.0;

	for(int i=0; i<num_points; i++)
	{
		mu_lidar(0) += gps(0,i);
		mu_lidar(1) += gps(1,i);
		mu_lidar(2) += gps(2,i);
	}
	for(int i=0; i<num_points; i++)
	{
		mu_camera(0) += orb(0,i);
		mu_camera(1) += orb(1,i);
		mu_camera(2) += orb(2,i);
	}

	mu_lidar = mu_lidar/num_points;
	mu_camera = mu_camera/num_points;


	MatrixXd lidar_centered = gps.colwise() - mu_lidar;
	MatrixXd camera_centered = orb.colwise() - mu_camera;



	Matrix3d cov = camera_centered * lidar_centered.transpose();

	//std::cout << cov << std::endl;

	JacobiSVD<MatrixXd> svd(cov, ComputeFullU | ComputeFullV);

	Matrix3d rotation;
	rotation = svd.matrixU() * svd.matrixV().transpose();
	if (rotation.determinant() < 0)
	{
		Vector3d diag_correct;
		diag_correct << 1.0, 1.0, -1.0;

		rotation = svd.matrixU() * diag_correct.asDiagonal() * svd.matrixV().transpose();
	}

	Vector3d translation = mu_camera - rotation * mu_lidar;

	// std::cout << "Rotation matrix: \n"
	// 		  << rotation << std::endl;
	// std::cout << "Translation: \n"
	// 		  << translation << std::endl;

	MatrixXd eltwise_error = (orb - ((rotation * gps).colwise() + translation)).array().square().colwise().sum();
	double error = sqrt(eltwise_error.sum() / num_points);
	std::cout << "RMSE: " << error << std::endl;

	Matrix4d T;
	T.setIdentity(4, 4);
	T.topLeftCorner(3, 3) = rotation;
	T.col(3).head(3) = translation;
	T.col(3)[3] = error;




	return T;
}
