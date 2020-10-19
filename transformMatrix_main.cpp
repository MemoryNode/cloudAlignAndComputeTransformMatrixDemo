#include<iostream>
#include<Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int getTransformMatrixAtoB(const Eigen::Quaterniond& qA, const Eigen::Vector3d& pA,
	const Eigen::Quaterniond& qB, const Eigen::Vector3d& pB, Eigen::Matrix4d& rt);

Eigen::Matrix4f quaternionToRot(float q[4]);

void rotToQuaternion(float R[3][3], float q[4]);

int main()
{
	double yaw = M_PI / 3, pitching = M_PI / 4, droll = M_PI / 6;


	//创建测试用的旋转矩阵，用于构建四元数
	Eigen::Vector3d ea0(yaw, pitching, droll);
	Eigen::Vector3d ea1(yaw + (M_PI / 6), pitching + (M_PI / 7), droll + (M_PI / 8));

	Eigen::Matrix3d R, R1;
	R = Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(ea0[1], Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX());

	R1 = Eigen::AngleAxisd(ea1[0], Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(ea1[1], Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(ea1[2], Eigen::Vector3d::UnitX());

	std::cout << R << std::endl << std::endl;
	std::cout << R1 << std::endl << std::endl;
	
	Eigen::Quaterniond qA;
	qA = R;
	Eigen::Quaterniond qB;
	qB = R1;
	Eigen::Vector3d pA(1.0f, 1.0f, 1.0f);
	Eigen::Vector3d pB(1.5f, 2.0f, 3.7f);
	Eigen::Matrix4d rt;

	getTransformMatrixAtoB(qA, pA, qB, pB, rt);

	std::cout << "q( w,x,y,z ), p( x,y,z )" << std::endl;
	std::cout << "qA( " << qA.w() << ',' << qA.x() << ',' << qA.y() << ',' << qA.z() 
		<< " ), pA( " << pA[0] << ',' << pA[1] << ',' << pA[2] << " )" << std::endl;
	std::cout << "qB( " << qB.w() << ',' << qB.x() << ',' << qB.y() << ',' << qB.z()
		<< " ), pB( " << pB[0] << ',' << pB[1] << ',' << pB[2] << " )" << std::endl << std::endl;
	
	std::cout <<"A to B rt: \n"<< rt << std::endl << std::endl;

	getchar();
	return 0;
}


int getTransformMatrixAtoB(const Eigen::Quaterniond& qA, const Eigen::Vector3d& pA,
	const Eigen::Quaterniond& qB, const Eigen::Vector3d& pB, Eigen::Matrix4d& rt)
{
	Eigen::Matrix4d rtA = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d rtB = Eigen::Matrix4d::Identity();
	rtA.block(0, 0, 3, 3) = qA.toRotationMatrix();
	rtA(0, 3) = pA.x();
	rtA(1, 3) = pA.y();
	rtA(2, 3) = pA.z();

	rtB.block(0, 0, 3, 3) = qB.toRotationMatrix();
	rtB(0, 3) = pB.x();
	rtB(1, 3) = pB.y();
	rtB(2, 3) = pB.z();

	rt = rtB*(rtA.inverse());

	return 0;
}


Eigen::Matrix4f quaternionToRot(float q[4])       //四元数转旋转矩阵
{
	Eigen::Matrix4f Rt;
	Rt(0, 0) = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	Rt(0, 1) = 2.0*(q[1] * q[2] - q[0] * q[3]);
	Rt(0, 2) = 2.0*(q[0] * q[2] + q[1] * q[3]);
	Rt(0, 3) = 0;
	Rt(1, 0) = 2.0*(q[0] * q[3] + q[1] * q[2]);
	Rt(1, 1) = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
	Rt(1, 2) = 2.0*(q[2] * q[3] - q[0] * q[1]);
	Rt(1, 3) = 0;
	Rt(2, 0) = 2.0*(q[1] * q[3] - q[0] * q[2]);
	Rt(2, 1) = 2.0*(q[0] * q[1] + q[2] * q[3]);
	Rt(2, 2) = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
	Rt(2, 3) = 0;
	Rt(3, 0) = 0;
	Rt(3, 1) = 0;
	Rt(3, 2) = 0;
	Rt(3, 3) = 1;

	return Rt;
}

void rotToQuaternion(float R[3][3], float q[4])   //旋转矩阵转四元数
{
	float tr = R[0][0] + R[1][1] + R[2][2];
	float temp = 0.0;
	if (tr > 0.0)
	{
		temp = 0.5f / sqrtf(tr + 1);
		q[0] = 0.25f / temp;
		q[1] = (R[2][1] - R[1][2]) * temp;
		q[2] = (R[0][2] - R[2][0]) * temp;
		q[3] = (R[1][0] - R[0][1]) * temp;
	}
	else
	{
		if (R[0][0] > R[1][1] && R[0][0] > R[2][2])
		{
			temp = 2.0f * sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]);
			q[0] = (R[2][1] - R[1][2]) / temp;
			q[1] = 0.25f * temp;
			q[2] = (R[0][1] + R[1][0]) / temp;
			q[3] = (R[0][2] + R[2][0]) / temp;
		}
		else if (R[1][1] >R[2][2])
		{
			temp = 2.0f * sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]);
			q[0] = (R[0][2] - R[2][0]) / temp;
			q[1] = (R[0][1] + R[1][0]) / temp;
			q[2] = 0.25f * temp;
			q[3] = (R[1][2] + R[2][1]) / temp;
		}
		else
		{
			temp = 2.0f * sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]);
			q[0] = (R[1][0] - R[0][1]) / temp;
			q[1] = (R[0][2] + R[2][0]) / temp;
			q[2] = (R[1][2] + R[2][1]) / temp;
			q[3] = 0.25f * temp;
		}
	}

}