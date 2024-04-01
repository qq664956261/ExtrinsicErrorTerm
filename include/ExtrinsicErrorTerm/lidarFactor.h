
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include "sophus/se3.h"
inline Eigen::Matrix3d skewd(const Eigen::Vector3d &x)
{
	Eigen::Matrix3d skew1 = Eigen::Matrix3d::Zero();
	skew1(0, 1) = -x[2];
	skew1(0, 2) = x[1];
	skew1(1, 0) = x[2];
	skew1(1, 2) = -x[0];
	skew1(2, 0) = -x[1];
	skew1(2, 1) = x[0];

	return skew1;
}
struct sonarEdgeFactor
{
	sonarEdgeFactor(Eigen::Vector3d point_a, Eigen::Vector3d point_b, Eigen::Quaterniond wq, Eigen::Vector3d wt)
		: point_a_(point_a), point_b_(point_b), wq_(wq), wt_(wt) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_12{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_12{t[0], t[1], t[2]};
		Eigen::Quaternion<T> q_corrected{T(wq_.w()), T(wq_.x()), T(wq_.y()), T(wq_.z())};
		Eigen::Matrix<T, 3, 1> t_corrected{T(wt_[0]), T(wt_[1]), T(wt_[2])};
		Eigen::Matrix<T, 3, 1> pa{T(point_a_[0]), T(point_a_[1]), T(point_a_[2])};
		Eigen::Matrix<T, 3, 1> pb{T(point_b_[0]), T(point_b_[1]), T(point_b_[2])};
		Eigen::Matrix<T, 3, 1> pc;
		pc = q_corrected * q_12 * pa + q_corrected * t_12 + t_corrected;
		Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residual);
		residuals.template block<3, 1>(0, 0) = pc - pb;

		// residual[0] = (pc.x() - pb.x());
		// residual[1] = pc.y() - pb.y();
		// residual[2] = pc.z() - pb.z();
		Eigen::Matrix<T, 3, 3> sqrt_info = T(100) * Eigen::Matrix<T, 3, 3>::Identity();
		sqrt_info(0, 0) = T(1);
		sqrt_info(1, 1) = T(1);
		sqrt_info(2, 2) = T(0);

		residuals.applyOnTheLeft(sqrt_info);

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d point_a, const Eigen::Vector3d point_b, const Eigen::Quaterniond wq, const Eigen::Vector3d wt)
	{
		return (new ceres::AutoDiffCostFunction<
				sonarEdgeFactor, 3, 4, 3>(
			new sonarEdgeFactor(point_a, point_b, wq, wt)));
	}

	Eigen::Vector3d point_a_;
	Eigen::Vector3d point_b_;
	Eigen::Quaterniond wq_;
	Eigen::Vector3d wt_;
};
class sonarFactor : public ceres::SizedCostFunction<3, 6>
{
public:
	sonarFactor(const Eigen::Vector3d point_a, const Eigen::Vector3d point_b, const Eigen::Quaterniond wq, const Eigen::Vector3d wt) : point_a_(point_a), point_b_(point_b), wq_(wq), wt_(wt) {}

	virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
	{
		// Eigen::Vector3d t_12(parameters[0][4], parameters[0][5], parameters[0][6]);
		// Eigen::Quaterniond q_12(parameters[0][3], parameters[0][0], parameters[0][1], parameters[0][2]);
		Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(parameters[0]);
		Sophus::SE3 T12 = Sophus::SE3::exp(lie);
		Sophus::SO3 R12 = T12.so3();

		Eigen::Quaternion<double> q_corrected{wq_.w(), wq_.x(), wq_.y(), wq_.z()};
		Eigen::Matrix<double, 3, 1> t_corrected{wt_[0], wt_[1], wt_[2]};

		Sophus::SE3 T_corrected(q_corrected, t_corrected);
		Sophus::SO3 R_corrected = T_corrected.so3();
		Eigen::Matrix<double, 3, 1> pa{point_a_[0], point_a_[1], point_a_[2]};
		Eigen::Matrix<double, 3, 1> pb{point_b_[0], point_b_[1], point_b_[2]};
		Eigen::Matrix<double, 3, 1> pc;
		// pc = q_corrected * q_12 * pa + q_corrected * t_12 + t_corrected;
		pc = T_corrected * T12 * pa;
		
		residuals[0] = pc.x() - pb.x();
		residuals[1] = pc.y() - pb.y();
		residuals[2] = pc.z() - pb.z();

		Eigen::Matrix3d Antisymmetrypa;
		Antisymmetrypa = skewd(pa);

		if (jacobians)
		{

			if (jacobians[0])
			{
				Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> jacobian_pose_T(jacobians[0]);
				jacobian_pose_T.setZero();
				Eigen::Matrix<double, 3, 3> k;
				k << -1, -1, -1,
					-1, -1, -1,
					-1, -1, -1;

				// Eigen::Vector3d p_result = R_corrected * R12 * pa;
				Eigen::Vector3d p_result = R12 * pa;
				Eigen::Matrix3d Antisymmetrypresult;
				Antisymmetrypresult = skewd(p_result);
				Eigen::Matrix3d R_w_front(q_corrected);
				jacobian_pose_T.block<3, 3>(0, 0) = R_w_front;
				// jacobian_pose_T.block<3, 3>(0, 3) = -Antisymmetrypresult;
				jacobian_pose_T.block<3, 3>(0, 3) = R_w_front * -Antisymmetrypresult;

				//jacobian_pose_T.col(1).setZero();
				// jacobian_pose_T.col(2).setZero();
				// jacobian_pose_T.col(3).setZero();
				// jacobian_pose_T.col(4).setZero();
				// jacobian_pose_T.col(5).setZero();
				//jacobian_pose_T.col(6).setZero();
			}
		}

		// if (jacobians)
		// {

		// 	if (jacobians[0])
		// 	{
		// 		Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_pose_R(jacobians[0]);
		// 		jacobian_pose_R.setZero();
		// 		Eigen::Matrix<double, 3, 3> k;
		// 		k << -1, -1, -1,
		// 			-1, -1, -1,
		// 			-1, -1, -1;

		// 		//jacobian_pose_R.block<3, 3>(0, 0) = q_corrected * q_12 * Antisymmetrypa * k;
		// 		Eigen::Vector3d p_result = q_corrected * q_12 * pa;
		// 		Eigen::Matrix3d Antisymmetrypresult;

		// 		Antisymmetrypresult = skewd(p_result);
		// 		//jacobian_pose_R.block<3, 3>(0, 0) = k * Antisymmetrypresult;
		// 	}
		// 	if (jacobians[1])
		// 	{
		// 		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_pose_t(jacobians[1]);
		// 		jacobian_pose_t.setZero();
		// 		Eigen::Matrix3d R_w_front(q_corrected);
		// 		//jacobian_pose_t.block<3, 3>(0, 0) = R_w_front;
		// 	}
		// }

		return true;
	}
	Eigen::Vector3d point_a_;
	Eigen::Vector3d point_b_;
	Eigen::Quaterniond wq_;
	Eigen::Vector3d wt_;
};

class SE3Param : public ceres::LocalParameterization
{
public:
	SE3Param() {}
	virtual ~SE3Param(){};

	// 李代数左乘更新
	virtual bool Plus(const double *x,
					  const double *delta,
					  double *x_plus_delta) const
	{
		// 通过Eigen::Map将double数组映射到Eigen::Vector6d结构
		Eigen::Map<const Eigen::Matrix<double, 6, 1>> se3(x);
		Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta_se3(delta);
		Eigen::Map<Eigen::Matrix<double, 6, 1>> x_plus_delta_se3(x_plus_delta);

		x_plus_delta_se3 = (Sophus::SE3::exp(delta_se3) * Sophus::SE3::exp(se3)).log();
		return true;
	}

	// x对delta的雅克比矩阵
	virtual bool ComputeJacobian(const double *x,
								 double *jacobian) const
	{
		Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> j(jacobian);
		j.setIdentity();
		//(j.topRows(6)).setIdentity();
		// (j.bottomRows(1)).setZero();
		return true;
	}

	// 参数x的自由度（可能有冗余），对于se3是6,对于四元数是4
	virtual int GlobalSize() const { return 6; }

	// delta_x所在正切空间的自由度，对于se3是6,对于四元数是3
	virtual int LocalSize() const { return 6; }
};


