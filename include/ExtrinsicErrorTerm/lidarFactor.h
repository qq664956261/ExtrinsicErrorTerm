
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>

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
		pc = q_corrected *  q_12 * pa + q_corrected * t_12 + t_corrected;

		residual[0] = pc.x() - pb.x();
		residual[1] = pc.y() - pb.y();
		residual[2] = pc.z() - pb.z();

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
