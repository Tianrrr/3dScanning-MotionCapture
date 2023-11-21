#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function (check gaussian.cpp for reference)
struct RegistrationCostFunction
{
	RegistrationCostFunction(const Point2D& point_p_,const Point2D& point_q_,const Weight& w_)
		: point_p(point_p_),point_q(point_q_),w(w_)
	{
	}

	template<typename T>
	bool operator()(const T* const angle, const T* const tx, const T* const ty, T* residual) const
	{
		// TODO: Implement the cost function
		const T trans_x = cos(angle[0])*point_p.x - sin(angle[0])*point_p.y + tx[0];
		const T trans_y = sin(angle[0])*point_p.x + cos(angle[0])*point_p.y + ty[0];
		residual[0] = w.w * ((trans_x - point_q.x)*(trans_x - point_q.x) + (trans_y- point_q.y)*(trans_y- point_q.y));
		return true;
	}


private:
	const Point2D point_p;
	const Point2D point_q;
	const Weight w;

};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// Read data points and the weights, and define the parameters of the problem
	const std::string file_path_1 = "../../Data/points_dragon_1.txt";
	const auto points1 = read_points_from_file<Point2D>(file_path_1);
	
	const std::string file_path_2 = "../../Data/points_dragon_2.txt";
	const auto points2 = read_points_from_file<Point2D>(file_path_2);
	
	const std::string file_path_weights = "../../Data/weights_dragon.txt";
	const auto weights = read_points_from_file<Weight>(file_path_weights);
	
	const double angle_initial = 0.0;
	const double tx_initial = 0.0;
	const double ty_initial = 0.0;
	
	double angle = angle_initial;
	double tx = tx_initial;
	double ty = ty_initial;

	ceres::Problem problem;

	// TODO: For each weighted correspondence create one residual block (check gaussian.cpp for reference)
	for (unsigned int i = 0;i<points1.size();i++)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<RegistrationCostFunction, 1, 1, 1, 1>(
				new RegistrationCostFunction(points1[i],points2[i],weights[i])
			),
			nullptr, &angle, &tx, &ty
		);
	}


	ceres::Solver::Options options;
	options.max_num_iterations = 25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	// Output the final values of the translation and rotation (in degree)
	std::cout << "Initial angle: " << angle_initial << "\ttx: " << tx_initial << "\tty: " << ty_initial << std::endl;
	std::cout << "Final angle: " << std::fmod(angle * 180 / M_PI, 360.0) << "\ttx: " << tx << "\tty: " << ty << std::endl;

	system("pause");
	return 0;
}
