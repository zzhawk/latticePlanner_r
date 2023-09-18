// Copyright 2023 watson.wang

#include <corecrt_math_defines.h>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "lattice.hpp"


namespace fr {
	Trajectory FrenetPath::generatePath()
	{
		vector<Trajectory> trajs = generateTrajectories();
		calcGlobalPaths(trajs);
		checkPaths(trajs);
		
		return findOptimal(trajs);
	}

	vector<Trajectory> FrenetPath::generateTrajectories()
	{
		vector<Trajectory> trajs;

		for (double di = -_para.max_road_width; di < _para.max_road_width; di += _para.max_road_sample_width)
		{
			for (double ti = _para.min_pred_time; ti < _para.max_pred_time; ti += _para.time_tick)
			{
				Trajectory tj;
				tj.lateral_polynomial = QuinticPolynomial(_sts.d, _sts.d_d, _sts.d_dd, di, 0.0, 0.0, ti);

				auto square_sum = [](vector<double> n) {
					double ans = 0.0;
					for (int i = 0; i < n.size(); ++i) {
						ans += n[i] * n[i];
					}
					return ans;
				};

				for (double tv = _para.target_speed - _para.target_speed_sample * _para.target_speed_num;
					tv < _para.target_speed + _para.target_speed_sample * _para.target_speed_num;
					tv += _para.target_speed_sample)
				{
					tj.longitudinal_polynomial = QuarticPolynomial(_sts.s, _sts.s_d, _sts.s_dd, tv, 0.0, ti);
					for (double t = 0; t < ti; t += _para.time_tick)
					{
						tj.samples.d.push_back(tj.lateral_polynomial->position(t));
						tj.samples.d_d.push_back(tj.lateral_polynomial->velocity(t));
						tj.samples.d_dd.push_back(tj.lateral_polynomial->acceleration(t));
						tj.samples.d_ddd.push_back(tj.lateral_polynomial->jerk(t));

						tj.samples.s.push_back(tj.longitudinal_polynomial->position(t));
						tj.samples.s_d.push_back(tj.longitudinal_polynomial->velocity(t));
						tj.samples.s_dd.push_back(tj.longitudinal_polynomial->acceleration(t));
						tj.samples.s_ddd.push_back(tj.longitudinal_polynomial->jerk(t));
					}

					auto Jp = square_sum(tj.samples.d_ddd);
					auto Js = square_sum(tj.samples.s_ddd);

					auto dp = pow(tj.samples.d.back(), 2.0);
					auto ds = pow(_para.target_speed - tj.samples.s_d.back(), 2.0);

					tj.cd = _para.K_J * Jp + _para.K_T * ti + _para.K_D * dp;
					tj.cv = _para.K_J * Js + _para.K_T * ti + _para.K_D * ds;
					tj.cf = _para.K_LAT * tj.cd + _para.K_LON * tj.cv;


					trajs.push_back(tj);
				}
			}
		}

		return trajs;
	}

	void FrenetPath::calcGlobalPaths(vector<Trajectory>& trajs)
	{
		for (auto &tj : trajs) {
			for (int i = 0; i < tj.samples.s.size(); ++i) {
				es::SpiralPoint pos = es::getEndPoint(tj.samples.s[i], _rfl.dCurv, _rfl.initCurv, _para.start_x, _para.start_y, _para.start_yaw);
				auto temp = pos.x + tj.samples.d[i] * cos(pos.t + M_PI_2);
				tj.global.x.push_back(pos.x + tj.samples.d[i] * cos(pos.t + M_PI_2));
				tj.global.y.push_back(pos.y + tj.samples.d[i] * sin(pos.t + M_PI_2));
			}

			for (int i = 1; i < tj.global.x.size(); ++i) {
				double tdx = tj.global.x[i] - tj.global.x[i - 1];
				double tdy = tj.global.y[i] - tj.global.y[i - 1];
				tj.samples.yaw.push_back(atan2(tdy, tdx));
				tj.samples.ds.push_back(hypot(tdy, tdx));
			}
			tj.samples.yaw.push_back(tj.samples.yaw.back());
			tj.samples.ds.push_back(tj.samples.ds.back());

			for (int i = 1; i < tj.samples.yaw.size(); ++i) {
				tj.samples.c.push_back(tj.samples.yaw[i] - tj.samples.yaw[i - 1]);
			}
		}
	}

	bool FrenetPath::isCollision(Trajectory & traj)
	{
		for (auto sobj : _obj->static_obstacles) {
			for (int i = 0; i < traj.global.x.size(); ++i) {
				geo_point gp_in(traj.global.x[i], traj.global.y[i]);
				if (boost::geometry::within(gp_in, sobj.getPoly())) return true;
			}
		}

		return false;
	}

	void FrenetPath::checkPaths(vector<Trajectory>& trajs)
	{
		for (auto &tj : trajs) {
			if (find_if(tj.samples.s_d.begin(), tj.samples.s_d.end(), [this](double isd) {
				return isd > _para.max_speed;
				}) != tj.samples.s_d.end()) {
				continue;
			}

			if (find_if(tj.samples.s_dd.begin(), tj.samples.s_dd.end(), [this](double isdd) {
				return isdd > _para.max_acceration;
				}) != tj.samples.s_dd.end()) {
				continue;
			}

			if (find_if(tj.samples.c.begin(), tj.samples.c.end(), [this](double ic) {
				return ic > _para.max_curvature;
				}) != tj.samples.c.end()) {
				continue;
			}

			if (isCollision(tj)) {
				continue;
			}

			tj.ok = true;
		}
	}

	Trajectory FrenetPath::findOptimal(vector<Trajectory>& trajs)
	{
		Trajectory path;
		double min_cf = DBL_MAX;
		for (auto &tj : trajs) {
			if (tj.ok) {
				if (tj.cf < min_cf) {
					min_cf = tj.cf;
					path = tj;
				}
			}
		}

		return path;
	}

	void FrenetPath::setStatus(Status sts)
	{
		_sts = sts;
	}
}