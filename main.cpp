// Copyright 2023 watson.wang
// This is the self test file.

#include <corecrt_math_defines.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "lattice.hpp"
#include "polynomials.hpp"

int main()
{
    std::ofstream lif("../spinline.csv");
    std::ofstream plf("../plan.csv");
    std::ofstream obf("../obstacle.csv");

	es::SpiralPoint start, goal;
	start.x = start.y = start.t = 0.0;

	goal.x = 200.0;
	goal.y = 10.0;
	goal.t = M_PI_2;

	es::SpiralParameter ip;
	es::SpiralParameter p = es::getParameter(start, goal, &ip);

	// std::cout << "# iparam " << ip.length << " " << ip.dCurv << " " << ip.initCurv << std::endl;
	// std::cout << "# param  " << p.length << " " << p.dCurv << " " << p.initCurv << std::endl;
    vector <es::SpiralPoint> rawpoints;

    for (double s = 0.0; s <= p.length; s += p.length / 100.0) {
        es::SpiralPoint pos;
        pos = es::getEndPoint(s, p.dCurv, p.initCurv, start.x, start.y, start.t);
        rawpoints.push_back(pos);
        lif << pos.x << "," << pos.y << "," << pos.t << std::endl;
    }

    auto obstacle = [&](es::SpiralPoint pos, es::SpiralPoint offset, double width, double len) {
       geo_ring ans;
       geo_ring rot;

       trans::rotate_transformer<boost::geometry::radian, double, 2, 2> rotate(-(pos.t + offset.t));
       trans::translate_transformer<double, 2, 2> translate(pos.x + offset.x, pos.y + offset.y);

       geo_point p0 = {  - len / 2.0, - width / 2.0 };
       geo_point p1 = {  + len / 2.0, - width / 2.0 };
       geo_point p2 = {  + len / 2.0, + width / 2.0 };
       geo_point p3 = {  - len / 2.0, + width / 2.0 };

       geo_ring org = { p0, p1, p2, p3, p0 };
       boost::geometry::transform(org, rot, rotate);
       boost::geometry::transform(rot, ans, translate);

       return ans;
    };

    es::SpiralPoint off1; off1.y = 0.5;
    es::SpiralPoint off2; off2.y = -0.5;
    auto obj = make_shared<ob::Constraints>();
    obj->static_obstacles.push_back(ob::stationaryObj(obstacle(rawpoints[20], off1, 3.5, 5.0)));
    obj->static_obstacles.push_back(ob::stationaryObj(obstacle(rawpoints[55], off2, 3.5, 5.0)));

    for (auto oo : obj->static_obstacles) {
       obf << dsv(oo.getPoly()) << endl;
    }

	fr::Parameters para;

    para.max_speed = 50.0/3.6;
    para.max_acceration = 2.0;
    para.max_curvature = 1.0;
    para.max_road_width = 7.0;
    para.max_road_sample_width = 1.0;
    para.time_tick = 0.2;
    para.max_pred_time = 5.0;
    para.min_pred_time = 4.0;
    para.target_speed = 30.0/3.6;
    para.target_speed_sample = 5.0/3.6;
    para.target_speed_num = 1;
    para.radius = 2.0;
    para.K_J = 0.1;
    para.K_T = 0.1;
    para.K_D = 1.0;
    para.K_LAT = 1.0;
    para.K_LON = 1.0;
    para.start_x = start.x;
    para.start_y = start.y;
    para.start_yaw = start.t;

    fr::Status newSts;
    newSts.t = 0.0;
    newSts.d = 2.0;
    newSts.d_d = 0.0;
    newSts.d_dd = 0.0;
    newSts.d_ddd = 0.0;
    newSts.s = 0.0;
    newSts.s_d = 10.0 / 3.6;
    newSts.s_dd = 0.0;
    newSts.s_ddd = 0.0;

    //fr::FrenetPath  pth(para, p, initSts);
    //auto ans = pth.generatePath();

    vector<fr::Trajectory> ans;
    
    while (true) {
        fr::FrenetPath pth(para, p, newSts, obj);
        auto traj = pth.generatePath();

        if (traj.ok) {
            newSts.d = traj.samples.d.back();
            newSts.d_d = traj.samples.d_d.back();
            newSts.d_dd = traj.samples.d_dd.back();
            newSts.d_ddd = traj.samples.d_ddd.back();
            newSts.s = traj.samples.s.back();
            newSts.s_d = traj.samples.s_d.back();
            newSts.s_dd = traj.samples.s_dd.back();
            newSts.s_ddd = traj.samples.s_ddd.back();

            ans.push_back(traj);
        }
        else {
            break;
        }

        bool exit = false;
        for (int i = 0; i < traj.global.x.size(); ++i) {
            if (hypot(goal.x - traj.global.x[i], goal.y - traj.global.y[i]) < 1.0) {
                exit = true;
                break;
            }
        }

        if (exit) break;
    }

    for (auto a: ans) {
        for (int i = 0; i < a.global.x.size(); ++i) {
            // std::cout << a.global.x[i] << "," << a.global.y[i] << std::endl;
           plf << a.global.x[i] << "," << a.global.y[i] << std::endl;
        }
    }

    return 0;
}