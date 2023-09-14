// Copyright 2023 watson.wang

#include <vector>
#include <optional>
#include "lane/euler_spiral.hpp"
#include "polynomials.hpp"

#ifndef LATTICE_HPP_
#define LATTICE_HPP_

using namespace std;

namespace fr {
   struct Parameters
   {
      double max_speed;
      double max_acceration;
      double max_curvature;
      double max_road_width;
      double max_road_sample_width;
      double time_tick;
      double max_pred_time;
      double min_pred_time;
      double target_speed;
      double target_speed_sample;
      double target_speed_num;
      double radius;

      double K_J;
      double K_T;
      double K_D;
      double K_LAT;
      double K_LON;

      double start_x;
      double start_y;
      double start_yaw;
   };

   struct FrenetLane
   {
      vector<double> t;
      vector<double> d;
      vector<double> d_d;
      vector<double> d_dd;
      vector<double> d_ddd;
      vector<double> s;
      vector<double> s_d;
      vector<double> s_dd;
      vector<double> s_ddd;

      vector<double> yaw;
      vector<double> ds;
      vector<double> c;
   };

   struct CartesianLane
   {
      vector<double> x;
      vector<double> y;
   };

   struct Status
   {
      double t;
      double d;
      double d_d;
      double d_dd;
      double d_ddd;
      double s;
      double s_d;
      double s_dd;
      double s_ddd;
   };

   class Trajectory
   {

   public:
      explicit Trajectory(){}
      virtual ~Trajectory() = default;

      Trajectory(const Trajectory& ) = default;
      Trajectory(Trajectory&&) noexcept = default;

      Trajectory& operator = (const Trajectory&) = default;
      Trajectory& operator = (Trajectory&&) noexcept = default;

      std::optional<QuinticPolynomial> lateral_polynomial{};
      std::optional<QuarticPolynomial> longitudinal_polynomial{};
      
      FrenetLane samples;
      CartesianLane global;

      double cd = 0.0;
      double cv = 0.0;
      double cf = 0.0;

      bool ok = false;
   };

   class FrenetPath
   {
   public:
      explicit FrenetPath(Parameters para, es::SpiralParameter rfl, Status sts) :_para(para), _rfl(rfl), _sts(sts) {}
      virtual ~FrenetPath() = default;

      FrenetPath(const FrenetPath&) = default;
      FrenetPath& operator = (const FrenetPath&) = default;

      FrenetPath(FrenetPath&&) noexcept = default;
      FrenetPath& operator = (FrenetPath&&) noexcept = default;

      Trajectory generatePath();
      void setStatus(Status sts);

   private:
      Parameters _para;
      es::SpiralParameter _rfl;
      Status _sts;

      vector<Trajectory> generateTrajectories();
      void calcGlobalPaths(vector<Trajectory>& trajs);
      bool isCollision(vector<Trajectory>& trajs);

      void checkPaths(vector<Trajectory>& trajs);
      Trajectory findOptimal(vector<Trajectory>& trajs);
   };
}

#endif