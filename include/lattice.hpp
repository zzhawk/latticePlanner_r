// Copyright 2023 watson.wang

#include <vector>
#include <optional>
#include "lane/euler_spiral.hpp"
#include "polynomials.hpp"
#include "obstacle.hpp"

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
      
      FrenetLane samples{};
      CartesianLane global{};

      double cd{};
      double cv{};
      double cf{};

      bool ok{};
   };

   class FrenetPath
   {
   public:
      explicit FrenetPath(Parameters para, es::SpiralParameter rfl, Status sts, shared_ptr<ob::Constraints> obj) :
          _para(para), _rfl(rfl), _sts(sts), _obj{std::move(obj)} {}
      virtual ~FrenetPath() = default;

      FrenetPath(const FrenetPath&) = default;
      FrenetPath& operator = (const FrenetPath&) = default;

      FrenetPath(FrenetPath&&) noexcept = default;
      FrenetPath& operator = (FrenetPath&&) noexcept = default;

      Trajectory generatePath();
      void setStatus(Status sts);

   private:
      Parameters _para{};
      es::SpiralParameter _rfl{};
      Status _sts{};
      std::shared_ptr<ob::Constraints> _obj{};

      vector<Trajectory> generateTrajectories();
      void calcGlobalPaths(vector<Trajectory>& trajs);
      bool isCollision(Trajectory &trajs);

      void checkPaths(vector<Trajectory>& trajs);
      Trajectory findOptimal(vector<Trajectory>& trajs);
   };
}

#endif