// Copyright 2023 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "polynomials.hpp"

namespace fr {
   // @brief Create a polynomial connecting the given initial and target configuration over the given
   // duration
   QuinticPolynomial::QuinticPolynomial(
      const double x0, const double x0v, const double x0a, const double xT, const double xTv,
      const double xTa, const double T)
   {
      d_ = x0a / 2;
      e_ = x0v;
      f_ = x0;

      const double T2 = T * T;
      const double T3 = T2 * T;
      const double T4 = T3 * T;
      const double T5 = T4 * T;

      Eigen::Matrix3d A;
      A << T5, T4, T3, 5 * T4, 4 * T3, 3 * T2, 20 * T3, 12 * T2, 6 * T;
      Eigen::Vector3d B;
      B << xT - x0 - x0v * T - 0.5 * x0a * T2, xTv - x0v - x0a * T, xTa - x0a;

      const Eigen::Vector3d X = A.colPivHouseholderQr().solve(B);
      a_ = X(0);
      b_ = X(1);
      c_ = X(2);
   }

   double QuinticPolynomial::position(const double t) const
   {
      const double t2 = t * t;
      const double t3 = t2 * t;
      const double t4 = t3 * t;
      const double t5 = t4 * t;
      return a_ * t5 + b_ * t4 + c_ * t3 + d_ * t2 + e_ * t + f_;
   }

   double QuinticPolynomial::velocity(const double t) const
   {
      const double t2 = t * t;
      const double t3 = t2 * t;
      const double t4 = t3 * t;
      return 5 * a_ * t4 + 4 * b_ * t3 + 3 * c_ * t2 + 2 * d_ * t + e_;
   }

   double QuinticPolynomial::acceleration(const double t) const
   {
      const double t2 = t * t;
      const double t3 = t2 * t;
      return 20 * a_ * t3 + 12 * b_ * t2 + 6 * c_ * t + 2 * d_;
   }

   double QuinticPolynomial::jerk(const double t) const
   {
      const double t2 = t * t;
      return 60 * a_ * t2 + 24 * b_ * t + 6 * c_;
   }




   QuarticPolynomial::QuarticPolynomial(
      const double x0, const double x0v, const double x0a, const double xTv,
      const double xTa, const double T)
   {
      c_ = x0a / 2;
      d_ = x0v;
      e_ = x0;
      f_ = 0.0;

      const double T2 = T * T;
      const double T3 = T2 * T;
      const double T4 = T3 * T;

      Eigen::Matrix2d A;
      A << 4*T3, 3*T2, 12 * T2, 6 * T;
      Eigen::Vector2d B;
      B << xTv - x0v - x0a * T, xTa - x0a;

      const Eigen::Vector2d X = A.colPivHouseholderQr().solve(B);
      a_ = X(0);
      b_ = X(1);
   }

   double QuarticPolynomial::position(const double t) const
   {
      const double t2 = t * t;
      const double t3 = t2 * t;
      const double t4 = t3 * t;
      return a_ * t4 + b_ * t3 + c_ * t2 + d_ * t + e_;
   }

   double QuarticPolynomial::velocity(const double t) const
   {
      const double t2 = t * t;
      const double t3 = t2 * t;
      return 4 * a_ * t3 + 3 * b_ * t2 + 2 * c_ * t + d_;
   }

   double QuarticPolynomial::acceleration(const double t) const
   {
      const double t2 = t * t;
      return 12 * a_ * t2 + 6 * b_ * t + 2 * c_;
   }

   double QuarticPolynomial::jerk(const double t) const
   {
      return 24 * a_ * t + 6 * b_;
   }
}