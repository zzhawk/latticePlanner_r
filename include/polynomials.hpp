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
//=====================================================================
//
// watson.wang: split into Quintic and Quartic


#ifndef PLOYNOMIALS_HPP_
#define PLOYNOMIALS_HPP_

namespace fr {
   class Polynomial
   {

   protected:
      /// @brief polynomial coefficients
      double a_ = 0.0;
      double b_ = 0.0;
      double c_ = 0.0;
      double d_ = 0.0;
      double e_ = 0.0;
      double f_ = 0.0;

   public:
      /// @brief Create a quintic polynomial between an initial and final state over a parameter length
      /// @param x0 initial position
      /// @param x0v initial velocity
      /// @param x0a initial acceleration
      /// @param xT final position
      /// @param xTv final velocity
      /// @param xTa final acceleration
      /// @param T parameter length (arc length or duration)
      Polynomial() {};
      // TODO(Maxime CLEMENT) add quartic case for when final position is not given

      /// @brief Get the position at the given time
      [[nodiscard]] virtual double position(const double t) const = 0;
      /// @brief Get the velocity at the given time
      [[nodiscard]] virtual double velocity(const double t) const = 0;
      /// @brief Get the acceleration at the given time
      [[nodiscard]] virtual double acceleration(const double t) const = 0;
      /// @brief Get the jerk at the given time
      [[nodiscard]] virtual double jerk(const double t) const = 0;
   };

   class QuinticPolynomial : public Polynomial
   {
   public:

      QuinticPolynomial(
         const double x0, const double x0v, const double x0a, const double xT, const double xTv,
         const double xTa, const double T);

      double position(const double t) const;
      double velocity(const double t) const;
      double acceleration(const double t) const;
      double jerk(const double t) const;
   };


   class QuarticPolynomial : public Polynomial
   {
   public:

      QuarticPolynomial(
         const double x0, const double x0v, const double x0a, const double xTv,
         const double xTa, const double T);

      double position(const double t) const;
      double velocity(const double t) const;
      double acceleration(const double t) const;
      double jerk(const double t) const;
   };
}


#endif