#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace frenet_planner
{
   struct Trajectory
   {

   };

   struct Path
   {

   };

   struct Point2d : public Eigen::Vector2d
   {
      Point2d() = default;
      Point2d(const double x, const double y) : Eigen::Vector2d(x, y) {}
   };

   class Spline2D
   {
      //std::vector<double> s_{};
      //Spline x_spline_{};
      //Spline y_spline_{};

      std::vector<Point2d> original_points_{};
   };

   struct FrenetState
   {
      sampler_common::FrenetPoint position = { 0, 0 };
      double lateral_velocity{};
      double longitudinal_velocity{};
      double lateral_acceleration{};
      double longitudinal_acceleration{};
   };

   struct SamplingParameter
   {
      double target_duration{};
      FrenetState target_state;
   };

   struct SamplingParameters
   {
      std::vector<SamplingParameter> parameters;
      double resolution;
   };

   /// @brief generate trajectories relative to the reference for the given initial state and sampling
   /// parameters
   std::vector<Trajectory> generateTrajectories(
      const sampler_common::transform::Spline2D& reference_spline, const FrenetState& initial_state,
      const SamplingParameters& sampling_parameters);
   /// @brief generate trajectories relative to the reference for the given initial state and sampling
   /// parameters
   std::vector<Trajectory> generateLowVelocityTrajectories(
      const sampler_common::transform::Spline2D& reference_spline, const FrenetState& initial_state,
      const SamplingParameters& sampling_parameters);
   /// @brief generate paths relative to the reference for the given initial state and sampling
   /// parameters
   std::vector<Path> generatePaths(
      const sampler_common::transform::Spline2D& reference_spline, const FrenetState& initial_state,
      const SamplingParameters& sampling_parameters);
   /// @brief generate a candidate path
   /// @details one polynomial for lateral motion (d) is calculated over the longitudinal displacement
   /// (s): d(s).
   Path generateCandidate(
      const FrenetState& initial_state, const FrenetState& target_state, const double s_resolution);
   /// @brief generate a candidate trajectory
   /// @details the polynomials for lateral motion (d) and longitudinal motion (s) are calculated over
   /// time: d(t) and s(t).
   Trajectory generateCandidate(
      const FrenetState& initial_state, const FrenetState& target_state, const double duration,
      const double time_resolution);
   /// @brief generate a low velocity candidate trajectory
   /// @details the polynomial for lateral motion (d) is calculated over the longitudinal displacement
   /// (s) rather than over time: d(s) and s(t).
   Trajectory generateLowVelocityCandidate(
      const FrenetState& initial_state, const FrenetState& target_state, const double duration,
      const double time_resolution);
   /// @brief calculate the cartesian frame of the given path
   void calculateCartesian(const sampler_common::transform::Spline2D& reference, Path& path);
   /// @brief calculate the cartesian frame of the given trajectory
   void calculateCartesian(
      const sampler_common::transform::Spline2D& reference, Trajectory& trajectory);

}  // namespace frenet_planner