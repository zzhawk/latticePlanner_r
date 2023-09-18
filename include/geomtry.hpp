#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/geometry.hpp>


using geo_point = boost::geometry::model::point<
	double, 2, boost::geometry::cs::cartesian>;
using geo_ring = boost::geometry::model::ring<geo_point>;

using boost::geometry::dsv;

namespace trans = boost::geometry::strategy::transform;