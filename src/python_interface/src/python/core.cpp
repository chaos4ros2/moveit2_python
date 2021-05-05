#include <boost/python.hpp>

namespace moveit {
namespace python {

void export_ros_init();

} }

BOOST_PYTHON_MODULE(_core)
{
	moveit::python::export_ros_init();
}