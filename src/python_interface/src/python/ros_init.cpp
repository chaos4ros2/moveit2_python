// https://github.com/chaos4ros2/moveit2/blob/main/moveit_ros/planning_interface/py_bindings_tools/src/roscpp_initializer.cpp#L142
#include <memory>
#include <boost/python.hpp>
#include <boost/thread/mutex.hpp>
// ##########################################
// ros1
// #include <ros/ros.h>
// ##########################################
// ros2 #425
#include "rclcpp/rclcpp.hpp"

#include "conversions.h"

// new 425
static std::vector<std::string>& ROScppArgs()
{
  static std::vector<std::string> args;
  return args;
}

namespace moveit {
namespace python {

class InitProxy {
public:
	static void init(const std::string& node_name="moveit_python_wrapper",
	                 const boost::python::dict& remappings = boost::python::dict());
	static void shutdown();

	~InitProxy();

private:
	InitProxy(const std::string& node_name, const boost::python::dict& remappings);

	static boost::mutex lock;
	static std::unique_ptr<InitProxy> singleton_instance;

// #################################################
// ros1 # 425
// private:
//  	std::unique_ptr<ros::AsyncSpinner> spinner;
// #################################################
};
boost::mutex InitProxy::lock;
std::unique_ptr<InitProxy> InitProxy::singleton_instance;

void InitProxy::init(const std::string& node_name, const boost::python::dict& remappings)
{
	boost::mutex::scoped_lock slock(lock);
	// ##############################################
	// ros1
	// if (!singleton_instance && !ros::isInitialized())
	// ##############################################
	// ros2 # 425
	if (!singleton_instance && !rclcpp::ok())
	// ##############################################
		singleton_instance.reset(new InitProxy(node_name, remappings));
}

void InitProxy::shutdown()
{
	boost::mutex::scoped_lock slock(lock);
	singleton_instance.reset();
}

InitProxy::InitProxy(const std::string& node_name, const boost::python::dict& remappings)
{
    // ###################################################
    // ros1
	// ros::init(fromDict<std::string>(remappings), node_name, ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
	// spinner.reset(new ros::AsyncSpinner(1));
	// spinner->start();
	// ####################################################
	// ####################################################
    // ros2 #425
	const std::vector<std::string>& args = ROScppArgs();
    int fake_argc = args.size();
    char** fake_argv = new char*[args.size()];
    for (std::size_t i = 0; i < args.size(); ++i)
      fake_argv[i] = strdup(args[i].c_str());

	rclcpp::init(fake_argc, NULL);
	rclcpp::executors::MultiThreadedExecutor executor;
	auto node = rclcpp::Node::make_shared(node_name);
	executor.add_node(node);
	executor.spin();
	// #####################################################
}

InitProxy::~InitProxy()
{
	// ###################################################
    // ros1
	// spinner->stop();
	// spinner.reset();
	// ####################################################
	// ros2
	rclcpp::shutdown();
	// ####################################################
}

BOOST_PYTHON_FUNCTION_OVERLOADS(rclcpp_init_overloads, InitProxy::init, 0, 2)

void export_ros_init()
{
	boost::python::def("rclcpp_init", InitProxy::init, rclcpp_init_overloads());
	boost::python::def("rclcpp_shutdown", &InitProxy::shutdown);
}

} }