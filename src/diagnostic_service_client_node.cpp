#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "diagnostic_service_client/diagnostic_service_client.h"
#include "std_msgs/String.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diagnostic_service_client_node");
  ros::NodeHandle nh;

  // Create a Publisher for diagnostics
  ros::Publisher diag_pub = nh.advertise<std_msgs::String>("/service_call_diagnostics", 10, true);

  // Create a DiagnosticServiceClient
  std::string service_name = "service_name";
  ros::M_string header_values;
  std::string service_md5sum = ros::service_traits::MD5Sum<std_srvs::SetBool>::value();
  ros::DiagnosticServiceClient client(service_name, false, header_values, service_md5sum, diag_pub);

  // Prepare the request and response
  std_srvs::SetBool srv;
  srv.request.data = true;

  // Call the service
  if (client.call(srv))
  {
    ROS_INFO("Response: success=%d, message=%s", srv.response.success, srv.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }

  ros::spin();

  return 0;
}
