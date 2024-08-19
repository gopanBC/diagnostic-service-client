#ifndef ROSCPP_DIAGNOSTIC_SERVICE_CLIENT_H
#define ROSCPP_DIAGNOSTIC_SERVICE_CLIENT_H

#include "ros/service_client.h"
#include <std_msgs/String.h>
#include <sstream>

namespace ros
{

/**
 * @brief A diagnostic-enhanced version of the ROS ServiceClient with topic publishing
 */
class DiagnosticServiceClient : public ServiceClient
{
public:
  DiagnosticServiceClient() : ServiceClient() {}

  DiagnosticServiceClient(const std::string& service_name, bool persistent, const M_string& header_values
                        , const std::string& service_md5sum, const ros::Publisher& pub)
    : ServiceClient(service_name, persistent, header_values, service_md5sum), diag_pub_(pub)
  {}

  DiagnosticServiceClient(const ServiceClient& rhs, const ros::Publisher& pub)
    : ServiceClient(rhs), diag_pub_(pub)
  {}

  ~DiagnosticServiceClient() {}

  DiagnosticServiceClient& operator=(const DiagnosticServiceClient& other) = default;

  /**
   * @brief Call the service with diagnostic logging and publish diagnostics.
   */
  template<class MReq, class MRes>
  bool call(MReq& req, MRes& res)
  {
    std::string node_name = ros::this_node::getName();
    std::string service_name = getService();
    ros::Time call_time = ros::Time::now();

    ROS_INFO("Service call initiated to [%s] by [%s]", service_name.c_str(), node_name.c_str());

    // Log request details manually since there's no __str__() method
    ROS_INFO("Request: data=%s", req.data ? "true" : "false");

    bool result = ServiceClient::call(req, res);

    std::ostringstream oss;
    oss << "Service: " << service_name << ", "
        << "Client Node: " << node_name << ", "
        << "Time: " << call_time << ", "
        << "Request: data=" << (req.data ? "true" : "false") << ", "
        << "Result: " << (result ? "Success" : "Failure") << ", "
        << "Response: success=" << res.success << ", message=" << res.message;

    std_msgs::String diag_msg;
    diag_msg.data = oss.str();
    diag_pub_.publish(diag_msg);

    if (result)
    {
      ROS_INFO("Service call to [%s] succeeded.", service_name.c_str());
      ROS_INFO("Response: success=%d, message=%s", res.success, res.message.c_str());
    }
    else
    {
      ROS_ERROR("Service call to [%s] failed.", service_name.c_str());
    }

    return result;
  }

  /**
   * @brief Call the service with diagnostic logging and publish diagnostics.
   */
  template<class Service>
  bool call(Service& service)
  {
    return call(service.request, service.response);
  }

private:
  ros::Publisher diag_pub_;

  void deserializeFailed(const std::exception& e)
  {
    ROS_ERROR("Failed to deserialize response: %s", e.what());
  }
};

} // namespace ros

#endif // ROSCPP_DIAGNOSTIC_SERVICE_CLIENT_H

