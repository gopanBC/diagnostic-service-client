# DiagnosticServiceClient for ROS

## Overview

The `DiagnosticServiceClient` is an enhanced ROS service client that integrates diagnostic capabilities directly into service interactions. This custom implementation extends the standard ROS `ServiceClient` to provide real-time diagnostic logging and improved visibility into service calls. 

This project aims to streamline debugging, performance monitoring, and system health analysis by publishing detailed diagnostic messages to a ROS topic.

## Features

- **Real-Time Diagnostic Logging:** Publishes detailed logs of service requests and responses to a specified ROS topic.
- **Enhanced Debugging:** Provides comprehensive information about service interactions, including timestamps and node names.
- **Customizable Logging:** Allows configuration of log verbosity and format.
- **Scalable Monitoring:** Efficiently handles multiple service clients and high-frequency service calls.


## Usage
Creating a Diagnostic Service Client

To use the DiagnosticServiceClient in your ROS node, include the header file and create an instance ofthe client with the desired configuration.

```cpp
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "diagnostic_service_client/diagnostic_service_client.h"
#include "std_msgs/String.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diagnostic_service_client_node");
  ros::NodeHandle nh;

  // Create a Publisher for diagnostics
  ros::Publisher diag_pub = nh.advertise<std_msgs::String>("service_call_diagnostics", 10);

  // Create a DiagnosticServiceClient
  std::string service_name = "set_bool";
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
```
