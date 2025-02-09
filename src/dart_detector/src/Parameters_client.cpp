//利用parameter server实现调参功能 使用colcon build --packages-select parameter_server 可以直接编译
//编译完一定要source install/local_setup.bash 然后ros2 run parameter_server server 调用该节点
//persistent参数存在dart-ros2-workspace/src/ros2_persist_parameter_server/server/param/parameter_server.yaml

#include <map>
#include <stdexcept>
#include <memory> 
#include "persist_parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include <rcl_interfaces/srv/set_parameters.hpp>
#include<cstring>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include "../include/detect/detect.h"

using namespace cv;
using namespace std;


// 输入图像
Mat img;
VideoCapture cap;
// HSV图像
Mat hsv;
// 色相
int hmin = 1;
int hmin_Max = 360;
int hmax = 180;
int hmax_Max = 180;
// 饱和度
int smin = 1;
int smin_Max = 255;
int smax = 255;
int smax_Max = 255;
// 亮度
int vmin = 1;
int vmin_Max = 255;
int vmax = 255;
int vmax_Max = 255;
// 最小圆心距离
int minDist = 1;
int minDist_Max = 400;
// 半径
int rmin = 1;
int rmin_Max = 400;
int rmax = 1;
int rmax_Max = 400;
// param1
int param1 = 1;
int param1_Max = 50;
// param2
int param2 = 1;
int param2_Max = 50;
//视觉相关参数 还未设定



/**
 * NoServerError
 *
 * The client will wait 5 seconds for the server to be ready.
 * If timeout, then throw an exception to terminate the endless waiting.
 */
struct NoServerError : public std::runtime_error
{
  public:
    NoServerError()
      : std::runtime_error("cannot connect to server"){}
};

/* 
 * SetOperationError
 *
 * When executing `set_parameter`, if the set operation failed, 
 * throw an exception to ignore the subsequent test.
 */
struct SetOperationError : public std::runtime_error
{
  public:
    SetOperationError()
      : std::runtime_error("set operation failed"){}
};

class TestPersistParameter
{
 private:
    // Save the result of each test operation.
void parameter_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg) {
        for (const auto &changed_parameter : msg->changed_parameters) {
            if (changed_parameter.name == "persistent.hmin") {
                hmin = changed_parameter.value.integer_value;
                RCLCPP_INFO(this->get_logger(), "Updated hmin to %d", hmin);
                sync_other_nodes_parameters("hmin", changed_parameter.value);
            }
            else if (changed_parameter.name == "persistent.hmax") {
                hmax = changed_parameter.value.integer_value;
                RCLCPP_INFO(this->get_logger(), "Updated hmax to %d", hmax);
                sync_other_nodes_parameters("hmax", changed_parameter.value);
            }
            else if (changed_parameter.name == "persistent.smin") {
                smin = changed_parameter.value.integer_value;
                RCLCPP_INFO(this->get_logger(), "Updated smin to %d", smin);
                sync_other_nodes_parameters("smin", changed_parameter.value);
            }
            else if (changed_parameter.name == "persistent.smax") {
                smax = changed_parameter.value.integer_value;
                RCLCPP_INFO(this->get_logger(), "Updated smax to %d", smax);
                sync_other_nodes_parameters("smax", changed_parameter.value);
            }
            else if (changed_parameter.name == "persistent.vmin") {
                vmin = changed_parameter.value.integer_value;
                RCLCPP_INFO(this->get_logger(), "Updated vmin to %d", vmin);
                sync_other_nodes_parameters("vmin", changed_parameter.value);
            }
            else if (changed_parameter.name == "persistent.vmax") {
                vmax = changed_parameter.value.integer_value;
                RCLCPP_INFO(this->get_logger(), "Updated vmax to %d", vmax);
                sync_other_nodes_parameters("vmax", changed_parameter.value);
            }
            else if (changed_parameter.name == "persistent.minDist") {
                minDist = changed_parameter.value.integer_value;
                RCLCPP_INFO(this->get_logger(), "Updated minDist to %d", minDist);
                sync_other_nodes_parameters("minDist", changed_parameter.value);
            }
            else if (changed_parameter.name == "persistent.rmin") {
                rmin = changed_parameter.value.integer_value;
                RCLCPP_INFO(this->get_logger(), "Updated rmin to %d", rmin);
                sync_other_nodes_parameters("rmin", changed_parameter.value);
            }
            else if (changed_parameter.name == "persistent.rmax") {
                rmax = changed_parameter.value.integer_value;
                RCLCPP_INFO(this->get_logger(), "Updated rmax to %d", rmax);
                sync_other_nodes_parameters("rmax", changed_parameter.value);
            }
            else if (changed_parameter.name == "persistent.param1") {
                param1 = changed_parameter.value.integer_value;
                RCLCPP_INFO(this->get_logger(), "Updated param1 to %d", param1);
                sync_other_nodes_parameters("param1", changed_parameter.value);
            }
            else if (changed_parameter.name == "persistent.param2") {
                param2 = changed_parameter.value.integer_value;
                RCLCPP_INFO(this->get_logger(), "Updated param2 to %d", param2);
                sync_other_nodes_parameters("param2", changed_parameter.value);
            }
        }
    }

    // 同步其它节点的参数
    void sync_other_nodes_parameters(const std::string &param_name, const rcl_interfaces::msg::ParameterValue &param_value) {
        // 假设我们要同步的节点列表（可以根据实际需要修改）
        std::vector<std::string> other_nodes = {"node_detector"};

        for (const auto& node : other_nodes) {
            try {
                // 调用服务设置其他节点的参数
                modify_parameter_other_nodes(node, param_name, param_value);
                RCLCPP_INFO(this->get_logger(), "Synced parameter '%s' to node '%s'.", param_name.c_str(), node.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to sync parameter '%s' to node '%s': %s", param_name.c_str(), node.c_str(), e.what());
            }
        }
    }

    // 修改其他节点的参数
    void modify_parameter_other_nodes(const std::string &node_name, const std::string &param_name, const rcl_interfaces::msg::ParameterValue &param_value) {
    
        auto client = persist_param_client_.create_client<rcl_interfaces::srv::SetParameters>(node_name + "/set_parameters");

        // 等待服务可用
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                throw std::runtime_error("Interrupted while waiting for service.");
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
        }

        // 创建请求并设置参数
        rcl_interfaces::srv::SetParameters::Request::SharedPtr request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter parameter;
        parameter.name = param_name;
        parameter.value = param_value;
        request->parameters.push_back(parameter);

        // 发送请求并等待结果
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(&(this->persist_param_client_), result) != rclcpp::FutureReturnCode::SUCCESS) {
            throw std::runtime_error("Failed to wait for service result.");
        }
        // 检查服务调用的实际结果
        auto response = result.get();
        for (const auto& result : response->results) {
            if (!result.successful) {
                throw std::runtime_error("Failed to set parameter: " + result.reason);
            }
        }
    }

    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_subscription_;
    PersistParametersClient persist_param_client_;
    static rclcpp::Logger client_logger_;
  public:
  	TestPersistParameter(
      const std::string & node_name, 
      const rclcpp::NodeOptions & options)
      : persist_param_client_(node_name, options)
    {
        rclcpp::Logger client_logger_ = rclcpp::get_logger("client");
      if(!wait_param_server_ready()) 
      {
        
        auto param_client = std::make_shared<PersistParametersClient>(node_name,options);
        parameter_event_subscription_ = param_client->create_subscription<rcl_interfaces::msg::ParameterEvent>(
            "/parameter_events", 10, [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr msg) {
                this->parameter_callback(msg);});
      }
            throw NoServerError();
      }
      
    

    inline bool wait_param_server_ready()
    {
      return persist_param_client_.wait_param_server_ready();
    }

    /*
    * Read the value of parameter.
    * @param param_name The name of parameter.
    * @param expect_str The value of the parameter that you expected, take std::string as example here.
    * If expect_str is equal to `nullptr` pointer, that means the parameter expected to be not exist.
    * @param testcase The test case description.
    */
    void do_read_and_check(const std::string & param_name, const char * expect_str, const std::string & testcase)
    {
      bool value = false;
      std::vector<rclcpp::Parameter> parameter;

      if(persist_param_client_.read_parameter(param_name, parameter)) {
        for(auto & param : parameter) {
          if(expect_str == nullptr) {
            if(param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
              value = true;
              break;
            }
          }else if(param.get_type() == rclcpp::ParameterType::PARAMETER_STRING && param.as_string() == expect_str) {
            value = true;
            break;
          }
        }
      }

      /*
       * Even if the Get operation failed, record it in result_map, and it shouldn't effect the
       * subsequent tests.
       */
      
    }

    /*
    * Change the value of parameter.
    * @param param_name The name of parameter.
    * @param changed_value The value that you want to set.
    * @param testcase The test case description.
    */
    void do_change_and_check(const std::string & param_name, const std::string & changed_value, const std::string & testcase)
    {
      bool ret = false;

      ret = persist_param_client_.modify_parameter<std::string>(param_name, changed_value);
      /*
       * If the Modify operation failed, record it in result_map, and no need to run the 
       * subsequent read tests.
       */
      if(!ret) {
        throw SetOperationError();
      }

      return do_read_and_check(param_name, changed_value.c_str(), testcase);
    }

  

    static inline rclcpp::Logger get_logger()
    {
      return client_logger_;
    }
    


};
int main(int argc, char ** argv)
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  std::shared_ptr<TestPersistParameter> test_client;

  int ret_code = 0;
  // In case of an exception is thrown when performing an operation after `ctrl-c` occured.
  try {
    test_client = std::make_shared<TestPersistParameter>("client", rclcpp::NodeOptions());
    
      
      RCLCPP_INFO(test_client->get_logger(), "Add a new parameter to parameter file : ");
      test_client->do_change_and_check("new_string", "Hello NewString", "e. Add New Normal parameter");
      test_client->do_change_and_check("persistent.new_string", "Hello NewString", "f. Add New Persistent parameter");
    

    // Waiting for the server to restart.
    std::this_thread::sleep_for(std::chrono::seconds(5));

    /*
    * Test : Reading parameter value again to confirm whether to store the modified persistent/normal parameter to the file.
    */
    {
      if(!test_client->wait_param_server_ready()) {
        throw NoServerError();
      }
      RCLCPP_INFO(test_client->get_logger(), "Last read the value of parameter after server restarts," 
        "to check whether changes stores to the file : ");
      test_client->do_read_and_check("a_string", "Hello world", "g. Test Normal Parameter Not Stores To File");
      test_client->do_read_and_check("persistent.a_string", "Hello", "h. Test Persistent Parameter Stores To File");
      test_client->do_read_and_check("new_string", nullptr, "i. Test New Added Normal Parameter Not Stores To File");
      test_client->do_read_and_check("persistent.new_string", "Hello NewString", "j. Test New Added Persistent Parameter Stores To File");
    }
  } catch (const rclcpp::exceptions::RCLError & e) {
    ret_code = -1;
    RCLCPP_ERROR(test_client->get_logger(), "unexpectedly failed: %s", e.what());
  } catch (const NoServerError & e) {
    ret_code = -2;
    RCLCPP_ERROR(test_client->get_logger(), "unexpectedly failed: %s", e.what());
  } catch (const SetOperationError & e) {
    ret_code = -3;
    RCLCPP_ERROR(test_client->get_logger(), "unexpectedly failed: %s", e.what());
  }

 
  rclcpp::shutdown();

  return ret_code;
}


