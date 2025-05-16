#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <fstream>
#include <vector>
#include <iomanip>  // std::fixed, std::setprecision

class RecorderNode {
public:
  RecorderNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
  {
    // 첫 샘플 전까지는 unset 상태
    first_sample_time_ = -1.0;

    // 토픽 구독
    sub_ = nh_.subscribe("/dial_gauge/raw", 1000, &RecorderNode::callback, this);

    // 서비스 서버
    export_srv_ = pnh_.advertiseService("export", &RecorderNode::exportData, this);
    reset_srv_  = pnh_.advertiseService("reset",  &RecorderNode::resetData,  this);

    ROS_INFO("RecorderNode initialized.");
  }

private:
  // 토픽 콜백: 상대 시간 계산 후 버퍼에 저장
  void callback(const std_msgs::Float64::ConstPtr& msg) {
    double t_now = ros::Time::now().toSec();

    if (first_sample_time_ < 0.0) {
      first_sample_time_ = t_now;
      ROS_INFO("Recorder: first_sample_time_ set to %.3f", first_sample_time_);
    }

    double t_rel = t_now - first_sample_time_;
    data_.emplace_back(t_rel, msg->data);
  }

  // 데이터 파일로 내보내기
  bool exportData(std_srvs::Trigger::Request& req,
                  std_srvs::Trigger::Response& res)
  {
    // 파라미터로 export_path_ 읽기 (GUI 에서 set_param 해 줘야 함)
    pnh_.param("export_path", export_path_, export_path_);

    if (export_path_.empty()) {
      res.success = false;
      res.message = "Export path not set. Please Browse first.";
      ROS_ERROR_STREAM(res.message);
      return true;
    }
    ROS_INFO_STREAM("Recorder export: writing to " << export_path_);

    std::ofstream ofs(export_path_);
    if (!ofs.is_open()) {
      res.success = false;
      res.message = "Cannot open file: " + export_path_;
      ROS_ERROR_STREAM(res.message);
      return true;
    }

    // 소수점 셋째 자리까지 고정
    ofs << std::fixed << std::setprecision(3);
    for (auto& kv : data_) {
      ofs << kv.first << "," << kv.second << "\n";
    }
    ofs.close();

    res.success = true;
    res.message = "Data exported (" + std::to_string(data_.size())
                    + " points) to " + export_path_;
    ROS_INFO_STREAM(res.message);
    return true;
  }

  // 내부 버퍼와 시간 기준을 완전히 리셋
  bool resetData(std_srvs::Trigger::Request& req,
                 std_srvs::Trigger::Response& res)
  {
    data_.clear();
    first_sample_time_ = -1.0;  // 기준 시간 리셋
    res.success = true;
    res.message = "Recorder data cleared and time reset";
    ROS_INFO("RecorderNode: data cleared, first_sample_time_ reset");
    return true;
  }

  // 멤버 변수
  ros::NodeHandle           nh_, pnh_;
  ros::Subscriber           sub_;
  ros::ServiceServer        export_srv_, reset_srv_;
  std::string               export_path_;
  std::vector<std::pair<double,double>> data_;
  double                    first_sample_time_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "recorder_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  RecorderNode node(nh, pnh);
  ros::spin();
  return 0;
}
