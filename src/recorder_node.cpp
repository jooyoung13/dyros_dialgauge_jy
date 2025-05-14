#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <fstream>
#include <vector>
#include <iomanip>         // std::fixed, std::setprecision
#include <ros/ros.h>       // ros::Time 등


class RecorderNode {
public:
  RecorderNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
  {
     // ▶ 퍼블리싱 시작 시각을 기록
    start_time_ = ros::Time::now();

    // 첫 샘플 오기 전까지는 기준 시간이 unset 상태
    first_sample_time_ = -1.0;

    // export_path_ 는 빈 문자열로 두고, GUI에서 반드시 설정하게 함
    export_path_.clear();  

    // 토픽 구독
    sub_ = nh_.subscribe("/dial_gauge/raw", 1000, &RecorderNode::callback, this);

    // 데이터 저장 서비스
    export_srv_ = pnh_.advertiseService("export", &RecorderNode::exportData, this);

    ROS_INFO_STREAM("RecorderNode initialized, default export_path=" << export_path_);
  }

private:
  // 토픽 콜백: 시간-값 쌍 버퍼에 저장
  void callback(const std_msgs::Float64::ConstPtr& msg) {
    double t_now = ros::Time::now().toSec();
    // 첫 샘플이면 기준 시간을 잡고 '0'으로 만들기
    if (first_sample_time_ < 0.0) {
      first_sample_time_ = t_now;
    }
    double t = t_now - first_sample_time_;  // 상대 시간(초)
    data_.emplace_back(t, msg->data);
  }

  bool exportData(std_srvs::Trigger::Request& req,
                std_srvs::Trigger::Response& res)
{
  // 1) 매 호출 시점에 최신 파라미터로 export_path_ 갱신
  pnh_.param("export_path", export_path_, export_path_);

  // 2) 경로가 설정되지 않았다면 에러 처리
  if (export_path_.empty()) {
    res.success = false;
    res.message = "Export path not set. Please select a path first.";
    ROS_ERROR_STREAM(res.message);
    return true;
  }
  ROS_INFO_STREAM("Recorder export: writing to " << export_path_);

  // 3) 파일 열기
  std::ofstream ofs(export_path_);
  if (!ofs.is_open()) {
    res.success = false;
    res.message = "Cannot open file: " + export_path_;
    ROS_ERROR_STREAM(res.message);
    return true;
  }

  // 4) 소수점 셋째 자리까지 고정된 CSV 형식으로 데이터 덤프
  ofs << std::fixed << std::setprecision(3);
  for (const auto& kv : data_) {
    ofs << kv.first << "," << kv.second << "\n";
  }
  ofs.close();

  // 5) 성공 응답
  res.success = true;
  res.message = "Data exported (" + std::to_string(data_.size()) +
                " points) to " + export_path_;
  ROS_INFO_STREAM(res.message);
  return true;
}



  ros::NodeHandle           nh_, pnh_;
  ros::Subscriber           sub_;
  ros::ServiceServer        export_srv_;
  std::string               export_path_;
  ros::Time start_time_;   
  std::vector<std::pair<double,double>> data_;
  double                    first_sample_time_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "recorder_node");
  ros::NodeHandle nh;        // global namespace
  ros::NodeHandle pnh("~");  // private namespace

  RecorderNode node(nh, pnh);
  ros::spin();
  return 0;
}
