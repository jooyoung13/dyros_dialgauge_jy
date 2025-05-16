#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float64.h>

#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QTimer>
#include <QFont>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>

#include <QSystemTrayIcon>
#include <QMenu>
#include <QAction>

#include <ros/package.h>

#include <QLabel>
#include <vector>       // full_data_ 를 사용할 경우
#include <QCloseEvent> 

#include <QFileDialog>
#include <ros/param.h>

#include <QLineEdit>        // @@ 추가
#include <QLabel>           // @@ 추가
#include <QHBoxLayout>      // @@ 추가

using namespace QtCharts;

class GuiNode : public QMainWindow {
  Q_OBJECT

public:
  explicit GuiNode(ros::NodeHandle& nh, QWidget* parent = nullptr)
    : QMainWindow(parent)
    , nh_(nh)
  {
    // 시작 시간 기록
    start_time_ = ros::Time::now();

    // @@ 추가: 현재 퍼블리시 중인지 플래그
    bool is_publishing_ = false;     

    // --- 기본 창 크기 설정 ---
    resize(1200, 800);
    setWindowTitle("DYROS Dial Gauge Monitor");

    // --- UI 세팅 ---
    auto* central = new QWidget;
    auto* layout  = new QVBoxLayout;
    layout->setContentsMargins(20, 20, 20, 20);
    layout->setSpacing(15);

    // 버튼 폰트 및 크기 설정
    QFont btn_font;
    btn_font.setPointSize(16);

    btn_open_   = new QPushButton("Open Port");
    btn_start_  = new QPushButton("Start Measure and Publish(rostopic /dial_gauge/raw)");
    btn_browse_ = new QPushButton("Browse Export Path");
    btn_export_ = new QPushButton("Export Data");

    btn_open_->setFont(btn_font);
    btn_start_->setFont(btn_font);
    btn_browse_->setFont(btn_font);
    btn_export_->setFont(btn_font);

    btn_open_->setMinimumHeight(60);
    btn_start_->setMinimumHeight(60);
    btn_export_->setMinimumHeight(60);
    btn_browse_->setMinimumHeight(60);


    layout->addWidget(btn_open_);
    layout->addWidget(btn_start_);

    // ------------------------------------------------

    lbl_filename_ = new QLabel("Filename:");
    le_filename_  = new QLineEdit;
    le_filename_->setPlaceholderText("e.g. dial_data.txt");

    btn_browse_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);

    auto* fileLayout = new QHBoxLayout;
    fileLayout->addWidget(btn_browse_);

    // 오른쪽 layout: 수직 구성
    auto* rightLayout = new QVBoxLayout;

    // 상단: Filename 레이블과 LineEdit을 가로로 배치
    auto* filenameLayout = new QHBoxLayout;
    filenameLayout->addWidget(lbl_filename_);
    filenameLayout->addWidget(le_filename_);

    // 수직으로 filename 라인 + export 버튼 배치
    rightLayout->addLayout(filenameLayout);
    rightLayout->addWidget(btn_export_);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    rightLayout->setSpacing(5);

    fileLayout->addLayout(rightLayout);

    // ✅ 중간 wrapper로 감싸서 전체 중앙 정렬
    auto* wrapperLayout = new QHBoxLayout;
    wrapperLayout->addLayout(fileLayout);
    wrapperLayout->setAlignment(Qt::AlignHCenter);  // <-- 핵심

    // ✅ 원래 있던 layout에 추가
    layout->addLayout(wrapperLayout);


    //--------------------------------------------------------------
    // ★ 차트 위쪽에 실시간 값 표시용 라벨 추가
    lbl_value_ = new QLabel("Time: 0.000s  Value: --   Δt: 0.00(0.0Hz)");
    QFont val_font;
    val_font.setPointSize(20);
    val_font.setBold(true);
    lbl_value_->setFont(val_font);
    lbl_value_->setAlignment(Qt::AlignCenter);

    lbl_value_->setTextFormat(Qt::RichText);            // ★ 추가 (HTML 사용 허용)

    lbl_value_->setStyleSheet(                            // ★ 추가
        "background:#ffffff;border:1px solid #ddd;"
        "border-radius:8px;padding:6px 10px;"
    );                 

    layout->addWidget(lbl_value_);

    // --- 그래프 설정 ---
    series_ = new QLineSeries;
    chart_  = new QChart;
    chart_->addSeries(series_);
    chart_->createDefaultAxes();

    series_->setPen(QPen(QColor("#0078d4"), 2));          // ★ 추가
    chart_->setBackgroundBrush(QColor("#f5f7fa"));        // ★ 추가

    // 차트 제목
    chart_->setTitle("Dialgauge: Live Data");
    QFont titleFont = chart_->titleFont();
    titleFont.setPointSize(18);
    chart_->setTitleFont(titleFont);

    // X/Y 축 포인터 추출 및 레이블
    axisX_ = qobject_cast<QValueAxis*>(chart_->axes(Qt::Horizontal).first());
    axisY_ = qobject_cast<QValueAxis*>(chart_->axes(Qt::Vertical).first());
    if (axisX_) {
      axisX_->setTitleText("Time (s)");
      axisX_->setTitleFont(titleFont);
      axisX_->setRange(0, 10);
    }
    if (axisY_) {
      axisY_->setTitleText("Gauge Value[mm]");
      axisY_->setTitleFont(titleFont);
      axisY_->setRange(-15, 15);
    }

    chart_->legend()->hide();

    auto* view = new QChartView(chart_);
    view->setRenderHint(QPainter::Antialiasing);
    view->setMinimumHeight(400);
    view->setMinimumWidth(1000);
    layout->addWidget(view);

    central->setLayout(layout);

    // central->setStyleSheet(
    //     "background:qlineargradient("
    //     "x1:0 y1:0 x2:0 y2:1,"
    //     "stop:0 #f5fbff stop:1 #dbeeff);"
    // );   

    

    setCentralWidget(central);

    // --- 서비스 클라이언트 생성 (절대 경로) ---
    open_client_   = nh_.serviceClient<std_srvs::SetBool>   ("/serial_node/open");
    start_client_  = nh_.serviceClient<std_srvs::Trigger>   ("/serial_node/start");
    stop_client_   = nh_.serviceClient<std_srvs::Trigger>   ("/serial_node/stop");
    export_client_ = nh_.serviceClient<std_srvs::Trigger>   ("/recorder_node/export");
    reset_client_  = nh_.serviceClient<std_srvs::Trigger>  ("/recorder_node/reset"); 

    // --- ROS 구독 ---
    sub_ = nh_.subscribe("/dial_gauge/raw", 1000, &GuiNode::onData, this);

    // --- 시그널·슬롯 연결 ---
    connect(btn_open_,  &QPushButton::clicked, this, &GuiNode::onOpen);
    connect(btn_start_, &QPushButton::clicked, this, &GuiNode::onStart);
    connect(btn_export_, &QPushButton::clicked, this, &GuiNode::onExport);
    connect(btn_browse_, &QPushButton::clicked, this, &GuiNode::onBrowse);

    // --- 타이머: ROS 콜백 처리 및 차트 슬라이딩 윈도우 갱신 ---
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &GuiNode::onTimer);
    timer_->start(5);

    // --- 시스템 트레이 아이콘 설정 추가 ---
    // 1) 패키지 경로에서 아이콘 파일 위치 구하기
    std::string pkg_path = ros::package::getPath("dyros_dialgauge_jy");
    QString icon_path = QString::fromStdString(pkg_path + "/src/icons/dial_gauge.png");
    ROS_INFO_STREAM("Tray icon path: " << icon_path.toStdString());
    this->setWindowIcon(QIcon(icon_path));
    // 2) QIcon 객체 생성
    QIcon trayIconImage(icon_path);
    // 3) 트레이아이콘에 아이콘 설정
    trayIcon_ = new QSystemTrayIcon(trayIconImage, this);

    // 2) 트레이 컨텍스트 메뉴
    trayMenu_   = new QMenu(this);
    actionShow_ = trayMenu_->addAction("Show Window");
    actionQuit_ = trayMenu_->addAction("Quit");

    trayIcon_->setContextMenu(trayMenu_);
    trayIcon_->show();

    // 3) 메뉴 액션 연결
    connect(actionShow_, &QAction::triggered, this, &QWidget::showNormal);
    connect(actionQuit_, &QAction::triggered, qApp, &QApplication::quit);

    // 4) 아이콘 더블클릭 시 창 보이기
    connect(trayIcon_, &QSystemTrayIcon::activated, this, [&](QSystemTrayIcon::ActivationReason r){
      if (r == QSystemTrayIcon::DoubleClick)
        this->showNormal();
    });
  }

private slots:
    void onOpen() {
        std_srvs::SetBool srv;

        /* 현재 버튼 글자 기준으로 열림 여부 판단 */
        bool currentlyOpen = (btn_open_->text() == "Port Opened");
        srv.request.data   = !currentlyOpen;      // true → 열기, false → 닫기

        /* 서비스 호출 */
        if (open_client_.call(srv)) {
            if (srv.response.success) {
                /* ───────────── 성공 ───────────── */
                btn_open_->setText(currentlyOpen ? "Open Port" : "Port Opened");

                /* 열렸을 때만 초록, 닫히면 기본색 */
                btn_open_->setStyleSheet(
                    currentlyOpen ? ""                          // 닫힘 = 기본
                                  : "background:#28a745;color:white;"   // 열림 = 초록
                );
            } else {
                /* ───────── 서비스 응답 실패 ───────── */
                btn_open_->setText("Error");
                btn_open_->setStyleSheet("background:#d9534f;color:white;");  // 빨강

                /* 1.5 초 뒤 원상 복구 */
                QTimer::singleShot(1500, this, [this]{
                    btn_open_->setStyleSheet("");
                    btn_open_->setText("Open Port");
                });

                ROS_WARN("openPort failed: %s", srv.response.message.c_str());
            }
        } else {
            /* ─────── 서비스 호출 자체 실패 ─────── */
            btn_open_->setText("Service Failed");
            btn_open_->setStyleSheet("background:#d9534f;color:white;");

            QTimer::singleShot(1500, this, [this]{
                btn_open_->setStyleSheet("");
                btn_open_->setText("Open Port");
            });

            ROS_ERROR("Failed to call /serial_node/open");
        }
    }



    void onStart() {
      // 1) 퍼블리시(시리얼 송수신) 시작 요청
      std_srvs::Trigger srv_start;
      if (start_client_.call(srv_start) && srv_start.response.success) {

        // 2) RecorderNode 데이터 초기화 요청 (custom reset 서비스)
        std_srvs::Trigger srv_reset;
        if (reset_client_.call(srv_reset) && srv_reset.response.success) {
          ROS_INFO("Recorder data cleared");
        } else {
          ROS_WARN("Failed to reset recorder data");
        }

        // 3) GUI 내부 상태 초기화
        start_time_      = ros::Time::now();
        data_.clear();
        is_publishing_   = true;
        has_prev_sample_ = false;    // ★ 추가: 이전 샘플 유무 초기화

        // 4) 버튼 스타일 & 레이블 업데이트
        btn_start_->setStyleSheet(
          "QPushButton{background:#d9534f;color:white;"
          "border:1px solid #999;border-radius:10px;padding:8px 12px;}"
          "QPushButton:hover{background:#c9302c;}"
        );
        btn_start_->setText("Measurement is running. Click to stop.");

        // 5) 클릭 시 onStop()이 호출되도록 시그널 토글
        disconnect(btn_start_, &QPushButton::clicked, this, &GuiNode::onStart);
        connect(btn_start_,    &QPushButton::clicked, this, &GuiNode::onStop);

        // 6) 포트 열기 버튼 비활성화
        btn_open_->setEnabled(false);

        ROS_INFO("Publishing started");
      } else {
        ROS_ERROR("onStart failed");
      }
    }

    // 1) onStop(): 퍼블리시만 멈추고, ros::shutdown() 절대 호출 안 함
    void onStop() {
    std_srvs::Trigger srv;
    if (stop_client_.call(srv) && srv.response.success) {
        btn_start_->setText("Start Measure and Publish (rostopic /dial_gauge/raw)");
        disconnect(btn_start_, &QPushButton::clicked, this, &GuiNode::onStop);
        connect(btn_start_, &QPushButton::clicked, this, &GuiNode::onStart);
        ROS_INFO("Publishing stopped");
        is_publishing_ = false;
        btn_start_->setStyleSheet("");                               // ★ 추가
        btn_open_->setEnabled(true);      
    } else {
        ROS_ERROR("onStop failed");
    }
    // ** 여기에 ros::shutdown() 절대 넣지 마세요! **
    }

    void onExport() {
        if (export_path_.isEmpty()) {
        ROS_WARN("No export path chosen. Please click Browse first.");
        btn_export_->setText("Choose Path!");
        btn_export_->setStyleSheet("background:#d9534f;color:white;");
        return;
        }

        // @@ 3) 파일명 입력란 값 확인
        QString fname = le_filename_->text().trimmed();
        if (fname.isEmpty()) {
        btn_export_->setText("Enter filename and [Press to export]");
        btn_export_->setStyleSheet("background:#d9534f;color:white;");
        ROS_WARN("filename textbox is empty. Please type the filename you want.");

        return;
        }

        // @@ 자동으로 .txt 확장자 붙이기
        if (!fname.endsWith(".txt", Qt::CaseInsensitive)) {
        fname += ".txt";
        }

        // @@ 4) 입력된 파일명과 디렉토리 결합
        QDir d(QFileInfo(export_path_).absolutePath());
        QString full = d.filePath(fname);

        // ROS 파라미터로 내보낼 경로 설정
        ros::param::set("/recorder_node/export_path", full.toStdString());
        ROS_INFO_STREAM("Set export path to: " << full.toStdString());

        std_srvs::Trigger srv;
        if (export_client_.call(srv) && srv.response.success) {
        btn_export_->setText("Exported");
        btn_export_->setStyleSheet("background:#28a745;color:white;");      // ★ 초록

        QTimer::singleShot(1500, this, [this]{                           
            btn_export_->setText("Export Data");
            btn_export_->setStyleSheet("");
        });
        } else {
        btn_export_->setText("Export Failed");
        ROS_ERROR("onExport failed");
        }
    }


  void onData(const std_msgs::Float64::ConstPtr& msg) {

    ros::Time now_ros = ros::Time::now();                       // 앞줄
    double t  = (now_ros - start_time_).toSec();

    double dt = has_prev_sample_ ? (now_ros - last_sample_time_).toSec() : 0.0;  // ★ 추가
    double hz = dt > 0.0 ? 1.0 / dt : 0.0;       
    
    //  실시간 값 라벨 업데이트
    lbl_value_->setText(
        QString("Time: %1&nbsp;s&nbsp;&nbsp;Value: "
                "<span style=\"color:#d9534f;\">%2</span>"
                "&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"   /* 3칸 여백 */
                "<span style=\"font-size:15pt;\">Δt: %3&nbsp;s&nbsp;(%4&nbsp;Hz)</span>")
            .arg(t, 0, 'f', 3)
            .arg(msg->data, 0, 'f', 3)
            .arg(dt, 0, 'f', 6)
            .arg(hz, 0, 'f', 1)
    );             
    last_sample_time_ = now_ros;           // ★ 추가
    has_prev_sample_  = true;              // ★ 추가

    QPointF pt(t, msg->data);

    // 차트용
    data_.push_back(pt);
    // // 제한할 경우:
    // if (data_.size() > 1000) data_.pop_front();

    // 전체 데이터 보관:  실제 파일 저장은 recorder_node 가 담당하므로 필요x 
    // full_data_.push_back(pt);
  }

    void onBrowse() {
    // 전체 파일 경로 선택
    QString file = QFileDialog::getSaveFileName(
        this,
        "Select export file",
        export_path_.isEmpty() ? QDir::homePath() + "/dial_data.txt"
                            : export_path_,
        "Text files (*.txt);;All files (*)"
    );
    if (file.isEmpty()) return;

    export_path_ = file;
    ros::param::set("/recorder_node/export_path", export_path_.toStdString());
    btn_export_->setStyleSheet("");  
    btn_browse_->setStyleSheet("background:#28a745;color:white;");  // ★ 추가 (녹색)
    // @@ 버튼에는 디렉토리만 표시하도록 수정
    QString dir = QFileInfo(export_path_).absolutePath();   // 전체에서 디렉토리만 꺼냄
    btn_browse_->setText("Save Path: " + dir);                              // 버튼에 디렉토리만 보여 줌

    btn_export_->setText("Export Data");
    }



  void onTimer() {
    ros::spinOnce();
    // @@ 시작: 퍼블리시 중일 때만 시간 축 갱신
    if (is_publishing_) {
      double now = (ros::Time::now() - start_time_).toSec();
      const double window = 30.0;
      if (axisX_) {
        if (now > window)
          axisX_->setRange(now - window, now);
        else
          axisX_->setRange(0, window);
      }
    }
    series_->replace(data_);
  }

// 2) closeEvent(): 오직 GUI 창 닫힐 때만 ROS 종료
protected:
    void closeEvent(QCloseEvent* event) override {
    std_srvs::Trigger srv;
    if (stop_client_.call(srv) && srv.response.success)
        ROS_INFO("Stopped publishing from closeEvent");
    ros::shutdown();
    ROS_INFO("ros::shutdown() called from closeEvent");
    QMainWindow::closeEvent(event);
    }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::ServiceClient open_client_, start_client_, stop_client_, export_client_;

  QPushButton* btn_open_;
  QPushButton* btn_start_;
  QPushButton* btn_export_;

  QLabel* lbl_value_;
  QLabel*    lbl_filename_;    // @@ 추가
  QLineEdit* le_filename_;     // @@ 추가
  std::vector<QPointF> full_data_; // (옵션) 전체 데이터 저장용

  QTimer* timer_;
  QChart* chart_;
  QLineSeries* series_;
  QValueAxis* axisX_;
  QValueAxis* axisY_;
  QVector<QPointF> data_;
  ros::Time start_time_;

  ros::Time last_sample_time_;       // ★ Δt 계산용
  bool      has_prev_sample_ = false;// ★ 첫 샘플 판별

  // 트레이 아이콘 관련 멤버
  QSystemTrayIcon* trayIcon_;
  QMenu*           trayMenu_;
  QAction*         actionShow_;
  QAction*         actionQuit_;

  QPushButton* btn_browse_;
  QString       export_path_;  // 사용자가 고른 경로

  bool is_publishing_ = false;

  ros::ServiceClient reset_client_;  // @@ 추가

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gui_node");
  ros::NodeHandle nh;
  QApplication app(argc, argv);


  
  qApp->setStyleSheet(
      "QWidget{background:#ffffff;}"
      /* ── 버튼 ─────────────────────────────────────────── */
      "QPushButton{border:1px solid #999;border-radius:10px;padding:8px 12px;}"
      "QPushButton:hover{background:#0078d4;color:white;}"
      "QPushButton:pressed{background:#005fa3;color:white;}"   /* 클릭‑중 색 */
      "QPushButton:disabled{background:#e0e0e0;color:#8c8c8c;}"/* 비활성 색 */
      /* ── 라인에디트 ───────────────────────────────────── */
      "QLineEdit{border:1px solid #ccc;border-radius:6px;padding:4px;}"
      "QLineEdit:focus{border:2px solid #0078d4;}"              /* 포커스 강조 */
      /* ── 상태바 ───────────────────────────────────────── */
      "QStatusBar{background:#f5f5f5;border-top:1px solid #ccc;}"
      /* ── 툴팁 ─────────────────────────────────────────── */
      "QToolTip{background:#333;color:#fff;border:none;padding:4px;border-radius:4px;}"
  );


  GuiNode window(nh);
  window.show();
  return app.exec();
}

#include "gui_node.moc"
