/**
 * @file radar_qt.cpp
 * @brief Qt GUI for radar simulator - 参数配置界面
 *
 * Usage:
 *   ./radar_client             # 本地模式（内嵌服务端）
 *   ./radar_client --remote 192.168.1.100:50051  # 远程模式
 *   ./radar_client --port 50052  # 修改本地端口
 */

#include <grpcpp/grpcpp.h>
#include <spdlog/spdlog.h>

#include <QApplication>
#include <QButtonGroup>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <memory>
#include <string>

#include "grpc/embedded_server.h"
#include "grpc/radar_service.grpc.pb.h"
#include "grpc/radar_service.pb.h"

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  MainWindow(const std::string &server_addr, bool local_mode,
             QWidget *parent = nullptr)
      : QMainWindow(parent), server_addr_(server_addr),
        local_mode_(local_mode) {

    setWindowTitle("雷达回波模拟器 - 参数配置");
    resize(1000, 800);

    auto *central = new QWidget(this);
    auto *main_layout = new QVBoxLayout(central);
    setCentralWidget(central);

    auto *info_label = new QLabel(
        local_mode_ ? "模式: 本地（内嵌服务端）"
                    : QString("模式: 远程 (%1)").arg(server_addr_.c_str()),
        this);
    info_label->setStyleSheet(
        "font-weight: bold; padding: 4px; background-color: #f0f0f0;");
    main_layout->addWidget(info_label);

    auto *scroll_area = new QScrollArea(this);
    scroll_area->setWidgetResizable(true);
    auto *scroll_content = new QWidget(this);
    auto *config_layout = new QVBoxLayout(scroll_content);
    scroll_area->setWidget(scroll_content);
    main_layout->addWidget(scroll_area, 1);

    create_system_params(config_layout);
    create_waveform_params(config_layout);
    create_antenna_params(config_layout);
    create_mech_scan_params(config_layout);
    create_noise_params(config_layout);
    create_clutter_params(config_layout);
    create_target_params(config_layout);

    auto *control_group = new QGroupBox("仿真控制", this);
    auto *control_layout = new QHBoxLayout(control_group);

    apply_btn_ = new QPushButton("应用配置", this);
    validate_btn_ = new QPushButton("验证配置", this);
    start_btn_ = new QPushButton("开始仿真", this);
    stop_btn_ = new QPushButton("停止仿真", this);
    stop_btn_->setEnabled(false);

    progress_bar_ = new QProgressBar(this);
    status_label_ = new QLabel("状态: 空闲", this);

    control_layout->addWidget(apply_btn_);
    control_layout->addWidget(validate_btn_);
    control_layout->addWidget(start_btn_);
    control_layout->addWidget(stop_btn_);
    control_layout->addWidget(progress_bar_, 1);
    control_layout->addWidget(status_label_);
    main_layout->addWidget(control_group);

    connect(apply_btn_, &QPushButton::clicked, this,
            &MainWindow::on_apply_config);
    connect(validate_btn_, &QPushButton::clicked, this,
            &MainWindow::on_validate);
    connect(start_btn_, &QPushButton::clicked, this, &MainWindow::on_start);
    connect(stop_btn_, &QPushButton::clicked, this, &MainWindow::on_stop);

    connect(this, &MainWindow::started, this, [this]() {
      start_btn_->setEnabled(false);
      stop_btn_->setEnabled(true);
    });
    connect(this, &MainWindow::stopped, this, [this]() {
      start_btn_->setEnabled(true);
      stop_btn_->setEnabled(false);
    });

    auto channel =
        grpc::CreateChannel(server_addr_, grpc::InsecureChannelCredentials());
    stub_ = radar::RadarService::NewStub(channel);

    auto *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::poll_status);
    timer->start(1000);
  }

signals:
  void started();
  void stopped();

private slots:
  void on_apply_config() {
    if (!send_system_params())
      return;
    if (!send_waveform_params())
      return;
    if (!send_antenna_params())
      return;
    if (!send_mech_scan_params())
      return;
    if (!send_noise_params())
      return;
    if (!send_clutter_params())
      return;

    status_label_->setText("状态: 配置已应用");
    QMessageBox::information(this, "成功", "配置已成功应用");
  }

  void on_validate() {
    radar::Empty request;
    radar::ValidationResult reply;
    grpc::ClientContext ctx;
    auto status = stub_->ValidateAll(&ctx, request, &reply);

    if (!status.ok()) {
      QMessageBox::critical(this, "gRPC错误",
                            QString::fromStdString(status.error_message()));
      return;
    }

    if (reply.valid()) {
      QMessageBox::information(this, "验证结果", "配置验证通过");
    } else {
      QString msg = "配置验证失败:\n";
      for (int i = 0; i < reply.errors_size(); ++i) {
        msg += "  - " + QString::fromStdString(reply.errors(i)) + "\n";
      }
      QMessageBox::warning(this, "验证失败", msg);
    }
  }

  void on_start() {
    radar::Empty request;
    radar::Status reply;
    grpc::ClientContext ctx;
    auto status = stub_->StartSimulation(&ctx, request, &reply);

    if (!status.ok()) {
      QMessageBox::critical(this, "gRPC错误",
                            QString::fromStdString(status.error_message()));
      return;
    }

    if (!reply.success()) {
      QMessageBox::warning(this, "启动错误",
                           QString::fromStdString(reply.message()));
      return;
    }

    emit started();
    status_label_->setText("状态: 运行中");
  }

  void on_stop() {
    radar::Empty request;
    radar::Status reply;
    grpc::ClientContext ctx;
    stub_->StopSimulation(&ctx, request, &reply);
    emit stopped();
    status_label_->setText("状态: 已停止");
  }

  void poll_status() {
    radar::Empty request;
    radar::SimStatus reply;
    grpc::ClientContext ctx;
    auto status = stub_->GetStatus(&ctx, request, &reply);

    if (!status.ok())
      return;

    if (reply.state() == radar::SimStatus::RUNNING) {
      progress_bar_->setValue(static_cast<int>(reply.progress()));
      status_label_->setText(QString("状态: 运行中 (%1/%2)")
                                 .arg(reply.current_scan())
                                 .arg(reply.total_scans()));
    }
  }

  void on_add_target() {
    radar::TargetParams request;
    request.set_id(next_target_id_++);
    request.set_range_m(target_range_->value() * 1000);
    request.set_azimuth_deg(target_azimuth_->value());
    request.set_elevation_deg(target_elevation_->value());
    request.set_radial_velocity_ms(target_velocity_->value());
    request.set_rcs_db(target_rcs_->value());

    QString swerling_text = target_swerling_->currentText();
    int swerling_type = 0;
    if (swerling_text == "Swerling0")
      swerling_type = 0;
    else if (swerling_text == "Swerling1")
      swerling_type = 1;
    else if (swerling_text == "Swerling2")
      swerling_type = 2;
    else if (swerling_text == "Swerling3")
      swerling_type = 3;
    else if (swerling_text == "Swerling4")
      swerling_type = 4;
    request.set_swerling_type(swerling_type);

    radar::Status reply;
    grpc::ClientContext ctx;
    auto status = stub_->AddTarget(&ctx, request, &reply);

    if (!status.ok()) {
      QMessageBox::critical(this, "gRPC错误",
                            QString::fromStdString(status.error_message()));
      return;
    }

    if (reply.success()) {
      update_target_table();
    } else {
      QMessageBox::warning(this, "添加目标失败",
                           QString::fromStdString(reply.message()));
    }
  }

  void on_remove_target() {
    int row = target_table_->currentRow();
    if (row < 0) {
      QMessageBox::warning(this, "提示", "请先选择要删除的目标");
      return;
    }

    int target_id = target_table_->item(row, 0)->text().toInt();
    radar::IntValue request;
    request.set_value(target_id);

    radar::Status reply;
    grpc::ClientContext ctx;
    auto status = stub_->RemoveTarget(&ctx, request, &reply);

    if (!status.ok()) {
      QMessageBox::critical(this, "gRPC错误",
                            QString::fromStdString(status.error_message()));
      return;
    }

    if (reply.success()) {
      update_target_table();
    }
  }

  void on_clear_targets() {
    radar::Empty request;
    radar::Status reply;
    grpc::ClientContext ctx;
    stub_->ClearTargets(&ctx, request, &reply);
    update_target_table();
  }

private:
  void create_system_params(QVBoxLayout *parent) {
    auto *group = new QGroupBox("系统参数", this);
    auto *layout = new QFormLayout(group);

    sys_fc_ = new QDoubleSpinBox(this);
    sys_fc_->setRange(1, 100);
    sys_fc_->setDecimals(3);
    sys_fc_->setValue(10.0);
    sys_fc_->setSuffix(" GHz");
    layout->addRow("载波频率:", sys_fc_);

    sys_prf_ = new QDoubleSpinBox(this);
    sys_prf_->setRange(100, 10000);
    sys_prf_->setDecimals(0);
    sys_prf_->setValue(1600);
    sys_prf_->setSuffix(" Hz");
    layout->addRow("脉冲重复频率 PRF:", sys_prf_);

    sys_bw_ = new QDoubleSpinBox(this);
    sys_bw_->setRange(1, 100);
    sys_bw_->setDecimals(3);
    sys_bw_->setValue(25.0);
    sys_bw_->setSuffix(" MHz");
    layout->addRow("信号带宽:", sys_bw_);

    sys_pulse_width_ = new QDoubleSpinBox(this);
    sys_pulse_width_->setRange(0.1, 100);
    sys_pulse_width_->setDecimals(1);
    sys_pulse_width_->setValue(2.0);
    sys_pulse_width_->setSuffix(" μs");
    layout->addRow("脉冲宽度:", sys_pulse_width_);

    sys_fs_ = new QDoubleSpinBox(this);
    sys_fs_->setRange(1, 200);
    sys_fs_->setDecimals(1);
    sys_fs_->setValue(60.0);
    sys_fs_->setSuffix(" MHz");
    layout->addRow("采样率:", sys_fs_);

    sys_min_range_ = new QDoubleSpinBox(this);
    sys_min_range_->setRange(0.1, 100);
    sys_min_range_->setDecimals(1);
    sys_min_range_->setValue(1.0);
    sys_min_range_->setSuffix(" km");
    layout->addRow("最小探测距离:", sys_min_range_);

    sys_max_range_ = new QDoubleSpinBox(this);
    sys_max_range_->setRange(1, 500);
    sys_max_range_->setDecimals(1);
    sys_max_range_->setValue(100.0);
    sys_max_range_->setSuffix(" km");
    layout->addRow("最大探测距离:", sys_max_range_);

    sys_scan_count_ = new QSpinBox(this);
    sys_scan_count_->setRange(1, 100);
    sys_scan_count_->setValue(10);
    layout->addRow("仿真扫描圈数:", sys_scan_count_);

    sys_peak_power_ = new QDoubleSpinBox(this);
    sys_peak_power_->setRange(1, 1e6);
    sys_peak_power_->setDecimals(0);
    sys_peak_power_->setValue(5000);
    sys_peak_power_->setSuffix(" W");
    layout->addRow("峰值发射功率:", sys_peak_power_);

    parent->addWidget(group);
  }

  void create_waveform_params(QVBoxLayout *parent) {
    auto *group = new QGroupBox("波形参数", this);
    auto *layout = new QFormLayout(group);

    wf_type_ = new QComboBox(this);
    wf_type_->addItem("LFM", 0);
    wf_type_->addItem("相位编码", 1);
    wf_type_->addItem("NLFM", 2);
    wf_type_->addItem("STARE", 3);
    layout->addRow("波形类型:", wf_type_);

    wf_code_length_ = new QSpinBox(this);
    wf_code_length_->setRange(2, 256);
    wf_code_length_->setValue(32);
    layout->addRow("相位码长度:", wf_code_length_);

    wf_phase_code_ = new QComboBox(this);
    wf_phase_code_->addItem("Barker", 0);
    wf_phase_code_->addItem("PN", 1);
    wf_phase_code_->addItem("Frank", 2);
    wf_phase_code_->addItem("Zadoff-Chu", 3);
    layout->addRow("相位码类型:", wf_phase_code_);

    wf_nlfm_window_ = new QComboBox(this);
    wf_nlfm_window_->addItem("矩形窗", 0);
    wf_nlfm_window_->addItem("Hamming", 1);
    wf_nlfm_window_->addItem("Hann", 2);
    wf_nlfm_window_->addItem("Blackman", 3);
    layout->addRow("NLFM窗函数:", wf_nlfm_window_);

    wf_polarization_ = new QComboBox(this);
    wf_polarization_->addItem("水平极化", 0);
    wf_polarization_->addItem("垂直极化", 1);
    wf_polarization_->addItem("圆极化", 2);
    layout->addRow("极化方式:", wf_polarization_);

    parent->addWidget(group);
  }

  void create_antenna_params(QVBoxLayout *parent) {
    auto *group = new QGroupBox("天线参数（反射面）", this);
    auto *layout = new QFormLayout(group);

    ant_az_beamwidth_ = new QDoubleSpinBox(this);
    ant_az_beamwidth_->setRange(0.1, 180.0);
    ant_az_beamwidth_->setDecimals(1);
    ant_az_beamwidth_->setValue(5.0);
    ant_az_beamwidth_->setSuffix(" °");
    layout->addRow("方位波束宽度:", ant_az_beamwidth_);

    ant_el_beamwidth_ = new QDoubleSpinBox(this);
    ant_el_beamwidth_->setRange(0.1, 180.0);
    ant_el_beamwidth_->setDecimals(1);
    ant_el_beamwidth_->setValue(5.0);
    ant_el_beamwidth_->setSuffix(" °");
    layout->addRow("俯仰波束宽度:", ant_el_beamwidth_);

    ant_gain_ = new QDoubleSpinBox(this);
    ant_gain_->setRange(20, 50);
    ant_gain_->setDecimals(1);
    ant_gain_->setValue(35.0);
    ant_gain_->setSuffix(" dB");
    layout->addRow("峰值增益:", ant_gain_);

    parent->addWidget(group);
  }

  void create_mech_scan_params(QVBoxLayout *parent) {
    auto *group = new QGroupBox("机械扫描参数", this);
    auto *layout = new QFormLayout(group);

    mech_rotation_rate_ = new QDoubleSpinBox(this);
    mech_rotation_rate_->setRange(1, 360);
    mech_rotation_rate_->setDecimals(1);
    mech_rotation_rate_->setValue(60.0);
    mech_rotation_rate_->setSuffix(" °/s");
    layout->addRow("天线转速:", mech_rotation_rate_);

    mech_az_start_ = new QDoubleSpinBox(this);
    mech_az_start_->setRange(0, 360);
    mech_az_start_->setDecimals(1);
    mech_az_start_->setValue(0.0);
    mech_az_start_->setSuffix(" °");
    layout->addRow("起始方位角:", mech_az_start_);

    mech_az_end_ = new QDoubleSpinBox(this);
    mech_az_end_->setRange(0, 360);
    mech_az_end_->setDecimals(1);
    mech_az_end_->setValue(360.0);
    mech_az_end_->setSuffix(" °");
    layout->addRow("终止方位角:", mech_az_end_);

    mech_elevation_ = new QDoubleSpinBox(this);
    mech_elevation_->setRange(-90, 90);
    mech_elevation_->setDecimals(1);
    mech_elevation_->setValue(0.0);
    mech_elevation_->setSuffix(" °");
    layout->addRow("固定俯仰角:", mech_elevation_);

    parent->addWidget(group);
  }

  void create_noise_params(QVBoxLayout *parent) {
    auto *group = new QGroupBox("噪声参数", this);
    auto *layout = new QFormLayout(group);

    noise_mode_ = new QComboBox(this);
    noise_mode_->addItem("禁用", 0);
    noise_mode_->addItem("复高斯", 1);
    noise_mode_->addItem("接收机温度", 2);
    noise_mode_->addItem("信噪比", 3);
    layout->addRow("噪声模式:", noise_mode_);

    noise_sigma_ = new QDoubleSpinBox(this);
    noise_sigma_->setRange(0.001, 10);
    noise_sigma_->setDecimals(4);
    noise_sigma_->setValue(0.1);
    layout->addRow("复高斯标准差:", noise_sigma_);

    noise_temp_ = new QDoubleSpinBox(this);
    noise_temp_->setRange(100, 500);
    noise_temp_->setDecimals(0);
    noise_temp_->setValue(290);
    noise_temp_->setSuffix(" K");
    layout->addRow("噪声温度:", noise_temp_);

    noise_power_ = new QDoubleSpinBox(this);
    noise_power_->setRange(1e-12, 1e-3);
    noise_power_->setDecimals(6);
    noise_power_->setValue(1e-6);
    noise_power_->setSuffix(" W");
    layout->addRow("噪声功率:", noise_power_);

    noise_bandwidth_ = new QDoubleSpinBox(this);
    noise_bandwidth_->setRange(1e6, 100e6);
    noise_bandwidth_->setDecimals(0);
    noise_bandwidth_->setValue(20e6);
    noise_bandwidth_->setSuffix(" Hz");
    layout->addRow("噪声带宽:", noise_bandwidth_);

    noise_seed_ = new QSpinBox(this);
    noise_seed_->setRange(0, 999999);
    noise_seed_->setValue(12345);
    layout->addRow("随机种子:", noise_seed_);

    parent->addWidget(group);
  }

  void create_clutter_params(QVBoxLayout *parent) {
    auto *group = new QGroupBox("杂波参数", this);
    auto *layout = new QFormLayout(group);

    clt_enabled_ = new QComboBox(this);
    clt_enabled_->addItem("禁用", 0);
    clt_enabled_->addItem("启用", 1);
    layout->addRow("海杂波:", clt_enabled_);

    clt_sea_state_ = new QSpinBox(this);
    clt_sea_state_->setRange(1, 9);
    clt_sea_state_->setValue(3);
    layout->addRow("海态等级:", clt_sea_state_);

    clt_doppler_center_ = new QDoubleSpinBox(this);
    clt_doppler_center_->setRange(-500, 500);
    clt_doppler_center_->setDecimals(1);
    clt_doppler_center_->setValue(0);
    clt_doppler_center_->setSuffix(" Hz");
    layout->addRow("多普勒中心:", clt_doppler_center_);

    clt_doppler_spread_ = new QDoubleSpinBox(this);
    clt_doppler_spread_->setRange(1, 200);
    clt_doppler_spread_->setDecimals(1);
    clt_doppler_spread_->setValue(50);
    clt_doppler_spread_->setSuffix(" Hz");
    layout->addRow("多普勒扩展:", clt_doppler_spread_);

    clt_range_min_ = new QDoubleSpinBox(this);
    clt_range_min_->setRange(0, 500);
    clt_range_min_->setDecimals(1);
    clt_range_min_->setValue(0);
    clt_range_min_->setSuffix(" km");
    layout->addRow("杂波最小距离:", clt_range_min_);

    clt_range_max_ = new QDoubleSpinBox(this);
    clt_range_max_->setRange(0, 500);
    clt_range_max_->setDecimals(1);
    clt_range_max_->setValue(100);
    clt_range_max_->setSuffix(" km");
    layout->addRow("杂波最大距离:", clt_range_max_);

    clt_seed_ = new QSpinBox(this);
    clt_seed_->setRange(0, 999999);
    clt_seed_->setValue(54321);
    layout->addRow("随机种子:", clt_seed_);

    clt_az_patch_step_ = new QDoubleSpinBox(this);
    clt_az_patch_step_->setRange(0.01, 5.0);
    clt_az_patch_step_->setDecimals(2);
    clt_az_patch_step_->setSingleStep(0.05);
    clt_az_patch_step_->setValue(0.25);
    clt_az_patch_step_->setSuffix(" deg");
    layout->addRow("方位patch步长:", clt_az_patch_step_);

    clt_active_gain_floor_ = new QDoubleSpinBox(this);
    clt_active_gain_floor_->setRange(-120.0, -1.0);
    clt_active_gain_floor_->setDecimals(1);
    clt_active_gain_floor_->setSingleStep(5.0);
    clt_active_gain_floor_->setValue(-40.0);
    clt_active_gain_floor_->setSuffix(" dB");
    layout->addRow("活动窗口门限:", clt_active_gain_floor_);

    clt_distribution_ = new QComboBox(this);
    clt_distribution_->addItem("瑞利/复高斯", 0);
    clt_distribution_->addItem("韦布尔", 1);
    clt_distribution_->addItem("对数正态", 2);
    clt_distribution_->addItem("K分布", 3);
    layout->addRow("杂波分布:", clt_distribution_);

    clt_weibull_shape_ = new QDoubleSpinBox(this);
    clt_weibull_shape_->setRange(0.1, 20.0);
    clt_weibull_shape_->setDecimals(2);
    clt_weibull_shape_->setSingleStep(0.1);
    clt_weibull_shape_->setValue(2.0);
    layout->addRow("Weibull p:", clt_weibull_shape_);

    clt_weibull_scale_ = new QDoubleSpinBox(this);
    clt_weibull_scale_->setRange(0.01, 100.0);
    clt_weibull_scale_->setDecimals(3);
    clt_weibull_scale_->setSingleStep(0.1);
    clt_weibull_scale_->setValue(1.414);
    layout->addRow("Weibull q:", clt_weibull_scale_);

    clt_lognormal_mu_ = new QDoubleSpinBox(this);
    clt_lognormal_mu_->setRange(-20.0, 20.0);
    clt_lognormal_mu_->setDecimals(3);
    clt_lognormal_mu_->setSingleStep(0.1);
    clt_lognormal_mu_->setValue(-1.0);
    layout->addRow("LogNormal mu:", clt_lognormal_mu_);

    clt_lognormal_sigma_ = new QDoubleSpinBox(this);
    clt_lognormal_sigma_->setRange(0.0, 10.0);
    clt_lognormal_sigma_->setDecimals(3);
    clt_lognormal_sigma_->setSingleStep(0.1);
    clt_lognormal_sigma_->setValue(1.0);
    layout->addRow("LogNormal sigma:", clt_lognormal_sigma_);

    clt_k_shape_nu_ = new QDoubleSpinBox(this);
    clt_k_shape_nu_->setRange(0.05, 100.0);
    clt_k_shape_nu_->setDecimals(3);
    clt_k_shape_nu_->setSingleStep(0.1);
    clt_k_shape_nu_->setValue(1.0);
    layout->addRow("K nu:", clt_k_shape_nu_);

    clt_k_texture_bandwidth_ = new QDoubleSpinBox(this);
    clt_k_texture_bandwidth_->setRange(0.01, 100.0);
    clt_k_texture_bandwidth_->setDecimals(3);
    clt_k_texture_bandwidth_->setSingleStep(0.5);
    clt_k_texture_bandwidth_->setValue(2.0);
    clt_k_texture_bandwidth_->setSuffix(" Hz");
    layout->addRow("K texture BW:", clt_k_texture_bandwidth_);

    clt_k_lut_size_ = new QSpinBox(this);
    clt_k_lut_size_->setRange(16, 65536);
    clt_k_lut_size_->setSingleStep(1024);
    clt_k_lut_size_->setValue(4096);
    layout->addRow("K LUT size:", clt_k_lut_size_);

    parent->addWidget(group);
  }

  void create_target_params(QVBoxLayout *parent) {
    auto *group = new QGroupBox("目标参数", this);
    auto *layout = new QVBoxLayout(group);

    auto *input_group = new QGroupBox("添加目标", this);
    auto *input_layout = new QFormLayout(input_group);

    target_range_ = new QDoubleSpinBox(this);
    target_range_->setRange(1, 500);
    target_range_->setDecimals(1);
    target_range_->setValue(50.0);
    target_range_->setSuffix(" km");
    input_layout->addRow("距离:", target_range_);

    target_azimuth_ = new QDoubleSpinBox(this);
    target_azimuth_->setRange(-180, 180);
    target_azimuth_->setDecimals(1);
    target_azimuth_->setValue(0.0);
    target_azimuth_->setSuffix(" °");
    input_layout->addRow("方位角:", target_azimuth_);

    target_elevation_ = new QDoubleSpinBox(this);
    target_elevation_->setRange(-90, 90);
    target_elevation_->setDecimals(1);
    target_elevation_->setValue(0.0);
    target_elevation_->setSuffix(" °");
    input_layout->addRow("俯仰角:", target_elevation_);

    target_velocity_ = new QDoubleSpinBox(this);
    target_velocity_->setRange(-500, 500);
    target_velocity_->setDecimals(1);
    target_velocity_->setValue(50.0);
    target_velocity_->setSuffix(" m/s");
    input_layout->addRow("径向速度:", target_velocity_);

    target_rcs_ = new QDoubleSpinBox(this);
    target_rcs_->setRange(-60, 60);
    target_rcs_->setDecimals(1);
    target_rcs_->setValue(0.0);
    target_rcs_->setSuffix(" dBsm");
    input_layout->addRow("RCS:", target_rcs_);

    target_swerling_ = new QComboBox(this);
    target_swerling_->addItem("Swerling0");
    target_swerling_->addItem("Swerling1");
    target_swerling_->addItem("Swerling2");
    target_swerling_->addItem("Swerling3");
    target_swerling_->addItem("Swerling4");
    target_swerling_->setCurrentText("Swerling1");
    input_layout->addRow("RCS起伏:", target_swerling_);

    auto *btn_layout = new QHBoxLayout();
    auto *add_btn = new QPushButton("添加目标", this);
    auto *remove_btn = new QPushButton("删除选中", this);
    auto *clear_btn = new QPushButton("清空所有", this);
    btn_layout->addWidget(add_btn);
    btn_layout->addWidget(remove_btn);
    btn_layout->addWidget(clear_btn);
    input_layout->addRow("", btn_layout);

    layout->addWidget(input_group);

    target_table_ = new QTableWidget(this);
    target_table_->setColumnCount(6);
    target_table_->setHorizontalHeaderLabels(
        {"ID", "距离(km)", "方位(°)", "俯仰(°)", "速度(m/s)", "RCS(dBsm)"});
    target_table_->setSelectionBehavior(QTableWidget::SelectRows);
    target_table_->setEditTriggers(QTableWidget::NoEditTriggers);
    layout->addWidget(target_table_);

    connect(add_btn, &QPushButton::clicked, this, &MainWindow::on_add_target);
    connect(remove_btn, &QPushButton::clicked, this,
            &MainWindow::on_remove_target);
    connect(clear_btn, &QPushButton::clicked, this,
            &MainWindow::on_clear_targets);

    parent->addWidget(group);
  }

  bool send_system_params() {
    radar::SystemParams request;
    request.set_fc_hz(sys_fc_->value() * 1e9);
    request.set_prf_hz(sys_prf_->value());
    request.set_bandwidth_hz(sys_bw_->value() * 1e6);
    request.set_pulse_width_s(sys_pulse_width_->value() * 1e-6);
    request.set_fs_hz(sys_fs_->value() * 1e6);
    request.set_min_range_m(sys_min_range_->value() * 1000);
    request.set_max_range_m(sys_max_range_->value() * 1000);
    request.set_scan_count(sys_scan_count_->value());
    request.set_peak_power_w(sys_peak_power_->value());

    radar::Status reply;
    grpc::ClientContext ctx;
    auto status = stub_->SetSystemParams(&ctx, request, &reply);

    if (!status.ok()) {
      QMessageBox::critical(this, "gRPC错误",
                            QString::fromStdString(status.error_message()));
      return false;
    }

    if (!reply.success()) {
      QMessageBox::warning(this, "系统参数错误",
                           QString::fromStdString(reply.message()));
      return false;
    }

    return true;
  }

  bool send_waveform_params() {
    radar::WaveformParams request;
    request.set_waveform_type(static_cast<radar::WaveformParams::WaveformType>(
        wf_type_->currentData().toInt()));
    request.set_phase_code_type(wf_phase_code_->currentData().toInt());
    request.set_nlfm_window_type(wf_nlfm_window_->currentData().toInt());
    request.set_polarization(wf_polarization_->currentData().toInt());

    radar::Status reply;
    grpc::ClientContext ctx;
    auto status = stub_->SetWaveformParams(&ctx, request, &reply);

    if (!status.ok()) {
      QMessageBox::critical(this, "gRPC错误",
                            QString::fromStdString(status.error_message()));
      return false;
    }

    if (!reply.success()) {
      QMessageBox::warning(this, "波形参数错误",
                           QString::fromStdString(reply.message()));
      return false;
    }

    return true;
  }

  bool send_antenna_params() {
    radar::AntennaParams request;
    request.set_az_beamwidth_deg(ant_az_beamwidth_->value());
    request.set_el_beamwidth_deg(ant_el_beamwidth_->value());
    request.set_peak_gain_db(ant_gain_->value());

    radar::Status reply;
    grpc::ClientContext ctx;
    auto status = stub_->SetAntennaParams(&ctx, request, &reply);

    if (!status.ok()) {
      QMessageBox::critical(this, "gRPC错误",
                            QString::fromStdString(status.error_message()));
      return false;
    }

    if (!reply.success()) {
      QMessageBox::warning(this, "天线参数错误",
                           QString::fromStdString(reply.message()));
      return false;
    }

    return true;
  }

  bool send_mech_scan_params() {
    radar::MechScanParams request;
    request.set_rotation_rate_dps(mech_rotation_rate_->value());
    request.set_az_start_deg(mech_az_start_->value());
    request.set_az_end_deg(mech_az_end_->value());
    request.set_elevation_deg(mech_elevation_->value());

    radar::Status reply;
    grpc::ClientContext ctx;
    auto status = stub_->SetMechScanParams(&ctx, request, &reply);

    if (!status.ok()) {
      QMessageBox::critical(this, "gRPC错误",
                            QString::fromStdString(status.error_message()));
      return false;
    }

    if (!reply.success()) {
      QMessageBox::warning(this, "扫描参数错误",
                           QString::fromStdString(reply.message()));
      return false;
    }

    return true;
  }

  bool send_noise_params() {
    radar::NoiseParams request;
    request.set_mode(static_cast<radar::NoiseParams::NoiseMode>(
        noise_mode_->currentData().toInt()));
    request.set_sigma_complex(noise_sigma_->value());
    request.set_temperature_k(noise_temp_->value());
    request.set_noise_power_w(noise_power_->value());
    request.set_noise_bandwidth_hz(noise_bandwidth_->value());
    request.set_seed(noise_seed_->value());

    radar::Status reply;
    grpc::ClientContext ctx;
    auto status = stub_->SetNoiseParams(&ctx, request, &reply);

    if (!status.ok()) {
      QMessageBox::critical(this, "gRPC错误",
                            QString::fromStdString(status.error_message()));
      return false;
    }

    if (!reply.success()) {
      QMessageBox::warning(this, "噪声参数错误",
                           QString::fromStdString(reply.message()));
      return false;
    }

    return true;
  }

  bool send_clutter_params() {
    radar::ClutterParams request;
    request.set_enabled(clt_enabled_->currentData().toInt() == 1);
    request.set_sea_state(clt_sea_state_->value());
    request.set_doppler_center_hz(clt_doppler_center_->value());
    request.set_doppler_sigma_hz(clt_doppler_spread_->value());
    request.set_ground_range_min_m(clt_range_min_->value() * 1000);
    request.set_ground_range_max_m(clt_range_max_->value() * 1000);
    request.set_seed(clt_seed_->value());
    request.set_az_patch_step_deg(clt_az_patch_step_->value());
    request.set_active_gain_floor_db(clt_active_gain_floor_->value());
    request.set_distribution(static_cast<radar::ClutterParams::ClutterDistribution>(
        clt_distribution_->currentData().toInt()));
    request.set_weibull_shape(clt_weibull_shape_->value());
    request.set_weibull_scale(clt_weibull_scale_->value());
    request.set_lognormal_mu(clt_lognormal_mu_->value());
    request.set_lognormal_sigma(clt_lognormal_sigma_->value());
    request.set_k_shape_nu(clt_k_shape_nu_->value());
    request.set_k_texture_bandwidth_hz(clt_k_texture_bandwidth_->value());
    request.set_k_lut_size(clt_k_lut_size_->value());

    radar::Status reply;
    grpc::ClientContext ctx;
    auto status = stub_->SetClutterParams(&ctx, request, &reply);

    if (!status.ok()) {
      QMessageBox::critical(this, "gRPC错误",
                            QString::fromStdString(status.error_message()));
      return false;
    }

    if (!reply.success()) {
      QMessageBox::warning(this, "杂波参数错误",
                           QString::fromStdString(reply.message()));
      return false;
    }

    return true;
  }

  void update_target_table() {
    radar::Empty request;
    radar::TargetListResponse reply;
    grpc::ClientContext ctx;
    auto status = stub_->GetTargets(&ctx, request, &reply);

    if (!status.ok())
      return;

    target_table_->setRowCount(reply.targets_size());
    for (int i = 0; i < reply.targets_size(); ++i) {
      const auto &t = reply.targets(i);
      target_table_->setItem(i, 0,
                             new QTableWidgetItem(QString::number(t.id())));
      target_table_->setItem(
          i, 1,
          new QTableWidgetItem(QString::number(t.range_m() / 1000, 'f', 1)));
      target_table_->setItem(
          i, 2, new QTableWidgetItem(QString::number(t.azimuth_deg(), 'f', 1)));
      target_table_->setItem(
          i, 3,
          new QTableWidgetItem(QString::number(t.elevation_deg(), 'f', 1)));
      target_table_->setItem(i, 4,
                             new QTableWidgetItem(QString::number(
                                 t.radial_velocity_ms(), 'f', 1)));
      target_table_->setItem(
          i, 5, new QTableWidgetItem(QString::number(t.rcs_db(), 'f', 1)));
    }
    target_table_->resizeColumnsToContents();
  }

  std::string server_addr_;
  bool local_mode_;
  std::unique_ptr<radar::RadarService::Stub> stub_;
  int next_target_id_ = 1;

  QPushButton *apply_btn_;
  QPushButton *validate_btn_;
  QPushButton *start_btn_;
  QPushButton *stop_btn_;
  QProgressBar *progress_bar_;
  QLabel *status_label_;
  QTableWidget *target_table_;

  QDoubleSpinBox *sys_fc_;
  QDoubleSpinBox *sys_prf_;
  QDoubleSpinBox *sys_bw_;
  QDoubleSpinBox *sys_pulse_width_;
  QDoubleSpinBox *sys_fs_;
  QDoubleSpinBox *sys_min_range_;
  QDoubleSpinBox *sys_max_range_;
  QSpinBox *sys_scan_count_;
  QDoubleSpinBox *sys_peak_power_;

  QComboBox *wf_type_;
  QSpinBox *wf_code_length_;
  QComboBox *wf_phase_code_;
  QComboBox *wf_nlfm_window_;
  QComboBox *wf_polarization_;

  QDoubleSpinBox *ant_az_beamwidth_;
  QDoubleSpinBox *ant_el_beamwidth_;
  QDoubleSpinBox *ant_gain_;

  QDoubleSpinBox *mech_rotation_rate_;
  QDoubleSpinBox *mech_az_start_;
  QDoubleSpinBox *mech_az_end_;
  QDoubleSpinBox *mech_elevation_;

  QComboBox *noise_mode_;
  QDoubleSpinBox *noise_sigma_;
  QDoubleSpinBox *noise_temp_;
  QDoubleSpinBox *noise_power_;
  QDoubleSpinBox *noise_bandwidth_;
  QSpinBox *noise_seed_;

  QComboBox *clt_enabled_;
  QSpinBox *clt_sea_state_;
  QDoubleSpinBox *clt_doppler_center_;
  QDoubleSpinBox *clt_doppler_spread_;
  QDoubleSpinBox *clt_range_min_;
  QDoubleSpinBox *clt_range_max_;
  QSpinBox *clt_seed_;
  QDoubleSpinBox *clt_az_patch_step_;
  QDoubleSpinBox *clt_active_gain_floor_;
  QComboBox *clt_distribution_;
  QDoubleSpinBox *clt_weibull_shape_;
  QDoubleSpinBox *clt_weibull_scale_;
  QDoubleSpinBox *clt_lognormal_mu_;
  QDoubleSpinBox *clt_lognormal_sigma_;
  QDoubleSpinBox *clt_k_shape_nu_;
  QDoubleSpinBox *clt_k_texture_bandwidth_;
  QSpinBox *clt_k_lut_size_;

  QDoubleSpinBox *target_range_;
  QDoubleSpinBox *target_azimuth_;
  QDoubleSpinBox *target_elevation_;
  QDoubleSpinBox *target_velocity_;
  QDoubleSpinBox *target_rcs_;
  QComboBox *target_swerling_;
};

namespace {
radar::EmbeddedServer *g_embedded = nullptr;

void cleanup_grpc() {
  radar::DestroyEmbeddedServer(g_embedded);
  g_embedded = nullptr;
}
} // namespace

int main(int argc, char *argv[]) {
  std::string remote_addr;
  int port = 50051;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--remote" && i + 1 < argc) {
      remote_addr = argv[++i];
    } else if (arg == "--port" && i + 1 < argc) {
      port = std::stoi(argv[++i]);
    }
  }

  bool local_mode = remote_addr.empty();
  std::string server_addr;

  if (local_mode) {
    server_addr = "localhost:" + std::to_string(port);
    g_embedded = radar::CreateEmbeddedServer(server_addr);
    qAddPostRoutine(cleanup_grpc);
  } else {
    server_addr = remote_addr;
  }

  QApplication app(argc, argv);
  MainWindow window(server_addr, local_mode);
  window.show();

  return app.exec();
}

#include "radar_qt.moc"
