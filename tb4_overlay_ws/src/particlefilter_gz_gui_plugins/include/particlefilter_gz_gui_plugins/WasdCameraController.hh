#pragma once

#include <QEvent>
#include <QTimer>

#include <atomic>
#include <filesystem>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/gui/Plugin.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/gui_camera.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

namespace particlefilter
{

class WasdCameraController : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(bool active READ Active WRITE SetActive NOTIFY ActiveChanged)
  Q_PROPERTY(QString statusText READ StatusText NOTIFY StatusTextChanged)
  Q_PROPERTY(double linearSpeed READ LinearSpeed WRITE SetLinearSpeed NOTIFY LinearSpeedChanged)
  Q_PROPERTY(double verticalSpeed READ VerticalSpeed WRITE SetVerticalSpeed NOTIFY VerticalSpeedChanged)
  Q_PROPERTY(double angularSpeed READ AngularSpeed WRITE SetAngularSpeed NOTIFY AngularSpeedChanged)

  public: WasdCameraController();
  public: ~WasdCameraController() override;

  public: bool Active() const;
  public: void SetActive(bool _active);

  public: QString StatusText() const;
  public: double LinearSpeed() const;
  public: void SetLinearSpeed(double _speed);
  public: double VerticalSpeed() const;
  public: void SetVerticalSpeed(double _speed);
  public: double AngularSpeed() const;
  public: void SetAngularSpeed(double _speed);

  protected: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;
  protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

  signals: void ActiveChanged();
  signals: void StatusTextChanged();
  signals: void LinearSpeedChanged();
  signals: void VerticalSpeedChanged();
  signals: void AngularSpeedChanged();

  private slots: void OnTick();

  private: struct CameraState
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
    bool valid{false};
  };

  private: void HandleKeyPress(int _key, bool _pressed);
  private: void UpdateStatus(const std::string &_status);
  private: void SetPoseFromMsg(const gz::msgs::Pose &_poseMsg);
  private: void SendCameraRequest(const CameraState &_state);
  private: bool MoveCameraSync(const CameraState &_state, unsigned int _timeoutMs, std::string &_error);
  private: void CaptureScreenshotAndWriteXmp(const CameraState &_state);
  private: bool CaptureScreenshotAndWriteXmpSync(const CameraState &_state, std::string &_error);
  private: void RunBatchCapture();
  private: static std::vector<CameraState> LoadCameraStatesFromCsv(
      const std::filesystem::path &_csvPath);
  private: static std::filesystem::path WaitForNewPng(
      const std::filesystem::path &_watchDir,
      std::filesystem::file_time_type _since,
      int _timeoutMs);
  private: static void WriteXmp(
      const std::filesystem::path &_imagePath,
      const CameraState &_state,
      const std::string &_posePrior,
      double _horizontalFovDeg);
  private: static void EulerToRotationMatrix(
      double _roll, double _pitch, double _yaw,
      double _r[3][3]);
  private: static void QuaternionToEuler(
      double _x, double _y, double _z, double _w,
      double &_roll, double &_pitch, double &_yaw);
  private: static void EulerToQuaternion(
      double _roll, double _pitch, double _yaw,
      double &_x, double &_y, double &_z, double &_w);
  private: static double WrapAngle(double _angle);

  private: mutable std::mutex mutex;
  private: CameraState cameraState;
  private: std::set<int> pressedKeys;
  private: gz::transport::Node node;
  private: QTimer tickTimer;
  private: std::atomic<bool> active{true};
  private: std::atomic<bool> requestInFlight{false};
  private: std::atomic<bool> captureInFlight{false};
  private: std::atomic<bool> batchInFlight{false};
  private: QString statusText;
  private: double linearSpeed{1.2};
  private: double verticalSpeed{1.0};
  private: double angularSpeed{0.9};
  private: double dt{1.0 / 30.0};
  private: std::filesystem::path captureDir;
  private: std::filesystem::path sourceCsv;
  private: std::string posePrior{"exact"};
  private: int screenshotTimeoutMs{5000};
  private: int settleMs{2500};
  private: double horizontalFovDeg{90.0};
};

}  // namespace particlefilter
