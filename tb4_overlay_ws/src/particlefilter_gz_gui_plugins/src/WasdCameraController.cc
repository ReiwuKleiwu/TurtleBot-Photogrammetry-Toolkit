#include "particlefilter_gz_gui_plugins/WasdCameraController.hh"

#include <cmath>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <QCoreApplication>
#include <QKeyEvent>
#include <QQmlContext>

#include <gz/gui/GuiEvents.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>

namespace particlefilter
{

namespace
{

constexpr double kViewToOptical[3][3] =
{
  {0.0, -1.0, 0.0},
  {0.0, 0.0, -1.0},
  {1.0, 0.0, 0.0}
};

}

//////////////////////////////////////////////////
WasdCameraController::WasdCameraController()
{
  this->title = "WASD Camera";
  this->tickTimer.setInterval(33);
  QObject::connect(&this->tickTimer, &QTimer::timeout, this, &WasdCameraController::OnTick);
}

//////////////////////////////////////////////////
WasdCameraController::~WasdCameraController()
{
  qApp->removeEventFilter(this);
}

//////////////////////////////////////////////////
bool WasdCameraController::Active() const
{
  return this->active.load();
}

//////////////////////////////////////////////////
void WasdCameraController::SetActive(bool _active)
{
  if (this->active.exchange(_active) == _active)
    return;
  this->UpdateStatus(_active ? "Camera control enabled" : "Camera control paused");
  emit this->ActiveChanged();
}

//////////////////////////////////////////////////
QString WasdCameraController::StatusText() const
{
  return this->statusText;
}

//////////////////////////////////////////////////
double WasdCameraController::LinearSpeed() const
{
  return this->linearSpeed;
}

//////////////////////////////////////////////////
void WasdCameraController::SetLinearSpeed(double _speed)
{
  const double clamped = std::clamp(_speed, 0.1, 20.0);
  if (std::abs(this->linearSpeed - clamped) < 1e-6)
    return;
  this->linearSpeed = clamped;
  emit this->LinearSpeedChanged();
}

//////////////////////////////////////////////////
double WasdCameraController::VerticalSpeed() const
{
  return this->verticalSpeed;
}

//////////////////////////////////////////////////
void WasdCameraController::SetVerticalSpeed(double _speed)
{
  const double clamped = std::clamp(_speed, 0.1, 20.0);
  if (std::abs(this->verticalSpeed - clamped) < 1e-6)
    return;
  this->verticalSpeed = clamped;
  emit this->VerticalSpeedChanged();
}

//////////////////////////////////////////////////
double WasdCameraController::AngularSpeed() const
{
  return this->angularSpeed;
}

//////////////////////////////////////////////////
void WasdCameraController::SetAngularSpeed(double _speed)
{
  const double clamped = std::clamp(_speed, 0.1, 6.0);
  if (std::abs(this->angularSpeed - clamped) < 1e-6)
    return;
  this->angularSpeed = clamped;
  emit this->AngularSpeedChanged();
}

//////////////////////////////////////////////////
void WasdCameraController::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  this->Context()->setContextProperty("WasdCameraController", this);
  qApp->installEventFilter(this);

  if (_pluginElem)
  {
    if (auto linearElem = _pluginElem->FirstChildElement("linear_speed"))
    {
      double value = this->linearSpeed;
      linearElem->QueryDoubleText(&value);
      this->SetLinearSpeed(value);
    }
    if (auto verticalElem = _pluginElem->FirstChildElement("vertical_speed"))
    {
      double value = this->verticalSpeed;
      verticalElem->QueryDoubleText(&value);
      this->SetVerticalSpeed(value);
    }
    if (auto angularElem = _pluginElem->FirstChildElement("angular_speed"))
    {
      double value = this->angularSpeed;
      angularElem->QueryDoubleText(&value);
      this->SetAngularSpeed(value);
    }
    double tickRate = 30.0;
    if (auto tickElem = _pluginElem->FirstChildElement("tick_rate"))
      tickElem->QueryDoubleText(&tickRate);
    if (tickRate > 1e-3)
    {
      this->dt = 1.0 / tickRate;
      this->tickTimer.setInterval(static_cast<int>(std::round(1000.0 / tickRate)));
    }

    this->cameraState.valid = true;
    if (auto elem = _pluginElem->FirstChildElement("initial_x"))
      elem->QueryDoubleText(&this->cameraState.x);
    if (auto elem = _pluginElem->FirstChildElement("initial_y"))
      elem->QueryDoubleText(&this->cameraState.y);
    if (auto elem = _pluginElem->FirstChildElement("initial_z"))
      elem->QueryDoubleText(&this->cameraState.z);
    if (auto elem = _pluginElem->FirstChildElement("initial_roll"))
      elem->QueryDoubleText(&this->cameraState.roll);
    if (auto elem = _pluginElem->FirstChildElement("initial_pitch"))
      elem->QueryDoubleText(&this->cameraState.pitch);
    if (auto elem = _pluginElem->FirstChildElement("initial_yaw"))
      elem->QueryDoubleText(&this->cameraState.yaw);
    if (auto elem = _pluginElem->FirstChildElement("capture_dir"))
    {
      if (const char *text = elem->GetText())
        this->captureDir = text;
    }
    if (auto elem = _pluginElem->FirstChildElement("source_csv"))
    {
      if (const char *text = elem->GetText())
        this->sourceCsv = text;
    }
    if (auto elem = _pluginElem->FirstChildElement("pose_prior"))
    {
      if (const char *text = elem->GetText())
        this->posePrior = text;
    }
    if (auto elem = _pluginElem->FirstChildElement("screenshot_timeout_ms"))
      elem->QueryIntText(&this->screenshotTimeoutMs);
    if (auto elem = _pluginElem->FirstChildElement("settle_ms"))
      elem->QueryIntText(&this->settleMs);
    if (auto elem = _pluginElem->FirstChildElement("horizontal_fov_deg"))
      elem->QueryDoubleText(&this->horizontalFovDeg);
  }

  this->node.Subscribe("/gui/camera/pose", &WasdCameraController::SetPoseFromMsg, this);
  this->tickTimer.start();
  this->UpdateStatus("Ready");
}

//////////////////////////////////////////////////
bool WasdCameraController::eventFilter(QObject *_obj, QEvent *_event)
{
  (void)_obj;
  if (!this->Active())
    return false;

  if (_event->type() == gz::gui::events::KeyPressOnScene::kType ||
      _event->type() == gz::gui::events::KeyReleaseOnScene::kType)
  {
    const bool pressed = _event->type() == gz::gui::events::KeyPressOnScene::kType;
    gz::common::KeyEvent key;
    if (pressed)
      key = static_cast<gz::gui::events::KeyPressOnScene *>(_event)->Key();
    else
      key = static_cast<gz::gui::events::KeyReleaseOnScene *>(_event)->Key();

    this->HandleKeyPress(key.Key(), pressed);
    if (pressed)
    {
      this->UpdateStatus("Scene key: " + key.Text());
    }
    return false;
  }

  if (_event->type() == QEvent::KeyPress || _event->type() == QEvent::KeyRelease)
  {
    auto *keyEvent = static_cast<QKeyEvent *>(_event);
    if (keyEvent->isAutoRepeat())
      return false;

    const bool pressed = _event->type() == QEvent::KeyPress;
    this->HandleKeyPress(keyEvent->key(), pressed);
    if (pressed)
      this->UpdateStatus("Qt key event received");
  }

  return false;
}

//////////////////////////////////////////////////
void WasdCameraController::HandleKeyPress(int _key, bool _pressed)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (_pressed)
    this->pressedKeys.insert(_key);
  else
    this->pressedKeys.erase(_key);

  if (_pressed && _key == Qt::Key_R)
  {
    this->cameraState.roll = 0.0;
    this->cameraState.pitch = 0.0;
    if (this->cameraState.valid && !this->requestInFlight.exchange(true))
      this->SendCameraRequest(this->cameraState);
  }
}

//////////////////////////////////////////////////
void WasdCameraController::OnTick()
{
  if (!this->Active())
    return;

  CameraState next;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->cameraState.valid || this->pressedKeys.empty())
      return;

    next = this->cameraState;
    const double cosYaw = std::cos(next.yaw);
    const double sinYaw = std::sin(next.yaw);

    if (this->pressedKeys.count(Qt::Key_W))
    {
      next.x += this->linearSpeed * this->dt * cosYaw;
      next.y += this->linearSpeed * this->dt * sinYaw;
    }
    if (this->pressedKeys.count(Qt::Key_S))
    {
      next.x -= this->linearSpeed * this->dt * cosYaw;
      next.y -= this->linearSpeed * this->dt * sinYaw;
    }
    if (this->pressedKeys.count(Qt::Key_A))
    {
      next.x += this->linearSpeed * this->dt * std::cos(next.yaw + M_PI_2);
      next.y += this->linearSpeed * this->dt * std::sin(next.yaw + M_PI_2);
    }
    if (this->pressedKeys.count(Qt::Key_D))
    {
      next.x += this->linearSpeed * this->dt * std::cos(next.yaw - M_PI_2);
      next.y += this->linearSpeed * this->dt * std::sin(next.yaw - M_PI_2);
    }
    if (this->pressedKeys.count(Qt::Key_Q))
      next.z += this->verticalSpeed * this->dt;
    if (this->pressedKeys.count(Qt::Key_E))
      next.z -= this->verticalSpeed * this->dt;
    if (this->pressedKeys.count(Qt::Key_J))
      next.yaw = WrapAngle(next.yaw + this->angularSpeed * this->dt);
    if (this->pressedKeys.count(Qt::Key_L))
      next.yaw = WrapAngle(next.yaw - this->angularSpeed * this->dt);
    if (this->pressedKeys.count(Qt::Key_I))
      next.pitch = std::clamp(next.pitch + this->angularSpeed * this->dt, -1.45, 1.45);
    if (this->pressedKeys.count(Qt::Key_K))
      next.pitch = std::clamp(next.pitch - this->angularSpeed * this->dt, -1.45, 1.45);
    if (this->pressedKeys.count(Qt::Key_U))
      next.roll = WrapAngle(next.roll + this->angularSpeed * this->dt);
    if (this->pressedKeys.count(Qt::Key_O))
      next.roll = WrapAngle(next.roll - this->angularSpeed * this->dt);

    this->cameraState = next;
  }

  if (!this->requestInFlight.exchange(true))
    this->SendCameraRequest(next);
}

//////////////////////////////////////////////////
void WasdCameraController::UpdateStatus(const std::string &_status)
{
  const QString next = QString::fromStdString(_status);
  if (this->statusText == next)
    return;
  this->statusText = next;
  emit this->StatusTextChanged();
}

//////////////////////////////////////////////////
void WasdCameraController::SetPoseFromMsg(const gz::msgs::Pose &_poseMsg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->cameraState.x = _poseMsg.position().x();
  this->cameraState.y = _poseMsg.position().y();
  this->cameraState.z = _poseMsg.position().z();
  QuaternionToEuler(
      _poseMsg.orientation().x(),
      _poseMsg.orientation().y(),
      _poseMsg.orientation().z(),
      _poseMsg.orientation().w(),
      this->cameraState.roll,
      this->cameraState.pitch,
      this->cameraState.yaw);
  this->cameraState.valid = true;
  QMetaObject::invokeMethod(this, [this]()
  {
    this->UpdateStatus("Active: W/S A/D Q/E J/L I/K U/O, R reset");
  }, Qt::QueuedConnection);
}

//////////////////////////////////////////////////
void WasdCameraController::SendCameraRequest(const CameraState &_state)
{
  std::thread([this, _state]()
  {
    std::string error;
    const bool ok = this->MoveCameraSync(_state, 1000u, error);

    this->requestInFlight.store(false);

    if (!ok)
    {
      QMetaObject::invokeMethod(this, [this, error]()
      {
        this->UpdateStatus(error);
      }, Qt::QueuedConnection);
      return;
    }

    QMetaObject::invokeMethod(this, [this]()
    {
      this->UpdateStatus("Camera moved");
    }, Qt::QueuedConnection);
  }).detach();
}

//////////////////////////////////////////////////
bool WasdCameraController::MoveCameraSync(
    const CameraState &_state, unsigned int _timeoutMs, std::string &_error)
{
  gz::msgs::GUICamera req;
  req.mutable_pose()->mutable_position()->set_x(_state.x);
  req.mutable_pose()->mutable_position()->set_y(_state.y);
  req.mutable_pose()->mutable_position()->set_z(_state.z);

  double qx, qy, qz, qw;
  EulerToQuaternion(_state.roll, _state.pitch, _state.yaw, qx, qy, qz, qw);
  req.mutable_pose()->mutable_orientation()->set_x(qx);
  req.mutable_pose()->mutable_orientation()->set_y(qy);
  req.mutable_pose()->mutable_orientation()->set_z(qz);
  req.mutable_pose()->mutable_orientation()->set_w(qw);

  gz::msgs::Boolean rep;
  bool result = false;
  const bool executed = this->node.Request(
      "/gui/move_to/pose", req, _timeoutMs, rep, result);

  if (!executed)
  {
    _error = "Request timeout: /gui/move_to/pose";
    return false;
  }

  if (!result || !rep.data())
  {
    _error =
        std::string("Move rejected: result=") + (result ? "true" : "false") +
        " rep=" + (rep.data() ? "true" : "false");
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void WasdCameraController::CaptureScreenshotAndWriteXmp(const CameraState &_state)
{
  std::thread([this, _state]()
  {
    std::string error;
    try
    {
      if (!this->CaptureScreenshotAndWriteXmpSync(_state, error))
      {
        this->captureInFlight.store(false);
        QMetaObject::invokeMethod(this, [this, error]()
        {
          this->UpdateStatus(error);
        }, Qt::QueuedConnection);
        return;
      }
      this->captureInFlight.store(false);
      QMetaObject::invokeMethod(this, [this]()
      {
        this->UpdateStatus("Captured");
      }, Qt::QueuedConnection);
    }
    catch (const std::exception &_err)
    {
      this->captureInFlight.store(false);
      QMetaObject::invokeMethod(this, [this, msg = std::string(_err.what())]()
      {
        this->UpdateStatus("Capture failed: " + msg);
      }, Qt::QueuedConnection);
    }
  }).detach();
}

//////////////////////////////////////////////////
bool WasdCameraController::CaptureScreenshotAndWriteXmpSync(
    const CameraState &_state, std::string &_error)
{
  namespace fs = std::filesystem;
  const fs::path watchDir = this->captureDir;
  std::error_code ec;
  fs::create_directories(watchDir, ec);
  const auto since = fs::file_time_type::clock::now();

  gz::msgs::StringMsg req;
  req.set_data("");
  gz::msgs::Boolean rep;
  bool result = false;
  const bool executed = this->node.Request(
      "/gui/screenshot", req, 4000u, rep, result);

  if (!executed)
  {
    _error = "Capture timeout: /gui/screenshot";
    return false;
  }

  if (!result || !rep.data())
  {
    _error =
        std::string("Capture rejected: result=") + (result ? "true" : "false") +
        " rep=" + (rep.data() ? "true" : "false");
    return false;
  }

  const fs::path imagePath = WaitForNewPng(watchDir, since, this->screenshotTimeoutMs);
  WriteXmp(imagePath, _state, this->posePrior, this->horizontalFovDeg);
  _error = imagePath.filename().string();
  return true;
}

//////////////////////////////////////////////////
void WasdCameraController::RunBatchCapture()
{
  try
  {
    const auto states = LoadCameraStatesFromCsv(this->sourceCsv);
    for (std::size_t i = 0; i < states.size(); ++i)
    {
      QMetaObject::invokeMethod(this, [this, i, total = states.size()]()
      {
        this->UpdateStatus(
            "Batch move " + std::to_string(i + 1) + "/" + std::to_string(total));
      }, Qt::QueuedConnection);

      std::string error;
      if (!this->MoveCameraSync(states[i], 1000u, error))
        throw std::runtime_error(error);

      std::this_thread::sleep_for(std::chrono::milliseconds(this->settleMs));

      CameraState snapshot;
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        snapshot = this->cameraState.valid ? this->cameraState : states[i];
      }

      if (!this->CaptureScreenshotAndWriteXmpSync(snapshot, error))
        throw std::runtime_error(error);

      QMetaObject::invokeMethod(this, [this, i, total = states.size()]()
      {
        this->UpdateStatus(
            "Batch captured " + std::to_string(i + 1) + "/" + std::to_string(total));
      }, Qt::QueuedConnection);
    }

    this->batchInFlight.store(false);
    QMetaObject::invokeMethod(this, [this]()
    {
      this->UpdateStatus("Batch capture complete");
    }, Qt::QueuedConnection);
  }
  catch (const std::exception &_err)
  {
    this->batchInFlight.store(false);
    QMetaObject::invokeMethod(this, [this, msg = std::string(_err.what())]()
    {
      this->UpdateStatus("Batch capture failed: " + msg);
    }, Qt::QueuedConnection);
  }
}

//////////////////////////////////////////////////
std::vector<WasdCameraController::CameraState> WasdCameraController::LoadCameraStatesFromCsv(
    const std::filesystem::path &_csvPath)
{
  std::ifstream in(_csvPath);
  if (!in)
    throw std::runtime_error("could not open source_csv");

  std::vector<CameraState> states;
  std::string line;
  bool header = true;
  while (std::getline(in, line))
  {
    if (header)
    {
      header = false;
      continue;
    }
    if (line.empty())
      continue;

    std::vector<std::string> cols;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ','))
      cols.push_back(item);
    if (cols.size() < 8)
      continue;

    CameraState state;
    state.x = std::stod(cols[1]);
    state.y = std::stod(cols[2]);
    state.z = std::stod(cols[3]);
    QuaternionToEuler(
        std::stod(cols[4]),
        std::stod(cols[5]),
        std::stod(cols[6]),
        std::stod(cols[7]),
        state.roll, state.pitch, state.yaw);
    state.valid = true;
    states.push_back(state);
  }

  if (states.empty())
    throw std::runtime_error("source_csv had no valid pose rows");
  return states;
}

//////////////////////////////////////////////////
std::filesystem::path WasdCameraController::WaitForNewPng(
    const std::filesystem::path &_watchDir,
    std::filesystem::file_time_type _since,
    int _timeoutMs)
{
  namespace fs = std::filesystem;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(_timeoutMs);
  while (std::chrono::steady_clock::now() < deadline)
  {
    fs::path newest;
    fs::file_time_type newestTime{};
    bool found = false;
    std::error_code ec;
    if (fs::exists(_watchDir, ec))
    {
      for (fs::recursive_directory_iterator it(_watchDir, ec), end; it != end; it.increment(ec))
      {
        if (ec)
          break;
        if (!it->is_regular_file(ec))
          continue;
        const fs::path candidate = it->path();
        if (candidate.extension() != ".png")
          continue;
        const auto mtime = it->last_write_time(ec);
        if (ec || mtime < _since)
          continue;
        if (!found || mtime > newestTime)
        {
          newest = candidate;
          newestTime = mtime;
          found = true;
        }
      }
    }
    if (found)
      return newest;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  throw std::runtime_error("no new screenshot appeared");
}

//////////////////////////////////////////////////
void WasdCameraController::WriteXmp(
    const std::filesystem::path &_imagePath,
    const CameraState &_state,
    const std::string &_posePrior,
    double _horizontalFovDeg)
{
  double rotC2w[3][3];
  EulerToRotationMatrix(_state.roll, _state.pitch, _state.yaw, rotC2w);

  double rotW2c[3][3];
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
      rotW2c[row][col] = rotC2w[col][row];
  }

  double rotRc[3][3];
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      rotRc[row][col] = 0.0;
      for (int k = 0; k < 3; ++k)
        rotRc[row][col] += kViewToOptical[row][k] * rotW2c[k][col];
    }
  }

  std::ostringstream rotStream;
  rotStream << std::setprecision(15);
  bool first = true;
  for (const auto &row : rotRc)
  {
    for (double value : row)
    {
      if (!first)
        rotStream << ' ';
      rotStream << value;
      first = false;
    }
  }

  std::ostringstream posStream;
  posStream << std::setprecision(15)
            << _state.x << ' ' << _state.y << ' ' << _state.z;

  const auto xmpPath = _imagePath.parent_path() / (_imagePath.stem().string() + ".xmp");
  std::ofstream out(xmpPath);
  out << "<?xpacket begin=\"\" id=\"W5M0MpCehiHzreSzNTczkc9d\"?>\n"
      << "<x:xmpmeta xmlns:x=\"adobe:ns:meta/\">\n"
      << "  <rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">\n"
      << "    <rdf:Description rdf:about=\"" << _imagePath.filename().string() << "\"\n"
      << "      xmlns:xcr=\"http://www.capturingreality.com/ns/xcr/1.1#\"\n"
      << "      xcr:Version=\"3\"\n"
      << "      xcr:Coordinates=\"absolute\"\n"
      << "      xcr:PosePrior=\"" << _posePrior << "\"\n"
      << "      xcr:CalibrationPrior=\"initial\"\n"
      << "      xcr:CalibrationGroup=\"-1\"\n"
      << "      xcr:DistortionGroup=\"-1\"\n"
      << "      xcr:DistortionModel=\"division\"\n"
      << "      xcr:DistortionCoeficients=\"0 0 0 0 0 0\"\n"
      << "      xcr:FocalLength35mm=\""
      << (36.0 / (2.0 * std::tan(_horizontalFovDeg * M_PI / 360.0))) << "\"\n"
      << "      xcr:Skew=\"0\"\n"
      << "      xcr:AspectRatio=\"1\"\n"
      << "      xcr:PrincipalPointU=\"0\"\n"
      << "      xcr:PrincipalPointV=\"0\"\n"
      << "      xcr:Rotation=\"" << rotStream.str() << "\">\n"
      << "      <xcr:Position>" << posStream.str() << "</xcr:Position>\n"
      << "    </rdf:Description>\n"
      << "  </rdf:RDF>\n"
      << "</x:xmpmeta>\n"
      << "<?xpacket end=\"w\"?>\n";
}

//////////////////////////////////////////////////
void WasdCameraController::EulerToRotationMatrix(
    double _roll, double _pitch, double _yaw,
    double _r[3][3])
{
  const double cr = std::cos(_roll);
  const double sr = std::sin(_roll);
  const double cp = std::cos(_pitch);
  const double sp = std::sin(_pitch);
  const double cy = std::cos(_yaw);
  const double sy = std::sin(_yaw);

  _r[0][0] = cy * cp;
  _r[0][1] = cy * sp * sr - sy * cr;
  _r[0][2] = cy * sp * cr + sy * sr;
  _r[1][0] = sy * cp;
  _r[1][1] = sy * sp * sr + cy * cr;
  _r[1][2] = sy * sp * cr - cy * sr;
  _r[2][0] = -sp;
  _r[2][1] = cp * sr;
  _r[2][2] = cp * cr;
}

//////////////////////////////////////////////////
void WasdCameraController::QuaternionToEuler(
    double _x, double _y, double _z, double _w,
    double &_roll, double &_pitch, double &_yaw)
{
  const double sinrCosp = 2.0 * (_w * _x + _y * _z);
  const double cosrCosp = 1.0 - 2.0 * (_x * _x + _y * _y);
  _roll = std::atan2(sinrCosp, cosrCosp);

  const double sinp = 2.0 * (_w * _y - _z * _x);
  if (std::abs(sinp) >= 1.0)
    _pitch = std::copysign(M_PI / 2.0, sinp);
  else
    _pitch = std::asin(sinp);

  const double sinyCosp = 2.0 * (_w * _z + _x * _y);
  const double cosyCosp = 1.0 - 2.0 * (_y * _y + _z * _z);
  _yaw = std::atan2(sinyCosp, cosyCosp);
}

//////////////////////////////////////////////////
void WasdCameraController::EulerToQuaternion(
    double _roll, double _pitch, double _yaw,
    double &_x, double &_y, double &_z, double &_w)
{
  const double cr = std::cos(_roll * 0.5);
  const double sr = std::sin(_roll * 0.5);
  const double cp = std::cos(_pitch * 0.5);
  const double sp = std::sin(_pitch * 0.5);
  const double cy = std::cos(_yaw * 0.5);
  const double sy = std::sin(_yaw * 0.5);
  _w = cr * cp * cy + sr * sp * sy;
  _x = sr * cp * cy - cr * sp * sy;
  _y = cr * sp * cy + sr * cp * sy;
  _z = cr * cp * sy - sr * sp * cy;
}

//////////////////////////////////////////////////
double WasdCameraController::WrapAngle(double _angle)
{
  return std::atan2(std::sin(_angle), std::cos(_angle));
}

}  // namespace particlefilter

GZ_ADD_PLUGIN(particlefilter::WasdCameraController, gz::gui::Plugin)
GZ_ADD_PLUGIN_ALIAS(particlefilter::WasdCameraController, "WasdCameraController")
