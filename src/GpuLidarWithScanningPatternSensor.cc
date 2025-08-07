#include "gz/sensors/GpuLidarWithScanningPatternSensor.hh"

#include <gz/msgs.hh>
#include <gz/transport/Node.hh>
#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/sensors/RenderingEvents.hh>

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

using namespace gz;
using namespace sensors;

/// \brief Private data for GpuLidarWithScanningPatternSensor
class gz::sensors::GpuLidarWithScanningPatternSensorPrivate
{
  /// \brief GPU rays for rendering
public:
  gz::rendering::GpuRaysPtr gpuRays{nullptr};

  /// \brief Point cloud message for publishing
public:
  gz::msgs::PointCloudPacked pointMsg;

  /// \brief Transport node for publishing
public:
  gz::transport::Node node;

  /// \brief Point cloud publisher
public:
  gz::transport::Node::Publisher pointPub;

  /// \brief Connection for new lidar frame events
public:
  gz::common::ConnectionPtr lidarFrameConnection;

  /// \brief Connection for scene change events
public:
  gz::common::ConnectionPtr sceneChangeConnection;

  /// \brief Event for new lidar frame
public:
  gz::common::EventT<void(const float *, unsigned int,
                          unsigned int, unsigned int, const std::string &)>
      lidarEvent;

  /// \brief All scanning points organized by frame
public:
  std::vector<std::vector<ScanPoint>> scanningFrames;

  /// \brief Current frame index
public:
  int currentFrameIndex{0};

  /// \brief Last update time for 10Hz cycling
public:
  std::chrono::steady_clock::duration lastFrameUpdateTime{0};

  /// \brief Frame update interval (1/10Hz = 100ms)
public:
  std::chrono::steady_clock::duration frameUpdateInterval{
      std::chrono::milliseconds(100)};

  /// \brief Path to scanning pattern CSV file
public:
  std::string patternFilePath{"avia_multi_frame_pattern.csv"};

  /// \brief Mutex for thread safety
public:
  std::mutex dataMutex;

  /// \brief Fill point cloud message from scan data
  /// \param[in] _laserBuffer Raw laser scan data
public:
  void FillPointCloudMsg(const float *_laserBuffer);
};

//////////////////////////////////////////////////
GpuLidarWithScanningPatternSensor::GpuLidarWithScanningPatternSensor()
    : dataPtr(std::make_unique<GpuLidarWithScanningPatternSensorPrivate>())
{
}

//////////////////////////////////////////////////
GpuLidarWithScanningPatternSensor::~GpuLidarWithScanningPatternSensor() = default;

//////////////////////////////////////////////////
bool GpuLidarWithScanningPatternSensor::Load(const sdf::Sensor &_sdf)
{
  if (!Lidar::Load(_sdf))
    return false;

  // Get pattern file path from SDF
  if (_sdf.Element()->HasElement("pattern_file_path"))
  {
    this->dataPtr->patternFilePath =
        _sdf.Element()->Get<std::string>("pattern_file_path");
  }

  gzdbg << "[GpuLidarWithScanningPattern] Starting pattern loading from: "
        << this->dataPtr->patternFilePath << std::endl;

  // Load scanning pattern from CSV - this happens at initialization for performance
  if (!this->LoadScanningPatternFromCSV(this->dataPtr->patternFilePath))
  {
    gzerr << "[GpuLidarWithScanningPattern] ERROR: Failed to load scanning pattern from "
          << this->dataPtr->patternFilePath << std::endl;
    return false;
  }

  gzdbg << "[GpuLidarWithScanningPattern] Pattern loading completed successfully! "
        << "Total frames: " << this->dataPtr->scanningFrames.size() << std::endl;

  // Initialize point cloud message
  msgs::InitPointCloudPacked(this->dataPtr->pointMsg, this->FrameId(), true,
                             {{"xyz", msgs::PointCloudPacked::Field::FLOAT32},
                              {"intensity", msgs::PointCloudPacked::Field::FLOAT32},
                              {"ring", msgs::PointCloudPacked::Field::UINT16}});

  if (this->Scene())
    this->CreateLidar();

  this->dataPtr->sceneChangeConnection =
      RenderingEvents::ConnectSceneChangeCallback(
          std::bind(&GpuLidarWithScanningPatternSensor::SetScene, this,
                    std::placeholders::_1));

  // Create the point cloud publisher
  this->SetTopic(this->Topic() + "/points");

  this->dataPtr->pointPub =
      this->dataPtr->node.Advertise<gz::msgs::PointCloudPacked>(
          this->Topic());

  if (!this->dataPtr->pointPub)
  {
    gzerr << "Unable to create publisher on topic["
          << this->Topic() << "].\n";
    return false;
  }

  gzdbg << "GPU Lidar with scanning pattern for [" << this->Name()
        << "] advertised on [" << this->Topic() << "]" << std::endl;

  this->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool GpuLidarWithScanningPatternSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool GpuLidarWithScanningPatternSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool GpuLidarWithScanningPatternSensor::LoadScanningPatternFromCSV(
    const std::string &_filePath)
{
  gzdbg << "[GpuLidarWithScanningPattern] Loading pattern from: " << _filePath << std::endl;

  std::ifstream file(_filePath);
  if (!file.is_open())
  {
    gzerr << "[GpuLidarWithScanningPattern] Cannot open file: " << _filePath << std::endl;
    return false;
  }

  std::string line;
  bool headerSkipped = false;
  int totalPoints = 0;

  // Clear existing data
  this->dataPtr->scanningFrames.clear();
  std::map<int, std::vector<ScanPoint>> frameMap;

  while (std::getline(file, line))
  {
    if (!headerSkipped)
    {
      headerSkipped = true;
      continue; // Skip CSV header
    }

    if (line.empty())
      continue;

    std::stringstream ss(line);
    std::string cell;
    std::vector<std::string> tokens;

    while (std::getline(ss, cell, ','))
    {
      tokens.push_back(cell);
    }

    if (tokens.size() >= 3) // frame_index, azimuth, elevation
    {
      try
      {
        int frameIndex = std::stoi(tokens[0]);
        float azimuth = std::stof(tokens[1]) * M_PI / 180.0f;   // Convert to radians
        float elevation = std::stof(tokens[2]) * M_PI / 180.0f; // Convert to radians

        ScanPoint point;
        point.frameIndex = frameIndex;
        point.azimuth = azimuth;
        point.elevation = elevation;

        frameMap[frameIndex].push_back(point);
        totalPoints++;
      }
      catch (const std::exception &e)
      {
        gzwarn << "[GpuLidarWithScanningPattern] Error parsing line: " << line << std::endl;
        continue;
      }
    }
  }

  file.close();

  // Convert map to vector for faster access
  this->dataPtr->scanningFrames.reserve(frameMap.size());
  for (const auto &pair : frameMap)
  {
    this->dataPtr->scanningFrames.push_back(pair.second);
  }

  gzdbg << "[GpuLidarWithScanningPattern] Loaded " << totalPoints
        << " points across " << this->dataPtr->scanningFrames.size()
        << " frames" << std::endl;

  return !this->dataPtr->scanningFrames.empty();
}

//////////////////////////////////////////////////
bool GpuLidarWithScanningPatternSensor::CreateLidar()
{
  this->dataPtr->gpuRays = this->Scene()->CreateGpuRays(this->Name());

  if (!this->dataPtr->gpuRays)
  {
    gzerr << "Unable to create gpu laser sensor\n";
    return false;
  }

  // Set basic GPU rays parameters
  this->dataPtr->gpuRays->SetNearClipPlane(this->RangeMin());
  this->dataPtr->gpuRays->SetFarClipPlane(this->RangeMax());
  this->dataPtr->gpuRays->SetClamp(false);

  // We'll use dynamic ray configuration based on current frame pattern
  if (!this->dataPtr->scanningFrames.empty())
  {
    const auto &firstFrame = this->dataPtr->scanningFrames[0];

    // Find azimuth and elevation ranges for the first frame (as example)
    float minAz = std::numeric_limits<float>::max();
    float maxAz = std::numeric_limits<float>::lowest();
    float minEl = std::numeric_limits<float>::max();
    float maxEl = std::numeric_limits<float>::lowest();

    for (const auto &point : firstFrame)
    {
      minAz = std::min(minAz, point.azimuth);
      maxAz = std::max(maxAz, point.azimuth);
      minEl = std::min(minEl, point.elevation);
      maxEl = std::max(maxEl, point.elevation);
    }

    this->dataPtr->gpuRays->SetAngleMin(minAz);
    this->dataPtr->gpuRays->SetAngleMax(maxAz);
    this->dataPtr->gpuRays->SetVerticalAngleMin(minEl);
    this->dataPtr->gpuRays->SetVerticalAngleMax(maxEl);

    // Set ray counts based on frame size
    this->dataPtr->gpuRays->SetRayCount(firstFrame.size());
    this->dataPtr->gpuRays->SetVerticalRayCount(1);
  }

  this->dataPtr->gpuRays->SetLocalPose(this->Pose());
  this->dataPtr->gpuRays->SetVisibilityMask(this->VisibilityMask());

  this->Scene()->RootVisual()->AddChild(this->dataPtr->gpuRays);

  // Set point cloud message dimensions
  if (!this->dataPtr->scanningFrames.empty())
  {
    const auto &currentFrame = this->dataPtr->scanningFrames[this->dataPtr->currentFrameIndex];
    this->dataPtr->pointMsg.set_width(currentFrame.size());
    this->dataPtr->pointMsg.set_height(1);
    this->dataPtr->pointMsg.set_row_step(
        this->dataPtr->pointMsg.point_step() * this->dataPtr->pointMsg.width());
  }

  this->dataPtr->lidarFrameConnection =
      this->dataPtr->gpuRays->ConnectNewGpuRaysFrame(
          std::bind(&GpuLidarWithScanningPatternSensor::OnNewLidarFrame, this,
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                    std::placeholders::_4, std::placeholders::_5));

  this->AddSensor(this->dataPtr->gpuRays);

  return true;
}

//////////////////////////////////////////////////
void GpuLidarWithScanningPatternSensor::UpdateCurrentFrame()
{
  if (this->dataPtr->scanningFrames.empty())
    return;

  // Simple cycling through frames at 10Hz
  this->dataPtr->currentFrameIndex =
      (this->dataPtr->currentFrameIndex + 1) % this->dataPtr->scanningFrames.size();
}

//////////////////////////////////////////////////
bool GpuLidarWithScanningPatternSensor::Update(
    const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("GpuLidarWithScanningPatternSensor::Update");

  if (!this->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->gpuRays)
  {
    gzerr << "GpuRays doesn't exist.\n";
    return false;
  }

  // Check if it's time to update frame (10Hz cycling)
  if (_now - this->dataPtr->lastFrameUpdateTime >= this->dataPtr->frameUpdateInterval)
  {
    this->UpdateCurrentFrame();
    this->dataPtr->lastFrameUpdateTime = _now;
  }

  this->Render();
  this->ApplyNoise();
  this->PublishLidarScan(_now);

  if (this->dataPtr->pointPub.HasConnections())
  {
    // Set timestamp and frame_id
    *this->dataPtr->pointMsg.mutable_header()->mutable_stamp() =
        msgs::Convert(_now);

    for (auto i = 0; i < this->dataPtr->pointMsg.mutable_header()->data_size(); ++i)
    {
      if (this->dataPtr->pointMsg.mutable_header()->data(i).key() == "frame_id" && this->dataPtr->pointMsg.mutable_header()->data(i).value_size() > 0)
      {
        this->dataPtr->pointMsg.mutable_header()->mutable_data(i)->set_value(
            0, this->FrameId());
      }
    }

    this->dataPtr->FillPointCloudMsg(this->laserBuffer);

    {
      this->AddSequence(this->dataPtr->pointMsg.mutable_header());
      GZ_PROFILE("GpuLidarWithScanningPatternSensor::Update Publish point cloud");
      this->dataPtr->pointPub.Publish(this->dataPtr->pointMsg);
    }
  }

  return true;
}

//////////////////////////////////////////////////
void GpuLidarWithScanningPatternSensor::OnNewLidarFrame(const float *_scan,
                                                        unsigned int _width, unsigned int _height, unsigned int _channels,
                                                        const std::string &_format)
{
  GZ_PROFILE("GpuLidarWithScanningPatternSensor::OnNewLidarFrame");
  std::lock_guard<std::mutex> lock(this->lidarMutex);

  unsigned int samples = _width * _height * _channels;
  unsigned int lidarBufferSize = samples * sizeof(float);

  if (!this->laserBuffer)
    this->laserBuffer = new float[samples];

  memcpy(this->laserBuffer, _scan, lidarBufferSize);

  if (this->dataPtr->lidarEvent.ConnectionCount() > 0)
  {
    this->dataPtr->lidarEvent(_scan, _width, _height, _channels, _format);
  }
}

//////////////////////////////////////////////////
void GpuLidarWithScanningPatternSensorPrivate::FillPointCloudMsg(
    const float *_laserBuffer)
{
  GZ_PROFILE("GpuLidarWithScanningPatternSensorPrivate::FillPointCloudMsg");

  if (this->scanningFrames.empty())
    return;

  // Get reference to current frame's scanning pattern (avoiding copy)
  const auto &currentFrame = this->scanningFrames[this->currentFrameIndex];

  uint32_t width = currentFrame.size();
  uint32_t height = 1;
  unsigned int channels = 3;

  std::string *msgBuffer = this->pointMsg.mutable_data();
  msgBuffer->resize(this->pointMsg.row_step() * this->pointMsg.height());
  char *msgBufferIndex = msgBuffer->data();

  bool isDense = true;

  // Use pre-computed scanning pattern - no trigonometric calculations in loop!
  for (uint32_t i = 0; i < width; ++i)
  {
    // Use pointer/reference for fast access to pre-loaded data
    const ScanPoint &point = currentFrame[i];

    auto index = i * channels;
    float depth = _laserBuffer[index];

    if (isDense)
      isDense = !(gz::math::isnan(depth) || std::isinf(depth));

    float intensity = _laserBuffer[index + 1];
    uint16_t ring = 0; // Could be derived from frame index if needed

    int fieldIndex = 0;

    // Convert spherical coordinates to Cartesian using pre-computed angles
    *reinterpret_cast<float *>(msgBufferIndex +
                               this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::cos(point.elevation) * std::cos(point.azimuth);

    *reinterpret_cast<float *>(msgBufferIndex +
                               this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::cos(point.elevation) * std::sin(point.azimuth);

    *reinterpret_cast<float *>(msgBufferIndex +
                               this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::sin(point.elevation);

    // Intensity
    *reinterpret_cast<float *>(msgBufferIndex +
                               this->pointMsg.field(fieldIndex++).offset()) = intensity;

    // Ring
    *reinterpret_cast<uint16_t *>(msgBufferIndex +
                                  this->pointMsg.field(fieldIndex++).offset()) = ring;

    msgBufferIndex += this->pointMsg.point_step();
  }

  this->pointMsg.set_is_dense(isDense);
}

//////////////////////////////////////////////////
gz::common::ConnectionPtr GpuLidarWithScanningPatternSensor::ConnectNewLidarFrame(
    std::function<void(const float *, unsigned int, unsigned int, unsigned int,
                       const std::string &)>
        _subscriber)
{
  return this->dataPtr->lidarEvent.Connect(_subscriber);
}

//////////////////////////////////////////////////
gz::rendering::GpuRaysPtr GpuLidarWithScanningPatternSensor::GpuRays() const
{
  return this->dataPtr->gpuRays;
}

//////////////////////////////////////////////////
bool GpuLidarWithScanningPatternSensor::HasConnections() const
{
  return Lidar::HasConnections() ||
         (this->dataPtr->pointPub && this->dataPtr->pointPub.HasConnections()) ||
         this->dataPtr->lidarEvent.ConnectionCount() > 0u;
}

//////////////////////////////////////////////////
int GpuLidarWithScanningPatternSensor::CurrentFrameIndex() const
{
  return this->dataPtr->currentFrameIndex;
}

//////////////////////////////////////////////////
int GpuLidarWithScanningPatternSensor::TotalFrames() const
{
  return this->dataPtr->scanningFrames.size();
}