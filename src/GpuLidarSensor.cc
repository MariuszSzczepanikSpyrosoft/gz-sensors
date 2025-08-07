/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#if defined(_MSC_VER)
  #pragma warning(push)
  #pragma warning(disable: 4005)
  #pragma warning(disable: 4251)
#endif
#include <gz/msgs/pointcloud_packed.pb.h>
#if defined(_MSC_VER)
  #pragma warning(pop)
#endif

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include "gz/sensors/GpuLidarSensor.hh"
#include "gz/sensors/SensorFactory.hh"

// Pattern scanning includes
#include <fstream>
#include <sstream>
#include <map>
#include <chrono>
#include <algorithm>
#include <cmath>

using namespace gz::sensors;

/// \brief Private data for the GpuLidar class
class gz::sensors::GpuLidarSensorPrivate
{
  /// \brief Fill the point cloud packed message
  /// \param[in] _laserBuffer Lidar data buffer.
  public: void FillPointCloudMsg(const float *_laserBuffer);

  /// \brief Fill point cloud message using pattern scanning
  /// \param[in] _laserBuffer Lidar data buffer.
  public: void FillPointCloudMsgWithPattern(const float *_laserBuffer);

  /// \brief Rendering camera
  public: gz::rendering::GpuRaysPtr gpuRays;

  /// \brief Connection to the Manager's scene change event.
  public: gz::common::ConnectionPtr sceneChangeConnection;

  /// \brief Event that is used to trigger callbacks when a new
  /// lidar frame is available
  public: gz::common::EventT<
          void(const float *_scan, unsigned int _width,
               unsigned int _height, unsigned int _channels,
               const std::string &_format)> lidarEvent;

  /// \brief Connection to gpuRays new lidar frame event
  public: gz::common::ConnectionPtr lidarFrameConnection;

  /// \brief The point cloud message.
  public: gz::msgs::PointCloudPacked pointMsg;

  /// \brief Transport node.
  public: gz::transport::Node node;

  /// \brief Publisher for the publish point cloud message.
  public: gz::transport::Node::Publisher pointPub;

  // Pattern scanning data
  /// \brief Multi-frame scanning pattern loaded from CSV
  public: std::vector<std::vector<GpuLidarSensor::ScanPoint>> multiFramePattern;
  
  /// \brief Current frame index for pattern scanning
  public: size_t currentPatternFrame = 0;
  
  /// \brief Path to the pattern file
  public: std::string patternFilePath;
  
  /// \brief Flag indicating if pattern scanning is enabled
  public: bool patternScanningEnabled = false;
  
  /// \brief Timestamp of last pattern update
  public: std::chrono::steady_clock::duration lastPatternUpdate{0};
  
  /// \brief Pattern update rate (Hz) - should match your 10Hz requirement
  public: double patternUpdateRate = 10.0;

  /// \brief Reference to parent sensor for accessing protected methods
  public: GpuLidarSensor* parentSensor = nullptr;
};

//////////////////////////////////////////////////
GpuLidarSensor::GpuLidarSensor()
  : dataPtr(new GpuLidarSensorPrivate())
{
  this->dataPtr->parentSensor = this;
}

//////////////////////////////////////////////////
GpuLidarSensor::~GpuLidarSensor()
{
  this->RemoveGpuRays(this->Scene());

  this->dataPtr->sceneChangeConnection.reset();

  if (this->laserBuffer)
  {
    delete [] this->laserBuffer;
    this->laserBuffer = nullptr;
  }
}

/////////////////////////////////////////////////
void GpuLidarSensor::SetScene(gz::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->lidarMutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    this->RemoveGpuRays(this->Scene());
    RenderingSensor::SetScene(_scene);

    if (this->initialized)
      this->CreateLidar();
  }
}

//////////////////////////////////////////////////
void GpuLidarSensor::RemoveGpuRays(
    gz::rendering::ScenePtr _scene)
{
  if (_scene)
  {
    _scene->DestroySensor(this->dataPtr->gpuRays);
  }
  this->dataPtr->gpuRays.reset();
  this->dataPtr->gpuRays = nullptr;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Load(const sdf::Sensor &_sdf)
{
  // Check if this is being loaded via "builtin" or via another sensor
  if (!Lidar::Load(_sdf))
  {
    return false;
  }

  // Pattern scanning configuration - check for pattern_file_path
  bool patternFound = false;
  
  // Approach 1: Check main sensor element (preferred)
  sdf::ElementPtr sensorElement = _sdf.Element();
  if (sensorElement)
  {
    gzdbg << "[GpuLidarSensor] Checking main sensor element for pattern_file_path..." << std::endl;
    
    if (sensorElement->HasElement("pattern_file_path"))
    {
      this->dataPtr->patternFilePath = sensorElement->Get<std::string>("pattern_file_path");
      patternFound = true;
      gzdbg << "[GpuLidarSensor] Found pattern_file_path in main sensor element: " 
            << this->dataPtr->patternFilePath << std::endl;
    }
  }
  
  // Approach 2: Check inside lidar element (fallback)
  if (!patternFound)
  {
    auto lidarSdf = _sdf.LidarSensor();
    if (lidarSdf)
    {
      auto lidarElement = lidarSdf->Element();
      if (lidarElement)
      {
        gzdbg << "[GpuLidarSensor] Checking lidar element for pattern_file_path..." << std::endl;
        
        if (lidarElement->HasElement("pattern_file_path"))
        {
          this->dataPtr->patternFilePath = lidarElement->Get<std::string>("pattern_file_path");
          patternFound = true;
          gzdbg << "[GpuLidarSensor] Found pattern_file_path in lidar element: " 
                << this->dataPtr->patternFilePath << std::endl;
        }
      }
    }
  }
  
  // Check for pattern_update_rate if pattern was found
  if (patternFound && sensorElement && sensorElement->HasElement("pattern_update_rate"))
  {
    this->dataPtr->patternUpdateRate = sensorElement->Get<double>("pattern_update_rate");
    gzdbg << "[GpuLidarSensor] Pattern update rate set to: " 
          << this->dataPtr->patternUpdateRate << " Hz" << std::endl;
  }
  
  // If pattern was found, try to load it
  if (patternFound)
  {
    gzmsg << "[GpuLidarSensor] ==> PATTERN SCANNING MODE DETECTED <==" << std::endl;
    gzmsg << "[GpuLidarSensor] Attempting to load scanning pattern from: " 
          << this->dataPtr->patternFilePath << std::endl;
    
    if (this->LoadScanningPattern(this->dataPtr->patternFilePath))
    {
      this->dataPtr->patternScanningEnabled = true;
      gzmsg << "[GpuLidarSensor] ✓ PATTERN SCANNING ENABLED!" << std::endl;
      gzmsg << "[GpuLidarSensor] ✓ Loaded " 
            << this->dataPtr->multiFramePattern.size() << " frames" << std::endl;
      gzmsg << "[GpuLidarSensor] ✓ Pattern update rate: " 
            << this->dataPtr->patternUpdateRate << " Hz" << std::endl;
    }
    else
    {
      gzerr << "[GpuLidarSensor] ✗ Failed to load pattern file!" << std::endl;
      gzerr << "[GpuLidarSensor] ✗ Falling back to standard scanning mode" << std::endl;
      this->dataPtr->patternScanningEnabled = false;
    }
  }
  else
  {
    gzdbg << "[GpuLidarSensor] No pattern_file_path found - using standard scanning mode" << std::endl;
    this->dataPtr->patternScanningEnabled = false;
  }

  // Initialize the point message.
  // \todo(anyone) The true value in the following function call forces
  // the xyz and rgb fields to be aligned to memory boundaries. This is need
  // by ROS1: https://github.com/ros/common_msgs/pull/77. Ideally, memory
  // alignment should be configured. This same problem is in the
  // RgbdCameraSensor.
  gz::msgs::InitPointCloudPacked(this->dataPtr->pointMsg, this->FrameId(), true,
      {{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32},
      {"intensity", gz::msgs::PointCloudPacked::Field::FLOAT32},
      {"ring", gz::msgs::PointCloudPacked::Field::UINT16}});

  if (this->Scene())
    this->CreateLidar();

  this->dataPtr->sceneChangeConnection =
    RenderingEvents::ConnectSceneChangeCallback(
        std::bind(&GpuLidarSensor::SetScene, this, std::placeholders::_1));

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

  gzdbg << "Lidar points for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  this->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool GpuLidarSensor::CreateLidar()
{
  this->dataPtr->gpuRays = this->Scene()->CreateGpuRays(
      this->Name());

  if (!this->dataPtr->gpuRays)
  {
    gzerr << "Unable to create gpu laser sensor\n";
    return false;
  }

  this->dataPtr->gpuRays->SetNearClipPlane(this->RangeMin());
  this->dataPtr->gpuRays->SetFarClipPlane(this->RangeMax());

  // Mask ranges outside of min/max to +/- inf, as per REP 117
  this->dataPtr->gpuRays->SetClamp(false);

  this->dataPtr->gpuRays->SetAngleMin(this->AngleMin().Radian());
  this->dataPtr->gpuRays->SetAngleMax(this->AngleMax().Radian());

  this->dataPtr->gpuRays->SetVerticalAngleMin(
      this->VerticalAngleMin().Radian());
  this->dataPtr->gpuRays->SetVerticalAngleMax(
      this->VerticalAngleMax().Radian());

  // For pattern scanning, we might want higher resolution to capture all pattern points
  if (this->dataPtr->patternScanningEnabled)
  {
    // Use higher resolution for better pattern coverage
    // You might want to adjust these values based on your pattern density
    unsigned int patternRayCount = std::max(this->RayCount(), static_cast<unsigned int>(720));
    unsigned int patternVerticalRayCount = std::max(this->VerticalRayCount(), static_cast<unsigned int>(180));
    
    this->dataPtr->gpuRays->SetRayCount(patternRayCount);
    this->dataPtr->gpuRays->SetVerticalRayCount(patternVerticalRayCount);
    
    gzdbg << "[GpuLidarSensor] Pattern mode - using enhanced resolution: " 
          << patternRayCount << "x" << patternVerticalRayCount << std::endl;
  }
  else
  {
    this->dataPtr->gpuRays->SetRayCount(this->RayCount());
    this->dataPtr->gpuRays->SetVerticalRayCount(this->VerticalRayCount());
  }

  this->dataPtr->gpuRays->SetLocalPose(this->Pose());

  this->Scene()->RootVisual()->AddChild(
      this->dataPtr->gpuRays);

  // Set the values on the point message.
  this->dataPtr->pointMsg.set_width(this->dataPtr->gpuRays->RangeCount());
  this->dataPtr->pointMsg.set_height(
      this->dataPtr->gpuRays->VerticalRangeCount());
  this->dataPtr->pointMsg.set_row_step(
      this->dataPtr->pointMsg.point_step() *
      this->dataPtr->pointMsg.width());
  this->dataPtr->gpuRays->SetVisibilityMask(this->VisibilityMask());

  this->dataPtr->lidarFrameConnection =
      this->dataPtr->gpuRays->ConnectNewGpuRaysFrame(
      std::bind(&GpuLidarSensor::OnNewLidarFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  this->AddSensor(this->dataPtr->gpuRays);

  return true;
}

/////////////////////////////////////////////////
void GpuLidarSensor::OnNewLidarFrame(const float *_scan,
    unsigned int _width, unsigned int _height, unsigned int _channels,
    const std::string &_format)
{
  GZ_PROFILE("GpuLidarSensor::OnNewLidarFrame");
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
bool GpuLidarSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("GpuLidarSensor::Update");
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

  // Handle pattern-based scanning
  if (this->dataPtr->patternScanningEnabled && !this->dataPtr->multiFramePattern.empty())
  {
    // Check if it's time to advance to next frame
    auto timeSinceLastUpdate = _now - this->dataPtr->lastPatternUpdate;
    auto updateInterval = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / this->dataPtr->patternUpdateRate));
    
    if (timeSinceLastUpdate >= updateInterval)
    {
      // Advance to next pattern frame
      this->dataPtr->currentPatternFrame = 
          (this->dataPtr->currentPatternFrame + 1) % this->dataPtr->multiFramePattern.size();
      
      this->dataPtr->lastPatternUpdate = _now;
      
      // Log every 100 frames to verify it's working
      if (this->dataPtr->currentPatternFrame % 100 == 0)
      {
        gzdbg << "[GpuLidarSensor] Advanced to pattern frame " 
              << this->dataPtr->currentPatternFrame 
              << " (total: " << this->dataPtr->multiFramePattern.size() << ")" << std::endl;
      }
    }
  }

  this->Render();

  // Apply noise before publishing the data.
  this->ApplyNoise();

  this->PublishLidarScan(_now);

  if (this->dataPtr->pointPub.HasConnections())
  {
    // Set the time stamp
    *this->dataPtr->pointMsg.mutable_header()->mutable_stamp() =
      gz::msgs::Convert(_now);
    // Set frame_id
    for (auto i = 0;
         i < this->dataPtr->pointMsg.mutable_header()->data_size();
         ++i)
    {
      if (this->dataPtr->pointMsg.mutable_header()->data(i).key() == "frame_id"
          && this->dataPtr->pointMsg.mutable_header()->data(i).value_size() > 0)
      {
        this->dataPtr->pointMsg.mutable_header()->mutable_data(i)->set_value(
              0,
              this->FrameId());
      }
    }

    // Use pattern-based or standard point cloud generation
    if (this->dataPtr->patternScanningEnabled)
    {
      this->dataPtr->FillPointCloudMsgWithPattern(this->laserBuffer);
    }
    else
    {
      this->dataPtr->FillPointCloudMsg(this->laserBuffer);
    }

    {
      this->AddSequence(this->dataPtr->pointMsg.mutable_header());
      GZ_PROFILE("GpuLidarSensor::Update Publish point cloud");
      this->dataPtr->pointPub.Publish(this->dataPtr->pointMsg);
    }
  }
  return true;
}

/////////////////////////////////////////////////
gz::common::ConnectionPtr GpuLidarSensor::ConnectNewLidarFrame(
          std::function<void(const float *_scan, unsigned int _width,
                  unsigned int _height, unsigned int _channels,
                  const std::string &/*_format*/)> _subscriber)
{
  return this->dataPtr->lidarEvent.Connect(_subscriber);
}

/////////////////////////////////////////////////
gz::rendering::GpuRaysPtr GpuLidarSensor::GpuRays() const
{
  return this->dataPtr->gpuRays;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::IsHorizontal() const
{
  return this->dataPtr->gpuRays->IsHorizontal();
}

//////////////////////////////////////////////////
gz::math::Angle GpuLidarSensor::HFOV() const
{
  return this->dataPtr->gpuRays->HFOV();
}

//////////////////////////////////////////////////
gz::math::Angle GpuLidarSensor::VFOV() const
{
  return this->dataPtr->gpuRays->VFOV();
}

//////////////////////////////////////////////////
bool GpuLidarSensor::HasConnections() const
{
  return Lidar::HasConnections() ||
     (this->dataPtr->pointPub && this->dataPtr->pointPub.HasConnections()) ||
     this->dataPtr->lidarEvent.ConnectionCount() > 0u;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::IsPatternScanningEnabled() const
{
  return this->dataPtr->patternScanningEnabled;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::LoadScanningPattern(const std::string &_patternFilePath)
{
  gzmsg << "[GpuLidarSensor] ======================================" << std::endl;
  gzmsg << "[GpuLidarSensor] LOADING SCANNING PATTERN" << std::endl;
  gzmsg << "[GpuLidarSensor] ======================================" << std::endl;
  gzmsg << "[GpuLidarSensor] File path: " << _patternFilePath << std::endl;
  
  std::ifstream file(_patternFilePath);
  if (!file.is_open()) 
  {
    gzerr << "[GpuLidarSensor] ✗ Could not open pattern file: " << _patternFilePath << std::endl;
    return false;
  }

  gzdbg << "[GpuLidarSensor] ✓ File opened successfully, parsing CSV..." << std::endl;

  std::string line;
  std::map<int, std::vector<ScanPoint>> frameMap;
  int lineNumber = 0;
  int successfulParses = 0;
  int failedParses = 0;
  int totalPointsLoaded = 0;

  // Read and process each line
  while (std::getline(file, line)) 
  {
    lineNumber++;
    
    // Trim whitespace
    line.erase(0, line.find_first_not_of(" \t\r\n"));
    line.erase(line.find_last_not_of(" \t\r\n") + 1);
    
    // Skip empty lines and header
    if (line.empty() || line.find("frame_id") != std::string::npos) 
    {
      if (lineNumber <= 5) // Log first few skips
      {
        gzdbg << "[GpuLidarSensor] Skipping line " << lineNumber << ": " << line << std::endl;
      }
      continue;
    }

    // Parse CSV line: frame_id,theta,phi,time
    std::stringstream ss(line);
    std::string item;
    std::vector<std::string> tokens;
    
    while (std::getline(ss, item, ',')) 
    {
      // Trim each token
      item.erase(0, item.find_first_not_of(" \t"));
      item.erase(item.find_last_not_of(" \t") + 1);
      tokens.push_back(item);
    }

    if (tokens.size() != 4) 
    {
      failedParses++;
      if (failedParses <= 5) // Log first few failures
      {
        gzwarn << "[GpuLidarSensor] Line " << lineNumber 
               << " - Expected 4 columns, got " << tokens.size() 
               << " (" << line << ")" << std::endl;
      }
      continue;
    }

    try 
    {
      int frameId = std::stoi(tokens[0]);
      double theta = std::stod(tokens[1]);
      double phi = std::stod(tokens[2]);
      double time = std::stod(tokens[3]);

      // Validate angles (basic sanity check)
      if (std::abs(theta) > 2*M_PI || std::abs(phi) > M_PI)
      {
        failedParses++;
        if (failedParses <= 5)
        {
          gzwarn << "[GpuLidarSensor] Line " << lineNumber 
                 << " - Invalid angles: theta=" << theta << ", phi=" << phi << std::endl;
        }
        continue;
      }

      ScanPoint point = {theta, phi, time};
      frameMap[frameId].push_back(point);
      successfulParses++;
      totalPointsLoaded++;
      
      // Log first few successful parses
      if (successfulParses <= 5)
      {
        gzdbg << "[GpuLidarSensor] ✓ Parsed line " << lineNumber 
              << ": frame=" << frameId << ", theta=" << theta 
              << ", phi=" << phi << ", time=" << time << std::endl;
      }
      
    } 
    catch (const std::exception &e) 
    {
      failedParses++;
      if (failedParses <= 5)
      {
        gzwarn << "[GpuLidarSensor] Parse error on line " << lineNumber 
               << ": " << e.what() << " (" << line << ")" << std::endl;
      }
    }
  }

  file.close();

  if (frameMap.empty()) 
  {
    gzerr << "[GpuLidarSensor] ✗ No valid data found in pattern file" << std::endl;
    return false;
  }

  // Convert map to vector for efficient access
  this->dataPtr->multiFramePattern.clear();
  this->dataPtr->multiFramePattern.reserve(frameMap.size());
  
  // Calculate statistics
  int minPointsPerFrame = INT_MAX;
  int maxPointsPerFrame = 0;
  
  for (const auto &framePair : frameMap) 
  {
    this->dataPtr->multiFramePattern.push_back(framePair.second);
    int frameSize = framePair.second.size();
    minPointsPerFrame = std::min(minPointsPerFrame, frameSize);
    maxPointsPerFrame = std::max(maxPointsPerFrame, frameSize);
    
    // Log first few frames
    if (framePair.first < 5)
    {
      gzdbg << "[GpuLidarSensor] Frame " << framePair.first 
            << " contains " << frameSize << " points" << std::endl;
    }
  }

  // Final statistics
  gzmsg << "[GpuLidarSensor] ======================================" << std::endl;
  gzmsg << "[GpuLidarSensor] PATTERN LOADING COMPLETE!" << std::endl;
  gzmsg << "[GpuLidarSensor] ======================================" << std::endl;
  gzmsg << "[GpuLidarSensor] ✓ Total lines processed: " << lineNumber << std::endl;
  gzmsg << "[GpuLidarSensor] ✓ Successful parses: " << successfulParses << std::endl;
  gzmsg << "[GpuLidarSensor] ✓ Failed parses: " << failedParses << std::endl;
  gzmsg << "[GpuLidarSensor] ✓ Total frames loaded: " << this->dataPtr->multiFramePattern.size() << std::endl;
  gzmsg << "[GpuLidarSensor] ✓ Total points loaded: " << totalPointsLoaded << std::endl;
  gzmsg << "[GpuLidarSensor] ✓ Points per frame - Min: " << minPointsPerFrame 
        << ", Max: " << maxPointsPerFrame 
        << ", Avg: " << (totalPointsLoaded / this->dataPtr->multiFramePattern.size()) << std::endl;
  gzmsg << "[GpuLidarSensor] ✓ Pattern update rate: " << this->dataPtr->patternUpdateRate << " Hz" << std::endl;
  
  if (failedParses > 0)
  {
    gzwarn << "[GpuLidarSensor] ⚠ " << failedParses << " lines failed to parse (see details above)" << std::endl;
  }
  
  gzmsg << "[GpuLidarSensor] ======================================" << std::endl;

  return true;
}

//////////////////////////////////////////////////
const std::vector<GpuLidarSensor::ScanPoint>& GpuLidarSensor::GetCurrentFramePattern() const
{
  // Return reference to current frame for efficient access (no copying!)
  if (this->dataPtr->multiFramePattern.empty())
  {
    static const std::vector<ScanPoint> emptyPattern;
    return emptyPattern;
  }
  
  return this->dataPtr->multiFramePattern[this->dataPtr->currentPatternFrame];
}

//////////////////////////////////////////////////
bool GpuLidarSensor::PatternAnglesToIndices(double _theta, double _phi, 
                                           unsigned int &_rayIndex, 
                                           unsigned int &_verticalIndex) const
{
  if (!this->dataPtr->gpuRays)
    return false;

  // Convert pattern angles to GPU ray grid indices
  double angleMin = this->dataPtr->gpuRays->AngleMin().Radian();
  double angleMax = this->dataPtr->gpuRays->AngleMax().Radian();
  double verticalAngleMin = this->dataPtr->gpuRays->VerticalAngleMin().Radian();
  double verticalAngleMax = this->dataPtr->gpuRays->VerticalAngleMax().Radian();
  
  unsigned int rayCount = this->dataPtr->gpuRays->RangeCount();
  unsigned int verticalRayCount = this->dataPtr->gpuRays->VerticalRangeCount();

  // Check if angles are within sensor FOV
  if (_theta < angleMin || _theta > angleMax || 
      _phi < verticalAngleMin || _phi > verticalAngleMax)
  {
    return false;
  }

  // Calculate indices
  double horizontalRatio = (_theta - angleMin) / (angleMax - angleMin);
  double verticalRatio = (_phi - verticalAngleMin) / (verticalAngleMax - verticalAngleMin);
  
  _rayIndex = static_cast<unsigned int>(horizontalRatio * (rayCount - 1));
  _verticalIndex = static_cast<unsigned int>(verticalRatio * (verticalRayCount - 1));
  
  // Ensure indices are within bounds
  _rayIndex = std::min(_rayIndex, rayCount - 1);
  _verticalIndex = std::min(_verticalIndex, verticalRayCount - 1);

  return true;
}

//////////////////////////////////////////////////
void GpuLidarSensorPrivate::FillPointCloudMsg(const float *_laserBuffer)
{
  // Original standard scanning implementation
  GZ_PROFILE("GpuLidarSensorPrivate::FillPointCloudMsg");
  uint32_t width = this->pointMsg.width();
  uint32_t height = this->pointMsg.height();
  unsigned int channels = 3;

  float angleStep =
    (this->gpuRays->AngleMax() - this->gpuRays->AngleMin()).Radian() /
    (this->gpuRays->RangeCount()-1);

  float verticleAngleStep = (this->gpuRays->VerticalAngleMax() -
      this->gpuRays->VerticalAngleMin()).Radian() /
    (this->gpuRays->VerticalRangeCount()-1);

  // Angles of ray currently processing, azimuth is horizontal, inclination
  // is vertical
  float inclination = this->gpuRays->VerticalAngleMin().Radian();

  std::string *msgBuffer = this->pointMsg.mutable_data();
  msgBuffer->resize(this->pointMsg.row_step() *
      this->pointMsg.height());
  char *msgBufferIndex = msgBuffer->data();
  // Set Pointcloud as dense. Change if invalid points are found.
  bool isDense { true };
  // Iterate over scan and populate point cloud
  for (uint32_t j = 0; j < height; ++j)
  {
    float azimuth = this->gpuRays->AngleMin().Radian();

    for (uint32_t i = 0; i < width; ++i)
    {
      // Index of current point, and the depth value at that point
      auto index = j * width * channels + i * channels;
      float depth = _laserBuffer[index];
      // Validate Depth/Radius and update pointcloud density flag
      if (isDense)
        isDense = !std::isinf(depth) && !std::isnan(depth);

      // Convert spherical coordinates to Cartesian for pointcloud
      // math: x = depth * cos(inclination) * cos(azimuth)
      //       y = depth * cos(inclination) * sin(azimuth)
      //       z = depth * sin(inclination)
      float pointX = depth * cosf(inclination) * cosf(azimuth);
      float pointY = depth * cosf(inclination) * sinf(azimuth);
      float pointZ = depth * sinf(inclination);

      // Intensity and ring values
      float intensity = _laserBuffer[index + 1];
      uint16_t ring = static_cast<uint16_t>(j);

      // Pack data into buffer - directly copy floats as bytes
      memcpy(msgBufferIndex, &pointX, sizeof(pointX));
      msgBufferIndex += sizeof(pointX);
      memcpy(msgBufferIndex, &pointY, sizeof(pointY));
      msgBufferIndex += sizeof(pointY);
      memcpy(msgBufferIndex, &pointZ, sizeof(pointZ));
      msgBufferIndex += sizeof(pointZ);
      memcpy(msgBufferIndex, &intensity, sizeof(intensity));
      msgBufferIndex += sizeof(intensity);
      memcpy(msgBufferIndex, &ring, sizeof(ring));
      msgBufferIndex += sizeof(ring);

      // Advance azimuth
      azimuth += angleStep;
    }
    // Advance elevation/inclination
    inclination += verticleAngleStep;
  }
  // Update pointcloud density status
  this->pointMsg.set_is_dense(isDense);
}

//////////////////////////////////////////////////
void GpuLidarSensorPrivate::FillPointCloudMsgWithPattern(const float *_laserBuffer)
{
  // Pattern-based scanning implementation
  GZ_PROFILE("GpuLidarSensorPrivate::FillPointCloudMsgWithPattern");
  
  if (!this->parentSensor || this->multiFramePattern.empty())
  {
    // Fallback to standard scanning
    this->FillPointCloudMsg(_laserBuffer);
    return;
  }

  const auto& currentFrame = this->multiFramePattern[this->currentPatternFrame];
  
  uint32_t width = this->pointMsg.width();
  uint32_t height = this->pointMsg.height();
  unsigned int channels = 3;

  // Prepare output buffer
  std::string *msgBuffer = this->pointMsg.mutable_data();
  msgBuffer->clear();
  msgBuffer->reserve(currentFrame.size() * this->pointMsg.point_step());
  
  bool isDense = true;
  int validPoints = 0;

  // Process each point in the current pattern frame
  for (const auto& patternPoint : currentFrame)
  {
    unsigned int rayIndex, verticalIndex;
    
    // Convert pattern angles to GPU ray grid indices
    if (!this->parentSensor->PatternAnglesToIndices(patternPoint.theta, patternPoint.phi, 
                                                   rayIndex, verticalIndex))
    {
      // Point is outside sensor FOV
      continue;
    }

    // Calculate buffer index for this ray
    auto index = verticalIndex * width * channels + rayIndex * channels;
    
    // Bounds check
    if (index + 2 >= width * height * channels)
    {
      continue;
    }

    float depth = _laserBuffer[index];
    
    // Validate depth
    if (std::isinf(depth) || std::isnan(depth))
    {
      isDense = false;
      continue;
    }

    // Convert spherical coordinates to Cartesian
    float pointX = depth * cosf(patternPoint.phi) * cosf(patternPoint.theta);
    float pointY = depth * cosf(patternPoint.phi) * sinf(patternPoint.theta);
    float pointZ = depth * sinf(patternPoint.phi);

    // Get intensity and set ring
    float intensity = _laserBuffer[index + 1];
    uint16_t ring = static_cast<uint16_t>(verticalIndex);

    // Append point data to buffer
    size_t currentPos = msgBuffer->size();
    msgBuffer->resize(currentPos + this->pointMsg.point_step());
    char *msgBufferIndex = msgBuffer->data() + currentPos;

    memcpy(msgBufferIndex, &pointX, sizeof(pointX));
    msgBufferIndex += sizeof(pointX);
    memcpy(msgBufferIndex, &pointY, sizeof(pointY));
    msgBufferIndex += sizeof(pointY);
    memcpy(msgBufferIndex, &pointZ, sizeof(pointZ));
    msgBufferIndex += sizeof(pointZ);
    memcpy(msgBufferIndex, &intensity, sizeof(intensity));
    msgBufferIndex += sizeof(intensity);
    memcpy(msgBufferIndex, &ring, sizeof(ring));

    validPoints++;
  }

  // Update point cloud dimensions
  this->pointMsg.set_width(validPoints);
  this->pointMsg.set_height(1);  // Single row for pattern-based scanning
  this->pointMsg.set_row_step(this->pointMsg.point_step() * validPoints);
  this->pointMsg.set_is_dense(isDense);

  // Log occasionally for debugging
  static int frameCounter = 0;
  frameCounter++;
  if (frameCounter % 100 == 0)
  {
    gzdbg << "[GpuLidarSensor] Pattern frame " << this->currentPatternFrame 
          << ": " << validPoints << " valid points from " << currentFrame.size() 
          << " pattern points" << std::endl;
  }
}