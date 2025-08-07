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

// NEW: Additional includes for pattern scanning
#include <fstream>
#include <sstream>
#include <map>
#include <chrono>

using namespace gz::sensors;

/// \brief Private data for the GpuLidar class
class gz::sensors::GpuLidarSensorPrivate
{
  /// \brief Fill the point cloud packed message
  /// \param[in] _laserBuffer Lidar data buffer.
  public: void FillPointCloudMsg(const float *_laserBuffer);

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

  // NEW: Pattern scanning data
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
};

//////////////////////////////////////////////////
GpuLidarSensor::GpuLidarSensor()
  : dataPtr(new GpuLidarSensorPrivate())
{
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
//////////////////////////////////////////////////
bool GpuLidarSensor::Load(const sdf::Sensor &_sdf)
{
  // Check if this is being loaded via "builtin" or via another sensor
  if (!Lidar::Load(_sdf))
  {
    return false;
  }

  // NEW: Check for pattern_file_path with multiple fallback approaches
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
  
  // Approach 3: Debug - list all available elements
  if (!patternFound)
  {
    gzdbg << "[GpuLidarSensor] pattern_file_path not found. Debugging available elements:" << std::endl;
    
    if (sensorElement)
    {
      gzdbg << "[GpuLidarSensor] Main sensor element exists" << std::endl;
      
      // List all child elements for debugging
      sdf::ElementPtr child = sensorElement->GetFirstElement();
      while (child)
      {
        gzdbg << "[GpuLidarSensor] Found child element: " << child->GetName() << std::endl;
        child = child->GetNextElement();
      }
    }
    else
    {
      gzdbg << "[GpuLidarSensor] Main sensor element is null" << std::endl;
    }
    
    gzdbg << "[GpuLidarSensor] No pattern_file_path found - using standard scanning mode" << std::endl;
  }
  
  // If pattern was found, try to load it
  if (patternFound)
  {
    gzdbg << "[GpuLidarSensor] Attempting to load scanning pattern from: " 
          << this->dataPtr->patternFilePath << std::endl;
    
    if (this->LoadScanningPattern(this->dataPtr->patternFilePath))
    {
      this->dataPtr->patternScanningEnabled = true;
      gzdbg << "[GpuLidarSensor] Pattern scanning enabled with " 
            << this->dataPtr->multiFramePattern.size() << " frames" << std::endl;
    }
    else
    {
      gzerr << "[GpuLidarSensor] Failed to load pattern file, falling back to standard scanning" << std::endl;
      this->dataPtr->patternScanningEnabled = false;
    }
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

  this->dataPtr->gpuRays->SetRayCount(this->RayCount());
  this->dataPtr->gpuRays->SetVerticalRayCount(
      this->VerticalRayCount());
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

  // NEW: Handle pattern-based scanning
  if (this->dataPtr->patternScanningEnabled)
  {
    gzdbg << "[GpuLidarSensor] >>> patternScanningEnabled " << std::endl;
    // Check if it's time to advance to next frame (10Hz)
    auto timeSinceLastUpdate = _now - this->dataPtr->lastPatternUpdate;
    auto updateInterval = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / this->dataPtr->patternUpdateRate));
    
    if (timeSinceLastUpdate >= updateInterval)
    {
      // Advance to next pattern frame (efficient - just increment index!)
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
    } else

    // Here you would modify the GPU rays configuration based on current pattern
    // The GetCurrentFramePattern() returns a const reference for efficiency
    const auto& currentFrame = this->GetCurrentFramePattern();
    
    // TODO: Apply the current frame pattern to GPU rays
    // This is where you'd configure the GpuRays object to use the specific
    // theta/phi angles from the current frame
    // Example (you'll need to implement the actual ray configuration):
    /*
    if (!currentFrame.empty())
    {
      // Configure GPU rays to match pattern
      // this->dataPtr->gpuRays->SetCustomAngles(currentFrame);
    }
    */
  } else {
    gzdbg << "[GpuLidarSensor] >>> not patternScanningEnabled " << std::endl;
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

    this->dataPtr->FillPointCloudMsg(this->laserBuffer);

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
void GpuLidarSensorPrivate::FillPointCloudMsg(const float *_laserBuffer)
{
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

// NEW: Pattern scanning methods
//////////////////////////////////////////////////
bool GpuLidarSensor::LoadScanningPattern(const std::string &_patternFilePath)
{
  gzdbg << "[GpuLidarSensor] Starting to load scanning pattern from: " 
        << _patternFilePath << std::endl;
  
  std::ifstream file(_patternFilePath);
  if (!file.is_open()) 
  {
    gzerr << "[GpuLidarSensor] Could not open pattern file: " << _patternFilePath << std::endl;
    return false;
  }

  gzdbg << "[GpuLidarSensor] File opened successfully, parsing CSV..." << std::endl;

  std::string line;
  std::map<int, std::vector<ScanPoint>> frameMap;
  int lineNumber = 0;
  int successfulParses = 0;
  int failedParses = 0;

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
      if (failedParses < 10) // Log first few failures
      {
        gzwarn << "[GpuLidarSensor] Line " << lineNumber 
               << " - Expected 4 columns, got " << tokens.size() << std::endl;
      }
      continue;
    }

    try 
    {
      int frameId = std::stoi(tokens[0]);
      double theta = std::stod(tokens[1]);
      double phi = std::stod(tokens[2]);
      double time = std::stod(tokens[3]);

      ScanPoint point = {theta, phi, time};
      frameMap[frameId].push_back(point);
      successfulParses++;
      
    } 
    catch (const std::exception &e) 
    {
      failedParses++;
      if (failedParses < 10)
      {
        gzwarn << "[GpuLidarSensor] Parse error on line " << lineNumber 
               << ": " << e.what() << std::endl;
      }
    }
  }

  file.close();

  if (frameMap.empty()) 
  {
    gzerr << "[GpuLidarSensor] No valid data found in pattern file" << std::endl;
    return false;
  }

  // Convert map to vector for efficient access
  this->dataPtr->multiFramePattern.clear();
  this->dataPtr->multiFramePattern.reserve(frameMap.size());
  
  for (const auto &framePair : frameMap) 
  {
    this->dataPtr->multiFramePattern.push_back(framePair.second);
  }

  gzdbg << "[GpuLidarSensor] ✓ Pattern loading COMPLETE!" << std::endl;
  gzdbg << "[GpuLidarSensor]   - Processed lines: " << lineNumber << std::endl;
  gzdbg << "[GpuLidarSensor]   - Successful parses: " << successfulParses << std::endl;
  gzdbg << "[GpuLidarSensor]   - Failed parses: " << failedParses << std::endl;
  gzdbg << "[GpuLidarSensor]   - Total frames: " << this->dataPtr->multiFramePattern.size() << std::endl;
  gzdbg << "[GpuLidarSensor]   - Average points per frame: " 
        << (this->dataPtr->multiFramePattern.empty() ? 0 : 
            successfulParses / this->dataPtr->multiFramePattern.size()) << std::endl;

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
bool GpuLidarSensor::IsPatternScanningEnabled() const
{
  return this->dataPtr->patternScanningEnabled;
}
