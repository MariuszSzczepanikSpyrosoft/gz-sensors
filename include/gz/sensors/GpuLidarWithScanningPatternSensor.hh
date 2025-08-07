#ifndef GZ_SENSORS_GPULIDARWITHSCANNINGPATTERNSENSOR_HH_
#define GZ_SENSORS_GPULIDARWITHSCANNINGPATTERNSENSOR_HH_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <mutex>

#include <sdf/sdf.hh>
#include <gz/utils/SuppressWarning.hh>

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable : 4251)
#endif
#include <gz/rendering/GpuRays.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include "gz/sensors/gpu_lidar_with_scanning_pattern/Export.hh"
#include "gz/sensors/RenderingEvents.hh"
#include "gz/sensors/Lidar.hh"

namespace gz
{
  namespace sensors
  {
    inline namespace GZ_SENSORS_VERSION_NAMESPACE
    {

      /// \brief Structure to hold a single scan point
      struct ScanPoint
      {
        float azimuth;   ///< Azimuth angle in radians
        float elevation; ///< Elevation angle in radians
        int frameIndex;  ///< Frame index for multi-frame pattern
      };

      /// \brief Forward declarations
      class GpuLidarWithScanningPatternSensorPrivate;

      /// \brief GPU Lidar Sensor with Custom Scanning Pattern
      ///
      /// This sensor extends the standard GPU lidar with support for custom
      /// scanning patterns loaded from CSV files (like Livox Avia multi-frame patterns).
      /// It provides high-performance scanning at 10Hz using pre-loaded pattern data.
      class GZ_SENSORS_GPU_LIDAR_WITH_SCANNING_PATTERN_VISIBLE GpuLidarWithScanningPatternSensor : public Lidar
      {
        /// \brief Constructor
      public:
        GpuLidarWithScanningPatternSensor();

        /// \brief Destructor
      public:
        virtual ~GpuLidarWithScanningPatternSensor();

        /// \brief Load the sensor based on data from an sdf::Sensor object.
        /// \param[in] _sdf SDF Sensor parameters.
        /// \return True on success
      public:
        virtual bool Load(const sdf::Sensor &_sdf) override;

        /// \brief Load the sensor based on data from an sdf::Element object.
        /// \param[in] _sdf SDF Sensor parameters.
        /// \return True on success
      public:
        virtual bool Load(sdf::ElementPtr _sdf) override;

        /// \brief Initialize values in the sensor
        /// \return True on success
      public:
        virtual bool Init() override;

        /// \brief Force the sensor to generate data
        /// \param[in] _now The current time
        /// \return true if the update was successful
      public:
        virtual bool Update(
            const std::chrono::steady_clock::duration &_now) override;

        /// \brief Connect a to the new lidar frame signal
        /// \param[in] _subscriber Callback for new frame data
        /// \return Connection pointer
      public:
        gz::common::ConnectionPtr ConnectNewLidarFrame(
            std::function<void(const float *_scan, unsigned int _width,
                               unsigned int _height, unsigned int _channels,
                               const std::string & /*_format*/)>
                _subscriber);

        /// \brief Get the GPU rays object
        /// \return Pointer to the GPU rays sensor
      public:
        gz::rendering::GpuRaysPtr GpuRays() const;

        /// \brief Check if sensor has connections
        /// \return True if there are subscribers
      public:
        virtual bool HasConnections() const override;

        /// \brief Get the current frame index
        /// \return Current frame index in the pattern
      public:
        int CurrentFrameIndex() const;

        /// \brief Get total number of frames in the pattern
        /// \return Total frames
      public:
        int TotalFrames() const;

        /// \brief Load scanning pattern from CSV file
        /// \param[in] _filePath Path to the CSV file
        /// \return True on success
      private:
        bool LoadScanningPatternFromCSV(const std::string &_filePath);

        /// \brief Create the GPU lidar sensor
        /// \return True on success
      private:
        bool CreateLidar();

        /// \brief Callback for new lidar frame data
        /// \param[in] _scan Raw scan data
        /// \param[in] _width Width of scan
        /// \param[in] _height Height of scan
        /// \param[in] _channels Number of channels
        /// \param[in] _format Data format
      private:
        void OnNewLidarFrame(const float *_scan,
                             unsigned int _width, unsigned int _height, unsigned int _channels,
                             const std::string &_format);

        /// \brief Update current frame for scanning pattern
      private:
        void UpdateCurrentFrame();

        /// \brief Private data pointer
      private:
        std::unique_ptr<GpuLidarWithScanningPatternSensorPrivate> dataPtr;
      };
    }
  }
}

#endif