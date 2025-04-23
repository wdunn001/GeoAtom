#ifndef COMPASS_INTERFACE_H
#define COMPASS_INTERFACE_H

#include <Arduino.h>
#include <QMC5883LCompass.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>

/**
 * @brief Abstract interface for compass/magnetometer sensors
 * 
 * This interface defines the common functionality for different
 * compass libraries to present a unified API to the application code.
 */
class CompassInterface {
public:
    /**
     * @brief Initialize the compass
     * @return true if initialization successful
     */
    virtual bool begin() = 0;
    
    /**
     * @brief Read new data from the compass
     * @return true if reading successful
     */
    virtual bool read() = 0;
    
    /**
     * @brief Get the current azimuth/heading in degrees (0-359)
     * @return int Heading in degrees, -1 if error
     */
    virtual int getAzimuth() = 0;
    
    /**
     * @brief Check if the compass data is valid
     * @return true if data is valid and ready to use
     */
    virtual bool isDataValid() = 0;
    
    /**
     * @brief Get the X-axis raw reading
     * @return float X-axis reading
     */
    virtual float getX() = 0;
    
    /**
     * @brief Get the Y-axis raw reading
     * @return float Y-axis reading
     */
    virtual float getY() = 0;
    
    /**
     * @brief Get the Z-axis raw reading
     * @return float Z-axis reading
     */
    virtual float getZ() = 0;
    
    /**
     * @brief Get the compass direction as text (N, NE, E, etc.)
     * @param buffer Character buffer to populate with direction text
     * @param azimuth Optional heading value, will use current heading if not provided
     */
    virtual void getDirection(char* buffer, int azimuth = -1) = 0;
    
    /**
     * @brief Checks if calibration is supported
     * @return true if calibration is supported
     */
    virtual bool supportsCalibration() = 0;
    
    /**
     * @brief Set calibration offsets (if supported)
     * @param x_offset X-axis offset
     * @param y_offset Y-axis offset
     * @param z_offset Z-axis offset
     * @return true if setting successful
     */
    virtual bool setCalibrationOffsets(float x_offset, float y_offset, float z_offset) = 0;
    
    /**
     * @brief Set calibration scales (if supported)
     * @param x_scale X-axis scale factor
     * @param y_scale Y-axis scale factor
     * @param z_scale Z-axis scale factor
     * @return true if setting successful
     */
    virtual bool setCalibrationScales(float x_scale, float y_scale, float z_scale) = 0;
    
    /**
     * @brief Calculate calibration from 8 measured compass points
     * @param pointsX Array of X values for the 8 calibration points (N,S,E,W,NE,SE,SW,NW)
     * @param pointsY Array of Y values for the 8 calibration points
     * @return true if calibration was calculated and applied successfully
     */
    virtual bool calculateCalibration(int* pointsX, int* pointsY) = 0;
    
    /**
     * @brief Get a string with sensor info for debugging/display
     * @return String sensor info
     */
    virtual String getSensorInfoString() = 0;
    
    /**
     * @brief Get the sensor type name
     * @return const char* sensor name
     */
    virtual const char* getSensorName() = 0;
    
    /**
     * @brief Virtual destructor for cleanup
     */
    virtual ~CompassInterface() {}
};

/**
 * @brief Implementation of CompassInterface for QMC5883L using mprograms/QMC5883LCompass library
 */
class QMC5883LCompassImpl : public CompassInterface {
private:
    QMC5883LCompass _compass;
    bool _initialized = false;
    int _last_x = 0;
    int _last_y = 0;
    int _last_z = 0;

public:
    bool begin() override {
        _compass.init();
        // Since init() doesn't return status, read once to check if we get data
        for(int i=0; i<3; i++) {
            _compass.read();
            // If any axis gives non-zero reading, consider it working
            if (_compass.getX() != 0 || _compass.getY() != 0 || _compass.getZ() != 0) {
                _initialized = true;
                break;
            }
            delay(10);
        }
        return _initialized;
    }
    
    bool read() override {
        if (!_initialized) return false;
        _compass.read();
        _last_x = _compass.getX();
        _last_y = _compass.getY();
        _last_z = _compass.getZ();
        return true;
    }
    
    int getAzimuth() override {
        if (!_initialized) return -1;
        return _compass.getAzimuth();
    }
    
    float getX() override {
        return _last_x;
    }
    
    float getY() override {
        return _last_y;
    }
    
    float getZ() override {
        return _last_z;
    }
    
    bool isDataValid() override {
        return _initialized && (_last_x != 0 || _last_y != 0 || _last_z != 0);
    }
    
    void getDirection(char* buffer, int azimuth = -1) override {
        if (!_initialized) {
            buffer[0] = 'E';
            buffer[1] = 'R';
            buffer[2] = 'R';
            buffer[3] = '\0';
            return;
        }
        
        if (azimuth == -1) {
            azimuth = getAzimuth();
        }
        
        // Get the text direction (N, NE, E, etc.)
        _compass.getDirection(buffer, azimuth);
    }
    
    bool supportsCalibration() override {
        return true;
    }
    
    bool setCalibrationOffsets(float x_offset, float y_offset, float z_offset) override {
        if (!_initialized) return false;
        _compass.setCalibrationOffsets(x_offset, y_offset, z_offset);
        return true;
    }
    
    bool setCalibrationScales(float x_scale, float y_scale, float z_scale) override {
        if (!_initialized) return false;
        _compass.setCalibrationScales(x_scale, y_scale, z_scale);
        return true;
    }
    
    bool calculateCalibration(int* pointsX, int* pointsY) override {
        if (!_initialized) return false;
        
        // Calculate offsets based on min/max ranges from 8 points
        int x_min = pointsX[0], x_max = pointsX[0];
        int y_min = pointsY[0], y_max = pointsY[0];
        
        for (int i=1; i<8; i++) {
            if (pointsX[i] < x_min) x_min = pointsX[i];
            if (pointsX[i] > x_max) x_max = pointsX[i];
            if (pointsY[i] < y_min) y_min = pointsY[i];
            if (pointsY[i] > y_max) y_max = pointsY[i];
        }
        
        float x_offset = (x_max + x_min) / 2.0f;
        float y_offset = (y_max + y_min) / 2.0f;
        float z_offset = 0; // QMC library doesn't really use Z in direction calc
        
        float x_delta = (x_max - x_min) / 2.0f;
        float y_delta = (y_max - y_min) / 2.0f;
        
        // Avoid division by zero if range is zero
        if (x_delta <= 0 || y_delta <= 0) {
            return false;
        }
        
        float avg_delta = (x_delta + y_delta) / 2.0f; // Average range
        
        float x_scale = avg_delta / x_delta;
        float y_scale = avg_delta / y_delta;
        float z_scale = 1.0f;
        
        // Apply calibration to QMC compass object
        setCalibrationOffsets(x_offset, y_offset, z_offset);
        setCalibrationScales(x_scale, y_scale, z_scale);
        
        return true;
    }
    
    String getSensorInfoString() override {
        if (!_initialized) return "QMC5883L: not initialized";
        return "QMC5883L: X=" + String(_last_x) + " Y=" + String(_last_y) + " Z=" + String(_last_z);
    }
    
    const char* getSensorName() override {
        return "QMC5883L";
    }
};

/**
 * @brief Implementation of CompassInterface for HMC5883L using Adafruit_HMC5883_Unified library
 */
class HMC5883LCompassImpl : public CompassInterface {
private:
    Adafruit_HMC5883_Unified _hmc;
    bool _initialized = false;
    sensors_event_t _last_event;
    float _declination_rad = 0.0f; // Default declination correction (0 = none)
    
    // Add calibration parameters
    float _x_offset = 0.0f;
    float _y_offset = 0.0f;
    float _z_offset = 0.0f;
    float _x_scale = 1.0f;
    float _y_scale = 1.0f;
    float _z_scale = 1.0f;
    bool _calibration_valid = false;

public:
    HMC5883LCompassImpl(int32_t sensor_id = 12345) : _hmc(sensor_id) {
        // Initialize last_event to zero values
        memset(&_last_event, 0, sizeof(sensors_event_t));
    }
    
    // Set magnetic declination for your location (in radians)
    void setDeclination(float declination_rad) {
        _declination_rad = declination_rad;
    }
    
    bool begin() override {
        _initialized = _hmc.begin();
        if (_initialized) {
            // Read once to verify we get data
            read();
        }
        return _initialized;
    }
    
    bool read() override {
        if (!_initialized) return false;
        
        bool success = _hmc.getEvent(&_last_event);
        
        // Apply calibration if valid
        if (success && _calibration_valid) {
            // Apply offset and scale corrections
            _last_event.magnetic.x = (_last_event.magnetic.x - _x_offset) * _x_scale;
            _last_event.magnetic.y = (_last_event.magnetic.y - _y_offset) * _y_scale;
            _last_event.magnetic.z = (_last_event.magnetic.z - _z_offset) * _z_scale;
        }
        
        return success;
    }
    
    int getAzimuth() override {
        if (!_initialized) return -1;
        
        // Calculate heading when the magnetometer is level
        float heading = atan2(_last_event.magnetic.y, _last_event.magnetic.x);
        
        // Add declination adjustment
        heading += _declination_rad;
        
        // Correct for wrap due to addition of declination
        if (heading < 0) heading += 2 * PI;
        if (heading > 2 * PI) heading -= 2 * PI;
        
        // Convert radians to degrees
        float headingDegrees = heading * 180 / M_PI;
        
        return (int)headingDegrees;
    }
    
    float getX() override {
        return _last_event.magnetic.x;
    }
    
    float getY() override {
        return _last_event.magnetic.y;
    }
    
    float getZ() override {
        return _last_event.magnetic.z;
    }
    
    bool isDataValid() override {
        return _initialized && (_last_event.magnetic.x != 0 || _last_event.magnetic.y != 0 || _last_event.magnetic.z != 0);
    }
    
    void getDirection(char* buffer, int azimuth = -1) override {
        if (!_initialized) {
            strcpy(buffer, "ERR");
            return;
        }
        
        if (azimuth == -1) {
            azimuth = getAzimuth();
        }
        
        // Implementation of basic 8-point compass directions
        if (azimuth >= 338 || azimuth < 23) strcpy(buffer, "N");
        else if (azimuth < 68) strcpy(buffer, "NE");
        else if (azimuth < 113) strcpy(buffer, "E");
        else if (azimuth < 158) strcpy(buffer, "SE");
        else if (azimuth < 203) strcpy(buffer, "S");
        else if (azimuth < 248) strcpy(buffer, "SW");
        else if (azimuth < 293) strcpy(buffer, "W");
        else if (azimuth < 338) strcpy(buffer, "NW");
    }
    
    bool supportsCalibration() override {
        // Now we support calibration for HMC5883L
        return true;
    }
    
    bool setCalibrationOffsets(float x_offset, float y_offset, float z_offset) override {
        if (!_initialized) return false;
        
        _x_offset = x_offset;
        _y_offset = y_offset;
        _z_offset = z_offset;
        _calibration_valid = true;
        
        return true;
    }
    
    bool setCalibrationScales(float x_scale, float y_scale, float z_scale) override {
        if (!_initialized) return false;
        
        _x_scale = x_scale;
        _y_scale = y_scale;
        _z_scale = z_scale;
        _calibration_valid = true;
        
        return true;
    }
    
    bool calculateCalibration(int* pointsX, int* pointsY) override {
        if (!_initialized) return false;
        
        // Calculate calibration from multiple points similar to QMC5883L
        float x_min = pointsX[0], x_max = pointsX[0];
        float y_min = pointsY[0], y_max = pointsY[0];
        
        // Find min/max values for X and Y
        int numPoints = 8; // Default to 8 points
        if (pointsX[15] != 0 || pointsY[15] != 0) numPoints = 16;
        else if (pointsX[7] == 0 && pointsY[7] == 0) numPoints = 4;
        
        for (int i = 1; i < numPoints; i++) {
            if (pointsX[i] < x_min) x_min = pointsX[i];
            if (pointsX[i] > x_max) x_max = pointsX[i];
            if (pointsY[i] < y_min) y_min = pointsY[i];
            if (pointsY[i] > y_max) y_max = pointsY[i];
        }
        
        // Calculate center offset (ellipse center)
        _x_offset = (x_max + x_min) / 2.0f;
        _y_offset = (y_max + y_min) / 2.0f;
        _z_offset = 0; // Z offset calculation would require 3D calibration
        
        // Calculate scaling factors to transform ellipse to circle
        float x_delta = (x_max - x_min) / 2.0f;
        float y_delta = (y_max - y_min) / 2.0f;
        
        // Avoid division by zero
        if (x_delta <= 0 || y_delta <= 0) {
            return false;
        }
        
        // Use average delta as the target circle radius
        float avg_delta = (x_delta + y_delta) / 2.0f;
        
        // Calculate scales to normalize axes to the same circle
        _x_scale = avg_delta / x_delta;
        _y_scale = avg_delta / y_delta;
        _z_scale = 1.0f; // Z scale unchanged
        
        _calibration_valid = true;
        
        return true;
    }
    
    String getSensorInfoString() override {
        if (!_initialized) return "HMC5883L: not initialized";
        
        String info = "HMC5883L: X=" + String(_last_event.magnetic.x) + 
               " Y=" + String(_last_event.magnetic.y) + 
               " Z=" + String(_last_event.magnetic.z) + " uT";
               
        if (_calibration_valid) {
            info += " [Cal]";
        }
        
        return info;
    }
    
    const char* getSensorName() override {
        return "HMC5883L";
    }
};

#endif // COMPASS_INTERFACE_H 