/**
 * Python wrapper for Trakstar PointATC3DG using pybind11
 * Author: Cheng Zhang <cz2874@columbia.edu>
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <vector>
#include <array>
#include "trakstar/PointATC3DG.hpp"

namespace py = pybind11;

// Helper class to wrap sensor data
struct SensorData {
    double x, y, z;
    std::vector<double> quaternion;  // [w, x, y, z]
    double azimuth, elevation, roll;
    std::vector<double> rotation_matrix;  // 3x3 matrix flattened to 9 elements
    
    SensorData() : x(0), y(0), z(0), azimuth(0), elevation(0), roll(0) {
        quaternion.resize(4, 0.0);
        quaternion[0] = 1.0;  // w component
        rotation_matrix.resize(9, 0.0);
    }
};

// Python wrapper class for PointATC3DG
class TrakstarPython {
private:
    std::unique_ptr<trakstar::PointATC3DG> bird_;
    int num_sensors_;
    
public:
    TrakstarPython() : num_sensors_(0) {
        bird_ = std::make_unique<trakstar::PointATC3DG>();
        if (bird_->ok()) {
            num_sensors_ = bird_->getNumberOfSensors();
        }
    }
    
    ~TrakstarPython() = default;
    
    // Check if device is initialized properly
    bool is_ok() const {
        return bird_->ok();
    }
    
    // Get number of sensors
    int get_number_of_sensors() const {
        return num_sensors_;
    }
    
    // Check if transmitter is attached
    bool transmitter_attached() const {
        return bird_->transmitterAttached();
    }
    
    // Check if specific sensor is attached
    bool sensor_attached(int sensor_id) const {
        return bird_->sensorAttached(sensor_id);
    }
    
    // Set measurement rate
    int set_measurement_rate(double rate) {
        return bird_->setMeasurementRate(rate);
    }
    
    // Set maximum range (true for 72 inch, false for 36 inch)
    int set_maximum_range(bool range_72inch) {
        return bird_->setMaximumRange(range_72inch);
    }
    
    // Set sensor hemisphere
    int set_sensor_hemisphere(int sensor_id, bool hemisphere_back) {
        char hemisphere = hemisphere_back ? HEMISPHERE_REAR : HEMISPHERE_FRONT;
        return bird_->setSensorHemisphere(sensor_id, hemisphere);
    }
    
    // Set sensor to output quaternion
    int set_sensor_quaternion(int sensor_id) {
        return bird_->setSensorQuaternion(sensor_id);
    }
    
    // Set sensor to output rotation matrix
    int set_sensor_rotation_matrix(int sensor_id) {
        return bird_->setSensorRotMat(sensor_id);
    }
    
    // Get coordinates with quaternion
    py::dict get_coordinates_quaternion(int sensor_id) {
        double x, y, z;
        double quat[4];
        
        int result = bird_->getCoordinatesQuaternion(sensor_id, x, y, z, quat);
        
        py::dict data;
        data["success"] = (result == 0);
        data["x"] = x;
        data["y"] = y;
        data["z"] = z;
        data["quaternion"] = std::vector<double>{quat[0], quat[1], quat[2], quat[3]}; // [w, x, y, z]
        
        return data;
    }
    
    // Get coordinates with Euler angles
    py::dict get_coordinates_angles(int sensor_id) {
        double x, y, z, azimuth, elevation, roll;
        
        int result = bird_->getCoordinatesAngles(sensor_id, x, y, z, azimuth, elevation, roll);
        
        py::dict data;
        data["success"] = (result == 0);
        data["x"] = x;
        data["y"] = y;
        data["z"] = z;
        data["azimuth"] = azimuth;
        data["elevation"] = elevation;
        data["roll"] = roll;
        
        return data;
    }
    
    // Get coordinates with rotation matrix
    py::dict get_coordinates_matrix(int sensor_id) {
        double x, y, z;
        double matrix[9];
        
        int result = bird_->getCoordinatesMatrix(sensor_id, x, y, z, matrix);
        
        py::dict data;
        data["success"] = (result == 0);
        data["x"] = x;
        data["y"] = y;
        data["z"] = z;
        data["rotation_matrix"] = std::vector<double>(matrix, matrix + 9);
        
        return data;
    }
    
    // Get data from all sensors
    py::dict get_all_sensors_data() {
        py::dict all_data;
        all_data["success"] = bird_->ok();
        all_data["num_sensors"] = num_sensors_;
        
        py::list sensors_data;
        for (int i = 0; i < num_sensors_; i++) {
            py::dict sensor_data = get_coordinates_quaternion(i);
            sensor_data["sensor_id"] = i;
            sensors_data.append(sensor_data);
        }
        all_data["sensors"] = sensors_data;
        
        return all_data;
    }
};

PYBIND11_MODULE(pytrak, m) {
    m.doc() = "Python wrapper for Trakstar PointATC3DG USB tracker";
    
    // Define constants
    m.attr("HEMISPHERE_FRONT") = py::int_(HEMISPHERE_FRONT);
    m.attr("HEMISPHERE_REAR") = py::int_(HEMISPHERE_REAR);
    
    // Define the main TrakstarPython class
    py::class_<TrakstarPython>(m, "Trakstar")
        .def(py::init<>())
        .def("is_ok", &TrakstarPython::is_ok, "Check if device is initialized properly")
        .def("get_number_of_sensors", &TrakstarPython::get_number_of_sensors, "Get number of connected sensors")
        .def("transmitter_attached", &TrakstarPython::transmitter_attached, "Check if transmitter is attached")
        .def("sensor_attached", &TrakstarPython::sensor_attached, "Check if specific sensor is attached", py::arg("sensor_id"))
        .def("set_measurement_rate", &TrakstarPython::set_measurement_rate, "Set measurement rate", py::arg("rate"))
        .def("set_maximum_range", &TrakstarPython::set_maximum_range, "Set maximum range", py::arg("range_72inch"))
        .def("set_sensor_hemisphere", &TrakstarPython::set_sensor_hemisphere, "Set sensor hemisphere", py::arg("sensor_id"), py::arg("hemisphere_back"))
        .def("set_sensor_quaternion", &TrakstarPython::set_sensor_quaternion, "Set sensor to output quaternion", py::arg("sensor_id"))
        .def("set_sensor_rotation_matrix", &TrakstarPython::set_sensor_rotation_matrix, "Set sensor to output rotation matrix", py::arg("sensor_id"))
        .def("get_coordinates_quaternion", &TrakstarPython::get_coordinates_quaternion, "Get coordinates with quaternion", py::arg("sensor_id"))
        .def("get_coordinates_angles", &TrakstarPython::get_coordinates_angles, "Get coordinates with Euler angles", py::arg("sensor_id"))
        .def("get_coordinates_matrix", &TrakstarPython::get_coordinates_matrix, "Get coordinates with rotation matrix", py::arg("sensor_id"))
        .def("get_all_sensors_data", &TrakstarPython::get_all_sensors_data, "Get data from all sensors");
    
    // Define SensorData helper class
    py::class_<SensorData>(m, "SensorData")
        .def(py::init<>())
        .def_readwrite("x", &SensorData::x)
        .def_readwrite("y", &SensorData::y)
        .def_readwrite("z", &SensorData::z)
        .def_readwrite("quaternion", &SensorData::quaternion)
        .def_readwrite("azimuth", &SensorData::azimuth)
        .def_readwrite("elevation", &SensorData::elevation)
        .def_readwrite("roll", &SensorData::roll)
        .def_readwrite("rotation_matrix", &SensorData::rotation_matrix);
}
