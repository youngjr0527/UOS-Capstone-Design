#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <queue>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <thread>


#include <boost/multiprecision/cpp_dec_float.hpp>
// #include <boost/multiprecision/detail/functions/pow.hpp>

#define CAL_DISTANCE(rssi, alpha, n, offset) (std::pow(10.0, ((alpha) - (rssi)) / (10.0 * (n))) + (offset))

////////////////////////////////////
// double rssi;
std::queue<double> rssiQueue;
// bool initialized = false;
std::vector<double> rssi_data;

///////////////////////////////////
const int avg_num = 15;

class KalmanFilter {
public:
    KalmanFilter(double processNoise = 0.05, double measurementNoise = 20)
        :  processNoise(processNoise), measurementNoise(measurementNoise),
        predictedRSSI(0), initialized(false),errorCovariance(100) {}

    inline double filtering(double rssi) {
    if (!initialized) {
        initialized = true;
        predictedRSSI = rssi;
    } else {
        double predictedRSSI = this->predictedRSSI;
        double errorCovariance = this->errorCovariance + processNoise;
        double kalmanGain = errorCovariance / (errorCovariance + measurementNoise);
        this->predictedRSSI = predictedRSSI + kalmanGain * (rssi - predictedRSSI);
        this->errorCovariance = (1 - kalmanGain) * errorCovariance;
    }
    return this->predictedRSSI;
}

private:
    bool initialized;
    double processNoise;
    double measurementNoise;
    double predictedRSSI;
    double errorCovariance;
};

class MovAvgFilter {
public:
    MovAvgFilter(int _n = avg_num) : n(_n) {
        for (int i = 0; i < _n; ++i) {
            xBuf.push(1.0);
        }
        prevAvg = 0.0;
    }

    double filtering(double x) {
        double front = xBuf.front();
        xBuf.pop();
        xBuf.push(x);
        double avg = prevAvg + (x - front) / n;
        prevAvg = avg;
        return avg;
    }

private:
    double prevAvg;
    std::queue<double> xBuf;
    int n;
};

class SmoothingFilter {
public:
    SmoothingFilter(double alpha = 0.2) : alpha(alpha), initialized(false),smoothedValue(0) {}

    double filtering(double value) {
        if (!initialized) {
            initialized = true;
            smoothedValue = value;
        } else {
            smoothedValue = alpha * value + (1 - alpha) * smoothedValue;
        }

        return smoothedValue;
    }

private:
    double alpha;
    bool initialized;
    double smoothedValue;
};



double cal_distance(double rssi, double alpha = -52.0, double n = 2.0, double offset = 0.0) {
    using namespace boost::multiprecision;
    cpp_dec_float_100 distance = pow(cpp_dec_float_100(10), (alpha - rssi) / (10.0 * n));
    return static_cast<double>(distance + offset);
}


KalmanFilter filter;
KalmanFilter filter_2;
KalmanFilter filter_3;

// pub; // 전역 변수로 pub 선언

int flag = 0;

double alpha;
int n;
// ros::param::get("alpha", alpha);  // "~"는 private namespace를 의미합니다.
// ros::param::get("n", n);

void rssi_callback(const std_msgs::String& data) {

    if (data.data.empty()) {
        return;
    } else if (data.data == "OK+CONNF") {
        return;
    } else if(data.data.find("OK+Get:") != std::string::npos) 

    {
        // Create a separate thread for filtering and logging

        std::string line = data.data;
        std::string remove_line = line.substr(7); // Remove "OK+Get:"
        double rssi = std::stod(remove_line);
        // rssiQueue.push(rssi);
        double filtered_rssi = filter.filtering(rssi);

        std::cout << "filtered_rssi: " << filtered_rssi << std::endl;

    }

    
}









int main(int argc, char** argv) {
    
    ros::init(argc, argv, "rssi_listener");
    ros::NodeHandle nh;
    
        
    std::string path_2 = "/home/kym/catkin_ws/src/arduino/src/rssi_raw_log.txt";
    std::ofstream logFileTemp(path_2);
    logFileTemp.close();
    KalmanFilter filter;
    try {
    std::cout << "hi ros " << std::endl;
   
    ros::Subscriber sub = nh.subscribe("/rssi", 10, rssi_callback);
    
    
    ros::spin();
    }
    
     catch (ros::Exception& e) {
        // Handle exceptions
    }

    return 0;
}


