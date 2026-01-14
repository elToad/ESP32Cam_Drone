#include <cmath>
#include <iostream>

class ComplementaryFilter {
public:
    ComplementaryFilter(double alpha = 0.98) 
        : alpha(alpha), roll(0.0), pitch(0.0) {}

    void update(double gx, double gy, double gz, 
                double ax, double ay, double az, 
                double dt) {
                    
        // Calculate roll and pitch from accelerometer
        double roll_acc = std::atan2(ay, az) * 180.0 / M_PI;
        double pitch_acc = std::atan2(-ax, std::sqrt(ay*ay + az*az)) * 180.0 / M_PI;

        // Integrate gyroscope data
        roll += gx * dt * 180.0 / M_PI;
        pitch += gy * dt * 180.0 / M_PI;

        // Apply complementary filter
        roll = alpha * roll + (1.0 - alpha) * roll_acc;
        pitch = alpha * pitch + (1.0 - alpha) * pitch_acc;
    }

    double getRoll() const { return roll; }
    double getPitch() const { return pitch; }

private:
    double alpha;  // Filter coefficient (gyro weight)
    double roll;   // In degrees
    double pitch;  // In degrees
};