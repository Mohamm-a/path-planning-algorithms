#include <vector>
#include <cmath>
#include <cstdint>

// -------------------------------------------------------------
// SENSORSTRUKTURER
// -------------------------------------------------------------

// Enskild mätpunkt från centrerad LiDAR (polära koordinater)
struct LidarPoint {
    float distance; // Avstånd i meter (r)
    float angle;    // Vinkel i radianer från rakt fram (-pi till +pi)
};

// IMU-data (Accelerometer + Gyroskop)
struct IMUData {
    float accel_x, accel_y, accel_z; // Linjär acceleration (m/s²)
    float gyro_x,  gyro_y,  gyro_z;  // Vinkelhastighet (rad/s)
};

// Drivlina & Styrning (Ackermann)
struct AckermannState {
    float speed;          // Hastighet från motorencoder (m/s)
    float steering_angle; // Framhjulens vinkel i radianer (delta)
};

// Samlad indata till varje SLAM-uppdatering
struct SLAMInput {
    std::vector<LidarPoint> lidar_scan;
    IMUData imu;
    AckermannState ackermann;
    float delta_time;     // Tid sedan förra anropet i sekunder (dt)
};

// -------------------------------------------------------------
// SLAM UTDATA OCH KARTA (För Ruttplanering)
// -------------------------------------------------------------

struct RobotPose {
    float x;      // Global X-position (meter)
    float y;      // Global Y-position (meter)
    float theta;  // Global orientering/vinkel (radianer)
};

enum class CellState : uint8_t {
    UNKNOWN = 0,
    FREE = 1,
    OBSTACLE = 2  // Hinder / Väggar / Banans kanter
};

struct OccupancyGrid {
    uint32_t width;               // Bredd i antal celler
    uint32_t height;              // Höjd i antal celler
    float resolution;             // Storlek per cell i meter (t.ex. 0.05 m = 5 cm)
    RobotPose origin;             // Kartans origo i globala koordinater
    std::vector<CellState> data;  // 1D-vektor som lagrar 2D-kartan (width * height)
};

struct SLAMOutput {
    RobotPose pose;      // Robotens slutgiltiga position (Startpunkt för A*)
    OccupancyGrid map;   // Hinderkartan (Sökyta för A*)
    bool is_tracking;    // Status om SLAM har god konvergens/positionering
};

// -------------------------------------------------------------
// SLAM FUNKTION
// -------------------------------------------------------------

class SLAMSystem {
private:
    const float wheelbase_ = 0.30f; // Robotens hjulbas i meter (L)
    RobotPose current_pose_{0.0f, 0.0f, 0.0f};
    OccupancyGrid map_;

    // Omvandlar polära LiDAR-punkter till kartesiska hinder i globala koordinater
    void integrateLidarScan(const std::vector<LidarPoint>& scan, const RobotPose& pose) {
        for (const auto& pt : scan) {
            if (pt.distance <= 0.0f) continue; // Filtrera ogiltiga mätningar

            // 1. Polärt -> Lokala kartesiska (Roboten i origo)
            float local_x = pt.distance * std::cos(pt.angle);
            float local_y = pt.distance * std::sin(pt.angle);

            // 2. Lokala -> Globala koordinater baserat på nuvarande position
            float global_x = pose.x + (local_x * std::cos(pose.theta) - local_y * std::sin(pose.theta));
            float global_y = pose.y + (local_x * std::sin(pose.theta) + local_y * std::cos(pose.theta));

            // 3. Markera cellen som OBSTACLE i OccupancyGrid (kartindexering)
            // ... Mappa global_x, global_y till grid-index och sätt map_.data[idx] ...
        }
    }

public:
    SLAMOutput update(const SLAMInput& input) {
        // STEP 1: Beräkna Odometri-prediktion (Dead Reckoning)
        // Kombinera Ackermann-kinematik med Gyroskop (gyro_z) för stabil svängningshastighet
        float yaw_rate_ackermann = (input.ackermann.speed / wheelbase_) * std::tan(input.ackermann.steering_angle);
        float fused_yaw_rate = 0.5f * (yaw_rate_ackermann + input.imu.gyro_z); // Enkel fusion / EKF

        // Uppdatera orientering och position
        current_pose_.theta += fused_yaw_rate * input.delta_time;
        current_pose_.x += input.ackermann.speed * std::cos(current_pose_.theta) * input.delta_time;
        current_pose_.y += input.ackermann.speed * std::sin(current_pose_.theta) * input.delta_time;

        // STEP 2: Korrigera posen med Scan Matching (LiDAR mot befintlig karta)
        // ... (t.ex. ICP eller Scan-to-Map Matching) ...

        // STEP 3: Rita in nya hinder i Occupancy Grid
        integrateLidarScan(input.lidar_scan, current_pose_);

        return { current_pose_, map_, true };
    }
};