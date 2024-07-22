#include "myServo.h"
#include <vector>

class leg {
    public:
        leg(int* link_lengths, int sda, int scl, int min_freq, int max_freq);
        struct Point3D { float x; float y; float z;};
        void start();
        void moveAngles(int servo_num, int angles[3]);
        std::vector<int> getPosition(int leg_num);
        Point3D movePosition(int leg_num, int* target_pos, float error);
        
    private:
        myServo servos;
        int LEG_DIMS[3];
        int servo_angles[6][3];
        struct Point2D { float x; float y; };
        Point2D doProjection(Point2D p1, Point2D p2, float len);

        float rad2deg(float rad);
        float deg2rad(float deg);
        float atan2_deg(float y, float x);


};