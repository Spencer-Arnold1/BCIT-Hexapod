#include "leg.h"
#include <math.h>

leg::leg(int* link_lengths, int sda, int scl, int min_freq, int max_freq) 
    : servos(sda, scl, min_freq, max_freq) 
    {
        for (int i = 0; i < 3; i++) 
        {
            LEG_DIMS[i] = link_lengths[i];
        }

        for (int i = 0; i < 6; i++) 
        {
            for (int n = 0; n < 3; n++) 
            {
                servo_angles[i][n] = 90;
            }
        }
    }

void leg::start()
{
    servos.start();
}

void leg::moveAngles(int leg_num, int* angles)
{
    int base_index = 3*leg_num;
    for(int n = 0; n < 3; n++)
    {
       servo_angles[leg_num][n] = angles[n]; 
    }
    //servo_angles[leg_num] = angles;
    servos.move(base_index,angles[0]);
    servos.move(base_index + 1,angles[1]);
    servos.move(base_index + 2,angles[2]);
}

std::vector<int> leg::getPosition(int leg_num)
{
    int angles[3];
    for (int n = 0; n < 3; n++)
    {
        angles[n] = servo_angles[leg_num][n];
    }
    int positions[] = {0,0,0};
    int angle_ele = 0;
}

leg::Point3D leg::movePosition(int leg_num, int* target_pos, float error)
{
    // Get first servo angle
    int angle1= atan2_deg(target_pos[1], target_pos[0]);

    // Convert previous servo angles to positions on the uv plane of the target
    Point2D pos1 = {LEG_DIMS[0], 0};
    Point2D pos2 = {pos1.x + LEG_DIMS[1] * cos(deg2rad(servo_angles[leg_num][1])), pos1.y + LEG_DIMS[1] * sin(deg2rad(servo_angles[leg_num][1]))};

    // Find uv coordinates of target
    Point2D target_uv = {sqrt(target_pos[0] * target_pos[0] + target_pos[1] * target_pos[1]), target_pos[2]};

    // Solve for middle joint position in uv coordinates
    float pos_error = 100;
    Point2D mid_pos = pos2;
    while (pos_error > error) {
        mid_pos = doProjection(target_uv, mid_pos, LEG_DIMS[2]);
        mid_pos = doProjection(pos1, mid_pos, LEG_DIMS[1]);
        Point2D end_test = doProjection(mid_pos, target_uv, LEG_DIMS[2]);
        pos_error = sqrt((target_uv.x - end_test.x) * (target_uv.x - end_test.x) + (target_uv.y - end_test.y) * (target_uv.y - end_test.y));
    }
    Point2D newPos[3] = {pos1, mid_pos, target_uv};

    // Get required servo angles from joint coordinates
    float x = newPos[1].x - newPos[0].x;
    float y = newPos[1].y - newPos[0].y;
    int angle2 = atan2_deg(y, x);

    x = newPos[2].x - newPos[1].x;
    y = newPos[2].y - newPos[1].y;
    int angle3 = atan2_deg(y, x) - angle2 + 90;
    
    int base_index = 3*leg_num;
    //invert servo orientation for left legs
    if (leg_num < 3)
    {
        angle2 = -angle2;
    }
    //invert servo orientation for right legs
    else if (leg_num >= 3 && leg_num < 7)
    {
        angle1 = -angle1;
        angle3 = -angle3;
    }

    //offset back leg angles
    if (leg_num == 0 || leg_num == 3 )
    {
        angle1 += 45;
    }
    //offset front leg angles
    else if (leg_num == 2 || leg_num == 5 )
    {
        angle1 -= 45;
    }

    //move servos into position
    servos.move(base_index,angle1);
    servos.move(base_index + 1,angle2);
    servos.move(base_index + 2,angle3);

    //return servo angles for debugging
    Point3D newAngles = {angle1,angle2,angle3};
    return newAngles;

}

leg::Point2D leg::doProjection(Point2D p1, Point2D p2, float len) {
    Point2D vec = {p2.x - p1.x, p2.y - p1.y};
    float dist = sqrt(vec.x * vec.x + vec.y * vec.y);
    Point2D norm = {vec.x / dist, vec.y / dist};
    Point2D pos = {p1.x + norm.x * len, p1.y + norm.y * len};
    return pos;
}



float leg::rad2deg(float rad) {
    return rad * (180.0 / M_PI);
}

float leg::deg2rad(float deg) {
    return deg * (M_PI / 180.0);
}

float leg::atan2_deg(float y, float x) {
    return rad2deg(atan2(y, x));
}