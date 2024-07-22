#include "leg.h"
#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
  uint8_t LX = 128;
  uint8_t LY = 128;
  uint8_t RX = 128;
  uint8_t RY = 128;
} struct_message;

struct_message myData;

int LEG_SEGMENT_LEGTHS[3] = {32, 124, 221};
leg legs(LEG_SEGMENT_LEGTHS,41,42,102,512);

int legOffset = 100;
int frontStep = 0;
int sideStep = 0;
int height = 200;
int stepHeight = 100;
int legWidth = 100;
int frontTilt = 0;
int sideTilt = 0;
int midSideOffset = 50;

int pose1lm[] = {legWidth,0,-height-sideTilt};
int pose2lm[] = {legWidth-sideStep,-frontStep,-height-sideTilt};
int pose3lm[] = {legWidth,0,-height+stepHeight-sideTilt};
int pose4lm[] = {legWidth+sideStep,frontStep,-height-sideTilt};

int pose1lb[] = {legWidth,-legOffset,-height-frontTilt-sideTilt};
int pose2lb[] = {legWidth-sideStep,-frontStep-legOffset,-height-frontTilt-sideTilt};
int pose3lb[] = {legWidth,-legOffset,-height+stepHeight-frontTilt-sideTilt};
int pose4lb[] = {legWidth+sideStep,frontStep-legOffset,-height-frontTilt-sideTilt};

int pose1lf[] = {legWidth,legOffset,-height+frontTilt-sideTilt};
int pose2lf[] = {legWidth-sideStep,-frontStep+legOffset,-height+frontTilt-sideTilt};
int pose3lf[] = {legWidth,legOffset,-height+stepHeight+frontTilt-sideTilt};
int pose4lf[] = {legWidth+sideStep,frontStep+legOffset,-height+frontTilt-sideTilt};

int pose1rm[] = {legWidth,0,-height+sideTilt};
int pose2rm[] = {legWidth+sideStep,-frontStep,-height+sideTilt};
int pose3rm[] = {legWidth,0,-height+stepHeight+sideTilt};
int pose4rm[] = {legWidth-sideStep,frontStep,-height+sideTilt};

int pose1rb[] = {legWidth,-legOffset,-height-frontTilt+sideTilt};
int pose2rb[] = {legWidth+sideStep,-frontStep-legOffset,-height-frontTilt+sideTilt};
int pose3rb[] = {legWidth,-legOffset,-height+stepHeight-frontTilt+sideTilt};
int pose4rb[] = {legWidth-sideStep,frontStep-legOffset,-height-frontTilt+sideTilt};

int pose1rf[] = {legWidth,legOffset,-height+frontTilt};
int pose2rf[] = {legWidth+sideStep,-frontStep+legOffset,-height+frontTilt+sideTilt};
int pose3rf[] = {legWidth,legOffset,-height+stepHeight+frontTilt+sideTilt};
int pose4rf[] = {legWidth-sideStep,frontStep+legOffset,-height+frontTilt+sideTilt};

int wait_time = 300;

int neutral[] = {0,0,0};


unsigned long previousMillis = 0;
const long interval = 30; 

enum StepState { STEP1, STEP2, STEP3, STEP4 };
StepState currentStep = STEP1;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
}

void moveInterpolate(int leg_num, int* start_pos, int* end_pos, float progress) {
    int target_pos[3];
    for (int i = 0; i < 3; ++i) {
        target_pos[i] = start_pos[i] + (end_pos[i] - start_pos[i]) * progress;
    }
    legs.movePosition(leg_num, target_pos, 1);
}

void updateLegPositions() {
    static unsigned long startMillis = 0;
    unsigned long currentMillis = millis();
    float progress = float(currentMillis - startMillis) / wait_time;
    if (progress > 1.0) progress = 1.0;

    switch (currentStep) {
        case STEP1:
            moveInterpolate(0, pose1lb, pose2lb, progress);
            moveInterpolate(2, pose1lf, pose2lf, progress);
            moveInterpolate(4, pose1rm, pose2rm, progress);

            moveInterpolate(1, pose3lm, pose4lm, progress);
            moveInterpolate(3, pose3rf, pose4rf, progress);
            moveInterpolate(5, pose3rb, pose4rb, progress);
            break;
        case STEP2:
            moveInterpolate(0, pose2lb, pose3lb, progress);
            moveInterpolate(2, pose2lf, pose3lf, progress);
            moveInterpolate(4, pose2rm, pose3rm, progress);

            moveInterpolate(1, pose4lm, pose1lm, progress);
            moveInterpolate(3, pose4rf, pose1rf, progress);
            moveInterpolate(5, pose4rb, pose1rb, progress);
            break;
        case STEP3:
            moveInterpolate(0, pose3lb, pose4lb, progress);
            moveInterpolate(2, pose3lf, pose4lf, progress);
            moveInterpolate(4, pose3rm, pose4rm, progress);

            moveInterpolate(1, pose1lm, pose2lm, progress);
            moveInterpolate(3, pose1rf, pose2rf, progress);
            moveInterpolate(5, pose1rb, pose2rb, progress);
            break;
        case STEP4:
            moveInterpolate(0, pose4lb, pose1lb, progress);
            moveInterpolate(2, pose4lf, pose1lf, progress);
            moveInterpolate(4, pose4rm, pose1rm, progress);

            moveInterpolate(1, pose2lm, pose3lm, progress);
            moveInterpolate(3, pose2rf, pose3rf, progress);
            moveInterpolate(5, pose2rb, pose3rb, progress);
            break;
    }

    if (progress >= 1.0) {
        startMillis = currentMillis;
        currentStep = static_cast<StepState>((currentStep + 1) % 4);
    }
}

void updatePose(int* pose, int x, int y, int z)
{
    pose[0] = x;
    pose[1] = y;
    pose[2] = z;
}

void updateTargets()
{
    //legOffset = 100;
    frontStep = ((100 * myData.LY) / 256) - 50 ;
    sideStep = ((-100 * myData.LX) / 256) + 50 ;

    sideTilt = ((80 * myData.RX) / 256) - 40 ;
    frontTilt = ((80 * myData.RY) / 256) - 40 ;

    Serial.print("front step:  ");
    Serial.print(frontStep);
    Serial.print("    side step:  ");
    Serial.print(sideStep);
    Serial.print("front tilt:  ");
    Serial.print(frontTilt);
    Serial.print("    side tilt:  ");
    Serial.println(sideTilt);
    //height = 150;
    //stepHeight = 70;
    //legWidth = 100;
    //frontTilt = 0;
    //sideTilt = 0;

    updatePose(pose1lm,legWidth+midSideOffset,0,-height-sideTilt);
    updatePose(pose2lm,legWidth-sideStep+midSideOffset,-frontStep,-height-sideTilt);
    updatePose(pose3lm,legWidth+midSideOffset,0,-height+stepHeight-sideTilt);
    updatePose(pose4lm,legWidth+sideStep+midSideOffset,frontStep,-height-sideTilt);

    updatePose(pose1lb,legWidth,-legOffset,-height-frontTilt-sideTilt);
    updatePose(pose2lb,legWidth-sideStep,-frontStep-legOffset,-height-frontTilt-sideTilt);
    updatePose(pose3lb,legWidth,-legOffset,-height+stepHeight-frontTilt-sideTilt);
    updatePose(pose4lb,legWidth+sideStep,frontStep-legOffset,-height-frontTilt-sideTilt);

    updatePose(pose1lf,legWidth,legOffset,-height+frontTilt-sideTilt);
    updatePose(pose2lf,legWidth-sideStep,-frontStep+legOffset,-height+frontTilt-sideTilt);
    updatePose(pose3lf,legWidth,legOffset,-height+stepHeight+frontTilt-sideTilt);
    updatePose(pose4lf,legWidth+sideStep,frontStep+legOffset,-height+frontTilt-sideTilt);

    updatePose(pose1rm,legWidth+midSideOffset,0,-height+sideTilt);
    updatePose(pose2rm,legWidth+sideStep+midSideOffset,-frontStep,-height+sideTilt);
    updatePose(pose3rm,legWidth+midSideOffset,0,-height+stepHeight+sideTilt);
    updatePose(pose4rm,legWidth-sideStep+midSideOffset,frontStep,-height+sideTilt);

    updatePose(pose1rb,legWidth,-legOffset,-height-frontTilt+sideTilt);
    updatePose(pose2rb,legWidth+sideStep,-frontStep-legOffset,-height-frontTilt+sideTilt);
    updatePose(pose3rb,legWidth,-legOffset,-height+stepHeight-frontTilt+sideTilt);
    updatePose(pose4rb,legWidth-sideStep,frontStep-legOffset,-height-frontTilt+sideTilt);

    updatePose(pose1rf,legWidth,legOffset,-height+frontTilt);
    updatePose(pose2rf,legWidth+sideStep,-frontStep+legOffset,-height+frontTilt+sideTilt);
    updatePose(pose3rf,legWidth,legOffset,-height+stepHeight+frontTilt+sideTilt);
    updatePose(pose4rf,legWidth-sideStep,frontStep+legOffset,-height+frontTilt+sideTilt);


}

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
  
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    Serial.println("Started listening ESP-NOW");
  
    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("PCA9685 Servo Test");
    legs.start();
    //servos.start();
  
    for (int i = 0; i < 6 ; i++)
    {
        legs.moveAngles(i, neutral);
    }
  
}

void loop() {
  
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        updateTargets();
        updateLegPositions();
    }
   
    














/*
if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        int firstCommaIndex = input.indexOf(',');
        int secondCommaIndex = input.indexOf(',', firstCommaIndex + 1);
        int thirdCommaIndex = input.indexOf(',', secondCommaIndex + 1);

        if (firstCommaIndex != -1 && secondCommaIndex != -1 && thirdCommaIndex != -1) {
            int legNumber = input.substring(0, firstCommaIndex).toInt();
            int angle1 = input.substring(firstCommaIndex + 1, secondCommaIndex).toInt();
            int angle2 = input.substring(secondCommaIndex + 1, thirdCommaIndex).toInt();
            int angle3 = input.substring(thirdCommaIndex + 1).toInt();

            if (legNumber >= 0 && legNumber < 7 ) {
                int angles[] = {angle1, angle2, angle3};
                //legs.moveAngles(legNumber, angles);
                leg::Point3D newAngles = legs.movePosition(legNumber, angles, 1);
                Serial.print("Leg ");
                Serial.print(legNumber);
                Serial.print(" set to position ");
                Serial.print(angle1);
                Serial.print(", ");
                Serial.print(angle2);
                Serial.print(", ");
                Serial.print(angle3);
                Serial.print("  |  Moved to angles ");
                Serial.print(newAngles.x);
                Serial.print(", ");
                Serial.print(newAngles.y);
                Serial.print(", ");
                Serial.println(newAngles.z);
            } else {
                Serial.println("Invalid leg number or angles");
            }
        } else {
            Serial.println("Invalid input format");
        }
    }
*/


/*
   if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int separatorIndex = input.indexOf(',');
    
    if (separatorIndex != -1) {
      int servoNumber = input.substring(0, separatorIndex).toInt();
      int angle = input.substring(separatorIndex + 1).toInt();
      
      if (servoNumber >= 0 && servoNumber < 32 && angle >= 0 && angle <= 180) {
        servos.move(servoNumber,angle);
        Serial.print("Servo ");
        Serial.print(servoNumber);
        Serial.print(" set to angle ");
        Serial.println(angle);
      } else {
        Serial.println("Invalid servo number or angle");
      }
    } else {
      Serial.println("Invalid input format");
    }
  }
  */
}
