package org.firstinspires.ftc.teamcode.SQT_Programming;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class TFB_LinearOpMode extends LinearOpMode {
    protected boolean redSide;

    protected ElapsedTime runtime = new ElapsedTime();
    protected ColorSensor colorSensor = null;
    protected Servo leftSkystoneArm, rightSkystoneArm,leftSkystoneHand, rightSkystoneHand;
    protected Servo foundationCaptureServoLeft, foundationCaptureServoRight;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    void initMethod(){
        if(redSide){
            // LEFT hand clamp is 0.04
// and unclamp is 0.5

/*
for larm, down is 0.4 and up is 0.8
*/
        }

        foundationCaptureServoLeft = hardwareMap.get(Servo.class, "l_found");
        foundationCaptureServoLeft.setPosition(0.6);
        foundationCaptureServoRight = hardwareMap.get(Servo.class, "r_found");
        foundationCaptureServoRight.setPosition(0.3);
        //colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        if(redSide) {
            leftSkystoneArm = hardwareMap.get(Servo.class, "left_sa");

            leftSkystoneHand = hardwareMap.get(Servo.class, "left_sh");
        }
        else {

            rightSkystoneArm = hardwareMap.get(Servo.class, "right_sa");
            rightSkystoneArm.setPosition(0);

            rightSkystoneHand = hardwareMap.get(Servo.class, "right_sh");
            rightSkystoneHand.setPosition(0.0);
        }

    }
}
