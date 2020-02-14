package org.firstinspires.ftc.teamcode.SQT_Programming;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class TFB_LinearOpMode extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime();
    protected ColorSensor colorSensor = null;
    protected Servo leftSkystoneArm, rightSkystoneArm,leftSkystoneHand, rightSkystoneHand;
    protected Servo foundationCaptureServoLeft, foundationCaptureServoRight;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    void initMethod(){
        foundationCaptureServoLeft = hardwareMap.get(Servo.class, "l_found");
        foundationCaptureServoLeft.setPosition(0.6);
        foundationCaptureServoRight = hardwareMap.get(Servo.class, "r_found");
        foundationCaptureServoRight.setPosition(0.3);
        //colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        rightSkystoneArm = hardwareMap.get(Servo.class, "right_sa");
        rightSkystoneArm.setPosition(0);
        //rightSkystoneArm = hardwareMap.get(Servo.class,"right_sa");

        rightSkystoneHand = hardwareMap.get(Servo.class, "right_sh");
        rightSkystoneHand.setPosition(0.0);
        //rightSkystoneHand = hardwareMap.get(Servo.class,"right_sh");

    }
}
