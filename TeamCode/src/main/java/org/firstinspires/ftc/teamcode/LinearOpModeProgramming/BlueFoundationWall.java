package org.firstinspires.ftc.teamcode.LinearOpModeProgramming;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Vector2Weighted;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Autonomous(name= "Blue Foundation Wall",group = "drive")
public class BlueFoundationWall extends LinearOpMode {

     SampleMecanumDriveBase drive;
     Servo leftFoundation;
     Servo rightFoundation;

     boolean foundBridge = false;
     ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {


        leftFoundation = hardwareMap.get(Servo.class, "l_foundation");
        rightFoundation = hardwareMap.get(Servo.class, "r_foundation");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);


        leftFoundation.setPosition(0);
        rightFoundation.setPosition(0);
        waitForStart();



        while (opModeIsActive()){

            drive.followTrajectorySync(drive.trajectoryBuilder().strafeRight(12).build());
            drive.followTrajectorySync(drive.trajectoryBuilder().back(40).build());
            leftFoundation.setPosition(0.35);
            rightFoundation.setPosition(0.35);
            sleep(750);
            drive.followTrajectorySync(drive.trajectoryBuilder().forward(40).build());
            leftFoundation.setPosition(0);
            rightFoundation.setPosition(0);
            sleep(750);
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeLeft(25).build());
            while(!foundBridge){
                moveToStonesThroughBridge();
            }
                drive.setMotorPowers(0,0,0,0);
            break;


        }



    }









    void moveToStonesThroughBridge() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if((blue>red)&&(blue>green)){
            telemetry.addLine("Blue Object Detected");
            foundBridge = true;
        }
        else if((red>blue)&&(red>green)){
            telemetry.addLine("Red Object Detected");
            foundBridge = true;
        }
        else{

                drive.setMotorPowers(-0.4, 0.4, -0.4, 0.4);

        }

    }


}




