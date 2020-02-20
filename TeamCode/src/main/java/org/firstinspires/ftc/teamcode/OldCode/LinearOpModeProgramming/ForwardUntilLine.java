package org.firstinspires.ftc.teamcode.OldCode.LinearOpModeProgramming;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Disabled

@Autonomous(name= "Forward Until Line",group = "drive")
public class ForwardUntilLine extends LinearOpMode {


    public ElapsedTime runtime = new ElapsedTime();

    boolean foundBridge = false;

    SampleMecanumDriveBase drive;
    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

      colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");



        waitForStart();



        while (opModeIsActive()){
            while(!foundBridge) {
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

            drive.setMotorPowers(-0.4, -0.4, -0.4, -0.4);

        }

    }


}




