package org.firstinspires.ftc.teamcode.OldCode.LinearOpModeProgramming;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OldCode.Vector2Weighted;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Autonomous(name= "MoveTest2",group = "drive")
public class StrafeTest extends LinearOpMode {

    Vector2Weighted v = new Vector2Weighted();


    public ElapsedTime runtime = new ElapsedTime();



     SampleMecanumDriveBase drive;

    @Override
    public void runOpMode() throws InterruptedException {




        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

/*
        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

 */

        // drive.trajectoryBuilder().
/*
        Trajectory tr = drive.trajectoryBuilder().
                strafeTo(v.createVector(10,10)).

                lineTo(v.createVector(50,50))
                .addMarker(2.0,() -> {
                    servo.setPosition(1);

                    return Unit.INSTANCE;
                })



                addMarker(), () -> {

            //Do stuff
            return Unit.INSTANCE;
        })


        .build();

        */

        //drive.trajectoryBuilder(new Vector2d(0,0)).addMarker(()->{



        //   return Unit.INSTANCE;});

        //Trajectory trajectory = drive.trajectoryBuilder().lineTo(vector2d).build();
        // Trajectory tr = drive.trajectoryBuilder().strafeTo(vector2d).build();




      /*  skystoneServo.setPosition(0);
        waitForStart();
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(28,-17)).build());
        skystoneServo.setPosition(0.35);
        sleep(500);
        drive.followTrajectorySync(drive.trajectoryBuilder().back(12).build());
        // drive.followTrajectorySync(drive.trajectoryBuilder().strafeLeft(60).build());
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13,35)).build());
        skystoneServo.setPosition(0);
        //the 24 is the second stone location -17
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13,-41)).build());


        //SS5
        skystoneServo.setPosition(0);
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(28,-39)).build());
        skystoneServo.setPosition(0.35);
        sleep(500);
        drive.followTrajectorySync(drive.trajectoryBuilder().back(12).build());
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13,35)).build());
        skystoneServo.setPosition(0);
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeRight(16).build());
        //drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13,0)).build());


       */

        waitForStart();

        while (opModeIsActive()){
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(Vector2Weighted.createVector(20,0)).build());
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(Vector2Weighted.createVector(0,0)).build());

            //drive.getPoseEstimate();
        }



    }





}




