package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;

public class RoadRunnerAdditions extends LinearOpMode {
    SampleMecanumDriveBase drive;
    double xScale = 1;
    double yScale = 1;
    //The Scale Factors are meant to be used if the robot does not move the right amount but still moves in a linear amount.
    public void runOpMode(){

    }
    public void initRobot(HardwareMap h){
        drive = new SampleMecanumDriveREVOptimized(h);
    }

    public void move(Vector2d[] points){
        for(Vector2d point:points){
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(point).build());
        }
    }

    public void move(Vector2d point){
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(point).build());

    }

    public void move(double x, double y){
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(new Vector2d(x,y)).build());
    }



    public void setPosition(int x, int y){
        drive.setPoseEstimate(new Pose2d(x,y));
    }

    public void turn(double angle){
        drive.turnSync(Math.toRadians(angle));
    }
    public boolean busy(){
        return drive.isBusy();
    }
    public void setMotorPowers(double a, double b, double c, double d){drive.setMotorPowers(a,b,c,d);}

    public void fastMove(Vector2d pointA, Vector2d pointB){
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(pointA).
                addMarker(pointA, () -> {
                   sleep(250);

                    return Unit.INSTANCE;
                })
                .strafeTo(pointB).build());
    }

    public void fastMove(double pointAX, double pointAY, double pointBX, double pointBY){
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(new Vector2d(pointAX,pointAY)).
                addMarker(new Vector2d(pointAX,pointAY), () -> {
                    sleep(250);

                    return Unit.INSTANCE;
                })
                .strafeTo(new Vector2d(pointBX,pointBY)).build());
    }


}
