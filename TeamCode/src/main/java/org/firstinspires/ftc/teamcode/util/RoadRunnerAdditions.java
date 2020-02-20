package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;

public class RoadRunnerAdditions extends LinearOpMode {
    SampleMecanumDriveBase drive;

    double yScale = 1;
    double xScale = 1;
    public void runOpMode(){

    }
    public void initRobot(HardwareMap h){
        drive = new SampleMecanumDriveREVOptimized(h);
        yScale = 1.075;
    }
    public void setScale(double x, double y){
        xScale = x;
        yScale = y;
    }



    public void flipSides(){
        xScale *= -1;
        yScale =1.075;
    }


    public void move(Vector2d point){
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(point).build());
    }
    public void move(Vector2d[] points){


        for(int i = 0; i<points.length; i++){
            Vector2d point = new Vector2d(points[i].getX()*xScale, points[i].getY()*yScale);
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(point).build());
        }
    }


    public void setPosition(int x, int y){
        drive.setPoseEstimate(new Pose2d(x*xScale,y*yScale));
    }

    public void turn(double angle){
        drive.turnSync(Math.toRadians(angle));
    }

    public boolean busy(){
        return drive.isBusy();
    }
    public void setMotorPowers(double a, double b, double c, double d){drive.setMotorPowers(a,b,c,d);}

    public void fastMove(Vector2d pointA, Vector2d pointB){
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(new Vector2d(pointA.getX()*xScale, pointA.getY()*yScale)).
                addMarker(pointA, () -> {
                   sleep(250);

                    return Unit.INSTANCE;
                })
                .strafeTo(new Vector2d(pointB.getX()*xScale,pointB.getY()*yScale)).build());
    }

}
