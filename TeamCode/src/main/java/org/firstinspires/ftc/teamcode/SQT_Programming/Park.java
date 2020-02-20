package org.firstinspires.ftc.teamcode.SQT_Programming;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.RoadRunnerAdditions;

@Autonomous(group = "A")
public class Park extends LinearOpMode {
    RoadRunnerAdditions roadrunner = new RoadRunnerAdditions();
    @Override
    public void runOpMode(){

        roadrunner.initRobot(hardwareMap);
        waitForStart();

        roadrunner.move(new Vector2d[]{new Vector2d(-10,0)});
        stop();
    }
}