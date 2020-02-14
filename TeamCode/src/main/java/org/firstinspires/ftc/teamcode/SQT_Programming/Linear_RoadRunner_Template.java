package org.firstinspires.ftc.teamcode.SQT_Programming;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.RoadRunnerAdditions;

@Autonomous
public class Linear_RoadRunner_Template extends LinearOpMode {
    RoadRunnerAdditions roadrunner = new RoadRunnerAdditions();
    @Override
    public void runOpMode(){

        roadrunner.initRobot(hardwareMap);
        waitForStart();
// Split
        Vector2d[] path0 = new Vector2d[]{
                new Vector2d(10,0)};
        roadrunner.move(path0);

        stop();
    }
}