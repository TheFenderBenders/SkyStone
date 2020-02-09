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

        while(opModeIsActive()){
            //roadrunner.move can be used in 3 ways:
            // Inputting an x and y coordinate(the position you want to move to)
            //Inputting a Vector2d of the point to want to move to
            //Inputting an array of Vector2d points. In this case, it will move one by one to each point. It will Decelerate
            roadrunner.move(10,10);

            roadrunner.move(new Vector2d(10,10));

            Vector2d[] path = new Vector2d[]{
              new Vector2d(10,10),
              new Vector2d(20,20)
            };
            roadrunner.move(path);

            roadrunner.turn(90);

            roadrunner.setMotorPowers(0.1,0.1,0.3,0.6);

            Vector2d pointA = new Vector2d(10,10);
            Vector2d pointB = new Vector2d(20,10);
            roadrunner.fastMove(pointA,pointB);
            // A Fast move will go to point A and then point B. However, It will NOT decelerate. It will wait 0.5 seconds but
            // will not slow down.
            stop();
        }
    }
}