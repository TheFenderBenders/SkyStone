package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

import static java.lang.Thread.sleep;

@Autonomous(name="Blue-Building-Neutral_side_Test", group ="TFB Auto")
public class BlueBuildingNeutralSide extends TFB_Autonomous {
    @Override
    public void init() {

        // this is where all the right variables are set for the correct inherited code to run
        alliance = ALLIANCE.BLUE;
        starting_position = STARTING_POSITION.LOADING_ZONE;

        under_bridge_position = UNDER_BRIDGE_POSITION.NEUTRAL_SIDE;
        runtime.reset();

        state = STATES.FETCH_AND_DELIVER_SKYSTONES;
        skystone_state = SKYSTONE_STATES.FIRST_SKYSTONE;

        one = new Point(230, 350);
        two = new Point(320, 350);

        skystoneCoOrdinates[0] = Vector2Weighted.createVector(28.5, 8);
        skystoneCoOrdinates[1] = Vector2Weighted.createVector(28.5, 0);
        skystoneCoOrdinates[2] = Vector2Weighted.createVector(28.5, -12);
        skystoneCoOrdinates[3] = Vector2Weighted.createVector(28.5, -20);
        skystoneCoOrdinates[4] = Vector2Weighted.createVector(28.5, -27);
        skystoneCoOrdinates[5] = Vector2Weighted.createVector(28.5, -37);

        frontOfStoneCoOrdinates[0] = Vector2Weighted.createVector(20, 8);
        frontOfStoneCoOrdinates[1] = Vector2Weighted.createVector(20, 0);
        frontOfStoneCoOrdinates[2] = Vector2Weighted.createVector(20, -12);
        frontOfStoneCoOrdinates[3] = Vector2Weighted.createVector(20, -20);
        frontOfStoneCoOrdinates[4] = Vector2Weighted.createVector(20, -27);
        frontOfStoneCoOrdinates[5] = Vector2Weighted.createVector(20, -37);

        buildingZoneSkystoneDropOff = Vector2Weighted.createVector(20, 50);

        super.init();

    }

    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
        super.start();
    }


    @Override
    public void loop()  {
        super.loop();
    }


    @Override
    public void stop(){

    }
}
