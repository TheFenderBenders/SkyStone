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

        one = new Point(150, 350);
        two = new Point(270, 350);

        skystoneCoOrdinates[0] = Vector2Weighted.createVector(28.5, 8);
        skystoneCoOrdinates[1] = Vector2Weighted.createVector(28.5, 0);
        skystoneCoOrdinates[2] = Vector2Weighted.createVector(28.5, -9);
        skystoneCoOrdinates[3] = Vector2Weighted.createVector(28.5, -17);
        skystoneCoOrdinates[4] = Vector2Weighted.createVector(28.5, -25);
        skystoneCoOrdinates[5] = Vector2Weighted.createVector(28.5, -33);

        frontOfStoneCoOrdinates[0] = Vector2Weighted.createVector(20, 8);
        frontOfStoneCoOrdinates[1] = Vector2Weighted.createVector(20, 0);
        frontOfStoneCoOrdinates[2] = Vector2Weighted.createVector(20, -9);
        frontOfStoneCoOrdinates[3] = Vector2Weighted.createVector(20, -17);
        frontOfStoneCoOrdinates[4] = Vector2Weighted.createVector(20, -25);
        frontOfStoneCoOrdinates[5] = Vector2Weighted.createVector(20, -33);

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