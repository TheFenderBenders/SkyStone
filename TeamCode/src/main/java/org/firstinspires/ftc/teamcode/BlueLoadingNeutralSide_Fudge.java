package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Point;

@Autonomous(name="BLN Fudge", group ="TFB Auto")
@Disabled
public class BlueLoadingNeutralSide_Fudge extends TFB_Autonomous {
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

        skystoneCoOrdinates[0] = Vector2Weighted.createVector(28.5, 9);
        skystoneCoOrdinates[1] = Vector2Weighted.createVector(28.5, 20);
        skystoneCoOrdinates[2] = Vector2Weighted.createVector(28.5, -9);
        skystoneCoOrdinates[3] = Vector2Weighted.createVector(28.5, -18);
        skystoneCoOrdinates[4] = Vector2Weighted.createVector(28.5, -25);
        skystoneCoOrdinates[5] = Vector2Weighted.createVector(28.5, -34);

        frontOfStoneCoOrdinates[0] = Vector2Weighted.createVector(20, 10);
        frontOfStoneCoOrdinates[1] = Vector2Weighted.createVector(20, 20);
        frontOfStoneCoOrdinates[2] = Vector2Weighted.createVector(20, -11);
        frontOfStoneCoOrdinates[3] = Vector2Weighted.createVector(20, -18);
        frontOfStoneCoOrdinates[4] = Vector2Weighted.createVector(20, -25);
        frontOfStoneCoOrdinates[5] = Vector2Weighted.createVector(20, -34);

        buildingZoneSkystoneDropOff = Vector2Weighted.createVector(20, 55);

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
