package org.firstinspires.ftc.teamcode.OldCode;

import org.firstinspires.ftc.teamcode.SQT_Programming.TFB_Auto;
import org.opencv.core.Point;

public class BlueLoadingWall extends TFB_Auto {

    @Override
    public void runOpMode() throws InterruptedException {

        // this is where all the right variables are set for the correct inherited code to run
        alliance = ALLIANCE.BLUE;
        starting_position = STARTING_POSITION.LOADING_ZONE;

        under_bridge_position = UNDER_BRIDGE_POSITION.NEUTRAL_SIDE;
        runtime.reset();

        state = STATES.FETCH_AND_DELIVER_SKYSTONES;
        skystone_state = SKYSTONE_STATES.FIRST_SKYSTONE;

        one = new Point(230, 350);
        two = new Point(320, 350);

        skystoneCoOrdinates[0] = Vector2Weighted.createVector(STONE_WALL_DISTANCE, 9);
        skystoneCoOrdinates[1] = Vector2Weighted.createVector(STONE_WALL_DISTANCE, 0);
        skystoneCoOrdinates[2] = Vector2Weighted.createVector(STONE_WALL_DISTANCE, -8);
        skystoneCoOrdinates[3] = Vector2Weighted.createVector(STONE_WALL_DISTANCE, -16);
        skystoneCoOrdinates[4] = Vector2Weighted.createVector(STONE_WALL_DISTANCE, -25);
        skystoneCoOrdinates[5] = Vector2Weighted.createVector(STONE_WALL_DISTANCE, -33);

        frontOfStoneCoOrdinates[0] = Vector2Weighted.createVector(STONE_WALL_DISTANCE - BACKUP_DISTANCE, 9);
        frontOfStoneCoOrdinates[1] = Vector2Weighted.createVector(STONE_WALL_DISTANCE - BACKUP_DISTANCE, 2);
        frontOfStoneCoOrdinates[2] = Vector2Weighted.createVector(STONE_WALL_DISTANCE - BACKUP_DISTANCE, -10);
        frontOfStoneCoOrdinates[3] = Vector2Weighted.createVector(STONE_WALL_DISTANCE - BACKUP_DISTANCE, -18);
        frontOfStoneCoOrdinates[4] = Vector2Weighted.createVector(STONE_WALL_DISTANCE - BACKUP_DISTANCE, -25);
        frontOfStoneCoOrdinates[5] = Vector2Weighted.createVector(STONE_WALL_DISTANCE - BACKUP_DISTANCE, -33);

        buildingZoneSkystoneDropOff = Vector2Weighted.createVector(20, 55);

        super.initMethod();
        super.runOpMode();

    }

}
