package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name="Blue-Building-Neutral_side", group ="TFB Auto")
public class BlueBuildingNeutralSide extends TFB_Autonomous {
    @Override
    public void init() {

        // this is where all the right variables are set for the correct inherited code to run
        starting_position = STARTING_POSITION.BUILDING_ZONE;
        alliance = ALLIANCE.BLUE;
        skystoneServo = skystoneServoRight;
        under_bridge_position = UNDER_BRIDGE_POSITION.NEUTRAL_SIDE;
        runtime.reset();
        state = STATES.REPOSITION_FOUNDATION;
        foundation_state = FOUNDATION_STATES.STRAFE_LEFT;

        one = new Point(300, 300);
        two = new Point(180, 300);

        skystoneCoOrdinates[0] = Vector2Weighted.createVector(28.5, 8);
        skystoneCoOrdinates[1] = Vector2Weighted.createVector(28.5, 0);
        skystoneCoOrdinates[2] = Vector2Weighted.createVector(28.5, -7.5);
        skystoneCoOrdinates[3] = Vector2Weighted.createVector(28.5, -13.5);
        skystoneCoOrdinates[4] = Vector2Weighted.createVector(28.5, -22);
        skystoneCoOrdinates[5] = Vector2Weighted.createVector(28.5, -30);

        buildingZoneSkystoneDropOff = Vector2Weighted.createVector(13.5, 50);
        skystoneFirstSetReferencePoint = Vector2Weighted.createVector(13.5, 0);
        skystoneSecondSetReferencePoint = Vector2Weighted.createVector(13.5, -24);

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
    public void loop() {
        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
