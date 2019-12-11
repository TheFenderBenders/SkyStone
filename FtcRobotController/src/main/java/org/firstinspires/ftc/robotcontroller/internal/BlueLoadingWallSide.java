package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue-Loading-Wall_side", group ="TFB Auto")
public class BlueLoadingWallSide extends TFB_Autonomous {
    @Override
    public void init() {
        super.init();

        // this is where all the right variables are set for the correct inherited code to run
        starting_position = STARTING_POSITION.LOADING_ZONE;
        alliance = ALLIANCE.BLUE;
        under_bridge_position = UNDER_BRIDGE_POSITION.WALL_SIDE;

        state = STATES.FETCH_AND_DELIVER_SKYSTONE;
        skystone_state = SKYSTONE_STATES.MOVE_TOWARD_STONE_LINE;
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
