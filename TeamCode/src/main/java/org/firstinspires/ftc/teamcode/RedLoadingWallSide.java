package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red-Loading-Wall_side", group ="TFB Auto")
public class RedLoadingWallSide extends TFB_Autonomous {
    @Override
    public void init() {
        runtime.reset();
        // this is where all the right variables are set for the correct inherited code to run
        starting_position = STARTING_POSITION.LOADING_ZONE;
        alliance = ALLIANCE.RED;
        under_bridge_position = UNDER_BRIDGE_POSITION.WALL_SIDE;
        state = STATES.FETCH_AND_DELIVER_SKYSTONES;
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
