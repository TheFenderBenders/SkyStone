package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red-Loading-Neutral_side", group ="TFB Auto")
public class RedLoadingNeutralSide extends TFB_Autonomous {
    @Override
    public void init() {
        runtime.reset();
        // this is where all the right variables are set for the correct inherited code to run
        starting_position = STARTING_POSITION.LOADING_ZONE;
        alliance = ALLIANCE.RED;
        under_bridge_position = UNDER_BRIDGE_POSITION.NEUTRAL_SIDE;
        state = STATES.FETCH_AND_DELIVER_SKYSTONE;
        skystone_state = SKYSTONE_STATES.MOVE_TOWARD_STONE_LINE;
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
