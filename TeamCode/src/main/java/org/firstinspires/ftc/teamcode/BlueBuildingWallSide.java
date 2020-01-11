package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Blue-Building-Wall_side", group ="TFB Auto")
@Disabled
public class BlueBuildingWallSide extends TFB_Autonomous {
    @Override
    public void init() {
        // this is where all the right variables are set for the correct inherited code to run
        starting_position = STARTING_POSITION.BUILDING_ZONE;
        alliance = ALLIANCE.BLUE;
        under_bridge_position = UNDER_BRIDGE_POSITION.WALL_SIDE;
        runtime.reset();
        state = STATES.REPOSITION_FOUNDATION;
        foundation_state = FOUNDATION_STATES.STRAFE_LEFT;
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
