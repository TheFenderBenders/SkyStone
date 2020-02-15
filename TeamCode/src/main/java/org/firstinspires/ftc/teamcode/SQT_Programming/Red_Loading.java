package org.firstinspires.ftc.teamcode.SQT_Programming;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Red_Loading extends TFB_Auto{
    @Override
    public void initMethod() {
        starting_position = STARTING_POSITION.LOADING_ZONE;
        alliance = ALLIANCE.RED;
        super.initMethod();
    }

    @Override
    public void runOpMode() throws InterruptedException{
        this.initMethod();
        super.runOpMode();
    }



}
