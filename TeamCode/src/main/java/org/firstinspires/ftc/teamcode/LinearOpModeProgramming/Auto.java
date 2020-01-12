package org.firstinspires.ftc.teamcode.LinearOpModeProgramming;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LinearOpModeProgramming.*;
import org.firstinspires.ftc.teamcode.R;

public class Auto extends LinearOpMode {
    public void runOpMode(){
        int currentHover = 0;
        String[] lines = new String[10];

        for(int i = 0; i<lines.length; i++) {
            if (currentHover == i) lines[i] = ">" + lines[i];
        }


        BlueFoundationNeutral blueFoundationNeutral = new BlueFoundationNeutral();
        BlueFoundationWall blueFoundationWall = new BlueFoundationWall();
        BlueLoadingWall blueLoadingWall = new BlueLoadingWall();

        RedFoundationNeutral redFoundationNeutral = new RedFoundationNeutral();
        RedFoundationWall redFoundationWall = new RedFoundationWall();
        RedLoadingNeutral redLoadingNeutral = new RedLoadingNeutral();

        // Menu System

        boolean runOnce = false;
        boolean side;

        /*
        Loading vs Building
        Red vs Blue
        Close vs Far
        (int) num of stones
        delay

         */




        waitForStart();
    }
}
