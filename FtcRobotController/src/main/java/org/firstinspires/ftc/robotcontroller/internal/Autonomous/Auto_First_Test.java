/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.internal.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Test", group="Linear Opmode")
@Disabled
public class Auto_First_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private  DcMotor backLeft = null;
    private  DcMotor backRight = null;

    CRServo slideServo = null;
    CRServo armServo = null;
    CRServo handServo = null;
    Servo leftFoundServo = null;
    Servo rightFoundServo = null;
    CRServo skystoneServo = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontRight  = hardwareMap.get(DcMotor.class, "front_left");
        frontLeft = hardwareMap.get(DcMotor.class, "front_right");
        backRight = hardwareMap.get(DcMotor.class, "back_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_right");


        slideServo = hardwareMap.get(CRServo.class,"slide");
        armServo = hardwareMap.get(CRServo .class, "arm");
        handServo = hardwareMap.get(CRServo.class, "hand");
        leftFoundServo = hardwareMap.get(Servo.class,"l_foundation");
        rightFoundServo = hardwareMap.get(Servo.class,"r_foundation");
        skystoneServo = hardwareMap.get(CRServo.class, "skystone");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);


        leftFoundServo.setPosition(-1);//POSITIVE is clockwise looking at the shaft
        rightFoundServo.setPosition(-1);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        final int FORWARD = 0;
        final int SERVO_DOWN = 1;
        final int BACK_OUT = 2;
        final int STRAFE = 3;
        final int SERVO_UP = 4;
        final int COMPLETE = -1;


        int currentState = FORWARD;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            switch (currentState) {


                case FORWARD:
                    move(1,0,0.3);
                    sleep(1300);
                    cutPower();

                    move(0, 1, 0.3);
                    sleep(3000);
                    cutPower();
                    currentState = SERVO_DOWN;
                    break;
                case SERVO_DOWN:
                    leftFoundServo.setPosition(leftFoundServo.getPosition() + 0.3);
                    rightFoundServo.setPosition(rightFoundServo.getPosition() + 0.3);
                    sleep(1000);
                    cutPower();
                    currentState = BACK_OUT;
                    break;

                case BACK_OUT:
                    move(0, -1, 0.4);
                    sleep(5000);
                    cutPower();
                    currentState = SERVO_UP;
                    break;


                case SERVO_UP:
                    leftFoundServo.setPosition(leftFoundServo.getPosition() - 0.3);
                    rightFoundServo.setPosition(rightFoundServo.getPosition() - 0.3);
                    sleep(1000);
                    cutPower();
                    currentState = STRAFE;
                    break;


                case STRAFE:

                    move(1,0,-0.4);
                    sleep(2600 );
                    cutPower();
                    move(0,1,0.3);
                    sleep(1300);
                    cutPower();

                    move(1,0,-0.4);
                    sleep(1400);
                    cutPower();
                    currentState = COMPLETE;
                    break;


                case COMPLETE:
                    telemetry.addLine("The Program Has Terminated");

                    break;

            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
    }

    void move(double x, double y, double speed){








        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        double v1 = r * Math.cos(robotAngle);
        double v2 = r * Math.sin(robotAngle);
        double v3 = r * Math.sin(robotAngle);
        double v4 = r * Math.cos(robotAngle);
        frontLeft.setPower(v1*speed);
        frontRight.setPower(v2*speed);
        backLeft.setPower(v3*speed);
        backRight.setPower(v4*speed);
    }

    void cutPower(){

        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        skystoneServo.setPower(0);

    }




}