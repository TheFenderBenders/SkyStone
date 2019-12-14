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

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="LM3 Iterative TeleOp", group="Iterative Opmode")

public class LM3_TeleOp_Iterative extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private  DcMotor leftBack = null;
    private  DcMotor rightBack = null;

    CRServo slideServo = null;
    CRServo armServo = null;
    CRServo handServo = null;
    Servo leftFoundServo = null;
    Servo rightFoundServo = null;
    Servo skystoneServo = null;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "front_left");
        rightFront = hardwareMap.get(DcMotor.class, "front_right");
        leftBack = hardwareMap.get(DcMotor.class, "back_left");
        rightBack = hardwareMap.get(DcMotor.class, "back_right");


        slideServo = hardwareMap.get(CRServo.class,"slide");
        armServo = hardwareMap.get(CRServo.class, "arm");
        handServo = hardwareMap.get(CRServo.class, "hand");
        leftFoundServo = hardwareMap.get(Servo.class,"l_foundation");
        rightFoundServo = hardwareMap.get(Servo.class,"r_foundation");
        skystoneServo = hardwareMap.get(Servo.class, "skystone");




        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        //rightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //GAMEPAD # 2 CONTROLS  This includes The Linear Slide, the Arm, and the Foundation Servos.


        if(gamepad2.left_stick_button){
            slideServo.setPower(Range.clip(-gamepad2.left_stick_y/2,-0.5,0.75));
        }
        else{
            slideServo.setPower(Range.clip(-gamepad2.left_stick_y,-0.5,0.75));
        }

        if(gamepad2.a) {

            rightFoundServo.setPosition(rightFoundServo.getPosition() + 0.01);
            leftFoundServo.setPosition(leftFoundServo.getPosition()+0.01);

        }

        if(gamepad2.y){

            rightFoundServo.setPosition(rightFoundServo.getPosition() - 0.01);
            leftFoundServo.setPosition(leftFoundServo.getPosition() -0.01);
        }

        if(gamepad2.x){
            rightFoundServo.setPosition(0);
            leftFoundServo.setPosition(0);
        }

        armServo.setPower(gamepad2.right_stick_y);


//GAMEPAD # 1 CONTROLS:This includes Movement(Strafing), the hand movement, and the skystone Servo


        if(gamepad1.a){
            skystoneServo.setPosition(0.35);
        }
        else if(gamepad1.y){
            skystoneServo.setPosition(0);
        }


        if(gamepad1.right_bumper){
            handServo.setPower(0.5);
        }
        else if(gamepad1.left_bumper){
            handServo.setPower(-0.5);
        }
        else{
            handServo.setPower(0);
        }



        double gx = -gamepad1.left_stick_x*0.75;
        double gy = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x/2;

        double r = Math.hypot(gx, gy);
        double robotAngle = Math.atan2(gy, gx) - Math.PI / 4;
        double rightX = rx;
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;



        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftBack.setPower(v3);
        rightBack.setPower(v4);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("v1", v1);
        telemetry.addData("v2", v2);
        telemetry.addData("v3", v3);
        telemetry.addData("v4", v4);

        telemetry.addData("Arm", armServo.getPortNumber());
        telemetry.addData("hand", handServo.getPortNumber());
        telemetry.addData("slide", slideServo.getPortNumber());
        telemetry.addData("skystone", skystoneServo.getPortNumber());
        telemetry.addData("lF", leftFoundServo.getPortNumber());
        telemetry.addData("rF", rightFoundServo.getPortNumber());

        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
