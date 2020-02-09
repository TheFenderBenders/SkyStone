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

package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="New Robot Tele", group="Iterative Opmode")

public class SQT_TeleOp_Iterative extends OpMode
{
    DotStarBridgedLED blinkinLedDriver;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private  DcMotor leftBack = null;
    private  DcMotor rightBack = null;
    DcMotor leftIntake = null;
    DcMotor rightIntake = null;
    Servo grabber = null;
    DcMotor slide = null;
    Servo swing = null;
    ElapsedTime slideTimer = new ElapsedTime();
    ElapsedTime internalSlideTimer = new ElapsedTime();

    boolean slideMovement = false;
    ElapsedTime reverseTimer = new ElapsedTime();

    int leftStickReverse = 1;

    int lit = 0;
    @Override
    public void init() {
        //blinkinLedDriver = hardwareMap.get(DotStarBridgedLED.class, "driver");

        telemetry.addData("Status", "Initialized");

        leftIntake = hardwareMap.get(DcMotor.class, "lIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rIntake");
        grabber = hardwareMap.get(Servo.class, "grabber");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        slide = hardwareMap.get(DcMotor.class, "slide");
        swing = hardwareMap.get(Servo.class, "swing");




        // Most robots need the motor on one side to be reversed, to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
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
        reverseTimer.reset();
        slideTimer.reset();
        runtime.reset();
        internalSlideTimer.reset();



    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
/*
        //blinkinLedDriver.pixels[lit] = new DotStarBridgedLED.Pixel(255,150,0);
        if(lit<blinkinLedDriver.pixels.length){
            lit ++;
        }
        else{
            lit = 0;
        }

 */

        if(gamepad2.left_bumper){
            grabber.setPosition(0);
        }
        else if (gamepad2.right_bumper){
            grabber.setPosition(0.23);
        }

        if(gamepad2.left_stick_button) {
            slide.setPower(-gamepad2.left_stick_y*4);
        }
        else{
            slide.setPower(-gamepad2.left_stick_y*0.25);
        }
    //slide is negative because of way string was strung
        if(gamepad1.a){
            if(reverseTimer.milliseconds()>500){
                reverseMotors();
                reverseTimer.reset();
            }
        }


        if(gamepad2.x)swing.setPosition(0);
        if(gamepad2.b)swing.setPosition(0.666666);




      //ishan on intake
      //me on grabber

      if(gamepad1.right_trigger!=0){
            leftIntake.setPower(-gamepad1.right_trigger);
            rightIntake.setPower(gamepad1.right_trigger);
        }
        else if (gamepad1.left_trigger!=0){
            rightIntake.setPower(-gamepad1.left_trigger);
            leftIntake.setPower(gamepad1.left_trigger);
        }
        else{
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }




        if(gamepad2.right_trigger!=0){
            grabber.setPosition(grabber.getPosition()-0.01);
        }
        else if(gamepad2.left_trigger!=0){
            grabber.setPosition(grabber.getPosition()+0.01);
        }
        telemetry.addData("Grabber Position", grabber.getPosition());
        telemetry.update();




        if(gamepad2.a){

            if(slideTimer.milliseconds()>1000) {
                internalSlideTimer.reset();
                slide.setPower(0.35);
                slideMovement = true;
            }
        }
        if(slideMovement){
            if(slideTimer.milliseconds()>500){
                slide.setPower(0);
                if(swing.getPosition()==0){
                    swing.setPosition(0.666666);
                }
                else{
                    swing.setPosition(0);
                }
                slideMovement = false;
                slideTimer.reset();
            }
        }










        double gx = 0.25*leftStickReverse * gamepad1.left_stick_x;
        double gy = -0.25*leftStickReverse * gamepad1.left_stick_y;

        if(gamepad1.left_stick_button) {
             gx = 0.25*leftStickReverse * gamepad1.left_stick_x;
             gy = -0.25*leftStickReverse * gamepad1.left_stick_y;
        }
        else{
             gx = leftStickReverse * gamepad1.left_stick_x;
             gy = -leftStickReverse * gamepad1.left_stick_y;
        }


        double rx = gamepad1.right_stick_x;

        if(gamepad1.right_stick_button){
            rx = gamepad1.right_stick_x*0.25;
        }
        else{
            rx = gamepad1.right_stick_x;
        }

        double r = Math.hypot(gx, gy);


        //double robotAngle = Math.atan2(gy, gx) - Math.PI / 4;
        double robotAngle =(Math.atan2(gy, gx) - Math.PI / 4);
        robotAngle = Math.toDegrees(robotAngle);
        robotAngle = Math.round(robotAngle/8)*8;
        robotAngle = Math.toRadians(robotAngle);
            double rightX = rx;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;

            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftBack.setPower(v3);
            rightBack.setPower(v4);





        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }



    void reverseMotors(){
        leftStickReverse = - leftStickReverse;
    }


    class SlideThread implements Runnable{
        @Override
        public void run() {

        }
    }






}
