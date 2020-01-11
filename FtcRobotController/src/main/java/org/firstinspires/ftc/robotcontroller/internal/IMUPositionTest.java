// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static java.lang.Thread.sleep;

@Autonomous(name="Drive Avoid Imu", group="Exercises")
@Disabled

public class IMUPositionTest extends LinearOpMode
{


    BNO055IMU.Parameters imuparameters;

    BNO055IMU imu;
    BNO055IMU imu2;
    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry.addLine("THREAD STARTED");
        telemetry.update();

        // IMU Initialization
        imuparameters = new BNO055IMU.Parameters();

        imuparameters.mode                = BNO055IMU.SensorMode.IMU;
        imuparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuparameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu hub 1");

        imu.initialize(imuparameters);

        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);


            sleep(100); //Changing modes requires a delay before doing anything else


        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);

        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);

        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

            sleep(100); //Changing modes again requires a delay

        // make sure the imu gyro is calibrated before continuing.

            while (!imu.isGyroCalibrated())
            {
                sleep(50);
            }















        telemetry.addLine("THREAD STARTED");
        telemetry.update();

        // IMU Initialization
        imuparameters = new BNO055IMU.Parameters();

        imuparameters.mode                = BNO055IMU.SensorMode.IMU;
        imuparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuparameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu2 = hardwareMap.get(BNO055IMU.class, "imu hub 2");

        imu2.initialize(imuparameters);

         AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
         AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        //Need to be in CONFIG mode to write to registers
        imu2.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);


        sleep(100); //Changing modes requires a delay before doing anything else


        //Write to the AXIS_MAP_CONFIG register
        imu2.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);

        //Write to the AXIS_MAP_SIGN register
        imu2.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);

        //Need to change back into the IMU mode to use the gyro
        imu2.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        sleep(100); //Changing modes again requires a delay

        // make sure the imu gyro is calibrated before continuing.

        while (!imu2.isGyroCalibrated())
        {
            sleep(50);
        }



        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.

        telemetry.update();

        // wait for start button.

        waitForStart();

        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // drive until end of period.

        while (opModeIsActive())
        {
            // Use gyro to drive in a straight line.
            telemetry.addData("Avg Accel", (imu.getAcceleration().xAccel+imu2.getAcceleration().xAccel)/2);
            telemetry.addData("A Accel", (imu.getAcceleration().xAccel));
            //telemetry.addData("VEL", imu.getVelocity().yVeloc);
            //telemetry.addData("IMU POS", imu.getPosition().x);
            telemetry.update();

        }

        // turn the motors off.
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */


}