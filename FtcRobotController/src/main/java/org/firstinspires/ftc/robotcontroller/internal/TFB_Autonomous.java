package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcontroller.internal.Autonomous.Skystone_Auto;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class TFB_Autonomous extends TFB_OpMode {

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30;
    protected double correction;
    protected double distanceFromSkystone = 100000;

    enum DIRECTION {
        LEFT, RIGHT
    }
    DIRECTION dir;


    enum STARTING_POSITION {
        LOADING_ZONE,
        BUILDING_ZONE
    }

    protected enum ALLIANCE {
        RED,
        BLUE
    }

    enum UNDER_BRIDGE_POSITION {
        WALL_SIDE,
        NEUTRAL_SIDE
    }

    protected ALLIANCE alliance;
    protected STARTING_POSITION starting_position;
    protected UNDER_BRIDGE_POSITION under_bridge_position;

    protected SKYSTONE_STATES skystone_states;
    protected FOUNDATION_STATES foundation_states;
    protected PARK_STATES park_states;

    protected STATES state;
    protected SKYSTONE_STATES skystone_state;
    protected FOUNDATION_STATES foundation_state;
    protected PARK_STATES park_state;

    enum STATES {
        FETCH_AND_DELIVER_SKYSTONE,
        PARK_UNDER_BRIDGE,
        REPOSITION_FOUNDATION,
        DONE
    }

    enum SKYSTONE_STATES {
        MOVE_TOWARD_STONE_LINE,
        STRAFE_UNTIL_SKYSTONE,
        MOVE_TOWARD_SKYSTONE,
        CAPTURE_SKYSTONE,
        BACK_OUT,
        TURN_TOWARD_BRIDGE,
        STRAFE_BASED_ON_PARKING_POSITION,
        MOVE_TO_BRIDGE,
        DROP_OFF_SKYSTONE,
    }

    enum FOUNDATION_STATES {
        MOVE_TOWARD_FOUNDATION,
        LATCH_ARM,
        BRING_FOUNDATION_BACK,
    }

    enum PARK_STATES {
        STRAFE,
        MOVE_TO_BRIDGE
    }

    protected static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    protected static final boolean PHONE_IS_PORTRAIT = true  ;
    protected static final String VUFORIA_KEY =
            "AW8pxAb/////AAABmc2pCnd0uUidheyLY5krCRBcvgnlBqrElE/ZP/pTLqZoxVQ8COgVDVpCp0pOmtF6HP9kyk7kh9Qjq0A6ND0F7A0iemGwWN2RxixFEOSWiDrSbc46XnYYpF+qCAkHx2w2e4tvJD4REtBPVTd7URXPnMEKqJde9cWVQc6D9gOFAa42CnkYsuvJZ2Kn2Lc51kuqyJ0szGwPjZUsA5vZ1vENH75y2tuym8jY4oRl2BYsmehEotnxApXt/6D+gdYsb7cGAyZuHxXx00zp+gGTlnrhJdEx9DnQVjI2HLBi6j848ayI200c8jCVqiVtv+NExtP3NCDY66YGGKp+so0pJ7MRUWYrJ7+4n4kGKg59erl+UIjO";
    protected static final float mmPerInch        = 25.4f;
    protected static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    protected static final float stoneZ = 2.00f * mmPerInch;
    protected OpenGLMatrix lastLocation = null;
    protected VuforiaLocalizer vuforia = null;
    protected VuforiaTrackables targetsSkyStone;
    protected VuforiaTrackable stoneTarget;
    protected List<VuforiaTrackable> allTrackables;
    protected VuforiaLocalizer.Parameters vuparameters;
    protected boolean targetVisible = false;
    protected float phoneXRotate    = 0;
    protected float phoneYRotate    = 0;
    protected float phoneZRotate    = 0;
    protected OpenGLMatrix robotLocationTransform;
    protected int cameraMonitorViewId;
    protected BNO055IMU.Parameters imuparameters;
    protected boolean IMU_initialized = false;

    @Override
    public void init() {

        // check for common initialiazation combinations that aren't implemented
        if ((starting_position == STARTING_POSITION.BUILDING_ZONE) && (state == STATES.FETCH_AND_DELIVER_SKYSTONE) ||
                (starting_position == STARTING_POSITION.LOADING_ZONE) && (state == STATES.REPOSITION_FOUNDATION)
        ) {
            String Error = "INCORRECT STATE COMBO. CHECK YOUR INIT. Your Starting Position Was: "
                    + starting_position + "But your State Was: " + state;

            telemetry.addLine(Error);
            telemetry.update();
            try {
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                telemetry.addLine("Sleep exception");
                telemetry.update();
            }
            state = STATES.DONE;
            return;
        }

        super.init();


        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        vuparameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuparameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(vuparameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, vuparameters.cameraDirection);
        }

        telemetry.addData("Status", "Vuforia Init done");
        telemetry.addLine("Calibrating IMU");
        telemetry.update();

        Thread t1 = new Thread(new IMULoader());
        t1.start();

/*        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

 */
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        super.start();
        //targetsSkyStone.activate();
        runtime.reset();
    }

    @Override
    public void loop() {

        telemetry.update();
        if (!IMU_initialized) {
            telemetry.addLine("IMU not initialized...PLEASE WAIT!");
            telemetry.update();
            return;
        }

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addLine("IMU initialized!");
        telemetry.update();

        switch (state) {
            case FETCH_AND_DELIVER_SKYSTONE:

                switch (skystone_state) {
                    case MOVE_TOWARD_STONE_LINE:
                        correction = checkDirection();
                        telemetry.addData("IMU Correction", correction);
                        telemetry.update();

                        if (runtime.milliseconds() < 4000) {
                            // move forward for 4 seconds
                            moveByGyro(0, 0.2);

                        }
                        else {
                            skystone_state = SKYSTONE_STATES.STRAFE_UNTIL_SKYSTONE;
                            runtime.reset();
                        }
                        break;

                    case STRAFE_UNTIL_SKYSTONE:

                        // check all the trackable targets to see which one (if any) is visible.
                        targetVisible = false;
                        for (VuforiaTrackable trackable : allTrackables) {
                            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                                telemetry.addData("Visible Target", trackable.getName());
                                targetVisible = true;

                                // getUpdatedRobotLocation() will return null if no new information is available since
                                // the last time that call was made, or if the trackable is not currently visible.
                                robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                                if (robotLocationTransform != null) {
                                    lastLocation = robotLocationTransform;
                                }
                                break;
                            }
                        }

                        // Provide feedback as to where the robot is located (if we know).
                        if (targetVisible) {
                            cutPower();

                            // express position (translation) of robot in inches.
                            VectorF translation = lastLocation.getTranslation();
                            //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            //      translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                            distanceFromSkystone = translation.get(1);

                            // express the rotation of the robot in degrees.
                            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                            //SkystonePosition = [translation.get(2), translation.get(1), translation.get(0)];
                            skystone_state = SKYSTONE_STATES.MOVE_TOWARD_SKYSTONE;
                            runtime.reset();
                        }
                        else {
                            //moveByGyro(90,0.225);
                            moveByGyro(90,0.25);
                            telemetry.addData("Skystone NOT visible", "none");
                        }
                        telemetry.update();



                        break;
                    case MOVE_TOWARD_SKYSTONE:

                        break;
                    case CAPTURE_SKYSTONE:

                        break;
                    case BACK_OUT:

                        break;
                    case TURN_TOWARD_BRIDGE:

                        break;

                    case STRAFE_BASED_ON_PARKING_POSITION:

                        break;
                    case MOVE_TO_BRIDGE:

                        moveToBridge(0, 0.2);
                        break;

                    case DROP_OFF_SKYSTONE:

                        break;
                }

                break;

            case REPOSITION_FOUNDATION:
                switch (foundation_state) {
                    case MOVE_TOWARD_FOUNDATION:

                        break;
                    case LATCH_ARM:

                        break;
                    case BRING_FOUNDATION_BACK:

                        break;
                }

                break;

            case PARK_UNDER_BRIDGE:
                switch (park_state) {
                    case STRAFE:

                        break;
                    case MOVE_TO_BRIDGE:

                        moveToBridge(0, 0.2);
                        state = STATES.DONE;
                        break;
                }

                break;

            case DONE:

                telemetry.addData("Status", "Done");
                telemetry.update();
                break;

        }

    }

    @Override
    public void stop() {
        super.stop();
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    protected double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    protected double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .05;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    protected void strafe(DIRECTION direction, double power, double correction) {
        switch (direction) {
            case LEFT:
                frontLeft.setPower(-power-correction);
                frontRight.setPower(power);
                backLeft.setPower(power-correction);
                backRight.setPower(-power);

                break;
            case RIGHT:
                break;
        }

    }

    protected void cutPower(){

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }


    void moveByGyro(double angle, double speed){
        /*

        0 = 45
         */
        double inputAngle = (angle*(Math.PI/180))+Math.PI/4;
        //double rightX = globalAngle/10;
        double rightX = checkDirection();
        double v1 = Math.cos(inputAngle) + rightX;
        double v2 = Math.sin(inputAngle) - rightX;
        double v3 = Math.sin(inputAngle) + rightX;
        double v4 = Math.cos(inputAngle) - rightX;

        backRight.setPower(v1*speed);
        backLeft.setPower(v2*speed);
        frontRight.setPower(v3*speed);
        frontLeft.setPower(v4*speed);
    }

    // moves in specified direction with the appropriate power till either red or blue gaffer tapes are seen.
    void moveToBridge(double angle, double speed) {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if((blue>red)&&(blue>green)){
            telemetry.addLine("Blue Object Detected");
            cutPower();
            state = STATES.DONE;
        }
        else if((red>blue)&&(red>green)){
            telemetry.addLine("Red Object Detected");
            cutPower();
            state = STATES.DONE;
        }
        else{
            moveByGyro(angle, speed);
        }


    }

    class IMULoader implements Runnable {
        public void run() {
            // IMU Initialization
            imuparameters = new BNO055IMU.Parameters();

            imuparameters.mode                = BNO055IMU.SensorMode.IMU;
            imuparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            imuparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imuparameters.loggingEnabled      = false;

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            imu.initialize(imuparameters);

            byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
            byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

            //Need to be in CONFIG mode to write to registers
            imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

            try {
                Thread.sleep(100); //Changing modes requires a delay before doing anything else
            } catch (InterruptedException e) {
                telemetry.addLine("Sleep exception");
                telemetry.update();
            }

            //Write to the AXIS_MAP_CONFIG register
            imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);

            //Write to the AXIS_MAP_SIGN register
            imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);

            //Need to change back into the IMU mode to use the gyro
            imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

            try {
                Thread.sleep(100); //Changing modes again requires a delay
            } catch (InterruptedException e) {
                telemetry.addLine("Sleep exception");
                telemetry.update();
            }

            // make sure the imu gyro is calibrated before continuing.
            try {
                while (!imu.isGyroCalibrated())
                {
                    Thread.sleep(50);
                }
            } catch (InterruptedException e) {
                telemetry.addLine("Sleep exception");
                telemetry.update();
            }
            IMU_initialized = true;
        }
    }


}
