package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
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

import static java.lang.Thread.currentThread;
import static java.lang.Thread.sleep;
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
    DIRECTION strafe_dir;


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

    protected STATES state;
    protected SKYSTONE_STATES skystone_state;
    protected FOUNDATION_STATES foundation_state;

    enum STATES {
        FETCH_AND_DELIVER_SKYSTONE,
        PARK_UNDER_BRIDGE,
        REPOSITION_FOUNDATION,
        DONE
    }

    enum SKYSTONE_STATES {
        MOVE_TOWARD_STONE_LINE,
        STRAFE_UNTIL_SKYSTONE,
        ALIGN_WITH_SKYSTONE,
        MOVE_TOWARD_SKYSTONE,
        CAPTURE_SKYSTONE,
        BACK_OUT,
        TURN_TOWARD_BRIDGE,
        STRAFE_BASED_ON_PARKING_POSITION,
        MOVE_TO_BRIDGE,
        DROP_OFF_SKYSTONE,
        DEBUG,
        PRINT_LOCATION
    }

    enum FOUNDATION_STATES {
        STRAFE_LEFT,
        MOVE_TOWARD_FOUNDATION,
        LATCH_ARM,
        WAIT,
        BRING_FOUNDATION_BACK,
        UNLATCH_ARM,
        STRAFE_OUT_OF_FOUNDATION,
        MOVE_TILL_BRIDGE
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
    VuforiaTrackable trackable;
    protected VuforiaLocalizer.Parameters vuparameters;
    OpenGLMatrix robotFromCamera;
    final float CAMERA_FORWARD_DISPLACEMENT = 5.5f * mmPerInch;
    final float CAMERA_VERTICAL_DISPLACEMENT = 2.5f * mmPerInch;   // eg: Camera is 2 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT = 1.5f;
    protected boolean targetVisible = false;
    protected float phoneXRotate    = 0;
    protected float phoneYRotate    = 0;
    protected float phoneZRotate    = 0;
    protected OpenGLMatrix robotLocationTransform;
    protected int cameraMonitorViewId;
    protected BNO055IMU.Parameters imuparameters;
    protected boolean IMU_initialized = false;
    protected ElapsedTime dutyCycleOnTime = new ElapsedTime();
    protected ElapsedTime dutyCycleOffTime = new ElapsedTime();
    protected int iterations = 0;
    protected int DUTY_CYCLE_ON_TIME=1100;
    protected int DUTY_CYCLE_OFF_TIME=600;
    protected boolean foundBridge = false;
    protected boolean neutralBridgeSeek = false;

    VectorF translation;

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
                sleep(5000);
            } catch (InterruptedException e) {
                telemetry.addLine("Sleep exception");
                telemetry.update();
            }
            state = STATES.DONE;
            return;
        }

        super.init();

        skystoneCaptureServo.setPosition(0);

        // start off IMU initialization on a background thread
        Thread t1 = new Thread(new IMULoader());
        t1.start();

        // no need for Vuforia in Loading Zone
       if (starting_position == STARTING_POSITION.LOADING_ZONE) {
            // initialize Vuforia
            cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            vuparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            vuparameters.vuforiaLicenseKey = VUFORIA_KEY;
            vuparameters.cameraDirection = CAMERA_CHOICE;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(vuparameters);

            // Load the data sets for the trackable objects. These particular data
            // sets are stored in the 'assets' part of our application.
            this.targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
            stoneTarget = targetsSkyStone.get(0);
            stoneTarget.setName("Stone Target");

            // For convenience, gather together all the trackable objects in one easily-iterable collection */
            allTrackables = new ArrayList<VuforiaTrackable>();
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
                phoneXRotate = 90;
            }

            robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

            /**  Let all the trackable listeners know where the phone is.  */
            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, vuparameters.cameraDirection);
            }
        }
//        CameraDevice.getInstance().setFlashTorchMode(true);

        telemetry.addData("Status", "Hit Play to Start.");
        telemetry.update();

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

        while (!imu.isGyroCalibrated()) {
            telemetry.addLine("IMU not initialized...PLEASE WAIT!");
            telemetry.update();
            return;
        }
        if (starting_position == STARTING_POSITION.LOADING_ZONE) {
            this.targetsSkyStone.activate();
        }
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("STATE", skystone_state);
        switch (state) {
            case FETCH_AND_DELIVER_SKYSTONE:
                switch (skystone_state) {
                    case MOVE_TOWARD_STONE_LINE:
                        correction = checkDirection();
                        telemetry.addData("IMU Correction", correction);
                        telemetry.update();

                        if (runtime.milliseconds() < 1400 ) {
                            moveByGyro(0, 0.3);
                        }
                        else {
                            skystone_state = SKYSTONE_STATES.STRAFE_UNTIL_SKYSTONE;
                            runtime.reset();
                        }
                        break;

                    case STRAFE_UNTIL_SKYSTONE:
                        if(dutyCycleOffTime.milliseconds() <DUTY_CYCLE_OFF_TIME) {
                            cutPower();
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
                            dutyCycleOnTime.reset();
                        }
                        else if(dutyCycleOnTime.milliseconds()<DUTY_CYCLE_ON_TIME){
                            if (alliance == ALLIANCE.BLUE) {
                                moveByGyro(90,0.3);
                            }
                            else {
                                moveByGyro(-90,0.3);
                            }
                        }
                        else {
                            dutyCycleOffTime.reset();
                        }

                        // Provide feedback as to where the robot is located (if we know).
                        if (targetVisible) {
                            cutPower();

                            // express position (translation) of robot in inches.
                            VectorF translation = lastLocation.getTranslation();
                            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                  translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                            distanceFromSkystone = translation.get(0);
                            telemetry.addData("Distance to Skystone", distanceFromSkystone);

                            // express the rotation of the robot in degrees.
                            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//                            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                            //SkystonePosition = [translation.get(2), translation.get(1), translation.get(0)];
                            skystone_state = SKYSTONE_STATES.ALIGN_WITH_SKYSTONE;
                            runtime.reset();
                        }
                        telemetry.update();

                        break;

                    case PRINT_LOCATION:
                        translation = null;
                        for (VuforiaTrackable trackable : allTrackables) {
                            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                                telemetry.addData("Visible Target", trackable.getName());
                                robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                                if (robotLocationTransform != null) {
                                    lastLocation = robotLocationTransform;
                                }
                                translation = lastLocation.getTranslation();
                            }
                        }

                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                        distanceFromSkystone = translation.get(0)/mmPerInch;
                        telemetry.addData("Distance to Skystone", distanceFromSkystone);
                        telemetry.update();
                        break;

                    case ALIGN_WITH_SKYSTONE:
                        translation = null;
                        for (VuforiaTrackable trackable : allTrackables) {
                            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                                telemetry.addData("Visible Target", trackable.getName());
                                robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                                if (robotLocationTransform != null) {
                                    lastLocation = robotLocationTransform;
                                }
                                translation = lastLocation.getTranslation();
                            }
                        }

                        if(translation != null){
                            if(translation.get(1)/mmPerInch < 1.5){
                                telemetry.addData("Position", "{0,1,2} = %.1f, %.1f,%.1f", translation.get(0)/ mmPerInch, translation.get(1)/ mmPerInch, translation.get(2)/ mmPerInch);
                                moveByGyro(-90,0.19);
                            }
                            else if(translation.get(1)/mmPerInch > 3){
                                moveByGyro(90,0.19);
                            }
                            else {
                                cutPower();
                                skystone_state = SKYSTONE_STATES.MOVE_TOWARD_SKYSTONE;
                                runtime.reset();
                            }
                        }

                        break;

                    case MOVE_TOWARD_SKYSTONE:
                        translation = null;
                        for (VuforiaTrackable trackable : allTrackables) {
                            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                                telemetry.addData("Visible Target", trackable.getName());
                                robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                                if (robotLocationTransform != null) {
                                    lastLocation = robotLocationTransform;
                                }
                                 translation = lastLocation.getTranslation();
                            }
                        }

                        if(translation != null){
                            if(translation.get(0)/mmPerInch < -11){
                                moveByGyro(0, 0.2);
                            }
                            else {
                                cutPower();
                                skystone_state = SKYSTONE_STATES.CAPTURE_SKYSTONE;
                                dutyCycleOffTime.reset();
                                runtime.reset();
                            }
                        }
                        break;

                    case CAPTURE_SKYSTONE:
                        if (dutyCycleOffTime.milliseconds() >= DUTY_CYCLE_OFF_TIME) { // sleep for a bit
                            skystoneCaptureServo.setPosition(0.35);
                            skystone_state = SKYSTONE_STATES.BACK_OUT;
                            runtime.reset();
                            dutyCycleOffTime.reset();
                        }
                        break;

                    case BACK_OUT:
                        if (dutyCycleOffTime.milliseconds() >= DUTY_CYCLE_OFF_TIME) { // sleep for a bit
                            if(under_bridge_position == UNDER_BRIDGE_POSITION.WALL_SIDE) {
                                if (runtime.milliseconds() < 4000) {
                                    moveByGyro(-180, 0.3);
                                } else {
                                    cutPower();
                                    skystone_state = SKYSTONE_STATES.TURN_TOWARD_BRIDGE;
                                    runtime.reset();
                                }
                            }
                            else if(under_bridge_position == UNDER_BRIDGE_POSITION.NEUTRAL_SIDE) {
                                if (runtime.milliseconds() < 1500) {
                                    moveByGyro(-180, 0.2);
                                } else {
                                    cutPower();
                                    skystone_state = SKYSTONE_STATES.TURN_TOWARD_BRIDGE;
                                    runtime.reset();
                                }
                            }


                        }
                        else {
                            runtime.reset();
                        }
                        break;

                    case TURN_TOWARD_BRIDGE:
                        telemetry.addData("ANGLE", getAngle());
                        telemetry.update();
                        if (alliance == ALLIANCE.BLUE) {
                            if (globalAngle < 90) {
                                frontLeft.setPower(-0.2);
                                backLeft.setPower(-0.2);
                                frontRight.setPower(0.2);
                                backRight.setPower(0.2);
                            } else {
                                cutPower();
                                resetAngle();
                                skystone_state = SKYSTONE_STATES.MOVE_TO_BRIDGE;
                            }
                        }
                        else {
                            if (globalAngle > -90) {
                                frontLeft.setPower(0.2);
                                backLeft.setPower(0.2);
                                frontRight.setPower(-0.2);
                                backRight.setPower(-0.2);
                            } else {
                                cutPower();
                                resetAngle();
                                skystone_state = SKYSTONE_STATES.MOVE_TO_BRIDGE;
                            }
                        }
                        break;

                    case STRAFE_BASED_ON_PARKING_POSITION:

                        break;
                    case MOVE_TO_BRIDGE:
                        moveToBridge(0,0.2);
                        if (foundBridge) { // continue
                            runtime.reset();
                            foundBridge = false;
                            skystone_state = SKYSTONE_STATES.DROP_OFF_SKYSTONE;
                        }
                        break;

                    case DROP_OFF_SKYSTONE:
                        if(runtime.milliseconds()<600){
                            moveByGyro(0,0.3);
                            skystoneCaptureServo.setPosition(0);
                        }
                        else{
                            state = STATES.PARK_UNDER_BRIDGE;
                            cutPower();
                        }
                        break;
                }

                break;

            case REPOSITION_FOUNDATION:
                switch (foundation_state) {
                    case STRAFE_LEFT:
                        if (runtime.milliseconds() < 2000) {
                            if(alliance == ALLIANCE.BLUE)
                                moveByGyro(-90, 0.3);
                            else
                                moveByGyro(90,0.3);
                        }
                        else {
                            cutPower();
                            foundation_state = FOUNDATION_STATES.MOVE_TOWARD_FOUNDATION;
                            runtime.reset();
                        }
                        break;
                    case MOVE_TOWARD_FOUNDATION:
                        if (runtime.milliseconds() < 2400) {
                            moveByGyro(0, 0.3);
                        }
                        else {
                            cutPower();
                            foundation_state = FOUNDATION_STATES.LATCH_ARM;
                            runtime.reset();
                        }
                        break;
                    case LATCH_ARM:
                        if (runtime.milliseconds() > 1000) {
                            foundationCaptureLeftServo.setPosition(1.0);
                            foundationCaptureRightServo.setPosition(1.0);
                            resetAngle();
                            runtime.reset();
                            foundation_state = FOUNDATION_STATES.WAIT;
                        }
                        break;
                    case WAIT:
                        if (runtime.milliseconds() > 2000) {
                            foundation_state = FOUNDATION_STATES.BRING_FOUNDATION_BACK;
                            runtime.reset();
                        }
                        break;
                    case BRING_FOUNDATION_BACK:
                        if (runtime.milliseconds() < 3000) {
                            moveByGyro(180, 0.4);
                        }
                        else {
                            foundation_state = FOUNDATION_STATES.UNLATCH_ARM;
                            cutPower();
                            runtime.reset();
                        }
                        break;
                    case UNLATCH_ARM:
                        if (runtime.milliseconds() > 1000) {
                            foundationCaptureLeftServo.setPosition(0);
                            foundationCaptureRightServo.setPosition(0);
                            resetAngle();
                            runtime.reset();
                            foundation_state = FOUNDATION_STATES.STRAFE_OUT_OF_FOUNDATION;
                        }
                        break;
                    case STRAFE_OUT_OF_FOUNDATION:
                        if(runtime.milliseconds()<2500){
                            if (alliance == ALLIANCE.BLUE) {
                                moveByGyro(90, 0.4);
                            }
                            else {
                                moveByGyro(-90, 0.4);
                            }
                        }
                        else{
                            cutPower();
                            runtime.reset();
                            foundation_state = FOUNDATION_STATES.MOVE_TILL_BRIDGE;
                        }
                        break;
                    case MOVE_TILL_BRIDGE:
                        if ((under_bridge_position == UNDER_BRIDGE_POSITION.WALL_SIDE) || (neutralBridgeSeek)) {
                                // strafe only
                                if (alliance == ALLIANCE.BLUE) {
                                    // strafe right
                                    moveToBridge(90, 0.5);
                                } else {
                                    // strafe left
                                    moveToBridge(-90, 0.5);
                                }
                                if (foundBridge) {
                                    runtime.reset();
                                    cutPower();
                                    state = STATES.DONE;
                                }
                        }
                        else { // Neutral side
                            if (runtime.milliseconds() < 1500) {
                                moveByGyro(0, 0.3);
                            }
                            else {
                                neutralBridgeSeek = true;
                            }
                        }

                        break;
                }

                break;

            case PARK_UNDER_BRIDGE:
                moveToBridge(180, 0.2);
                if(foundBridge) {
                    state = STATES.DONE;
                    cutPower();
                    runtime.reset();
                }
                break;

            case DONE:

                cutPower();
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
                backLeft.setPower(power);
                backRight.setPower(-power+correction);

                break;
            case RIGHT:
                frontLeft.setPower(power);
                frontRight.setPower(-power+correction);
                backLeft.setPower(-power-correction);
                backRight.setPower(power);

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
        double inputAngle = (-angle*(Math.PI/180))+Math.PI/4;
        //double rightX = globalAngle/10;
        double rightX = -checkDirection();

        double v1 = Math.cos(inputAngle) + rightX;
        double v2 = Math.sin(inputAngle) - rightX;
        double v3 = Math.sin(inputAngle) + rightX;
        double v4 = Math.cos(inputAngle) - rightX;

    /*    double v1 = Math.cos(inputAngle);
        double v2 = Math.sin(inputAngle);
        double v3 = Math.sin(inputAngle);
        double v4 = Math.cos(inputAngle);
*/

        frontLeft.setPower(v1*speed);
        frontRight.setPower(v2*speed);
        backLeft.setPower(v3*speed);
        backRight.setPower(v4*speed);
    }

    // moves in specified direction with the appropriate power till either red or blue gaffer tapes are seen.
    void moveToBridge(double angle, double speed) {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if((blue>red)&&(blue>green)){
            telemetry.addLine("Blue Object Detected");
            foundBridge = true;
        }
        else if((red>blue)&&(red>green)){
            telemetry.addLine("Red Object Detected");
            foundBridge = true;
        }
        else{
            moveByGyro(angle, speed);
        }


    }

    class IMULoader implements Runnable {
        public void run() {
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
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            imu.initialize(imuparameters);

            byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
            byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

            //Need to be in CONFIG mode to write to registers
            imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

            try {
                sleep(100); //Changing modes requires a delay before doing anything else
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
                sleep(100); //Changing modes again requires a delay
            } catch (InterruptedException e) {
                telemetry.addLine("Sleep exception");
                telemetry.update();
            }

            // make sure the imu gyro is calibrated before continuing.
            try {
                while (!imu.isGyroCalibrated())
                {
                    sleep(50);
                }
            } catch (InterruptedException e) {
                telemetry.addLine("Sleep exception");
                telemetry.update();
            }
            IMU_initialized = true;
        }

        void cutPower(){
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }


}
