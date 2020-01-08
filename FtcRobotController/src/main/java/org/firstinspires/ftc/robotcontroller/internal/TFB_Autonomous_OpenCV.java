package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import java.util.ArrayList;
import java.util.List;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Core;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.CvType;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;

import java.util.ArrayList;
import java.util.List;


public class TFB_Autonomous_OpenCV extends TFB_OpMode {



    OpenCvCamera phoneCam;
    Point one, two, three, four, five, six;

    int size = 2;
    int currentSize = 0;
    double[] pointArray = new double[(int)(size*size-1)];

    private Mat hsvThresholdOutput = new Mat();
    private Mat blurOutput = new Mat();
    private Mat cvErodeOutput = new Mat();
    double data1[], data2[], data3[], data4[], data5[], data6[];


    protected BNO055IMU.Parameters imuparameters;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30;
    protected double correction;
    protected double distanceFromSkystone = 100000;

    boolean foundBridge = false;

    boolean IMU_initialized = false;

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





    @Override
    public void init() {

        if(starting_position == STARTING_POSITION.LOADING_ZONE) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            phoneCam.openCameraDevice();
            phoneCam.setPipeline(new TFB_Autonomous_OpenCV.SamplePipeline());
            phoneCam.showFpsMeterOnViewport(false);
            phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

            one = new Point(240, 548);
            two = new Point(240, 457);
            three = new Point(240, 366);
            four = new Point(240, 274);
            five = new Point(240, 183);
            six = new Point(240, 91);
        }



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

        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("STATE", skystone_state);
        switch (state) {
            case FETCH_AND_DELIVER_SKYSTONE:
                switch (skystone_state) {

                }

                break;

            case REPOSITION_FOUNDATION:
                switch (foundation_state) {

                }

                break;

            case PARK_UNDER_BRIDGE:

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


    class SamplePipeline extends OpenCvPipeline {
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        @Override
        public Mat processFrame(Mat input) {

            Mat hsvThresholdInput = input;
            double[] hsvThresholdHue = {0.0, 81.70648464163823};
            double[] hsvThresholdSaturation = {103.1924460431655, 255.0};
            double[] hsvThresholdValue = {114.65827338129498, 255.0};
            hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

            data1 = hsvThresholdOutput.get((int)one.y, (int)one.x);
            data2 = hsvThresholdOutput.get((int)two.y, (int)two.x);
            data3 = hsvThresholdOutput.get((int)three.y, (int)three.x);
            data4 = hsvThresholdOutput.get((int)four.y, (int)four.x);
            data5 = hsvThresholdOutput.get((int)five.y,(int)five.x);
            data6 = hsvThresholdOutput.get((int)six.y, (int)six.x);

            /*
            if(currentSize<size){
                pointArray[currentSize] = data1[currentSize];
            }
            */

            Imgproc.circle(input, one, 10, new Scalar((data1[0]>200)?255:0, (data1[0]<=200)?255:0, 0), 4);
            Imgproc.circle(input, two, 10, new Scalar((data2[0]>200)?255:0, (data2[0]<=200)?255:0, 0),  4);
            Imgproc.circle(input, three, 10, new Scalar((data3[0]>200)?255:0, (data3[0]<=200)?255:0, 0),  4);
            Imgproc.circle(input, four, 10, new Scalar((data4[0]>200)?255:0, (data4[0]<=200)?255:0, 0),  4);
            Imgproc.circle(input, five, 10, new Scalar((data5[0]>200)?255:0, (data5[0]<=200)?255:0, 0),  4);
            Imgproc.circle(input, six, 10, new Scalar((data6[0]>200)?255:0, (data6[0]<=200)?255:0, 0),  4);

            return input;
        }

        /**
         * This method is a generated getter for the output of a HSV_Threshold.
         * @return Mat output from HSV_Threshold.
         */
        public Mat hsvThresholdOutput() {
            return hsvThresholdOutput;
        }

        /**
         * This method is a generated getter for the output of a Blur.
         * @return Mat output from Blur.
         */
        public Mat blurOutput() {
            return blurOutput;
        }

        /**
         * This method is a generated getter for the output of a CV_erode.
         * @return Mat output from CV_erode.
         */
        public Mat cvErodeOutput() {
            return cvErodeOutput;
        }


        /**
         * Segment an image based on hue, saturation, and value ranges.
         *
         * @param input The image on which to perform the HSL threshold.
         * @param hue The min and max hue
         * @param sat The min and max saturation
         * @param val The min and max value
         * @param out The image in which to store the output.
         */
        private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                                  Mat out) {
            Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
            Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                    new Scalar(hue[1], sat[1], val[1]), out);
        }

        /**
         * Softens an image using one of several filters.
         * @param input The image on which to perform the blur.
         * @param type The blurType to perform.
         * @param doubleRadius The radius for the blur.
         * @param output The image in which to store the output.
         */
        private void blur(Mat input, EasyOpenCV_Test.BlurType type, double doubleRadius,
                          Mat output) {
            int radius = (int)(doubleRadius + 0.5);
            int kernelSize;
            switch(type){
                case BOX:
                    kernelSize = 2 * radius + 1;
                    Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                    break;
                case GAUSSIAN:
                    kernelSize = 6 * radius + 1;
                    Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                    break;
                case MEDIAN:
                    kernelSize = 2 * radius + 1;
                    Imgproc.medianBlur(input, output, kernelSize);
                    break;
                case BILATERAL:
                    Imgproc.bilateralFilter(input, output, -1, radius, radius);
                    break;
            }
        }

        /**
         * Expands area of lower value in an image.
         * @param src the Image to erode.
         * @param kernel the kernel for erosion.
         * @param anchor the center of the kernel.
         * @param iterations the number of times to perform the erosion.
         * @param borderType pixel extrapolation method.
         * @param borderValue value to be used for a constant border.
         * @param dst Output Image.
         */
        private void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
                             int borderType, Scalar borderValue, Mat dst) {
            if (kernel == null) {
                kernel = new Mat();
            }
            if (anchor == null) {
                anchor = new Point(-1,-1);
            }
            if (borderValue == null) {
                borderValue = new Scalar(-1);
            }
            Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
        }



    }




}
