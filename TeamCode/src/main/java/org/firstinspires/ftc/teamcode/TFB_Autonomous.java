package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import org.firstinspires.ftc.teamcode.MainCode.MovementTest;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import com.qualcomm.robotcore.hardware.Servo;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class TFB_Autonomous extends TFB_OpMode {

    protected OpenCvCamera phoneCam;
    protected Point one, two;
    protected Mat hsvThresholdOutput = new Mat();
    protected Mat blurOutput = new Mat();
    protected Mat cvErodeOutput = new Mat();
    protected double data1[], data2[];
    protected int cameraMonitorViewId;
    protected SampleMecanumDriveBase drive;
    protected Vector2d skystoneCoOrdinates[];
    protected Vector2d buildingZoneSkystoneDropOff;
    protected Vector2d skystoneFirstSetReferencePoint;
    protected Vector2d skystoneSecondSetReferencePoint;

    protected Servo skystoneServo; // pointer to the correct Skystone servo

    boolean stoneWall[] = {true,true,true,true,true,true}; // indicates presence of stones/skystones
    int skystone1 = -1;
    int skystone2 = -1;

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

    protected FOUNDATION_STATES foundation_states;

    protected STATES state;
    protected FOUNDATION_STATES foundation_state;

    enum STATES {
        FETCH_AND_DELIVER_SKYSTONES,
        PARK_UNDER_BRIDGE,
        REPOSITION_FOUNDATION,
        DONE
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

        // check for common initialiazation combinations that aren't implemented
        if ((starting_position == STARTING_POSITION.BUILDING_ZONE) && (state == STATES.FETCH_AND_DELIVER_SKYSTONES) ||
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

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new SamplePipeline());
        phoneCam.showFpsMeterOnViewport(false);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        telemetry.addData("Status", "Hit Play to Start.");        telemetry.update();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop()  {

        switch (state) {
            case FETCH_AND_DELIVER_SKYSTONES:

                // get the first Skystone
                drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(skystoneCoOrdinates[skystone1]).build());
                skystoneServo.setPosition(0.35);
                try {
                    sleep(250);
                } catch (InterruptedException e) {
                    telemetry.addLine("Sleep exception");
                    telemetry.update();
                }
                drive.followTrajectorySync(drive.trajectoryBuilder().back(15).build()); // back up a bit
                drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(buildingZoneSkystoneDropOff).build());
                skystoneServo.setPosition(0.25);
                stoneWall[skystone1] = false;
                drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(skystoneSecondSetReferencePoint).build());

                // get the second Skystone
                drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(skystoneCoOrdinates[skystone2]).build());
                skystoneServo.setPosition(0.35);
                try {
                    sleep(250);
                } catch (InterruptedException e) {
                    telemetry.addLine("Sleep exception");
                    telemetry.update();
                }
                drive.followTrajectorySync(drive.trajectoryBuilder().back(15).build()); // back up a bit
                drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(buildingZoneSkystoneDropOff).build());
                skystoneServo.setPosition(0.25);
                stoneWall[skystone2] = false;
                drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(skystoneFirstSetReferencePoint).build());

                // find the next two available stones to pick...assuming we can only deliver four at this point
                int index = -1;
                for (int i=0; i<2; i++) {
                    do {
                        // skip till you find the next available stone
                        index++;
                    } while (!stoneWall[index]);
                     drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(skystoneCoOrdinates[index]).build());
                     skystoneServo.setPosition(0.35);
                     try {
                        sleep(250);
                    } catch (InterruptedException e) {
                        telemetry.addLine("Sleep exception");
                        telemetry.update();
                    }
                    drive.followTrajectorySync(drive.trajectoryBuilder().back(15).build()); // back up a bit
                    drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(buildingZoneSkystoneDropOff).build());
                    skystoneServo.setPosition(0.25);
                    stoneWall[index] = false;
                    drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(skystoneFirstSetReferencePoint).build());
                }

                state = STATES.DONE;
                break;

            case REPOSITION_FOUNDATION:

                break;

            case PARK_UNDER_BRIDGE:
                break;

            case DONE  :

                break;
        }
    }

    @Override
    public void stop() {
        super.stop();
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

            // 0-based indexes
            if(data1[0]>200){ // first stone is a Skystone and so is the fourth
                skystone1 = 0;
                skystone2 = 3;
            }
            if(data2[0]>200){ // second stone is a Skystone and so is the fifth
                skystone1 = 1;
                skystone2 = 4;
            }
            else{ // third stone is a Skystone and so is the sixth
                skystone1 = 2;
                skystone2 = 5;
            }
            Imgproc.circle(input, one, 10, new Scalar((data1[0]>200)?255:0, (data1[0]<=200)?255:0, 0), 4);
            Imgproc.circle(input, two, 10, new Scalar((data2[0]>200)?255:0, (data2[0]<=200)?255:0, 0),  4);


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
