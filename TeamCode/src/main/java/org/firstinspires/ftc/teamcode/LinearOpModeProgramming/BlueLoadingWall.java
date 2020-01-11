package org.firstinspires.ftc.teamcode.LinearOpModeProgramming;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.TFB_Autonomous;
import org.firstinspires.ftc.teamcode.Vector2Weighted;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Vector2Weighted;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
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

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Autonomous(name= "Blue Loading Wall",group = "drive")
public class BlueLoadingWall extends LinearOpMode {
    int index;

    boolean foundBridge = false;

    ColorSensor colorSensor = null;
    OpenCvCamera phoneCam;
    Point one, two;

    boolean stoneWall[] = {true,true,true,true,true,true}; // indicates presence of stones/skystones
    int skystone1 = -1;
    int skystone2 = -1;

    int next_stone;
    int size = 2;
    int currentSize = 0;
    double[] pointArray = new double[(int)(size*size-1)];

    private Mat hsvThresholdOutput = new Mat();
    private Mat blurOutput = new Mat();
    private Mat cvErodeOutput = new Mat();
    double data1[], data2[];

    boolean forward = true;
    public static double DISTANCE = 60;
    Servo rightSkystoneServo;
    Servo leftSkystoneServo;

    Thread t1;

     Vector2d skystoneCoOrdinates[] = new Vector2d[6];
     Vector2d frontOfStoneCoOrdinates[] = new Vector2d[6];

     double STONE_WALL_DISTANCE = 28.5;
     double BACKUP_DISTANCE = 9.0;

     Vector2d buildingZoneSkystoneDropOff;


    public ElapsedTime runtime = new ElapsedTime();


    int skystone = -1;
    boolean FirstSkystone  = false;
    boolean SecondSkystone = false;
     SampleMecanumDriveBase drive;

    @Override
    public void runOpMode() throws InterruptedException {

        skystoneCoOrdinates[0] = Vector2Weighted.createVector(STONE_WALL_DISTANCE, 9);
        skystoneCoOrdinates[1] = Vector2Weighted.createVector(STONE_WALL_DISTANCE, 0);
        skystoneCoOrdinates[2] = Vector2Weighted.createVector(STONE_WALL_DISTANCE, -8);
        skystoneCoOrdinates[3] = Vector2Weighted.createVector(STONE_WALL_DISTANCE, -16);
        skystoneCoOrdinates[4] = Vector2Weighted.createVector(STONE_WALL_DISTANCE, -25);
        skystoneCoOrdinates[5] = Vector2Weighted.createVector(STONE_WALL_DISTANCE, -33);

        frontOfStoneCoOrdinates[0] = Vector2Weighted.createVector(STONE_WALL_DISTANCE - BACKUP_DISTANCE, 9);
        frontOfStoneCoOrdinates[1] = Vector2Weighted.createVector(STONE_WALL_DISTANCE - BACKUP_DISTANCE, 2);
        frontOfStoneCoOrdinates[2] = Vector2Weighted.createVector(STONE_WALL_DISTANCE - BACKUP_DISTANCE, -10);
        frontOfStoneCoOrdinates[3] = Vector2Weighted.createVector(STONE_WALL_DISTANCE - BACKUP_DISTANCE, -18);
        frontOfStoneCoOrdinates[4] = Vector2Weighted.createVector(STONE_WALL_DISTANCE - BACKUP_DISTANCE, -25);
        frontOfStoneCoOrdinates[5] = Vector2Weighted.createVector(STONE_WALL_DISTANCE - BACKUP_DISTANCE, -33);

        buildingZoneSkystoneDropOff = Vector2Weighted.createVector(20, 55);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new SamplePipeline());
        phoneCam.showFpsMeterOnViewport(false);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);

        one = new Point(230, 350);
        two = new Point(320, 350);


        rightSkystoneServo = hardwareMap.get(Servo.class, "r_skystone");
        leftSkystoneServo = hardwareMap.get(Servo.class, "l_skystone");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        t1 = new Thread(new ArmResetter());

        rightSkystoneServo.setPosition(0);
        leftSkystoneServo.setPosition(0.35);


        waitForStart();
        phoneCam.stopStreaming();


        while (opModeIsActive()){


            drive.followTrajectorySync(drive.trajectoryBuilder().forward(28.5).build());
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(skystoneCoOrdinates[skystone1]).build());
            rightSkystoneServo.setPosition(0.35);
            sleep(250);
            drive.followTrajectorySync(drive.trajectoryBuilder().back(BACKUP_DISTANCE).build());
            dropOffStone();
            stoneWall[skystone1] = false;
            resetPosition();
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(frontOfStoneCoOrdinates[skystone2]).build());
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(skystoneCoOrdinates[skystone2]).build());
            rightSkystoneServo.setPosition(0.35);
            sleep(250);
            drive.followTrajectorySync(drive.trajectoryBuilder().back(BACKUP_DISTANCE).build()); // back up a bit
            dropOffStone();
            stoneWall[skystone2] = false;
            resetPosition();
            next_stone = findNextAvailableStone();
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(frontOfStoneCoOrdinates[next_stone]).build());
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(skystoneCoOrdinates[next_stone]).build());
            rightSkystoneServo.setPosition(0.35);
            sleep(250);
            drive.followTrajectorySync(drive.trajectoryBuilder().back(BACKUP_DISTANCE).build()); // back up a bit
            dropOffStone();
            stoneWall[index] = false;
            foundBridge = false;

            while (!foundBridge) { // continue
                moveToStonesThroughBridge();
            }

            drive.setMotorPowers(0,0,0,0);
            foundBridge = false;

            break;


        }



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
            if(data1[0]<100){ // first stone is a Skystone and so is the fourth
                skystone1 = 0;
                skystone2 = 3;
            }
            else if(data2[0]<100){ // second stone is a Skystone and so is the fifth
                skystone1 = 1;
                skystone2 = 4;
            }
            else { // third stone is a Skystone and so is the sixth
                skystone1 = 2;
                skystone2 = 5;
            }
            Imgproc.circle(input, one, 10, new Scalar((data1[0]>200)?255:0, (data1[0]<=200)?255:0, 0), 4);
            Imgproc.circle(input, two, 10, new Scalar((data2[0]>200)?255:0, (data2[0]<=200)?255:0, 0),  4);
            telemetry.addData("Stone1:",skystone1);
            telemetry.addData("Stone2:",skystone2);
            telemetry.update();

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
    class ArmResetter implements Runnable {
        public void run() {
            rightSkystoneServo.setPosition(0);
        }
    }

    void dropOffStone () {
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(buildingZoneSkystoneDropOff).build());
        t1.start();

    }

    void resetPosition(){

        foundBridge = false;
        while(!foundBridge) {
            moveToStonesThroughBridge();
        }

        drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 33));
        foundBridge = false;

    }

    void moveToStonesThroughBridge() {
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

                drive.setMotorPowers(0.4, -0.4, 0.4, -0.4);

        }

    }

    int findNextAvailableStone() {
        index = 0;
        while (stoneWall[index] == false) {
            index++;
        }
        return index;
    }

}




