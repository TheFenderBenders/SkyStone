package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Autonomous(name= "MoveTest",group = "drive")
public class MovementTest extends LinearOpMode {
    OpenCvCamera phoneCam;
    Point one, two;


    int size = 2;
    int currentSize = 0;
    double[] pointArray = new double[(int)(size*size-1)];

    private Mat hsvThresholdOutput = new Mat();
    private Mat blurOutput = new Mat();
    private Mat cvErodeOutput = new Mat();
    double data1[], data2[];

    boolean forward = true;
    public static double DISTANCE = 60;
    Servo skystoneServo;
    Vector2Weighted v = new Vector2Weighted();


    public ElapsedTime runtime = new ElapsedTime();


    int skystone = -1;
    boolean FirstSkystone  = false;
    boolean SecondSkystone = false;
     SampleMecanumDriveBase drive;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new SamplePipeline());
        phoneCam.showFpsMeterOnViewport(false);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        one = new Point(300, 300);
        two = new Point(180, 300);



        skystoneServo = hardwareMap.get(Servo.class, "skystone_servo");



        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

/*
        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

 */

        // drive.trajectoryBuilder().
/*
        Trajectory tr = drive.trajectoryBuilder().
                strafeTo(v.createVector(10,10)).

                lineTo(v.createVector(50,50))
                .addMarker(2.0,() -> {
                    servo.setPosition(1);

                    return Unit.INSTANCE;
                })



                addMarker(), () -> {

            //Do stuff
            return Unit.INSTANCE;
        })


        .build();

        */

        //drive.trajectoryBuilder(new Vector2d(0,0)).addMarker(()->{



        //   return Unit.INSTANCE;});

        //Trajectory trajectory = drive.trajectoryBuilder().lineTo(vector2d).build();
        // Trajectory tr = drive.trajectoryBuilder().strafeTo(vector2d).build();




      /*  skystoneServo.setPosition(0);
        waitForStart();
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(28,-17)).build());
        skystoneServo.setPosition(0.35);
        sleep(500);
        drive.followTrajectorySync(drive.trajectoryBuilder().back(12).build());
        // drive.followTrajectorySync(drive.trajectoryBuilder().strafeLeft(60).build());
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13,35)).build());
        skystoneServo.setPosition(0);
        //the 24 is the second stone location -17
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13,-41)).build());


        //SS5
        skystoneServo.setPosition(0);
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(28,-39)).build());
        skystoneServo.setPosition(0.35);
        sleep(500);
        drive.followTrajectorySync(drive.trajectoryBuilder().back(12).build());
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13,35)).build());
        skystoneServo.setPosition(0);
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeRight(16).build());
        //drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13,0)).build());


       */
        skystoneServo.setPosition(0);

        telemetry.addData("Skystone", skystone);
        waitForStart();
        phoneCam.stopStreaming();



        if(data1[0]<200){
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(28.5,8)).build());
        }
        else if(data2[0]<200){
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(28.5,0)).build());
        }
        else {
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(28.5,-7.5)).build());
        }
        skystoneServo.setPosition(0.35);
        sleep(250);
        drive.followTrajectorySync(drive.trajectoryBuilder().back(15).build());


        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13.5,50)).build());
        skystoneServo.setPosition(0.25);
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13.5,-24)).build());

        if(data1[0]<200){
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(28.5,-13.5)).build());
        }
        else if(data2[0]<200){
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(28.5,-22)).build());
        }
        else {
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(28.5,-30)).build());
        }
        skystoneServo.setPosition(0.35);
        sleep(250);
        drive.followTrajectorySync(drive.trajectoryBuilder().back(15).build());
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13.5,50)).build());
        skystoneServo.setPosition(0.25);
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13.5, 0)).build());

        if(data1[0]<200){
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(28.5,0)).build());
        }
        else if(data2[0]<200){
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(28.5,-8)).build());
        }
        else {
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(29.5,15)).build());
        }

        skystoneServo.setPosition(0.35);
        sleep(250);
        drive.followTrajectorySync(drive.trajectoryBuilder().back(15).build());
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13.5, 50)).build());
        skystoneServo.setPosition(0.25);
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13.5, 0)).build());

        if(data1[0]<200){
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(28.5,-8)).build());
        }
        else if(data2[0]<200){
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(29.5,-15)).build());
        }
        else {
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(29.5,-22)).build());
        }
        skystoneServo.setPosition(0.35);
        drive.followTrajectorySync(drive.trajectoryBuilder().back(15).build());
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13.5, 50)).build());
        skystoneServo.setPosition(0.25);
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13.5, 0)).build());

        //drive.followTrajectorySync(drive.trajectoryBuilder().forward(28).build());
        //drive.followTrajectorySync(drive.trajectoryBuilder().strafeLeft(16).build());
        //drive.setPoseEstimate(new Pose2d(0,0));
        //deliver(2, true);
        //deliver(5, false);
        while (opModeIsActive()){
            telemetry.addData("",skystone);
            telemetry.addData("Data 1", data1[0]);
            telemetry.addData("Data 2", data2[0]);
            telemetry.update();

            if(!drive.isBusy()){


            }

            //drive.getPoseEstimate();
        }



    }

    void deliver(int stone, boolean firstStone){
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(28.5,-0.5-7.7*stone)).build());
        skystoneServo.setPosition(0.35);
        sleep(500);
        drive.followTrajectorySync(drive.trajectoryBuilder().back(12).build());
        // drive.followTrajectorySync(drive.trajectoryBuilder().strafeLeft(60).build());
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13,35)).build());
        skystoneServo.setPosition(0);
        //the 24 is the second stone location -17
        if(firstStone) drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(v.createVector(13,-41)).build());
        else drive.followTrajectorySync(drive.trajectoryBuilder().strafeRight(18).build());
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


            /*
            if(currentSize<size){
                pointArray[currentSize] = data1[currentSize];
            }
            */

            if(data1[0]>200){
                skystone = 1;
            }
            if(data2[0]>200){
                skystone =  2;
            }

            else{
                skystone = 3;
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




