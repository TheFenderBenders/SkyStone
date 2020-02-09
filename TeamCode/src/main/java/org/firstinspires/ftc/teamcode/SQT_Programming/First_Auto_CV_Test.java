package org.firstinspires.ftc.teamcode.SQT_Programming;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OldCode.LinearOpModeProgramming.BlueLoadingWall;
import org.firstinspires.ftc.teamcode.util.RoadRunnerAdditions;
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

@Autonomous
public class First_Auto_CV_Test extends LinearOpMode {

    pointArray mainPoints = new pointArray();


    OpenCvCamera phoneCam;
    Point one, two;

    boolean[] stoneWall = {true, true, true, true, true, true}; // indicates presence of stones/skystones
    int skystone1 = -1;
    int skystone2 = -1;


    private Mat hsvThresholdOutput = new Mat();
    private Mat blurOutput = new Mat();
    private Mat cvErodeOutput = new Mat();
    double[] data1;
    double[] data2;


    RoadRunnerAdditions roadrunner = new RoadRunnerAdditions();

    @Override
    public void runOpMode() {
        mainPoints.add("S1", -8,30);
        mainPoints.add("S2", 0,30);
        mainPoints.add("S3", 8,30);
        mainPoints.add("S4", 16,30);
        mainPoints.add("S5", 24,30);
        mainPoints.add("S6", 32,30);
        mainPoints.add("Dropoff Point", -65,20);





        one = new Point(200,400);
        two = new Point(325,400);

        roadrunner.initRobot(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new SamplePipeline());
        phoneCam.showFpsMeterOnViewport(false);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
        waitForStart();
        telemetry.addData("First Skystone", skystone1);
        telemetry.update();
        phoneCam.stopStreaming();

        while(opModeIsActive()) {
            Vector2d[] path0 = new Vector2d[]{
                    new Vector2d(0, 30)};
            roadrunner.move(path0);

            Vector2d[] path1 = new Vector2d[]{
                    new Vector2d(0,18),
                    new Vector2d(-60,18)
            };
            roadrunner.move(path1);

            stop();
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

            data1 = hsvThresholdOutput.get((int) one.y, (int) one.x);
            data2 = hsvThresholdOutput.get((int) two.y, (int) two.x);

            // 0-based indexes
            if (data1[0] < 100) { // first stone is a Skystone and so is the fourth
                skystone1 = 0;
                skystone2 = 3;
            } else if (data2[0] < 100) { // second stone is a Skystone and so is the fifth
                skystone1 = 1;
                skystone2 = 4;
            } else { // third stone is a Skystone and so is the sixth
                skystone1 = 2;
                skystone2 = 5;
            }
            Imgproc.circle(input, one, 10, new Scalar((data1[0] > 200) ? 255 : 0, (data1[0] <= 200) ? 255 : 0, 0), 4);
            Imgproc.circle(input, two, 10, new Scalar((data2[0] > 200) ? 255 : 0, (data2[0] <= 200) ? 255 : 0, 0), 4);
            telemetry.addData("Stone1:", skystone1);
            telemetry.addData("Stone2:", skystone2);
            telemetry.update();

            return input;
        }

        /**
         * This method is a generated getter for the output of a HSV_Threshold.
         *
         * @return Mat output from HSV_Threshold.
         */
        public Mat hsvThresholdOutput() {
            return hsvThresholdOutput;
        }

        /**
         * This method is a generated getter for the output of a Blur.
         *
         * @return Mat output from Blur.
         */
        public Mat blurOutput() {
            return blurOutput;
        }

        /**
         * This method is a generated getter for the output of a CV_erode.
         *
         * @return Mat output from CV_erode.
         */
        public Mat cvErodeOutput() {
            return cvErodeOutput;
        }


        /**
         * Segment an image based on hue, saturation, and value ranges.
         *
         * @param input The image on which to perform the HSL threshold.
         * @param hue   The min and max hue
         * @param sat   The min and max saturation
         * @param val   The min and max value
         * @param out   The image in which to store the output.
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
         *
         * @param src         the Image to erode.
         * @param kernel      the kernel for erosion.
         * @param anchor      the center of the kernel.
         * @param iterations  the number of times to perform the erosion.
         * @param borderType  pixel extrapolation method.
         * @param borderValue value to be used for a constant border.
         * @param dst         Output Image.
         */
        private void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
                             int borderType, Scalar borderValue, Mat dst) {
            if (kernel == null) {
                kernel = new Mat();
            }
            if (anchor == null) {
                anchor = new Point(-1, -1);
            }
            if (borderValue == null) {
                borderValue = new Scalar(-1);
            }
            Imgproc.erode(src, dst, kernel, anchor, (int) iterations, borderType, borderValue);
        }


    }


    public class pointArray{
        ArrayList<String> names = new ArrayList<>(100);
        ArrayList<Vector2d> points = new ArrayList<>(100);
        void add(String name, int x, int y){
            names.add(name);
            points.add(new Vector2d(x,y));
        }
        Vector2d get(String getter){
            if(names.contains(getter)) {
                return points.get(names.indexOf(getter));
            }
            else{
                telemetry.addLine("ERROR! The Point You Requested Does Not Exist");
                telemetry.update();
                sleep(5000);
                stop();
                return null;
            }
        }
        Vector2d getBehind(String getter){
            if(names.contains(getter)) {
                double x  = points.get(names.indexOf(getter)).getX();
                double y  = points.get(names.indexOf(getter)).getY();
                return (new Vector2d(x,y-10));
            }
            else{
                telemetry.addLine("ERROR! The Point You Requested Does Not Exist");
                telemetry.update();
                sleep(5000);
                stop();
                return null;
            }




        }
        void remove(String name){
            int position = names.indexOf(name);
            names.add(position, "");
        }
    }
}