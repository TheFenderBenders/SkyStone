package org.firstinspires.ftc.teamcode.OldCode.LinearOpModeProgramming;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.imgproc.Imgproc.approxPolyDP;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.circle;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.minEnclosingCircle;
import static org.opencv.imgproc.Imgproc.moments;
import static org.opencv.imgproc.Imgproc.putText;
import static org.opencv.imgproc.Imgproc.rectangle;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Disabled

@Autonomous(name= "YEET FAST",group = "drive")
public class OpenCvMiddle extends LinearOpMode {

    OpenCvCamera phoneCam;
     SampleMecanumDriveBase drive;
    int skystone1 = -1;
    int skystone2 = -1;

    int next_stone;
    int size = 2;
    Point one, two;

    private Mat hsvThresholdOutput = new Mat();
    private Mat blurOutput = new Mat();
    private Mat cvErodeOutput = new Mat();
    private Mat contourLines;
    double[] data1;
    double[] data2;


    @Override
    public void runOpMode() throws InterruptedException {
        one = new Point(230, 350);
        two = new Point(320, 350);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new SamplePipeline());
        phoneCam.showFpsMeterOnViewport(true);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        waitForStart();



        while (opModeIsActive()){

        }



    }


     class SamplePipeline extends OpenCvPipeline {

        private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
         private ArrayList<MatOfPoint> filteredContours = new ArrayList<MatOfPoint>();

        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();

        @Override
        public Mat processFrame(Mat input) {

            Mat hsvThresholdInput = input;
            double[] hsvThresholdHue = {0.0, 74.56706281833615};
            double[] hsvThresholdSaturation = {158.22841726618702, 255.0};
            double[] hsvThresholdValue = {0.0, 255.0};
            int stones = 0;

            Mat blurInput = input;

            double blurRadius = 9.909909909909912;
            blur(blurInput, "BOX", blurRadius, blurOutput);

            hsvThreshold(blurOutput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

            Mat findContoursInput = hsvThresholdOutput;

            boolean findContoursExternalOnly = false;
            findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

/*            int size = findContoursOutput.size();
            Size s = new Size(480,640);
            contourLines = Mat.zeros(s, input.type());

            // exclude small contours filteredContours
            for (int i=0; i < findContoursOutput.size(); i++) {
                MatOfPoint contour = findContoursOutput.get(i);
                Rect boundRect = boundingRect(contour);
                if (boundRect.area() > 100) {
                    filteredContours.add(contour);
                    drawContours(input, filteredContours, filteredContours.size()-1, new Scalar(255, 0, 0), 6);
                }
            }
*/

            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[findContoursOutput.size()];
            Rect[] boundRect = new Rect[findContoursOutput.size()];
            Point[] centers = new Point[findContoursOutput.size()];
            float[][] radius = new float[findContoursOutput.size()][1];

            for (int i=0; i < findContoursOutput.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                approxPolyDP(new MatOfPoint2f(findContoursOutput.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                centers[i] = new Point();
                minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
            }

            for (int i=0; i < findContoursOutput.size(); i++) {
                if (boundRect[i].area() > 10000) {
                    stones++;
                    rectangle(input, boundRect[i].tl(), boundRect[i].br(), new Scalar(255, 0, 0), 6);
//                    circle(input, centers[i], (int) radius[i][0], new Scalar(255, 0, 0), 6);
                    circle(input, centers[i], 5, new Scalar(255, 0, 0), 6);
                }
            }

            telemetry.addData("Stones", stones);
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

         private void blur(Mat input, String type, double doubleRadius,
                           Mat output) {
             int radius = (int)(doubleRadius + 0.5);
             int kernelSize;
             switch(type){
                 case "BOX":
                     kernelSize = 2 * radius + 1;
                     Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                     break;
                 case "GAUSSIAN":
                     kernelSize = 6 * radius + 1;
                     Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                     break;
                 case "MEDIAN":
                     kernelSize = 2 * radius + 1;
                     Imgproc.medianBlur(input, output, kernelSize);
                     break;
                 case "BILATERAL":
                     Imgproc.bilateralFilter(input, output, -1, radius, radius);
                     break;
             }
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

         private void findContours(Mat input, boolean externalOnly,
                                   List<MatOfPoint> contours) {
             Mat hierarchy = new Mat();
             contours.clear();
             int mode;
             if (externalOnly) {
                 mode = Imgproc.RETR_EXTERNAL;
             }
             else {
                 mode = Imgproc.RETR_LIST;
             }
             int method = Imgproc.CHAIN_APPROX_SIMPLE;
             Imgproc.findContours(input, contours, hierarchy, mode, method);
         }



    }





}




