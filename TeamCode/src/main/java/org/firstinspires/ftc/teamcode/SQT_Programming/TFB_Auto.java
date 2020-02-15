package org.firstinspires.ftc.teamcode.SQT_Programming;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.OldCode.Vector2Weighted;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.RoadRunnerAdditions;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


import java.util.List;
import java.util.ArrayList;


public class TFB_Auto extends TFB_LinearOpMode {
/**************** READ ME *************
    This autonomous program uses 2 unique features:
    First, it uses inheritance. This means the whole of the program is in TFB_LinearOpMode,
    TFB_Auto, and another program, named  [ALLIANCE]_[STARTING SIDE]_[(OPTIONAL)PARKING_POSITION]
    Most of the code, however, is in TFB_Auto.

    It also uses RoadRunnerAdditions, a class created to make programming roadrunner easier.



 */

//**********     OPEN CV     **********
    protected OpenCvCamera phoneCam;
    protected Point one, two;
    protected boolean[] stoneWall = {true, true, true, true, true, true}; // indicates presence of stones/skystones
    protected int skystone1 = -1;
    protected int skystone2 = -1;
    protected Mat hsvThresholdOutput = new Mat();
    protected Mat blurOutput = new Mat();
    protected Mat cvErodeOutput = new Mat();
    protected double[] data1;
    protected double[] data2;

//**********     INHERITANCE     **********
protected enum DIRECTION {
    LEFT, RIGHT
}
    protected DIRECTION dir;
    protected DIRECTION strafe_dir;

    protected enum STARTING_POSITION {
        LOADING_ZONE,
        BUILDING_ZONE
    }

    protected enum ALLIANCE {
        RED,
        BLUE
    }

    protected enum UNDER_BRIDGE_POSITION {
        WALL_SIDE,
        NEUTRAL_SIDE
    }

    protected ALLIANCE alliance;
    protected STARTING_POSITION starting_position;
    protected UNDER_BRIDGE_POSITION under_bridge_position;

    protected FOUNDATION_STATES foundation_states;

    protected SKYSTONE_STATES skystone_state;
    protected FOUNDATION_STATES foundation_state;

    protected int threadStatus; // 0 means thread not started, 1 means thread running, 2 thread done



    protected enum SKYSTONE_STATES {
        FIRST_SKYSTONE,
        DROPOFF_FIRST_SKYSTONE,
        RESET_FOR_SECOND_SKYSTONE,
        SECOND_SKYSTONE,
        DROPOFF_SECOND_SKYSTONE,
        RESET_FOR_THIRD_STONE,
        THIRD_STONE,
        DROPOFF_THIRD_SKYSTONE,
        FOURTH_STONE,
        DROPOFF_FOURTH_SKYSTONE,
        STONES_DONE,
        PARK,
        PULL_FOUNDATION,

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

//**********     POINTS     **********
    protected pointArray mainPoints = new pointArray();
    protected String StoneOne = "S1";
    protected String StoneTwo = "S2";
    protected String StoneThree = "S3";
    protected String StoneFour = "S4";
    protected String StoneFive = "S5";
    protected String StoneSix = "S6";
    protected String DropoffPoint = "Dropoff Point";
    protected String PlacementPoint = "Placement Point";
    protected String latchOff = "latch";
    protected String FoundationPosition = "Foundation Position";
    protected String LeftFoundation = "lf";

//**********     MISC.     **********
    protected RoadRunnerAdditions roadrunner = new RoadRunnerAdditions();
    protected boolean foundBridge = false;

    @Override
    void initMethod()  {
        if(alliance == ALLIANCE.RED){
            redSide = true;
        }
        else{
            redSide = false;
        }
        super.initMethod();
        skystone_state = SKYSTONE_STATES.FIRST_SKYSTONE;
        roadrunner.initRobot(hardwareMap);
        if(alliance == ALLIANCE.BLUE) {
            one = new Point(200, 350);
            two = new Point(325, 350);
        }
        else{
            two = new Point(150, 350);
            one = new Point(275, 350);
        }
        if(alliance == ALLIANCE.RED){
            roadrunner.flipSides();
        }
        mainPoints.add("S1", -8,30);
        mainPoints.add("S2", 0,30);
        mainPoints.add("S3", 8,30);
        mainPoints.add("S4", 16,30);
        mainPoints.add("S5", 24,30);
        mainPoints.add("S6", 32,30);
        mainPoints.add("Dropoff Point", -86,24);
        mainPoints.add("Placement Point", -86,34);//30 is norm
        mainPoints.add("Foundation Position",-92,0);
        mainPoints.add("Wall Park", -15,0);
        mainPoints.add("latch", -86,40);
        mainPoints.add("lf", -92,24);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new SamplePipeline());
        phoneCam.showFpsMeterOnViewport(false);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);

        waitForStart();

        phoneCam.stopStreaming();
        while (opModeIsActive()) {

            if(starting_position == STARTING_POSITION.LOADING_ZONE){
                switch(skystone_state){

                    case FIRST_SKYSTONE:
                        if(alliance == ALLIANCE.BLUE) {
                            rightSkystoneHand.setPosition(0.25);
                        }
                        else{
                            leftSkystoneHand.setPosition(0.18);
                        }
                        skystone_state = SKYSTONE_STATES.DROPOFF_FIRST_SKYSTONE;
                        roadrunner.move(mainPoints.get(StoneTwo));
                        switch (skystone1){
                            case 0:
                                roadrunner.move(mainPoints.get(StoneOne));
                                PickUp();
                                roadrunner.move(mainPoints.getBehind(StoneOne));
                                break;
                            case 1:
                                roadrunner.move(mainPoints.get(StoneTwo));
                                PickUp();
                                roadrunner.move(mainPoints.getBehind(StoneTwo));
                                break;
                            case 2:
                                roadrunner.move(mainPoints.get(StoneThree));
                                PickUp();
                                roadrunner.move(mainPoints.getBehind(StoneThree));
                                break;


                        }
                        break;
                    case DROPOFF_FIRST_SKYSTONE:
                        roadrunner.move(mainPoints.get(DropoffPoint));
                        roadrunner.move(mainPoints.get(PlacementPoint));
                        DropOff();
                        skystone_state = SKYSTONE_STATES.RESET_FOR_SECOND_SKYSTONE;

                        break;
                    case RESET_FOR_SECOND_SKYSTONE:
                        roadrunner.move(mainPoints.get(DropoffPoint));
                        if(alliance == ALLIANCE.BLUE) {
                            rightSkystoneHand.setPosition(0.25);
                        }
                        else{
                            leftSkystoneHand.setPosition(0.18);
                        }
                        skystone_state = SKYSTONE_STATES.SECOND_SKYSTONE;
                        break;
                    case SECOND_SKYSTONE:
                        skystone_state = SKYSTONE_STATES.DROPOFF_SECOND_SKYSTONE;
                        switch (skystone2){
                            case 3:
                                roadrunner.move(new Vector2d[]{
                                        mainPoints.getBehind(StoneFour),
                                        mainPoints.get(StoneFour)
                                });
                                PickUp();
                                roadrunner.move(mainPoints.getBehind(StoneFour));


                                break;
                            case 4:
                                roadrunner.move(new Vector2d[]{
                                        mainPoints.getBehind(StoneFive), mainPoints.get(StoneFive)
                                });
                                PickUp();
                                roadrunner.move(mainPoints.getBehind(StoneFive));


                                break;
                            case 5:
                                roadrunner.move(new Vector2d[]{
                                        mainPoints.getBehind(StoneSix), mainPoints.get(StoneSix)
                                });
                                PickUp();
                                roadrunner.move(mainPoints.getBehind(StoneSix));
                                break;
                        }
                        break;
                    case DROPOFF_SECOND_SKYSTONE:
                        Vector2d newDropoff = new Vector2d(mainPoints.get(DropoffPoint).getX()+12,mainPoints.get(DropoffPoint).getY());
                        Vector2d newPlacement = new Vector2d(mainPoints.get(PlacementPoint).getX()+12,mainPoints.get(PlacementPoint).getY());
                        roadrunner.move(new Vector2d[]{newDropoff,newPlacement});
                        DropOff();
                        roadrunner.move(new Vector2d[]{newDropoff});
                        skystone_state = SKYSTONE_STATES.PULL_FOUNDATION;
                        break;
                    case PULL_FOUNDATION:
                        Vector2d latchDropoff = new Vector2d(mainPoints.get(DropoffPoint).getX()+12,mainPoints.get(DropoffPoint).getY()+4);
                        Vector2d latchPlacement = new Vector2d(mainPoints.get(PlacementPoint).getX()+12,mainPoints.get(PlacementPoint).getY()+4);

                        if(alliance == ALLIANCE.BLUE) {
                            rightSkystoneHand.setPosition(0.25);
                        }
                        else{
                            leftSkystoneHand.setPosition(0.18);
                        }

                        if(alliance == ALLIANCE.BLUE) {
                            roadrunner.turn(-90);
                        }
                        else{
                            roadrunner.turn(-90);
                        }
                        roadrunner.strafeLeft(-12);
                        roadrunner.forward(-16);
                        foundationCaptureServoLeft.setPosition(0.3);
                        foundationCaptureServoRight.setPosition(0.7);

                        roadrunner.move(latchPlacement);
                        foundationCaptureServoLeft.setPosition(0.1);
                        foundationCaptureServoRight.setPosition(0.85);
                        sleep(500);
                        roadrunner.move(mainPoints.get(FoundationPosition));
                        foundationCaptureServoLeft.setPosition(0.6);
                        foundationCaptureServoRight.setPosition(0.3);
                        sleep(500);

                       // roadrunner.move(new Vector2d[]{mainPoints.get("Wall Park")});
                        skystone_state = SKYSTONE_STATES.PARK;
                        break;
                    case PARK:
                        break;
                }
            }
            else if(starting_position == STARTING_POSITION.BUILDING_ZONE){

            }

        }
    }


    class ArmResetter implements Runnable {
        public void run() {
            if (alliance == ALLIANCE.BLUE) {
                leftSkystoneHand.setPosition(0);
                sleep(250);
                leftSkystoneArm.setPosition(0);
            }
            else {
            }
        }
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
            if (alliance == ALLIANCE.BLUE) {
                roadrunner.setMotorPowers(0.4, -0.4, 0.4, -0.4);
            }
            else {
                roadrunner.setMotorPowers(-0.4, 0.4, 0.4, -0.4);
            }
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

    public class pointArray{
        ArrayList<String> names = new ArrayList<>(100);
        ArrayList<Vector2d> points = new ArrayList<>(100);
        public void add(String name, int x, int y){
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
                return (new Vector2d(x,y-6));
                //return (new Vector2d(x,y-10));

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

    void PickUp(){
        if (alliance == ALLIANCE.BLUE) {
            rightSkystoneArm.setPosition(0.33333);
            sleep(500);
            rightSkystoneHand.setPosition(0.4);
            sleep(500);
            rightSkystoneArm.setPosition(0);

        }
        else {
            leftSkystoneArm.setPosition(0);
            sleep(500);
            leftSkystoneHand.setPosition(0);
            sleep(500);
            leftSkystoneArm.setPosition(0.4);
        }
    }

    void DropOff(){
        if (alliance == ALLIANCE.BLUE) {
            rightSkystoneArm.setPosition(0.33333);
            sleep(250);
            rightSkystoneHand.setPosition(0);
            sleep(500);
            rightSkystoneArm.setPosition(0);

        }
        else {
            leftSkystoneArm.setPosition(0);
            sleep(500);
            leftSkystoneHand.setPosition(0.4);
            sleep(500);
            leftSkystoneArm.setPosition(0.4);
        }
    }



}
