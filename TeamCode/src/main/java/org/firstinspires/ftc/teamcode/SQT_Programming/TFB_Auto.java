package org.firstinspires.ftc.teamcode.SQT_Programming;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

    //**************** OPEN CV VARIABLES ****************
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

    //**************** INHERITANCE VARIABLES ****************
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
        DONE,
        TURN_TO_FOUNDATION

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

    //**************** MISC. VARIABLES ****************

    protected RoadRunnerAdditions roadrunner = new RoadRunnerAdditions();

    protected boolean foundBridge = false;
     pointArray skystonePoints = new pointArray();
     pointArray foundationPoints = new pointArray();

     String StoneOne;
     String StoneTwo;
     String StoneThree;
     String StoneFour;
     String StoneFive;
     String StoneSix;
     String DropoffPoint;
     String PlacementPoint;
     String FoundationPosition;
     String WallSideParking;
     String InTriangle;
     String InFrontOfFoundation;
     String SplitPoint; //This is to make it easier to park in the middle if necessary
     String WallParkPoint;
     String NeutralParkPoint;

    
    @Override
    void initMethod()  {
        super.initMethod();
        skystone_state = SKYSTONE_STATES.FIRST_SKYSTONE;
        roadrunner.initRobot(hardwareMap);
        one = new Point(200,400);
        two = new Point(325,400);
        //One and two are the points that OpenCV uses to look for stone/skystone.

        skystonePoints.add("S1", -8,30);
        skystonePoints.add("S2", 0,30);
        skystonePoints.add("S3", 8,30);
        skystonePoints.add("S4", 16,30);
        skystonePoints.add("S5", 24,30);
        skystonePoints.add("S6", 32,30);
        skystonePoints.add("Dropoff Point", -86,20);
        skystonePoints.add("Placement Point", -86,28);
        skystonePoints.add("Foundation Position",-80,-30);
        skystonePoints.add("Wall Park", -15,-30);

        StoneOne = "S1";
        StoneTwo = "S2";
        StoneThree = "S3";
        StoneFour = "S4";
        StoneFive = "S5";
        StoneSix = "S6";
        DropoffPoint = "Dropoff Point";
        PlacementPoint = "Placement Point";
        FoundationPosition = "Foundation Position";
        WallSideParking = "Wall Park";
    }

    @Override
    public void runOpMode() throws InterruptedException {

/* *********** READ ***********
    to move, use roadrunner.move
    For example:
    Vector2d[] path1 = new Vector2d[]{
    new Vector2d(0,0),
    new Vector2d(10,10)
    };
    roadrunner.move(path1)

    OR
    Vector2d point = new Vector2d(10,10);
   roadrunner.move(point);
    (This is more commonly used)

    To turn, use roadrunner.turn(angle);

    If you want to follow two trajectories without deaccelerating, using roadrunner.fastMove(pointA, pointB);

    If you want to change your position, use roadrunner.setPosition(new Pose2d(x,y,rotation));

    There is also an object called loading/skystone points. This stores points as Vector2ds with names as strings.
    In this, you can call a point by the String name or the variable name using the .get function.
    This will return a coordinate. You can also use .getBehind to get the point 10 inches behind the point(mainly for stones).

    THINGS TO DO:
    Foundation Side Points
    Color Sensor mount/program setup
    add support for close/far parking sides
 ****************************
*/
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
                        skystone_state = SKYSTONE_STATES.DROPOFF_FIRST_SKYSTONE;
                        roadrunner.move(skystonePoints.get(StoneTwo));
                        switch (skystone1){
                            case 0:
                                roadrunner.move(skystonePoints.get(StoneOne));
                                PickUpStone();
                                roadrunner.move(skystonePoints.getBehind(StoneOne));
                                break;
                            case 1:
                                roadrunner.move(skystonePoints.get(StoneTwo));
                                PickUpStone();
                                roadrunner.move(skystonePoints.getBehind(StoneTwo));
                                break;
                            case 2:
                                roadrunner.move(skystonePoints.get(StoneThree));
                                PickUpStone();
                                roadrunner.move(skystonePoints.getBehind(StoneThree));
                                break;
                        }
                        break;
                    case DROPOFF_FIRST_SKYSTONE:
                        roadrunner.move(skystonePoints.get(DropoffPoint));
                        roadrunner.move(skystonePoints.get(PlacementPoint));
                        DropOffStone();
                        skystone_state = SKYSTONE_STATES.RESET_FOR_SECOND_SKYSTONE;

                        break;
                    case RESET_FOR_SECOND_SKYSTONE:
                        roadrunner.move(skystonePoints.get(DropoffPoint));
                        skystone_state = SKYSTONE_STATES.SECOND_SKYSTONE;
                        break;
                    case SECOND_SKYSTONE:
                        skystone_state = SKYSTONE_STATES.DROPOFF_SECOND_SKYSTONE;
                        switch (skystone2){
                            case 3:
                                Vector2d[] goToStone = new Vector2d[]{skystonePoints.getBehind(StoneFour),skystonePoints.get(StoneFour)};
                                roadrunner.move(goToStone);
                                PickUpStone();
                                roadrunner.move(skystonePoints.getBehind(StoneFour));
                                break;
                            case 4:
                                goToStone = new Vector2d[]{skystonePoints.getBehind(StoneFive),skystonePoints.get(StoneFive)};
                                roadrunner.move(goToStone);
                                PickUpStone();
                                roadrunner.move(skystonePoints.getBehind(StoneFive));
                                break;
                            case 5:
                                goToStone = new Vector2d[]{skystonePoints.getBehind(StoneSix),skystonePoints.get(StoneSix)};
                                roadrunner.move(goToStone);
                                PickUpStone();
                                roadrunner.move(skystonePoints.getBehind(StoneSix));
                                break;
                        }
                        break;
                    case DROPOFF_SECOND_SKYSTONE:
                        Vector2d newDropoff = new Vector2d(skystonePoints.get(DropoffPoint).getX()+12,skystonePoints.get(DropoffPoint).getY());
                        Vector2d newPlacement = new Vector2d(skystonePoints.get(PlacementPoint).getX()+12,skystonePoints.get(PlacementPoint).getY());
                        //The Purpose of the "New" variables above is to make sure the skystones don't stack on the foundation and fall
                        //The variables are points but shifted to the left(near the foundation) by 12 inches
                        roadrunner.move(new Vector2d[]{newDropoff,newPlacement});
                        DropOffStone();
                        roadrunner.move(newDropoff);
                        skystone_state = SKYSTONE_STATES.TURN_TO_FOUNDATION;
                        break;
                        
                    case TURN_TO_FOUNDATION:
                        roadrunner.turn(-90);
                        skystone_state = SKYSTONE_STATES.PULL_FOUNDATION;
                        break;
                    case PULL_FOUNDATION:
                        Vector2d foundationLatchPosition = new Vector2d(skystonePoints.get(PlacementPoint).getX()+12,skystonePoints.get(PlacementPoint).getY());
                        foundationCaptureServoLeft.setPosition(0.4);
                        foundationCaptureServoRight.setPosition(0.7);
                        //^ This is the "Half Latched" Position. This will ensure that the stones do not block the foundation servos
                        roadrunner.move(foundationLatchPosition);
                        foundationCaptureServoLeft.setPosition(0.05);
                        foundationCaptureServoRight.setPosition(0.85);
                        //^ This is the Fully Latched Position
                        sleep(500);
                        roadrunner.move(skystonePoints.get(FoundationPosition));
                        foundationCaptureServoLeft.setPosition(0.6);
                        foundationCaptureServoRight.setPosition(0.3);
                        //^ This is the Fully In Position
                        sleep(500);
                        skystone_state = SKYSTONE_STATES.PARK;
                        break;
                    case PARK:
                        roadrunner.move(skystonePoints.get(WallSideParking));
                        skystone_state = SKYSTONE_STATES.DONE;
                        break;
                    case DONE:
                        telemetry.addLine("Finished");
                        telemetry.update();
                        break;
                }
            }
            else if(starting_position == STARTING_POSITION.BUILDING_ZONE){
                switch(foundation_states){
                    case STRAFE_LEFT:
                        break;
                        
                   /* STRAFE_LEFT,
                            MOVE_TOWARD_FOUNDATION,
                            LATCH_ARM,
                            WAIT,
                            BRING_FOUNDATION_BACK,
                            UNLATCH_ARM,
                            STRAFE_OUT_OF_FOUNDATION,
                            MOVE_TILL_BRIDGE
                    */
                   
                }
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

    
    void PickUpStone(){
        if (alliance == ALLIANCE.BLUE) {
            rightSkystoneArm.setPosition(0.33333);
            sleep(250);
            rightSkystoneHand.setPosition(0.4);
            sleep(500);
            rightSkystoneArm.setPosition(0);

        }
        else {
        }
    }

    void DropOffStone(){
        if (alliance == ALLIANCE.BLUE) {
            rightSkystoneArm.setPosition(0.33333);
            sleep(250);
            rightSkystoneHand.setPosition(0);
            sleep(500);
            rightSkystoneArm.setPosition(0);

        }
        else {
        }
    }







}
