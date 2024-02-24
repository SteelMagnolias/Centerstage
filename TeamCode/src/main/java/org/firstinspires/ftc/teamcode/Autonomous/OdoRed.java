package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortal.CameraState;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import android.util.Size;

@Autonomous(name = "OdoRed", group="Iterative OpMode")
public class OdoRed extends LinearOpMode {

    // declare motors!
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private DcMotor wrist; // core hex on wrist

    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor backEncoder;

    private DcMotor verticalArm;
    private DcMotor verticalArm2;

    // servos
    private CRServo intakeClawRight;
    private CRServo intakeClawLeft;

    //cameras
    private WebcamName camera1;
    private WebcamName camera2;

    //camera variables
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private static final String TFOD_MODEL_ASSET = "CenterstageHeartTraining2.tflite";
    private static final String[] LABELS = {
            "heartBlue",
            "heartRed"
    };
    double x;
    double y;
    int numObDet = 0;
    ElapsedTime cameraTimer = new ElapsedTime();

    // bot constraints:
    double TRACK_WIDTH = 20; //(cm)!
    double TRACK_WIDTH_DELTA = 0;
    double Y_OFFSET = -13.5; //(cm)!
    double Y_OFFSET_DELTA = 0;
    double WHEEL_LEFT_DIAMETER = 3.469; // centimeters!
    double WHEEL_RIGHT_DIAMETER = 3.315;
    double WHEEL_BACK_DIAMETER = 3.471;
    double CPR = 8192; // counts per rotation!
    double WHEEL_CIRCUMFERENCE_LEFT = Math.PI * WHEEL_LEFT_DIAMETER;
    double WHEEL_CIRCUMFERENCE_RIGHT = Math.PI * WHEEL_RIGHT_DIAMETER;
    double WHEEL_CIRCUMFERENCE_BACK = Math.PI * WHEEL_BACK_DIAMETER;

    // current pose!
    double[] pose = {0,0,Math.toRadians(270)};

    // previous encoder positions!
    double prevLeftEncoder = 0;
    double prevRightEncoder = 0;
    double prevBackEncoder = 0;

    // auton step / action!
    int step = 0;
    int stackedStep = 0;
    int spikeMark = 3;
    int alliance=1;
    int location = 1;
    int logCount = 0;
    double adjustablePow; // used to slow bot down when we get close to position in certain movements that need to be extra precise.

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        wrist = hardwareMap.get(DcMotor.class, "wrist");

        leftBack.setDirection(DcMotor.Direction.REVERSE );


        verticalArm = hardwareMap.get(DcMotor.class, "verticalArm");
        verticalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // hold position
        verticalArm2 = hardwareMap.get(DcMotor.class, "verticalArm2");
        verticalArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // hold position

        verticalArm.setDirection(DcMotor.Direction.REVERSE);
        verticalArm2.setDirection(DcMotor.Direction.REVERSE);


        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intakeClawRight = hardwareMap.get(CRServo.class, "intakeClawRight");
        intakeClawLeft = hardwareMap.get(CRServo.class, "intakeClawLeft");

        camera1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera2 = hardwareMap.get(WebcamName.class, "Webcam 2");

        // fix encoders resets - wrong motors reset might be issue!
        leftEncoder = rightFront;
        rightEncoder = verticalArm2;
        backEncoder = rightBack;

        prevLeftEncoder = -leftEncoder.getCurrentPosition();
        prevRightEncoder = -rightEncoder.getCurrentPosition();
        prevBackEncoder = -backEncoder.getCurrentPosition();

        cameraTimer.reset();

        initTfod();
        while(!opModeIsActive() && !isStopRequested()) {
            if (tfod != null && visionPortal.getCameraState().equals(CameraState.STREAMING)) {
                switchCameras();
                tfodtelemetry();
                if (cameraTimer.milliseconds() <= 2000 || cameraTimer.milliseconds() >= 100) {
                    setSpikeMark();
                }
                telemetry.update();
            }
            else {
                //tfod didn't initialize or camera not yet streaming
                telemetry.addLine("camera not on");
                telemetry.update();
            }
        }

        waitForStart();
        // repeating code - contains state machine!
        while(!isStopRequested()){
            runOdometry();

            logCount++;
            //RobotLog.d("LogCount: " + logCount + "    Coordinates: (" + pose[0] + ", " + pose[1] + ")");


            switch(step) {
                case 0: // drive backward to give space for rotation!
                    drive(-0.3);
                    if (pose[1] >= 5) {
                        drive(0);
                        step++;
                    }

                    //setPIDSettings(1,0,0); // set the kp, ki, and kd for forward movemement
                    //drivePID(0); // keep angle at 0 (moving forward in straight line)
                    break;
                case 1:
                    if (spikeMark == 1) {
                        position1();
                    } else if (spikeMark == 2) {
                        position2();
                    } else {
                        position3();
                    }
                    break;
                case 2: // strafe left towards airplanes
                    if (pose[0] <= -34) {
                        adjustablePow = -0.2;
                    } else {
                        adjustablePow = -0.3;
                    }
                    strafe(adjustablePow);
                    if (pose[0] <= -44) {
                        strafe(0);
                        step++;
                    }
                    break;
                case 3: // rotate until forward to avoid stacks
                    rotate(0.1);
                    if (pose[2] <= Math.toRadians(100)) {
                        rotate(0);
                        step++;
                    }
                    break;
                case 4: // drive forward to middle but stop around first spike mark for a readjust
                    drive(0.5);
                    if (pose[1] >= 60) {
                        drive(0);
                        step++;
                    }
                    break;
                case 5: // rotate until forward to avoid stacks
                    rotate(0.1);
                    if (pose[2] <= Math.toRadians(100)) {
                        rotate(0);
                        step++;
                    }
                    break;
                case 6: // drive forward to middle
                    drive(0.5);
                    if (pose[1] >= 115) {
                        drive(0);
                        step++;
                    }

                    break;
                case 7: // strafe to not hit pixel stacks
                    strafe(0.3);
                    if (pose[0] >= 0) {
                        drive(0);
                        step++;
                    }
                    break;
                case 8: // turn to go through truss
                    rotate(0.3);
                    if (pose[2] <= Math.toRadians(0)) {
                        rotate(0);
                        step++;
                    }
                    break;
                case 9: // through stage door
                    drive(0.3);
                    if (pose[0] >= 180) {
                        drive(0);
                        step++;
                    }
                    break;
                case 10: // turn to face board
                    rotate(0.3);
                    if (pose[2] <= Math.toRadians(-190)) {
                        rotate(0);
                        step++;
                    }
                    break;
                case 11://move in front of board
                    strafe(-0.3);
                    if (pose[1] <= 30) {
                        strafe(0);
                        step++;
                    }
                    break;
                case 12: // apriltag read
                    step++;
                    break;
                case 13: //arm up
                    step++;
                    break;
                case 14: // move in to board
                    drive(-0.3);
                    if (pose[0] >= 220) {
                        drive(0);
                        step++;
                    }
                    break;
                case 15:// place pixel
                    step++;
                    break;
                case 16:
                    drive(0.3);
                    if (pose[0] <= 180) {
                        drive(0);
                        step++;
                    }
                    break;
                case 17: //arm down
                    step++;
                    break;
                case 18:
                    strafe(-0.3);
                    if (pose[1] <= 102) {
                        strafe(0);
                        step++;
                    }
                    break;
                case 19:
                    drive(0.3);
                    if (pose[0] >= 220) {
                        drive(0);
                        step++;
                    }
                    break;
                default: // do nothing!
                    drive(0);
                    stop();
            }
        }

        telemetry.addData("Pose0", pose[0]);
        telemetry.addData("Pose1", pose[1]);
        telemetry.addData("Pose2", Math.toDegrees(pose[2]));
        telemetry.addData("Case", step);

        telemetry.update();
    }

    public void runOdometry() {
        // runs odometry!
        // robot front faces 90 degrees
        // x direction is horizontal is parallel to driver station
        // y direction is perpendicular to driver station

        // distance wheel turns in cm!
        double rawLeftEncoder = -leftEncoder.getCurrentPosition();
        double rawRightEncoder = -rightEncoder.getCurrentPosition();
        double rawBackEncoder = backEncoder.getCurrentPosition();

        /*telemetry.addData("Raw Left", rawLeftEncoder);
        telemetry.addData("Raw Right", rawRightEncoder);
        telemetry.addData("Raw Back", rawBackEncoder);*/

        double rawChangeLeft = ((rawLeftEncoder - prevLeftEncoder) / CPR) * WHEEL_CIRCUMFERENCE_LEFT;
        double rawChangeRight = ((rawRightEncoder - prevRightEncoder) / CPR) * WHEEL_CIRCUMFERENCE_RIGHT;
        double rawChangeBack = ((rawBackEncoder - prevBackEncoder) / CPR) * WHEEL_CIRCUMFERENCE_BACK;

        /*telemetry.addData("Raw Left Change", rawChangeLeft);
        telemetry.addData("Raw Right Change", rawChangeRight);
        telemetry.addData("Raw Back Change", rawChangeBack);*/

        // find change in theta!
        double deltaTheta = -(rawChangeLeft - rawChangeRight) / (TRACK_WIDTH + TRACK_WIDTH_DELTA);
        telemetry.addData("deltaTheta", deltaTheta);

        // find change of x (center)!
        double xCenter = (rawChangeLeft + rawChangeRight) / 2;
        telemetry.addData("xCenter", xCenter);

        // find change in x perpendicular!
        double xPerp = rawChangeBack - ((Y_OFFSET + Y_OFFSET_DELTA) * deltaTheta);
        telemetry.addData("xPerp", xPerp);

        /*
        NOTICE - yChange and xChange (below) are swapped from GM0
        This is because in GM0, they swap the x and y axis
        I wanted it to be like a normal graph
         */

        //find change in x!
        double xChange = xCenter * Math.cos(pose[2]) - xPerp * Math.sin(pose[2]);
        telemetry.addData("xChange", xChange);

        // find changein y!
        double yChange = xCenter * Math.sin(pose[2]) + xPerp * Math.cos(pose[2]);
        telemetry.addData("yChange", yChange);

        pose[0] += xChange;
        pose[1] += yChange;
        pose[2] += deltaTheta;

        prevLeftEncoder = rawLeftEncoder;
        prevRightEncoder = rawRightEncoder;
        prevBackEncoder = rawBackEncoder;

        /*telemetry.addData("prevLeftEncoder", rawChangeLeft);
        telemetry.addData("Raw Left Change", rawChangeLeft);
        telemetry.addData("Raw Left Change", rawChangeLeft);*/

       // RobotLog.d("values: " + rawLeftEncoder + ", " + rawRightEncoder + ", " + rawBackEncoder + ", " + pose[0] + ", " + pose[1] + ", " + pose[2]);
    }


    public void drive(double pow) {
        // drive forward or backward
        leftFront.setPower(pow);
        rightFront.setPower(pow);
        leftBack.setPower(pow);
        rightBack.setPower(pow);

    }

    public void strafe(double pow) {
        // strafe left
        leftFront.setPower(pow);
        rightFront.setPower(-pow);
        leftBack.setPower(-pow);
        rightBack.setPower(pow);
    }

    public void rotate(double pow) {
        // rotate left or right counter clockwise
        leftFront.setPower(pow);
        rightFront.setPower(-pow);
        leftBack.setPower(pow);
        rightBack.setPower(-pow);
    }

    public void position2() {
        telemetry.addData("stackedStep", stackedStep);
        RobotLog.d("Pose:" + pose[0] + "," + pose[1] + "," +  pose[2] + "Stacked Step: " + stackedStep);
        switch(stackedStep) {
            case 0:
                rotate(0.3); // rotate 180 degrees to face spike mark 2
                if (pose[2] <= Math.toRadians(100)) {
                    rotate(0);
                    stackedStep++;
                }
                break;
            case 1:
                //arm stuff
                stackedStep++;
                break;
            case 2:
                drive(0.3);
                if (pose[1] >= 63){
                    drive(0);
                    stackedStep++;
                }
                break;
            case 3:
                //arm stuff
                stackedStep++;
                break;
            case 4:
                drive(-0.3);
                if(pose[1] <= 10){
                    drive(0);
                    step++;
                }
                break;
        }
    }

    public void position1() {
        telemetry.addData("stackedStep", stackedStep);
        RobotLog.d("Pose:" + pose[0] + "," + pose[1] + "," + pose[2] + "Stacked Step: " + stackedStep);
        switch (stackedStep) {
            case 0:
                rotate(0.3); // rotate 90 degrees to face spike mark 1
                if (pose[2] <= Math.toRadians(190)) {
                    rotate(0);
                    stackedStep++;
                }
                break;
            case 1:
                //arm stuff
                stackedStep++;
                break;
            case 2:
                strafe(0.3);
                if (pose[1] >= 63) {
                    drive(0);
                    stackedStep++;
                }
                break;
            case 3:
                drive(-0.2);
                if (pose[0] >= -3) {
                    drive(0);
                    stackedStep++;
                }
                break;
            case 4:
                //arm stuff
                stackedStep++;
                break;
            case 5:
                strafe(-0.3);
                if (pose[1] <= 10) {
                    drive(0);
                    stackedStep++;
                }
                break;
            case 6:
                rotate(0.3);
                if (pose[2] <= Math.toRadians(100)) {
                    rotate(0);
                    step++;
                }
                break;
        }
    }

    public void position3() {
        telemetry.addData("stackedStep", stackedStep);
        RobotLog.d("Pose:" + pose[0] + "," + pose[1] + "," +  pose[2] + "Stacked Step: " + stackedStep);
        switch(stackedStep) {
            case 0:
                rotate(-0.3); // rotate 90 degrees to face spike mark 1
                if (pose[2] >= Math.toRadians(350)) {
                    rotate(0);
                    stackedStep++;
                }
                break;
            case 1:
                //arm stuff
                stackedStep++;
                break;
            case 2:
                strafe(-0.3);
                if (pose[1] >= 63){
                    drive(0);
                    stackedStep++;
                }
                break;
            case 3:
                //arm stuff
                stackedStep++;
                break;
            case 4:
                strafe(0.3);
                if(pose[1] <= 10){
                    drive(0);
                    stackedStep++;
                }
                break;
            case 5:
                rotate(-0.3);
                if (pose[2] >= Math.toRadians(440)) {
                    rotate(0);
                    step++;
                    pose[2] -= (2 * Math.PI); // to get back to the normal circle
                }
                break;
        }
    }
    //functions for cameras
    private void initTfod() {
        // start building custom tfod
        TfodProcessor.Builder builder = new TfodProcessor.Builder();
        //custom training model file import
        builder.setModelAssetName(TFOD_MODEL_ASSET);
        //get names of objects
        builder.setModelLabels(LABELS);
        // verifies the type of tensor flow model???? not really sure
        builder.setIsModelTensorFlow2(true);
        // no clue
        builder.setIsModelQuantized(true);
        // how many minutes of model will be used.
        builder.setModelInputSize(300);
        //set camera aspect ratio
        builder.setModelAspectRatio(16.0 / 9.0);
        //make custom tfod
        tfod = builder.build();

        //start building custom vision portal
        VisionPortal.Builder builder2 = new VisionPortal.Builder();
        //pick a camera
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(camera1, camera2);
        builder2.setCamera(switchableCamera);
        // set camera resolution
        //this one already has pre calibrated apriltag info
        builder2.setCameraResolution(new Size(640, 480));
        //allow you to see the camera feed on driver hub
        builder2.enableLiveView(true);
        // streaming bandwidth
        builder2.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        // don't auto stop camera feed
        builder2.setAutoStopLiveView(false);
        //add custom tfod to vision portal
        builder2.addProcessor(tfod);
        //build vision portal
        visionPortal = builder2.build();
        //must be at least 75% confident to display data
        tfod.setMinResultConfidence(0.75f);

        //processor is on
        visionPortal.setProcessorEnabled(tfod, true);
        telemetry.addData("camera state", visionPortal.getCameraState());
    }

    //function for tfod telemetry
    private void tfodtelemetry() {
        telemetry.addData("camera #", visionPortal.getActiveCamera());

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        telemetry.addData("# Objects Detected", currentRecognitions.size());
        numObDet = currentRecognitions.size();

        for (Recognition recognition : currentRecognitions) {
            //average x coordinates
            x = (recognition.getLeft() + recognition.getRight()) / 2;
            //average y coordinates
            y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            //determine alliance
            if (recognition.getLabel().equals("heartRed")) {
                alliance = 1;
                telemetry.addLine("red alliance (1)");
            }
            else if (recognition.getLabel().equals("heartBlue")) {
                alliance = -1;
                telemetry.addLine("blue alliance (-1)");
            }
        }
        currentRecognitions = null;
    }

    private void setSpikeMark(){
       if (numObDet == 1 && x != 0) {
           // determine spike mark
           if (visionPortal.getActiveCamera().equals(camera1)) {
               if (x < 300) {
                   spikeMark = 1;
                   telemetry.addLine("Spike Mark Left-1");
               } else if (x > 299) {
                   spikeMark = 2;
                   telemetry.addLine("Spike Mark Middle (Left)-2");
               }
           } else if (visionPortal.getActiveCamera().equals(camera2)) {
               if (x < 300) {
                   spikeMark = 2;
                   telemetry.addLine("Spike Mark Middle (Right)-2");
               } else if (x > 299) {
                   spikeMark = 3;
                   telemetry.addLine("Spike Mark Right-3");
               }
           }
       }
       telemetry.addData("spike Marker", spikeMark);
        x = 0;
        numObDet = 0;
    }
    private void switchCameras() {
        if (cameraTimer.milliseconds() >= 2500) {
            visionPortal.stopStreaming();
            if (visionPortal.getActiveCamera().equals(camera1)) {
                visionPortal.setActiveCamera(camera2);
            } else if (visionPortal.getActiveCamera().equals(camera2)) {
                visionPortal.setActiveCamera(camera1);
            }
            visionPortal.resumeStreaming();
            cameraTimer.reset();
        }
    }
}