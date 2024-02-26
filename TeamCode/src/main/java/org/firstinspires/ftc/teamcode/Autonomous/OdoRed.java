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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static final String TFOD_MODEL_ASSET = "CenterstageHeartTraining2.tflite";
    private static final String[] LABELS = {
            "heartBlue",
            "heartRed"
    };
    private AprilTagDetection desiredTag = null;
    double x;
    double y;
    int numObDet = 0;
    int numTagDet = 0;
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
    final double desiredDistance = 4;
    double rangeError = -2;
    double rangeBuffer = 1.25;
    double headingError = -2;
    double headingBuffer = 1.5;
    double yawError = -2;
    double yawBuffer = 2;
    boolean targetFound = false;
    int spikeMark = 1;
    private static int desiredTagID = 5;

    int alliance=1;
    int location = 1;
    int logCount = 0;
    double adjustablePow; // used to slow bot down when we get close to position in certain movements that need to be extra precise.

    // PID stuff
    double integralSum = 0;
    double integralSum2 = 0;

    double lasterror = 0;
    double lasterror2 = 0;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();

    double KpWristDown = 0.1;
    double KiWristDown = 0;
    double KdWristDown = 0;

    double KpWristTuck = 0;
    double KiWristTuck = 0;
    double KdWristTuck = 0;

    double KpWristUp = 0.1;
    double KiWristUp = 0;
    double KdWristUp = 0;
    // ticks
    int referenceWristDown = -131;
    int referenceWristUp = 95;
    double referenceWristTuck = 1.17;
    double wristSpeedLimit = 0.6;
    double wristPowerPID = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        wrist = hardwareMap.get(DcMotor.class, "wrist");
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        intakeClawLeft.setDirection(CRServo.Direction.REVERSE);

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

        initCameras();
        visionPortal.getProcessorEnabled(tfod);
        while(!opModeIsActive() && !isStopRequested()) {
            if (tfod != null && visionPortal.getCameraState().equals(CameraState.STREAMING)) {
                switchCameras();
                tfodtelemetry();
                if (cameraTimer.milliseconds() <= 2000 && cameraTimer.milliseconds() >= 500) {
                    setSpikeMark();
                }
                telemetry.addData("spike mark", spikeMark);
                telemetry.update();
            }
            else {
                //tfod didn't initialize or camera not yet streaming
                telemetry.addLine("camera not on");
                telemetry.update();
            }
        }

        waitForStart();
        telemetry.clearAll();

        visionPortal.getProcessorEnabled(aprilTag);
        if (spikeMark == 3){
            visionPortal.setActiveCamera(camera1);
            desiredTagID = 5;
        } else if (spikeMark == 2) {
            visionPortal.setActiveCamera(camera2);
            desiredTagID = 6;
        } else{
            visionPortal.setActiveCamera(camera2);
            desiredTagID = 5;
        }

        // repeating code - contains state machine!
        while(!isStopRequested()){
            runOdometry();

            // keep wrist in position when necessary
            if (step >= 12 && step <= 15) {
                wristPowerPID = PIDControlWristDown(KpWristUp, KiWristUp, KdWristUp, referenceWristUp, wrist.getCurrentPosition());
                wrist.setPower(0); // pid was being weird and i dont know enough to attempt to fix it
            }
            else if (stackedStep >= 1) {
                wristPowerPID = PIDControlWristDown(KpWristDown, KiWristDown, KdWristDown, referenceWristDown, wrist.getCurrentPosition());
                wrist.setPower(wristPowerPID);

            }

            // keep claws tight until their drops
            if (step < 15) {
                intakeClawLeft.setPower(0.5);
            }

            if ((spikeMark == 1 && stackedStep < 4) || (spikeMark == 2 && stackedStep < 3) || (spikeMark == 3 && stackedStep < 3)) {
                intakeClawRight.setPower(0.5);
            }


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
                    if (spikeMark == 3) {
                        position3();
                    } else if (spikeMark == 2) {
                        position2();
                    } else {
                        position1();
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
                    rotate(0.7);
                    if (pose[2] <= Math.toRadians(20)) {
                        rotate(0);
                        step++;
                    }
                    break;
                case 9: // through stage door
                    drive(0.9);
                    if (pose[0] >= 175) {
                        drive(0);
                        step++;
                    }
                    break;
                case 10: // turn to face board
                    rotate(0.9);
                    if (pose[2] <= Math.toRadians(-190)) {
                        rotate(0);
                        step++;
                    }
                    break;
                case 11://move in front of board
                    strafe(-0.4);
                    if (pose[1] <= 40) {
                        strafe(0);
                        step++;
                    }
                    break;
                case 12: //arm up
                    verticalArm.setPower(-1);
                    verticalArm2.setPower(-1);
                    sleep(1000);
                    verticalArm2.setPower(-0.05);
                    verticalArm.setPower(-0.05);
                    step++;
                    if (step == 13){
                        cameraTimer.reset();
                    }
                    break;
                case 13: // apriltag read
                    if (cameraTimer.milliseconds() >= 10000){
                        drive(0);
                        step++;
                    }
                    apriltagTelemetry();
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    if (numTagDet == 0){
                        drive(0);
                    }
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null) {
                            if ((desiredTagID == detection.id)) {
                                targetFound = true;
                                desiredTag = detection;
                                break;
                            } else {
                                telemetry.addLine("Not the right tag");
                                targetFound = false;
                            }
                        } else {
                            telemetry.addLine("cant find tag id");
                            targetFound = false;
                        }
                    }
                    if (targetFound == true) {
                        rangeError = (desiredTag.ftcPose.range - desiredDistance);
                        headingError = (desiredTag.ftcPose.bearing);
                        yawError = desiredTag.ftcPose.yaw;

                        telemetry.addData("range error", rangeError);
                        telemetry.addData("heading error", headingError);
                        telemetry.addData("yaw error", yawError);

                        driveToTag();

                        if (rangeError > -rangeBuffer && rangeError < rangeBuffer && yawError > -yawBuffer && yawError < yawBuffer && headingError > -headingBuffer && headingError < headingBuffer){
                            drive(0);
                            telemetry.addLine("lined up to tag");
                            step++;
                        }
                    } else {
                        strafe(-0.35);
                        telemetry.addLine("tag not found");
                    }

                    headingError = 0;
                    yawError = 0;
                    rangeError = 0;
                    targetFound = false;
                    break;
                case 14:// place pixel
                    intakeClawLeft.setPower(-1);
                    sleep(450);
                    intakeClawLeft.setPower(0);
                    sleep(100);
                    intakeClawLeft.setPower(1);
                    sleep(450);
                    intakeClawLeft.setPower(0);
                    step++;
                    break;
                case 15: //arm down
                    verticalArm.setPower(0.7);
                    verticalArm2.setPower(0.7);
                    sleep(1500);
                    verticalArm2.setPower(0);
                    verticalArm.setPower(0);
                    step++;
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
    public void customDrive(double lf, double rf, double lb, double rb) {
        // rotate left or right counter clockwise
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
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
                    verticalArm.setPower(-0.5);
                    verticalArm.setPower(-0.5);
                    sleep(1000);
                }
                break;
            case 1:
                //arm stuff
                verticalArm.setPower(0);
                verticalArm2.setPower(0);

                stackedStep++;
                break;
            case 2:
                drive(0.3);
                if (pose[1] >= 54){
                    drive(0);
                    stackedStep++;
                }
                break;
            case 3:
                //arm stuff

                intakeClawRight.setPower(-0.3);
                sleep(500);
                intakeClawRight.setPower(0.3);
                sleep(500);
                intakeClawRight.setPower(0);
                stackedStep++;
                break;
            case 4:
                drive(-0.3);
                if(pose[1] <= 10){
                    drive(0);
                    stackedStep++;
                }
                break;
            case 5:
                rotate(-0.3);
                if (pose[2] >= Math.toRadians(80)) {
                    rotate(0);
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
                    verticalArm.setPower(-0.5);
                    verticalArm.setPower(-0.5);
                    sleep(1000);
                }
                break;
            case 1:
                //arm stuff
                verticalArm.setPower(0);
                verticalArm2.setPower(0);

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
                intakeClawRight.setPower(-0.3);
                sleep(500);
                intakeClawRight.setPower(0.3);
                sleep(500);
                intakeClawRight.setPower(0);
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
                    verticalArm.setPower(-0.5);
                    verticalArm.setPower(-0.5);
                    sleep(1000);
                }
                break;
            case 1:
                drive(-0.3);
                if (pose[0] <= -10) {
                    drive(0);
                    stackedStep++;
                }
                break;
            case 2:
                //arm stuff
                verticalArm.setPower(0);
                verticalArm2.setPower(0);
                stackedStep++;
                break;
            case 3:
                strafe(-0.3);
                if (pose[1] >= 67){
                    drive(0);
                    stackedStep++;
                }
                break;
            case 4:
                drive(0.3);
                if (pose[0] >= 0) {
                    drive(0);
                    stackedStep++;
                }
                break;
            case 5:
                //arm stuff
                intakeClawRight.setPower(-0.3);
                sleep(500);
                intakeClawRight.setPower(0.3);
                sleep(500);
                intakeClawRight.setPower(0);
                stackedStep++;
                break;
            case 6:
                drive(-0.3);
                if (pose[0] <= -10) {
                    drive(0);
                    stackedStep++;
                }
                break;
            case 7:
                strafe(0.3);
                if(pose[1] <= 10){
                    drive(0);
                    stackedStep++;
                }
                break;
            case 8:
                rotate(-0.3);
                if (pose[2] >= Math.toRadians(425)) {
                    rotate(0);
                    step++;
                    pose[2] -= (2 * Math.PI); // to get back to the normal circle
                }
                break;
        }
    }

    //functions for cameras
    private void initCameras() {
        TfodProcessor.Builder builder = new TfodProcessor.Builder();
        builder.setModelAssetName(TFOD_MODEL_ASSET);
        builder.setModelLabels(LABELS);
        builder.setIsModelTensorFlow2(true);
        builder.setIsModelQuantized(true);
        builder.setModelInputSize(300);
        builder.setModelAspectRatio(16.0 / 9.0);
        tfod = builder.build();

        AprilTagProcessor.Builder builder2 = new AprilTagProcessor.Builder();
        builder2.setDrawAxes(true);
        builder2.setDrawCubeProjection(false);
        builder2.setDrawTagOutline(true);
        builder2.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11);
        builder2.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary());
        builder2.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
        aprilTag = builder2.build();

        VisionPortal.Builder builder3 = new VisionPortal.Builder();
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(camera1, camera2);
        builder3.setCamera(switchableCamera);
        builder3.setCameraResolution(new Size(640, 480));
        builder3.enableLiveView(true);
        builder3.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder3.setAutoStopLiveView(false);
        builder3.addProcessor(tfod);
        builder3.addProcessor(aprilTag);
        visionPortal = builder3.build();
        tfod.setMinResultConfidence(0.75f);

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
        else {
            telemetry.addLine("no camera change");
        }
    }

    private void apriltagTelemetry (){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        numTagDet = currentDetections.size();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
            else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        telemetry.update();
    }

    private void driveToTag (){
        double slow = 0.15;
        double medium = 0.3;
        if (yawError > yawBuffer && visionPortal.getActiveCamera().equals(camera1)){
            customDrive(-slow, 0, -slow, 0);
        } else if (yawError < -yawBuffer && visionPortal.getActiveCamera().equals(camera1)) {
            customDrive(slow, 0, slow, 0);
        } else if (yawError > yawBuffer && visionPortal.getActiveCamera().equals(camera2)){
            customDrive(0, slow, 0, slow);
        } else if (yawError < -yawBuffer && visionPortal.getActiveCamera().equals(camera2)) {
            customDrive(0, -slow, 0, -slow);
        } else if ( headingError > headingBuffer+3){
            customDrive(medium, -medium, -medium, medium);
        } else if (headingError < -headingBuffer-3){
            customDrive(-medium, medium, medium, -medium);
        } else if ( headingError > headingBuffer){
            customDrive(slow, -slow, -slow, slow);
        } else if (headingError < -headingBuffer){
            customDrive(-slow, slow, slow, -slow);
        } else if (rangeError > rangeBuffer+3){
            customDrive(-medium, -medium, -medium, -medium);
        } else if (rangeError < -rangeBuffer-3) {
            customDrive(medium, medium, medium, medium);
        } else if (rangeError > rangeBuffer){
            customDrive(-slow, -slow, -slow, -slow);
        } else if (rangeError < -rangeBuffer){
            customDrive(slow, slow, slow, slow);
        }
    }

    public double PIDControlWristDown(double Kp, double Ki, double Kd, double reference, int state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lasterror ) / timer.seconds();
        lasterror = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum *Ki);


        if (output > wristSpeedLimit) {
            output = wristSpeedLimit;
        }
        else if (output < -wristSpeedLimit) {
            output = -wristSpeedLimit;
        }

        telemetry.addData("PIDOutPutWristDown", output);
        return output;

    }

    public double PIDControlWristUp(double Kp, double Ki, double Kd, double reference, int state){
        double error = reference - state;
        integralSum2 += error * timer2.seconds();
        double derivative = (error - lasterror2 ) / timer2.seconds();
        lasterror2 = error;

        timer2.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum2 *Ki);
        telemetry.addData("PIDOutputWristUp", output);


        if (output > wristSpeedLimit) {
            output = wristSpeedLimit;
        }
        else if (output < -wristSpeedLimit) {
            output = -wristSpeedLimit;
        }

        return output;

    }
}