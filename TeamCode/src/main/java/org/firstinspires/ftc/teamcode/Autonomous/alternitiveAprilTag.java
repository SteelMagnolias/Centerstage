package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "alternitiveAprilTag", group="Iterative OpMode")
public class alternitiveAprilTag extends LinearOpMode {

    // declare motors!
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

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
    final double desiredDistance = 4;
    double rangeError = -2;
    double rangeBuffer = 1.25;
    double headingError = -2;
    double headingBuffer = 1.5;
    double yawError = -2;
    double yawBuffer = 2;
    boolean targetFound = false;
    int spikeMark = 3;
    private static int desiredTagID = 5;
    int alliance=1;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftBack.setDirection(DcMotor.Direction.REVERSE );

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        camera1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera2 = hardwareMap.get(WebcamName.class, "Webcam 2");

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
                telemetry.update();
            }
            else {
                //tfod didn't initialize or camera not yet streaming
                telemetry.addLine("camera not on");
                telemetry.update();
            }
        }

        waitForStart();
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

        while (!isStopRequested()){
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
            } else {
                drive(0);
                telemetry.addLine("tag not found");
            }
            if (rangeError > -rangeBuffer && rangeError < rangeBuffer && yawError > -yawBuffer && yawError < yawBuffer && headingError > -headingBuffer && headingError < headingBuffer){
                drive(0);
                telemetry.addLine("lined up to tag");
                //step++;
            }
            headingError = 0;
            yawError = 0;
            rangeError = 0;
            targetFound = false;
        }

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
}