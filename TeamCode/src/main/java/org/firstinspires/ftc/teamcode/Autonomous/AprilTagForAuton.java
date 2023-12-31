package org.firstinspires.ftc.teamcode.Autonomous;


//import importaint things
import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;
import java.util.concurrent.TimeUnit;


@Autonomous(name = "AprilTagForAuton" ,  group="Linear OpMode")
public class AprilTagForAuton extends LinearOpMode {
    // how far from tag we want to be
    final double desiredDistance = 6;
    //forward speed control
    final double driveGain = 0.02;
    //strafe speed control
    final double strafeGain = 0.015;
    // turn speed control
    final double turnGain = 0.01;
    //max drive speed
    final double maxDrive = 0.1875;
    //max strafe speed
    final double maxStrafe = 0.1875;
    //max turn speed
    final double maxTurn = 0.1125;

    double rangeError = -2;
    double headingError = -2;
    double yawError = -2;

    // declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private TouchSensor allianceSwitch;


    //custom apriltag variable
    private AprilTagProcessor aprilTag;


    //custom visionportal variable
    private VisionPortal visionPortal;
    private AprilTagDetection desiredTag = null;
    //desired tag Id middle red will change with allience
    private static int desiredTagID = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        allianceSwitch = hardwareMap.get(TouchSensor.class, "allianceSwitch");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        if (allianceSwitch.isPressed()) {
            // if alliance switch is pressed, we are on the blue alliance and desired tag is changed
            desiredTagID = 2;
        }

        // initiate apriltag see function for more
        initAprilTag();


        boolean targetFound = false;
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        apriltagTelemetry();

        waitForStart();

        //drive through gate/to middle of backstage side
        //turn too far abiut 45 degrees face the corner of the tile

        while (rangeError < -1 || rangeError > 1 || headingError < -1 || headingError > 1 || yawError < -1 || yawError > 1) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
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
                }
            }
            if (targetFound = false) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            } else if (targetFound = true) {
                rangeError = (desiredTag.ftcPose.range - desiredDistance);
                headingError = desiredTag.ftcPose.bearing;
                yawError = desiredTag.ftcPose.yaw;

                telemetry.addData("range error", rangeError);
                telemetry.addData("heading error", headingError);
                telemetry.addData("yaw error", yawError);

                drive = Range.clip(rangeError * driveGain, -maxDrive, maxDrive);
                turn = Range.clip(headingError * turnGain, -maxTurn, maxTurn);
                strafe = Range.clip(-yawError * strafeGain, -maxStrafe, maxStrafe);

                telemetry.addData("forwards value", drive);
                telemetry.addData("turn value", turn);
                telemetry.addData("strafe value", strafe);

                tagDrive(drive, turn, strafe);

                apriltagTelemetry();

            }
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        //place pixel on board and park
    }

    private void apriltagTelemetry (){

        //list all detected apriltags
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        //display number of tags detected
        telemetry.addData("# AprilTags Detected", currentDetections.size());


        //list all detections
        for (AprilTagDetection detection : currentDetections) {
            //if there is tags with meta data (measurments
            if (detection.metadata != null) {
                //detection id and tag name
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                //location in xyz cords
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                // tilt in xyz degrees
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                //how far to turn to get to each
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
            //if tags dont have metadata
            else {
                //detection id
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                // location in xy cords
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }


        //telemetry code
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");


        //update telemetry
        telemetry.update();
    }

    private void initAprilTag(){
        //start duilding custom apriltag
        AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder();
        //draw axes on camera stream
        builder.setDrawAxes(true);
        //draw a 3d cube around the tag
        builder.setDrawCubeProjection(false);
        //draw tag outine on camera stream
        builder.setDrawTagOutline(true);
        //pick a family not sure what these are
        builder.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11);
        //use centerstage tag library to ID tags
        builder.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary());
        //set output units
        builder.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
        //create custom apriltag
        aprilTag = builder.build();


        //start creating custom vision portal
        VisionPortal.Builder builder2 = new VisionPortal.Builder();
        //use webcam 1
        builder2.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        //set camera resolution
        builder2.setCameraResolution(new Size(640, 480));
        //allow to be seem on driver hub
        builder2.enableLiveView(true);
        //set bandwith
        builder2.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        //do not autostop
        builder2.setAutoStopLiveView(false);
        //add custom apriltag processor
        builder2.addProcessor(aprilTag);
        //create custom vision portal
        visionPortal = builder2.build();
    }
    private void tagDrive(double x, double y, double yaw){
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        telemetry.addData("left front power", leftFrontPower);
        telemetry.addData("left back power", leftBackPower);
        telemetry.addData("right back power", rightBackPower);
        telemetry.addData("right front power", rightFrontPower);
    }
}