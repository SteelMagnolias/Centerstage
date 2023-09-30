package org.firstinspires.ftc.teamcode.TeleOp;

//import importaint things
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp (name = "AprilTagTest" , group = "Iterative Opmode")
public class AprilTagTest extends LinearOpMode {

    //custom apriltag variable
    private AprilTagProcessor aprilTag;

    //custom visionportal variable
    private VisionPortal visionPortal;

    public void runOpMode() {
        // initiate apriltag see function for more
        initAprilTag();

        waitForStart();
        while (true) {
            //send telemetry see function for more
            aprilTagTelemetry();
        }
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
        //calibration?????????
        builder.setLensIntrinsics(578.272, 578.272, 402.145, 221.506);
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

    private void aprilTagTelemetry() {
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

}