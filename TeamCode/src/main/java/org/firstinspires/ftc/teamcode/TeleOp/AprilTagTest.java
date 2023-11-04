package org.firstinspires.ftc.teamcode.TeleOp;


//import importaint things
import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
@TeleOp (name = "AprilTagTest" , group = "Iterative Opmode")
public class AprilTagTest extends OpMode {
    //DD=desired distance
    final double desiredDistance = 12.0;
    //forward speed control
    final double driveGain  =  0.02;
    //strafe speed control
    final double strafeGain =  0.015;
    // turn speed control
    final double turnGain   =  0.01;
    //max drive speed
    final double maxDrive = 0.5;
    //max strafe speed
    final double maxStrafe = 0.5;
    //max turn speed
    final double maxTurn = 0.3;
    //tells which allience we are on 1 red -1 blue
    final double allience = 1;


    // declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;


    //custom apriltag variable
    private AprilTagProcessor aprilTag;


    //custom visionportal variable
    private VisionPortal visionPortal;
    private AprilTagDetection desiredTag = null;
    //desired tag Id
    private static int DTID = -1;


    public void init() {
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftFront.setDirection(DcMotor.Direction.REVERSE );
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        // initiate apriltag see function for more
        initAprilTag();

        boolean targetFound = false;
    }

    public void loop(){

        double lt1 = gamepad1.left_trigger; // this is the value of the left trigger on gamepad1
        boolean b1 = gamepad1.b; // this is the value of the a button on gamepad1
        boolean x1 = gamepad1.x; // this is the value of the x button on gamepad1
        boolean y1 = gamepad1.y; // this is the value of the y button on gamepad1

        telemetry.addData("Gamepad:", 1);
        telemetry.addData("b1", b1);
        telemetry.addData("y1", y1);
        telemetry.addData("x1", x1);

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

        //start of actuall code

        if (x1 && allience == 1){
            DTID = 6;
        }
        else if (y1&& allience == 1){
            DTID = 7;
        }
        else if (b1 && allience == 1){
            DTID = 8;
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

}