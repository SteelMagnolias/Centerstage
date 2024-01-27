package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;



@Autonomous(name = "OdometryTest", group="Iterative OpMode")
public class OdometryTest extends OpMode {

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


    // bot constraints:
    double trackWidth = 19.2; //(cm)!
    double yOffSet = -18.85; //(cm)!
    double wheelRadius = 1.75; // centimeters!
    double cpr = 8192; // counts per rotation!
    double wheelCircumference = 2 * Math.PI * wheelRadius;

    // current pose!
    double[] pose = {0,0,90};

    // previous encoder positions!
    double prevLeftEncoder = 0;
    double prevRightEncoder = 0;
    double prevBackEncoder = 0;

    // auton step / action!
    int step = 0;

    int logCount = 0;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        wrist = hardwareMap.get(DcMotor.class, "wrist");

        leftBack.setDirection(DcMotor.Direction.REVERSE );

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalArm = hardwareMap.get(DcMotor.class, "verticalArm");
        verticalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // hold position
        verticalArm2 = hardwareMap.get(DcMotor.class, "verticalArm");
        verticalArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // hold position

        verticalArm.setDirection(DcMotor.Direction.REVERSE);
        verticalArm2.setDirection(DcMotor.Direction.REVERSE);


        intakeClawRight = hardwareMap.get(CRServo.class, "intakeClawRight");
        intakeClawLeft = hardwareMap.get(CRServo.class, "intakeClawLeft");

        // fix encoders resets - wrong motors reset might be issue!
        leftEncoder = rightFront;
        rightEncoder = wrist;
        backEncoder = rightBack;

        prevLeftEncoder = leftEncoder.getCurrentPosition();
        prevRightEncoder = rightEncoder.getCurrentPosition();
        prevBackEncoder = -backEncoder.getCurrentPosition();
    }


    @Override
    public void loop() {
        // repeating code - contains state machine!

        runOdometry();

        logCount++;
        RobotLog.d("LogCount: " + logCount + "    Coordinates: (" + pose[0] + ", " + pose[1] + ")");


        switch(step) {
            case 0: // drive forward!
                drive(0.3);
                if (pose[1] >= 30) {
                    drive(0);
                    step++;
                }

                //setPIDSettings(1,0,0); // set the kp, ki, and kd for forward movemement
                //drivePID(0); // keep angle at 0 (moving forward in straight line)
                break;
            case 1: // strafe right!
                strafe(0.3);
                if (pose[0] >= 30) {
                    strafe(0);
                    step++;
                }
                break;
            case 2: // drive back!
                drive(-0.3);
                if (pose[1] <= 0) {
                    drive(0);
                    step++;
                }
                break;
            case 3: // strafe left!
                strafe(-0.3);
                if(pose[0] <= 0) {
                    strafe(0);
                    step++;
                }
                break;
            case 4: // rotate clockwise!
                rotate(0.3);
                if (pose[2] <= Math.toRadians(0)) {
                    rotate(0);
                    step++;
                }
                break;
            case 5: // strafe to center of field
                strafe(-0.3);
                if (pose[1] >= 30) {
                    strafe(0);
                    step++;
                }
                break;
            case 6: // strafe back to original position
                strafe(0.3);
                if (pose[1] <= 0) {
                    strafe(0);
                    step++;
                }
                break;
            case 7: // rotate counter-clockwise
                rotate(-0.3);
                if (pose[2] >= Math.toRadians(180)) {
                    rotate(0);
                    step++;
                }
                break;
            case 8: // strafe towards center of field
                strafe(0.3);
                if (pose[1] >= 30) {
                    strafe(0);
                    step++;
                }
                break;
            case 9: // strafe back to starting position
                strafe(-0.3);
                if (pose[1] <= 0) {
                    strafe(0);
                    step++;
                }
                break;
            case 10: // rotate to face truss
                rotate(-0.3);
                if (pose[2] >= Math.toRadians(360)) {
                    rotate(0);
                    step++;
                }
                break;
            case 11: // move towards truss
                drive(0.3);
                if (pose[0] >= 30) {
                    drive(0);
                    step++;
                }
                break;
            case 12: // move back from truss to origin position
                drive(-0.3);
                if (pose[0] <= 0) {
                    drive(0);
                    step++;
                }
                break;
            case 13: // rotate to face field wall
                rotate(0.3);
                if (pose[2] <= Math.toRadians(180)) {
                    rotate(0);
                    step++;
                }
                break;
            case 14: // move forward towards field wall
                drive(0.3);
                if (pose[0] <= -30) {
                    drive(0);
                    step++;
                }
                break;
            case 15: // move backward to origin point
                drive(-0.3);
                if (pose[0] >= 0) {
                    drive(0);
                    step++;
                }
                break;
            case 16: // rotate to 110 degrees
                rotate(0.3);
                if (pose[2] <= Math.toRadians(110)) {
                    rotate(0);
                    step++;
                }
                break;
            case 17: // drive forward
                drive(0.3);
                if (pose[1] >= 30) {
                    drive(0);
                    step++;
                }
                break;
            case 18: // drive backwards
                drive(-0.3);
                if (pose[1] <= 0) {
                    drive(0);
                    step++;
                }
                break;
            case 19: // rotate to 45 degrees
                rotate(0.3);
                if (pose[2] <= Math.toRadians(45)) {
                    rotate(0);
                    step++;
                }
                break;
            case 20: // strafe left
                strafe(-0.3);
                if (pose[0] >= 30) {
                    strafe(0);
                    step++;
                }
                break;
            case 21: // strafe right
                strafe(0.3);
                if (pose[0] <= 0) {
                    strafe(0);
                    step++;
                }
                break;
            case 22: // rotate to -45 degrees
                rotate(0.3);
                if (pose[2] <= Math.toRadians(-45)) {
                    rotate(0);
                    step++;
                }
                break;
            case 23: // move backwards until 30 cm
                drive(-0.3);
                if (pose[1] >= 30) {
                    drive(0);
                    step++;
                }
                break;
            case 24: // move forwards until back at origin
                drive(0.3);
                if (pose[1] <= 0) {
                    drive(0);
                    step++;
                }
                break;
            case 25: // rotate to be -110 degrees
                rotate(0.3);
                if (pose[2] <= -110) {
                    rotate(0);
                    step++;
                }
                break;
            case 26: // drive backwards until 30 cm from field wall
                drive(-0.3);
                if (pose[1] >= 30) {
                    drive(0);
                    step++;
                }
                break;
            case 27: // drive forwards until back at origin
                drive(0.3);
                if (pose[1] <= 0) {
                    drive(0);
                    step++;
                }
                break;
            default: // do nothing!
                drive(0);
                stop();
        }

        telemetry.addData("Pose0", pose[0]);
        telemetry.addData("Pose1", pose[1]);
        telemetry.addData("Pose2", pose[2]);
        telemetry.addData("Case", step);

        telemetry.update();
    }


    @Override
    public void stop() {
        // stops code
    }

    public void runOdometry() {
        // runs odometry!
        // robot front faces 90 degrees
        // x direction is horizontal is parallel to driver station
        // y direction is perpendicular to driver station

        // distance wheel turns in cm!
        double rawLeftEncoder = leftEncoder.getCurrentPosition();
        double rawRightEncoder = rightEncoder.getCurrentPosition();
        double rawBackEncoder = backEncoder.getCurrentPosition();

        telemetry.addData("Raw Left", rawLeftEncoder);
        telemetry.addData("Raw Right", rawRightEncoder);
        telemetry.addData("Raw Back", rawBackEncoder);

        double rawChangeLeft = ((rawLeftEncoder - prevLeftEncoder) / cpr) * wheelCircumference;
        double rawChangeRight = ((rawRightEncoder - prevRightEncoder) / cpr) * wheelCircumference;
        double rawChangeBack = ((rawBackEncoder - prevBackEncoder) / cpr) * wheelCircumference;

        telemetry.addData("Raw Left Change", rawChangeLeft);
        telemetry.addData("Raw Right Change", rawChangeRight);
        telemetry.addData("Raw Back Change", rawChangeBack);

        // find change in theta!
        double deltaTheta = (rawChangeLeft - rawChangeRight) / trackWidth;
        telemetry.addData("deltaTheta", deltaTheta);

        // find change of x (center)!
        double xCenter = (rawChangeLeft + rawChangeRight) / 2;
        telemetry.addData("xCenter", xCenter);

        // find change in x perpendicular!
        double xPerp = rawChangeBack - (yOffSet * deltaTheta);
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

        telemetry.addData("prevLeftEncoder", rawChangeLeft);
        telemetry.addData("Raw Left Change", rawChangeLeft);
        telemetry.addData("Raw Left Change", rawChangeLeft);

        logCount++;
        RobotLog.d("LogCount: " + logCount + "    rawLeftEncoder: " + rawLeftEncoder);
        logCount++;
        RobotLog.d("LogCount: " + logCount + "    rawRightEncoder: " + rawRightEncoder);
        logCount++;
        RobotLog.d("LogCount: " + logCount + "    rawBackEncoder: " + rawBackEncoder);
        logCount++;
        RobotLog.d("LogCount: " + logCount + "    pose[0]: " + pose[0]);
        logCount++;
        RobotLog.d("LogCount: " + logCount + "    pose[1]: " + pose[1]);
        logCount++;
        RobotLog.d("LogCount: " + logCount + "    pose[2]: " + pose[2]);
        logCount++;
        RobotLog.d("LogCount: " + logCount + "    ");

        logCount++;
        RobotLog.d("LogCount: " + logCount + "    Coordinates: (" + pose[0] + ", " + pose[1] + ")");
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
        leftFront.setPower(-pow);
        rightFront.setPower(pow);
        leftBack.setPower(pow);
        rightBack.setPower(-pow);
    }

    public void rotate(double pow) {
        // rotate left or right counter clockwise
        leftFront.setPower(-pow);
        rightFront.setPower(pow);
        leftBack.setPower(-pow);
        rightBack.setPower(pow);
    }
}