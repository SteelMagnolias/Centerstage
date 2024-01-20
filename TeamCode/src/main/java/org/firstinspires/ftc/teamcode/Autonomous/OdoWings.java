package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;



@Autonomous(name = "OdoWings", group="Iterative OpMode")
public class OdoWings extends OpMode {

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

    // sensors
    private TouchSensor locationSwitch;

    // bot constraints:
    double trackWidth = 20.3; //(cm)!
    double yOffSet = -21; //(cm)!
    double wheelRadius = 1.75; // centimeters!
    double cpr = 8192; // counts per rotation!
    double wheelCircumference = 2 * Math.PI * wheelRadius;

    // current pose!
    double[] pose = {0,0,0};

    // previous encoder positions!
    double prevLeftEncoder = 0;
    double prevRightEncoder = 0;
    double prevBackEncoder = 0;

    // auton step / action!
    int step = 0;

    int logCount = 0;

    int spikeMark = 2;

    int location;

    @Override
    public void init() {
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

        locationSwitch = hardwareMap.get(TouchSensor.class, "allianceSwitch");
        intakeClawRight = hardwareMap.get(CRServo.class, "intakeClawRight");
        intakeClawLeft = hardwareMap.get(CRServo.class, "intakeClawLeft");

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // fix encoders resets - wrong motors reset might be issue!
        leftEncoder = rightFront;
        rightEncoder = verticalArm2;
        backEncoder = rightBack;


        prevLeftEncoder = leftEncoder.getCurrentPosition();
        prevRightEncoder = rightEncoder.getCurrentPosition();
        prevBackEncoder = backEncoder.getCurrentPosition();

        if (locationSwitch.isPressed()){
            location = -1;
        }
    }


    @Override
    public void loop() {
        // repeating code - contains state machine!

        runOdometry();

        logCount++;
        RobotLog.d("LogCount: " + logCount + "    Coordinates: (" + pose[0] + ", " + pose[1] + ")");


        switch(step) {
            case 0: // drive forward, away from wall
                drive(-0.3);
                if (pose[1] >= 20) {
                    drive(0);
                    if (spikeMark == 1) {
                        step++;
                    }
                    else if (spikeMark == 2) {
                        step+=5;
                    }
                    else {
                        step+=10;
                    }
                }
                break;
            case 1: // turn 270 degree to face SM 1
                // spike mark 1
                rotate(0.3);
                if (pose[2] <= Math.toRadians(270)) {
                    rotate(0);
                    step++;
                }
                break;
            case 2: // move to SM 1
                strafe(0.3);
                if (pose[1] >= (67)) {
                    rotate(0);
                    step++;
                }
                break;
            case 3: //unhook wrist, drop pixel, rehook wrist
                //do not odo things
                step++;
                break;
            case 4: // strafe back to not hit pixel
                strafe(-0.3);
                if (pose[1] <= (27)) {
                    rotate(0);
                    step+= 11;
                }
                break;
            case 5: // rotate to face SM2
                // spike mark 2
                rotate(0.3);
                if (pose[2] >=(Math.toRadians(170))) {
                    rotate(0);
                    step++;
                }
                break;
            case 6: // drive to reach SM2
                drive(0.3);
                if (pose[1] >= (55)){
                    drive(0);
                    step++;
                }
                break;
            case 7://unhook wrist, drop pixel, rehook wrist
                //do not odo things
                step++;
                break;
            case 8: // back away from pixel
                drive(-0.3);
                if (pose[1] <= (27)) {
                    drive(0);
                    step++;
                }
                break;
            case 9: // rotate to face o=correct direction
                rotate(0.3);
                if (pose[2] >= (Math.toRadians(260))){
                    rotate(0);
                    step=15;
                }
                break;
            case 10: //rotate to face SM3
                //spike mark 3
                rotate(0.3);
                if (pose[2] >= Math.toRadians(90)) {
                    rotate(0);
                    step++;
                }
                break;
            case 11: // strafe to line up with SM3
                strafe(-0.3);
                if (pose[1] >= (64)) {
                    rotate(0);
                    step++;
                }
                break;
            case 12://unhook wrist, drop pixel, rehook wrist
                //do not odo things
                step++;
                break;
            case 13: // move back to not hit pixel
                strafe(0.3);
                if (pose[1] <= (27)) {
                    rotate(0);
                    step++;
                }
                break;
            case 14: // turn to face correct direction
                rotate(0.3);
                if (pose[2] <=(Math.toRadians(270))) {
                    rotate(0);
                    step++;
                }
                break;
            case 15: // move back to not hit pixel
                //wings
                if (location == -1){
                    step+=9;
                    break;
                }
                drive(0.3);
                if (pose[0] <= (-25)){
                    drive (0);
                    step++;
                }
                break;
            case 16: // strafe to line up with stage door
                strafe(-0.3);
                if(pose[1] <= (-40)){
                    strafe(0);
                    step++;
                }
                break;
            case 17: // fix rotation
                rotate(0.3);
                if (pose[2] >= 200) {
                    rotate(0);
                    step++;
                }
                break;
            case 18: // wait in case auton conflicts
                //timer
                step++;
                break;
            case 19: // drive through stage door
                drive(-0.3);
                if(pose[0] >= (202)){
                    drive(0);
                    step++;
                }
                break;
            case 20: // line up with apriltags
                //apriltag stuff
                step++;
                break;
            case 21: // move forward to be at board
                drive(-0.2);
                if (pose[0] >= (213)){
                    drive(0);
                    step++;
                }
                break;
            case 22: // put pixel on board
                //arm stuff
                step++;
                break;
            case 23: // park for wings
                if (location == -1){
                    step++;
                    break;
                }
                strafe(0.3);
                if (pose[1] >= (117)){
                    strafe(0);
                }
                break;
            case 24: // park for backstage
                strafe(-0.3);
                if (pose[1] <= (5)){
                    strafe (0);
                }
                break;
            case 25: // backstage drive past pixel to do not hit it
                drive(-0.3);
                if (pose[0] >= (56)){
                    drive(0);
                    step=20;
                }
                break;
            default: // do nothing!
                drive(0);
                stop();
        }

        telemetry.addData("Pose0", pose[0]);
        telemetry.addData("Pose1", pose[1]);
        telemetry.addData("Pose2", Math.toDegrees(pose[2]));
        telemetry.addData("Case", step);

        telemetry.update();
    }


    @Override
    public void stop() {
        // stops code
    }

    public void runOdometry() {
        // runs odometry!

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
        This is because in GM0, they swap the x and y axis.
        I wanted it to be like a normal graph :)
         */

        //find change in y!
        double yChange = xCenter * Math.cos(pose[2]) - xPerp * Math.sin(pose[2]);
        telemetry.addData("xChange", yChange);

        // find changein x!
        double xChange = xCenter * Math.sin(pose[2]) + xPerp * Math.cos(pose[2]);
        telemetry.addData("yChange", xChange);

        pose[0] += -xChange; // negated to quickly make left negative and right positive
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