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

    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor backEncoder;

    //private DcMotor verticalArm;
    //private DcMotor hangArm;

    //private CRServo hangBolts;



    // servos
    //private CRServo intakeClawRight;
    //private CRServo intakeClawLeft;


    // bot constraints:
    double trackWidth = 20.32; //(cm)!
    double yOffSet = -19.0; //(cm)!
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

    ElapsedTime timer = new ElapsedTime();

    // PID STUFF
    double integralSum = 0;
    double Kp = 0;
    double Ki= 0;
    double Kd = 0;
    //could add another controller if needed
    double reference = 0.8;


    ElapsedTime PIDTimer = new ElapsedTime();
    double lasterror = 0;

    @Override
    public void init() {
        // init!
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftBack.setDirection(DcMotor.Direction.REVERSE );

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //verticalArm = hardwareMap.get(DcMotor.class, "verticalArm");
        //verticalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // hold position

        //intakeClawRight = hardwareMap.get(CRServo.class, "intakeClawRight");
        //intakeClawLeft = hardwareMap.get(CRServo.class, "intakeClawLeft");

        //hangArm = hardwareMap.get(DcMotor.class, "hangArm");
        //hangArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //hangBolts = hardwareMap.get(CRServo.class, "hangBolts");

        leftEncoder = leftBack;
        rightEncoder = leftFront;
        backEncoder = leftBack;

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
                if (pose[0] >= 6000000) {
                    drive(0);
                    step++;
                }

                //setPIDSettings(1,0,0); // set the kp, ki, and kd for forward movemement
                //drivePID(0); // keep angle at 0 (moving forward in straight line)
                break;
            case 1: // strafe right!
                strafe(-0.3);
                if (pose[1] >= 60) {
                    strafe(0);
                    step++;
                }
                break;
            case 2: // drive back!
                drive(-0.3);
                if (pose[0] <= 0) {
                    drive(0);
                    step++;
                }
                break;
            case 3: // strafe left!
                strafe(0.3);
                if(pose[1] <= 0) {
                    strafe(0);
                    step++;
                }
                break;
            case 4: // rotate right!
                rotate(0.3);
                if (pose[2] >= (Math.PI / 6)) {
                    rotate(0);
                    step++;
                }
                break;
            case 5: // rotate left!
                rotate(-0.3);
                if (pose[2] <=(Math.PI / -6)) {
                    rotate(0);
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

        // distance wheel turns in cm!
        double rawLeftEncoder = leftEncoder.getCurrentPosition();
        double rawRightEncoder = -rightEncoder.getCurrentPosition();
        double rawBackEncoder = -backEncoder.getCurrentPosition();

        telemetry.addData("Raw Left", rawLeftEncoder);
        telemetry.addData("Raw Right", rawRightEncoder);
        telemetry.addData("Raw Back", rawBackEncoder);

        double rawChangeLeft = rawLeftEncoder - prevLeftEncoder;
        double rawChangeRight = rawRightEncoder - prevRightEncoder;
        double rawChangeBack = rawBackEncoder - prevBackEncoder;

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

    public void setTimer(double duration) {
        timer.reset();
        while (true) { // this is bad practice, don't do it in any other sense
            if (timer.seconds() >= duration) {
                break;
            }
        }
    }

    public void drivePID(double angle) {
        double error = PIDcontrol(angle, pose[2]);

        // now you can put the 0.8 on a pid as well to track the y position at the same time

        leftFront.setPower(0.8 + error);
        rightFront.setPower(0.8 - error);
        leftBack.setPower(0.8 + error);
        rightBack.setPower(0.8 - error);
    }

    public void setPIDSettings (double p,  double i, double d) {
        integralSum = 0;
        Kp = p;
        Ki= i;
        Kd = d;


        PIDTimer = new ElapsedTime();
        lasterror = 0;
    }

    public double PIDcontrol(double reference, double state){
        double error = reference - state;
        integralSum += error * PIDTimer.seconds();
        double derivative = (error - lasterror ) / PIDTimer.seconds();
        lasterror = error;

        PIDTimer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum *Ki);
        return output;

    }
}