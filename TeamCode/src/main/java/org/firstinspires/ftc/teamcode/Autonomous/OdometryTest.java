package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;


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


    // bot constraints:
    double trackWidth = 21.5; //(cm)!
    double yOffSet = 10.5; //(cm)!
    double wheelRadius = 4.8; // centimeters!
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


    @Override
    public void init() {
        // init!
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftEncoder = leftFront;
        rightEncoder = rightFront;
        backEncoder = leftBack;
    }


    @Override
    public void loop() {
        // repeating code - contains state machine!
        runOdometry();

        switch(step) {
            case 0: // drive forward!
                drive(0.3);
                if (pose[1] >= 120 && pose[1] <= 123) {
                    drive(0);
                    step++;
                }
                break;
            case 1: // strafe right!
                strafe(0.5);
                if (pose[0] >= 120 && pose[0] <= 123) {
                    strafe(0);
                    step++;
                }
                break;
            case 2: // drive back!
                drive(-0.3);
                if (pose[1] >= -2 && pose[1] <= 1) {
                    drive(0);
                    step++;
                }
                break;
            case 3: // strafe left!
                strafe(-0.5);
                if(pose[0] >= -2 && pose[0] <= 1) {
                    strafe(0);
                    step++;
                }
                break;
            case 4: // rotate right!
                rotate(0.3);
                if (pose[2] >= (Math.PI / 6) && pose[2] <= ((8 * Math.PI) / 45)) {
                    rotate(0);
                    step++;
                }
                break;
            case 5: // rotate left!
                rotate(-0.3);
                if (pose[2] <=(Math.PI / -6) && pose[2] >= ((-8 * Math.PI) / 45)) {
                    rotate(0);
                    step++;
                }
                break;
            default: // do nothing!
                drive(0);
                stop();
        }
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

        double rawChangeLeft = ((rawLeftEncoder - prevLeftEncoder) / cpr) * wheelRadius;
        double rawChangeRight = ((rawRightEncoder - prevRightEncoder) / cpr) * wheelRadius;
        double rawChangeBack = ((rawBackEncoder - prevBackEncoder) / cpr) * wheelRadius;

        // find change in theta!
        double deltaTheta = (rawChangeLeft - rawChangeRight) / trackWidth;

        // find change of x (center)!
        double xCenter = (rawChangeLeft + rawChangeRight) / 2;

        // find change in x perpendicular!
        double xPerp = rawChangeBack - (yOffSet * deltaTheta);

        //find change in x!
        double xChange = xCenter * Math.cos(deltaTheta) - xPerp * Math.sin(deltaTheta);

        // find changein y!
        double yChange = xCenter * Math.sin(deltaTheta) + xPerp * Math.cos(deltaTheta);

        pose[0] += xChange;
        pose[1] += yChange;
        pose[2] += deltaTheta;

        prevLeftEncoder = rawLeftEncoder;
        prevRightEncoder = rawRightEncoder;
        prevBackEncoder = rawBackEncoder;
    }

    public void drive(double pow) {
        // drive forward or backward
        leftFront.setPower(pow);
        rightFront.setPower(pow);
        leftBack.setPower(pow);
        rightBack.setPower(pow);

    }

    public void strafe(double pow) {
        // strafe left or right
        leftFront.setPower(-pow);
        rightFront.setPower(pow);
        leftBack.setPower(pow);
        rightBack.setPower(-pow);
    }

    public void rotate(double pow) {
        // rotate left or right
        leftFront.setPower(-pow);
        rightFront.setPower(pow);
        leftBack.setPower(-pow);
        rightBack.setPower(pow);
    }
}