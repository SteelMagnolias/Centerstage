package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "OdoWings", group="Iterative OpMode")
public class OdoWings extends OpMode {

    // declare motors!
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor backEncoder;

    private DcMotor verticalArm;
    private CRServo intakeClawLeft;
    private CRServo intakeClawRight;


    // bot constraints:
    double trackWidth = 20.37; //(cm)!
    double yOffSet = -12.7; //(cm)!
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

    // make a timer
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void init() {
        // init!
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        verticalArm = hardwareMap.get(DcMotor.class, "verticalArm");
        intakeClawLeft = hardwareMap.get(CRServo.class, "intakeClawLeft");
        intakeClawRight = hardwareMap.get(CRServo.class, "intakeClawRight");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEncoder = leftFront;
        rightEncoder = rightFront;
        backEncoder = leftBack;

        prevLeftEncoder = leftEncoder.getCurrentPosition();
        prevRightEncoder = rightEncoder.getCurrentPosition();
        prevBackEncoder = -backEncoder.getCurrentPosition();
    }


    @Override
    public void loop() {
        // repeating code - contains state machine!
        runOdometry();

        switch(step) {
            case 0: // strafe over to D2
                strafe(-0.3);
                if (pose[0] >= 129.54) {
                    strafe(0);
                    step++;
                }
                break;
            case 1: // forward through gate to D5
                drive(-0.3);
                if (pose[1] <= -178) {
                    drive(0);
                    step++;
                }
                break;
            case 2: // strafe to E5
                strafe(0.3);
                if (pose[0] <= 68.78) {
                    strafe(0);
                    step++;
                }
                break;
            case 3: // forward a little to drop
                drive(-0.3);
                if (pose[1] <= -208.28) {
                    drive(0);
                    step++;
                }
                break;
            case 4: // do the drop
                // raise arm
                verticalArm.setPower(0.3);
                setTimer(3);
                verticalArm.setPower(0);
                // intake drop both
                intakeClawLeft.setPower(1);
                intakeClawRight.setPower(1);
                setTimer(3);
                // stop moving intake
                intakeClawLeft.setPower(0);
                intakeClawRight.setPower(0);

                // move arm down
                verticalArm.setPower(-0.3);
                setTimer(3);
                // stop moving arm
                verticalArm.setPower(0);

                step++;
                break;
            case 5: // backward from drop
                drive(0.3);
                if (pose[1] >= -178) {
                    drive(0);
                    step++;
                }
            case 6: // strafe to D5
                strafe(-0.3);
                if(pose[0] <=  129.4) {
                    strafe(0);
                    step++;
                }
                break;
            case 7: // forward to D6
                drive(-0.3);
                if (pose[1] >= 243.84) {
                    drive(0);
                    step++;
                }
                break;
            default: // do nothing!
                drive(0);
                strafe(0);
                rotate(0);
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
        double rawRightEncoder = rightEncoder.getCurrentPosition();
        double rawBackEncoder = -backEncoder.getCurrentPosition();

        telemetry.addData("Raw Left", rawLeftEncoder);
        telemetry.addData("Raw Right", rawRightEncoder);
        telemetry.addData("Raw Back", rawBackEncoder);

        double rawChangeLeft = (((rawLeftEncoder - prevLeftEncoder) / cpr) * wheelCircumference);
        double rawChangeRight = (((rawRightEncoder - prevRightEncoder) / cpr) * wheelCircumference);
        double rawChangeBack = (((rawBackEncoder - prevBackEncoder) / cpr) * wheelCircumference);

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
}