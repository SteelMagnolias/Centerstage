package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "alternativeOdometry", group="Iterative OpMode")
public class alternativeOdometry extends OpMode {

    // declare motors!
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor verticalArm1;
    private DcMotor verticalArm2;

    //declare servos
    private CRServo intakeClawRight;
    private CRServo intakeClawLeft;
    private CRServo wrist1;
    private CRServo wrist2;

    // declare encoders
    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor backEncoder;
    private DcMotor armEncoder;
    private DcMotor wristEncoder;


    // auton step / action!
    int step = 0;

    // calculation variables in millimeters or degrees
    double distancePerTick = 0.0134;
    double anglePerTick = 0.00714;
    double yDistanceTraveled;
    double xDistanceTraveled;
    double angleTurned;
    double xPreviousDistanceTraveled = 0;
    double yPreviousDistanceTraveled = 0;
    double previousAngleTurned;

    @Override
    public void init() {
        // init!
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        verticalArm1 = hardwareMap.get(DcMotor.class, "verticalArm");
        verticalArm2 = hardwareMap.get(DcMotor.class, "verticalArm");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeClawRight = hardwareMap.get(CRServo.class, "intakeClawRight");
        intakeClawLeft = hardwareMap.get(CRServo.class, "intakeClawLeft");

        leftEncoder = leftFront;
        rightEncoder = rightFront;
        backEncoder = leftBack;
        armEncoder = verticalArm1;
        wristEncoder = verticalArm2;
    }

    @Override
    public void loop() {

        directionValue();

        switch (step) {
            case 0: //Left
                move(1295, 0, 0);
                break;
            case 1: //Forward
                move(1295, 1829, 0);
                break;
            case 2: //right
                move(610, 1829, 0);
                break;
            case 3://scoot Forward
                move(610, 2083, 0);
                break;
            case 4: //arm
                arm(4096, 4096);
                break;
            case 5:
                dropPixel();
                break;
            case 6:
                arm(0, 0);
                break;
            case 7: // scoot back
                move(610, 1829, 0);
                break;
            case 8: // left
                move(1295, 1829, 0);
                break;
            case 9://forward
                move(1295, 2438, 0);
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

    public void directionValue() {
        //encoder values
        double CurrentLeftEncoder = leftEncoder.getCurrentPosition();
        double CurrentRightEncoder = rightEncoder.getCurrentPosition();
        double currentBackEncoder = backEncoder.getCurrentPosition();

        //y value
        double yEncoder = (CurrentLeftEncoder + CurrentRightEncoder) / 2;
        double yDistanceTraveledLoop = yEncoder * distancePerTick;
        yDistanceTraveled = yDistanceTraveledLoop + yPreviousDistanceTraveled;
        yPreviousDistanceTraveled = yDistanceTraveled;

        //x value
        double xEncoder = currentBackEncoder;
        double xDistanceTraveledLoop = xEncoder * distancePerTick;
        xDistanceTraveled = xDistanceTraveledLoop + xPreviousDistanceTraveled;
        xPreviousDistanceTraveled = xDistanceTraveled;

        //turn value
        double turnEncoder = (CurrentLeftEncoder - CurrentRightEncoder) / 2;
        double angleTurnedLoop = turnEncoder * anglePerTick;
        angleTurned = angleTurnedLoop + previousAngleTurned;
        if (angleTurned > 360){
            previousAngleTurned = angleTurned-360;
        }
        else{
            previousAngleTurned = angleTurned;
        }
    }

    public void move(int x, int y, int t) {
        if (t + 1 > angleTurned) {
            rotate(-0.3);
        } else if (t - 1 < angleTurned) {
            rotate(0.3);
        } else if (y - 10 < yDistanceTraveled) {
            drive(0.3);
        } else if (y + 10 > yDistanceTraveled) {
            drive(-0.3);
        } else if (x + 10 > xDistanceTraveled) {
            strafe(-0.3);
        } else if (x - 10 < xDistanceTraveled) {
            strafe(0.3);
        } else {
            drive(0);
            step++;
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

    public void arm(int a, int w){
        double currentArmEncoder = armEncoder.getCurrentPosition();
        double currentWristEncoder = wristEncoder.getCurrentPosition();
        if (currentArmEncoder + 5 <= a){
            verticalArm1.setPower(0.8);
            verticalArm2.setPower(0.8);
        } else if (currentArmEncoder - 5 >= a){
            verticalArm1.setPower(-0.8);
            verticalArm2.setPower(-0.8);
        } else {
            verticalArm1.setPower(0);
            verticalArm2.setPower(0);
        }

        if (currentWristEncoder + 5 <= w){
            wrist1.setPower(1);
            wrist2.setPower(1);
        } else if (currentWristEncoder - 5 >= w) {
            wrist1.setPower(-1);
            wrist2.setPower(-1);
        } else {
            wrist1.setPower(0);
            wrist2.setPower(0);
        }

        if (currentArmEncoder - 5 >= a && currentArmEncoder + 5 <= a && currentWristEncoder - 5 >= w && currentWristEncoder + 5 <= w){
            step++;
        }
    }


    public void dropPixel() {

    }
}