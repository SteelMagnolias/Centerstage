package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Autonomous(name = "EasyBackstage", group="Linear OpMode")
    public class EasyBackstage extends LinearOpMode {


    // declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private CRServo intakeClaw;

    // declare sensors
    private TouchSensor allianceSwitch; // determines what alliance we are on.

    // constant reverse, if we are on the blue side, then will be -1
    private int REVERSE = 1;

    private double drivePow = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addLine("if blue alliance hold button during init");

        // initialization of variables
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setDirection(DcMotor.Direction.REVERSE );
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        allianceSwitch = hardwareMap.get(TouchSensor.class, "allianceSwitch");
        intakeClaw = hardwareMap.get(CRServo.class, "intakeClaw");


        if (allianceSwitch.isPressed()) {
            // if alliance switch is pressed, we are on the blue alliance
            REVERSE = -1;
        }


        telemetry.addData("leftFront", leftFront.getPower());
        telemetry.addData("rightFront", rightFront.getPower());
        telemetry.addData("leftBack", leftBack.getPower());
        telemetry.addData("rightBack", rightBack.getPower());
        telemetry.addData("intakeClaw" , intakeClaw.getPower());

        telemetry.addData("REVERSE (if -1, blue alliance)", REVERSE);

        waitForStart();

        //drive forwards
        drive(drivePow , drivePow , drivePow, drivePow , 1500);

        //Drop 2 pixels in backstage
        dropPixel();

        //Drive into backstage
        drive(drivePow, drivePow, drivePow, drivePow, 500);

        telemetry.update();


    }

    public void drive(double lf, double rf, double lb, double rb, double time) {
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
        sleep((int) time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(10);

    }
    public void dropPixel (){
        intakeClaw.setPower(1);
        sleep(1000);
        intakeClaw.setPower(0);
    }
}