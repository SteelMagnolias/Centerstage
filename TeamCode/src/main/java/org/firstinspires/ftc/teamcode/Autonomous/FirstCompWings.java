package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Autonomous(name = "FirstCompWings", group="Linear OpMode")
public class FirstCompWings extends LinearOpMode{


    // declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private DcMotor verticalArm;
    private DcMotor verticalArm2;

    private CRServo intakeClawLeft;
    private CRServo intakeClawRight;

    // declare sensors
    private TouchSensor allianceSwitch; // determines what alliance we are on.

    private double drivePow=0.3;


    // constant reverse, if we are on the blue side, then will be -1
    private int REVERSE = 1;


    @Override
    public void runOpMode() throws InterruptedException {


        // initialization of variables
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        verticalArm = hardwareMap.get(DcMotor.class, "verticalArm");
        verticalArm2 = hardwareMap.get(DcMotor.class, "verticalArm2");
        intakeClawLeft = hardwareMap.get(CRServo.class, "intakeClawLeft");
        intakeClawRight = hardwareMap.get(CRServo.class, "intakeClawRight");

        leftBack.setDirection(DcMotor.Direction.REVERSE );
        verticalArm.setDirection(DcMotor.Direction.REVERSE);
        verticalArm2.setDirection(DcMotor.Direction.REVERSE);



        allianceSwitch = hardwareMap.get(TouchSensor.class, "allianceSwitch");


        if (allianceSwitch.isPressed()) {
            // if alliance switch is pressed, we are on the blue alliance
            REVERSE = -1;
            telemetry.addData("REVERSE", REVERSE);
            telemetry.update();
        }


        telemetry.addData("leftFront", leftFront.getPower());
        telemetry.addData("rightFront", rightFront.getPower());
        telemetry.addData("leftBack", leftBack.getPower());
        telemetry.addData("rightBack", rightBack.getPower());


        telemetry.addData("REVERSE (if -1, blue alliance)", REVERSE);


        waitForStart();


        // strafe inwards until in line with flip door
        drive(-drivePow*REVERSE, +drivePow*REVERSE, +drivePow*REVERSE, -drivePow*REVERSE, 4400);

        //Turn to correct strafe if on red alliance (not blue)
        if (REVERSE == 1) {
            drive(+drivePow*REVERSE, -drivePow*REVERSE, +drivePow*REVERSE, -drivePow*REVERSE, 100);

        }
        else {
            drive(-drivePow*REVERSE, +drivePow*REVERSE, -drivePow*REVERSE, +drivePow*REVERSE, 0); // got rid of turn comp
        }

        // drive forward until through door and into backstage area tapes
        drive(+drivePow , +drivePow , +drivePow , +drivePow , 5750);

        // back up a little
        verticalArm.setPower(0.3);
        sleep(2000);
        verticalArm.setPower(0);


        // drop pixel
        //dropPixel(2000);


        // back up a little
        //verticalArm.setPower(0.7);
        //sleep(2500);
        //verticalArm.setPower(0);



        telemetry.update();
    }


    public void drive (double lf, double rf, double lb, double rb, double time){
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
        sleep ((int)time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(10);
    }
    public void closePixel () {

        intakeClawLeft.setPower(1);
        intakeClawRight.setPower(-1);
    }

    public void dropPixel (int time){
        intakeClawLeft.setPower(-1);
        intakeClawRight.setPower(1);
        sleep(time);
        intakeClawLeft.setPower(0);
        intakeClawRight.setPower(0);

    }
}