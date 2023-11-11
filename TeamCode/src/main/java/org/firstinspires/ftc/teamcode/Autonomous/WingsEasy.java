package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Autonomous(name = "WingsEasy", group="Linear OpMode")
public class WingsEasy extends LinearOpMode{


    // declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private CRServo intakeClaw;

    // declare sensors
    private TouchSensor allianceSwitch; // determines what alliance we are on.

    private double drivePow=1;


    // constant reverse, if we are on the blue side, then will be -1
    private int REVERSE = 1;


    @Override
    public void runOpMode() throws InterruptedException {


        // initialization of variables
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        intakeClaw = hardwareMap.get(CRServo.class, "intakeClaw");

        leftBack.setDirection(DcMotor.Direction.REVERSE );


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
        drive(-drivePow*REVERSE, +drivePow*REVERSE, +drivePow*REVERSE, -drivePow*REVERSE, 3900);

        //Turn to correct strafe
        drive(+drivePow*REVERSE, -drivePow*REVERSE, +drivePow*REVERSE, -drivePow*REVERSE, 110);

        // drive forward until through door and into backstage area tapes
        drive(+drivePow , +drivePow , +drivePow , +drivePow , 5000);

        // drop pixel
        dropPixel(2000);


        // back up a little
        drive(drivePow, drivePow, drivePow, drivePow, 500);

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
        intakeClaw.setPower(-1);
    }

    public void dropPixel (int time){
        intakeClaw.setPower(-1);
        sleep(time);
        intakeClaw.setPower(0);

    }
}