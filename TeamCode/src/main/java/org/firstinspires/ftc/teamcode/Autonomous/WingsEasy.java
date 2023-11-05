package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Autonomous(name = "WingsEasy", group="Linear OpMode")
public class WingsEasy extends LinearOpMode{


    // declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    //private DcMotor intake;
    //private DcMotor hook;
    //private DcMotor horizontalArm;
    //private DcMotor verticalArm;

    private Servo intakeClaw;

    private Servo wrist;


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
        //intake = hardwareMap.get(DcMotor.class, "intake");
        //hook = hardwareMap.get(DcMotor.class, "hook");
        //horizontalArm = hardwareMap.get(DcMotor.class, "horizontalArm");
        //verticalArm = hardwareMap.get(DcMotor.class, "verticalArm");
        // intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

        leftBack.setDirection(DcMotor.Direction.REVERSE );


        allianceSwitch = hardwareMap.get(TouchSensor.class, "allianceSwitch");


        if (allianceSwitch.isPressed()) {
            // if alliance switch is pressed, we are on the blue alliance
            REVERSE = -1;
        }


        telemetry.addData("leftFront", leftFront.getPower());
        telemetry.addData("rightFront", rightFront.getPower());
        telemetry.addData("leftBack", leftBack.getPower());
        telemetry.addData("rightBack", rightBack.getPower());
        //telemetry.addData("intake", intake.getPower());
        //telemetry.addData("hook", hook.getPower());
        //telemetry.addData("horizontalArm", horizontalArm.getPower());
        //telemetry.addData("verticalArm", verticalArm.getPower());


        telemetry.addData("REVERSE (if -1, blue alliance)", REVERSE);


        waitForStart();


        // drive forward until in line with flip door
        drive(drivePow, drivePow, drivePow, drivePow, 5000);

        // rotate clockwise until intake faces door
        drive(-drivePow * REVERSE, +drivePow * REVERSE, -drivePow * REVERSE, +drivePow * REVERSE, 1000);

        // drive through door
        drive(drivePow, drivePow, drivePow, drivePow, 7000);

        // drop pixel




        // back up a little
        drive(-drivePow, -drivePow, -drivePow, -drivePow, 1000);

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
        intakeClaw.setPosition(0);
    }

    public void dropPixel (){
        intakeClaw.setPosition(1);

    }
}