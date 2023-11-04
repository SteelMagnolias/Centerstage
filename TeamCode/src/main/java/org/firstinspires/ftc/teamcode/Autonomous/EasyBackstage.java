package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Autonomous(name = "EasyBackstage", group="Linear OpMode")
    public class EasyBackstage extends LinearOpMode {


    // declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    /*
    private Servo intakeservo;

     */
    //private DcMotor intake;
    //private DcMotor hook;
    //private DcMotor horizontalArm;
    //private DcMotor verticalArm;


    // declare sensors
    private TouchSensor allianceSwitch; // determines what alliance we are on.

    // constant reverse, if we are on the blue side, then will be -1
    private int REVERSE = 1;

    private double drivePow = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {


        // initialization of variables
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setDirection(DcMotor.Direction.REVERSE );
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        /*
        intakeservo = hardwareMap.get(Servo.class, "intakeClaw");

         */
        //intake=hardwareMap.get(DcMotor.class, "intake");
        //hook=hardwareMap.get(DcMotor.class, "hook");
        //horizontalArm=hardwareMap.get(DcMotor.class, "horizontalArm");
        //verticalArm=hardwareMap.get(DcMotor.class, "verticalArm");


        allianceSwitch = hardwareMap.get(TouchSensor.class, "allianceSwitch");


        if (allianceSwitch.isPressed()) {
            // if alliance switch is pressed, we are on the blue alliance
            REVERSE = -1;
        }


        telemetry.addData("leftFront", leftFront.getPower());
        telemetry.addData("rightFront", rightFront.getPower());
        telemetry.addData("leftBack", leftBack.getPower());
        telemetry.addData("rightBack", rightBack.getPower());
       /*
        telemetry.addData("intakeServo" , intakeservo.getPosition());
        */
        //telemetry.addData("intake", intake.getPower());
        //telemetry.addData("hook", hook.getPower());
        //telemetry.addData("horizontalArm", horizontalArm.getPower());
        //telemetry.addData("verticalArm", verticalArm.getPower());


        telemetry.addData("REVERSE (if -1, blue alliance)", REVERSE);

        waitForStart();

        //drive away from wall
        drive(+drivePow * REVERSE, +drivePow * REVERSE, +drivePow * REVERSE, +drivePow * REVERSE, 500);


        //Rotating 90 degrees right
        drive(+drivePow * REVERSE, -drivePow * REVERSE, +drivePow * REVERSE, -drivePow * REVERSE, 1400);

        //drive forward a square in a half
        drive(+drivePow * REVERSE, +drivePow * REVERSE, +drivePow * REVERSE, +drivePow * REVERSE, 2000);

        /*
        //Drop 2 pixels in backstage
        dropPixel();
*/
        //Strafe away from pixels so not touching lol
        drive(+drivePow * REVERSE, -drivePow * REVERSE, drivePow * REVERSE, +drivePow * REVERSE, 1000);

        //Drive into backstage
        drive(+drivePow * REVERSE, +drivePow * REVERSE, +drivePow * REVERSE, +drivePow * REVERSE, 1000);

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

/*
    public void dropPixel (){
        intakeservo.setPosition(0);




    }
    */
}