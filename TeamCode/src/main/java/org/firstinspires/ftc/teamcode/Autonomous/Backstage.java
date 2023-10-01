package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name = "Backstage", group="LinearOpMode")
public class Backstage extends LinearOpMode{

    // declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor intake;
    private DcMotor hook;
    private DcMotor horizontalArm;
    private DcMotor verticalArm;

    // declare sensors
    private TouchSensor allianceSwitch; // determines what alliance we are on.

    // declare integer
    private int spikeMark=0;
    private double drivePow=0.3;

    // constant reverse, if we are on the blue side, then will be -1
    private int REVERSE = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        // initialization of variables
        leftFront=hardwareMap.get(DcMotor.class, "leftFront");
        rightFront=hardwareMap.get(DcMotor.class, "rightMotor");
        leftBack=hardwareMap.get(DcMotor.class, "leftBack");
        rightBack=hardwareMap.get(DcMotor.class, "rightBack");
        intake=hardwareMap.get(DcMotor.class, "intake");
        hook=hardwareMap.get(DcMotor.class, "hook");
        horizontalArm=hardwareMap.get(DcMotor.class, "horizontalArm");
        verticalArm=hardwareMap.get(DcMotor.class, "verticalArm");

        allianceSwitch = hardwareMap.get(TouchSensor.class, "allianceSwitch");

        if (allianceSwitch.isPressed()) {
            // if alliance switch is pressed, we are on the blue alliance
            REVERSE = -1;
        }

        telemetry.addData("leftFront", leftFront.getPower());
        telemetry.addData("rightFront", rightFront.getPower());
        telemetry.addData("leftBack", leftBack.getPower());
        telemetry.addData("rightBack", rightBack.getPower());
        telemetry.addData("intake", intake.getPower());
        telemetry.addData("hook", hook.getPower());
        telemetry.addData("horizontalArm", horizontalArm.getPower());
        telemetry.addData("verticalArm", verticalArm.getPower());

        telemetry.addData("REVERSE (if -1, blue alliance)", REVERSE);


        waitForStart();
        if(spikeMark==1){
            // left
            // 90 degree counter-clockwise turn
            drive(-drivePow * REVERSE,+drivePow * REVERSE,-drivePow * REVERSE,+drivePow * REVERSE, 1000);
            // strafe right
            drive(+drivePow * REVERSE, -drivePow * REVERSE, -drivePow * REVERSE, +drivePow * REVERSE, 1000);
            // change as you will
            drive(drivePow, drivePow, drivePow, drivePow, 1000);
            // drop purple pixel
            dropPixel();
            // strafe left
            drive(-drivePow * REVERSE, +drivePow * REVERSE, +drivePow * REVERSE, -drivePow * REVERSE, 1000);
            // 180 degree clockwise turn
            drive(+drivePow * REVERSE, -drivePow * REVERSE, +drivePow * REVERSE, -drivePow * REVERSE, 1000);

        }
        else if(spikeMark==2){
            // drive forward
            drive(drivePow, drivePow, drivePow, drivePow, 1000);
            // change as you will
            drive(drivePow, drivePow, drivePow, drivePow, 1000);
            dropPixel();
            // drive backward
            drive(-drivePow, -drivePow, -drivePow, -drivePow, 1000);
            // 90 degree clockwise turn
            drive(+drivePow * REVERSE, -drivePow * REVERSE, +drivePow * REVERSE, -drivePow * REVERSE, 1000);
        }
        else{
            // 90 degree clockwise turn
            drive(+drivePow * REVERSE, -drivePow * REVERSE, +drivePow * REVERSE, -drivePow * REVERSE, 1000);
            // change as you will
            drive(drivePow * REVERSE, drivePow * REVERSE, drivePow * REVERSE, drivePow * REVERSE, 1000);
            dropPixel();
            // strafe right
            drive(+drivePow * REVERSE, -drivePow * REVERSE, -drivePow * REVERSE, +drivePow * REVERSE, 1000);
        }
        // drive forward 2 square
        drive(drivePow, drivePow, drivePow, drivePow, 2000);
        // drop yellow pixel
        dropPixel();

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

    public void dropPixel (){

    }
}
