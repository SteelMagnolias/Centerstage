package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Backstage", group="Linear OpMode")
public class Backstage extends LinearOpMode{

    // declare variables
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor intake;
    DcMotor hook;
    DcMotor horizontalArm;
    DcMotor verticalArm;

    // declare integer
    int spikeMark=0;
    double drivePow;

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

        waitForStart();
        if(spikeMark==1){
            // left
            // 90 degree counter-clockwise turn
            drive(-drivePow,+drivePow,-drivePow,+drivePow, 1000);
            // strafe right
            drive(+drivePow, -drivePow, -drivePow, +drivePow, 1000);
            // change as you will
            drive(drivePow, drivePow, drivePow, drivePow, 1000);
            // drop purple pixel
            dropPixel();
            // strafe left
            drive(-drivePow, +drivePow, +drivePow, -drivePow, 1000);
            // 180 degree clockwise turn
            drive(+drivePow, -drivePow, +drivePow, -drivePow, 1000);

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
            drive(+drivePow, -drivePow, +drivePow, -drivePow, 1000);
        }
        else{
            // 90 degree clockwise turn
            drive(+drivePow, -drivePow, +drivePow, -drivePow, 1000);
            // change as you will
            drive(drivePow, drivePow, drivePow, drivePow, 1000);
            dropPixel();
            // strafe right
            drive(+drivePow, -drivePow, -drivePow, +drivePow, 1000);
        }
        // drive forward 2 square
        drive(drivePow, drivePow, drivePow, drivePow, 2000);
        // drop yellow pixel
        dropPixel();
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
