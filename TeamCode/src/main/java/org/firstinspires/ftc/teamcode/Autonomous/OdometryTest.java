package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;


@Autonomous(name = "OdometryTest", group="Iterative OpMode")
public class OdometryTest extends OpMode {

    // declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;


    // bot constraints:
    double trackWidth = 21.5; //(cm)
    double yOffSet = 10.5; //(cm)
    double wheelRadius = 4.8; // centimeters
    double cpr = 8192; // counts per rotation
    double wheelCircumference = 2 * Math.PI * wheelRadius;


    @Override
    public void init() {
        // init


    }


    @Override
    public void loop() {
        // repeating code - contains state machine
    }


    @Override
    public void stop() {
        // stops code
    }


    public void runOdometry() {
        // runs odometry
    }
}