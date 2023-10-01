package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Drive" , group = "Iterative Opmode")
public class Drive extends OpMode {

    // declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor intake;
    private DcMotor hook;
    private DcMotor horizontalArm;
    private DcMotor verticalArm;

    // servos
    private Servo intakeClaw;

    // sensors
    private DistanceSensor distanceSensor;

    // cameras

    public void init() {
        // init
        leftBack = hardwareMap.get(DcMotor.class, "leftBack"); // in config --> port 1 --> "leftBack"
    }

    public void loop(){
        // main code
    }

    public void stop() {
        // stop code
    }
}
