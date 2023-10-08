
//Imports

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

    // reverse by multiplying by this number
    private static final int REVERSE = -1;
    private static final double DEAD_ZONE = 0.1;
    private static final double OFF = 0;

    @Override
    public void init() {

        //Motor shizzzzzzzzz
        telemetry.addData("Status", "Initialized");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        intake = hardwareMap.get(DcMotor.class, "intake");
        hook = hardwareMap.get(DcMotor.class, "hook");
        horizontalArm = hardwareMap.get(DcMotor.class, "horizontalArm");
        verticalArm = hardwareMap.get(DcMotor.class, "verticalArm");

        //Servo shizzzz
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

        //distance sensor shizzzzz
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        telemetry.addData("Status", "Initialized");

        leftBack.setPower(OFF);
        rightBack.setPower(OFF);
        leftFront.setPower(OFF);
        rightFront.setPower(OFF);

        intake.setPower(OFF);
        hook.setPower(OFF);
        horizontalArm.setPower(OFF);
        verticalArm.setPower(OFF);
        intakeClaw.setPosition(OFF);
    }

    @Override
    public void loop() {
        // Assigning & Data
        double lefty1 = -(gamepad1.left_stick_y); // this is the value of gamepad1's left joystick y value
            double leftx1 = gamepad1.left_stick_x; // this is the value of gamepad1's left joystick x value
            double rightx1 = gamepad1.right_stick_x; // this is the value of gamepad1's right joystick x value
            double righty1 = (gamepad1.right_stick_y); // this the value of gamepad1's right joystick y value
            double lefty2 = -(gamepad2.left_stick_y); // this is the value of gamepad2's left joystick y value
            double leftx2 = gamepad2.left_stick_x; // this is the value of gamepad2's left joystick x value
            double rightx2 = gamepad2.right_stick_x; // this the value of gamepad2's right joystick x value
            double righty2 = (gamepad2.right_stick_y); // this is the value of gamepad2's right joystick y value
            boolean buttonUp = gamepad1.dpad_up; // this is the value of gamepad1's up button on the dpad
            boolean buttonDown = gamepad1.dpad_down; // this is the value of gamepad1's down button on the dpad
            boolean buttonLeft = gamepad1.dpad_left; // this is the value of the gamepad1's left button on the dpad
            boolean buttonRight = gamepad1.dpad_right; // this is the value of the gamepad1's right button on the dpad
            boolean lb = gamepad1.left_bumper; // this is the value of the gamepad1's left bumper
            boolean rb = gamepad1.right_bumper; // this is the value of the gamepad1's right bumper
            boolean a1 = gamepad1.a; // this is the value of the a button on gamepad1
            boolean x1 = gamepad1.x; // this is the value of the x button on gamepad1
            boolean y1 = gamepad1.y; // this is the value of the y button on gamepad1
            boolean rt = gamepad1.right_stick_button; // this is the value of the button behind the right stick on gamepad1

        boolean buttonUp2 = gamepad2.dpad_up; // this is the value of the up button on gamepad2
        boolean buttonDown2 = gamepad2.dpad_down; // this is  the value of the down button on gamepad2
        boolean b2 = gamepad2.b; // this is the value of the b button on gamepad2
        boolean a2 = gamepad2.a; // this is the value of the a button on gamepad2
        boolean y2 = gamepad2.y; // this is the value of the y button on gamepad2
        boolean x2 = gamepad2.x; // this is the value of the x button on gamepad2
        boolean rb2 =gamepad2.right_bumper;// Reset
        boolean lb2 = gamepad2.left_bumper; // levels for stack of cones, keep clicking until reached prefered level

        // print values to console
        telemetry.addData("lefty1", lefty1);
        telemetry.addData("leftx1", leftx1);
        telemetry.addData("rightx1", rightx1);
        telemetry.addData("lefty2", lefty2);
        telemetry.addData("leftx2", leftx2);
        telemetry.addData("rightx2", rightx2);
        telemetry.addData("buttonUp", buttonUp);
        telemetry.addData("buttonDown", buttonDown);
        telemetry.addData("buttonRight", buttonRight);
        telemetry.addData("buttonLeft", buttonLeft);
        telemetry.addData("lb", lb);
        telemetry.addData("rb", rb);
        telemetry.addData("a", a2);
        telemetry.addData("b", b2);
        telemetry.addData("x", x2);
        telemetry.addData("y", y2);
        telemetry.addData("rt", rt);
        telemetry.addData("lb2", lb2);


    }
}