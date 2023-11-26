package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp (name = "Drive2" , group = "Iterative Opmode")
public class Drive2 extends OpMode {

    private DcMotor verticalArm;

    // servos
    private CRServo intakeClawLeft;
    private CRServo intakeClawRight;

    double armPow = 0.5; // arm power




    public void init() {

        verticalArm = hardwareMap.get(DcMotor.class, "verticalArm");
        verticalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // hold position

        intakeClawLeft = hardwareMap.get(CRServo.class, "intakeClawLeft");
        intakeClawRight = hardwareMap.get(CRServo.class, "intakeClawRight");
    }


    public void loop(){
        // main code


        // gamepad 1
        double lefty1 = -(gamepad1.left_stick_y); // this is the value of gamepad1's left joystick y value
        double leftx1 = gamepad1.left_stick_x; // this is the value of gamepad1's left joystick x value
        double rightx1 = gamepad1.right_stick_x; // this is the value of gamepad1's right joystick x value
        double righty1 = -(gamepad1.right_stick_y); // this the value of gamepad1's right joystick y value
        boolean buttonUp1 = gamepad1.dpad_up; // this is the value of gamepad1's up button on the dpad
        boolean buttonDown1 = gamepad1.dpad_down; // this is the value of gamepad1's down button on the dpad
        boolean buttonLeft1 = gamepad1.dpad_left; // this is the value of the gamepad1's left button on the dpad
        boolean buttonRight1 = gamepad1.dpad_right; // this is the value of the gamepad1's right button on the dpad
        boolean lb1 = gamepad1.left_bumper; // this is the value of the gamepad1's left bumper
        boolean rb1 = gamepad1.right_bumper; // this is the value of the gamepad1's right bumper
        boolean a1 = gamepad1.a; // this is the value of the a button on gamepad1
        boolean x1 = gamepad1.x; // this is the value of the x button on gamepad1
        boolean y1 = gamepad1.y; // this is the value of the y button on gamepad1
        boolean b1 = gamepad1.b; // this is the value of the b button on gamepad1


        // gamepad 2
        double lefty2 = -(gamepad2.left_stick_y); // this is the value of gamepad2's left joystick y value
        double leftx2 = gamepad2.left_stick_x; // this is the value of gamepad2's left joystick x value
        double rightx2 = gamepad2.right_stick_x; // this the value of gamepad2's right joystick x value
        double righty2 = -(gamepad2.right_stick_y); // this is the value of gamepad2's right joystick y value
        boolean a2 = gamepad2.a; // this is the value of the a button on gamepad2
        boolean x2 = gamepad2.x; // this is the value of the x button on gamepad2
        boolean y2 = gamepad2.y; // this is the value of the y button on gamepad2
        boolean b2 = gamepad2.b; // this is the value of the b button on gamepad2
        boolean rb2 = gamepad2.right_bumper; // this is the value of the right bumper
        boolean lb2 = gamepad2.left_bumper; // this is the value of the left bumper
        boolean buttonup2 = gamepad2.dpad_up;
        boolean buttondown2 = gamepad2.dpad_down;

        // telemetry
        telemetry.addData("Gamepad:", 1);
        telemetry.addData("lefty1", lefty1);
        telemetry.addData("leftx1", leftx1);
        telemetry.addData("righty1", righty1);
        telemetry.addData("rightx1", rightx1);
        telemetry.addData("buttonUp1", buttonUp1);
        telemetry.addData("buttonDown1", buttonDown1);
        telemetry.addData("buttonLeft1", buttonLeft1);
        telemetry.addData("buttonRight1", buttonRight1);
        telemetry.addData("lb1", lb1);
        telemetry.addData("rb1", rb1);
        telemetry.addData("a1", a1);
        telemetry.addData("b1", b1);
        telemetry.addData("x1", x1);
        telemetry.addData("y1", y1);


        telemetry.addData("Gamepad:", 2);
        telemetry.addData("lefty2", lefty2);
        telemetry.addData("leftx2", leftx2);
        telemetry.addData("righty2", righty2);
        telemetry.addData("rightx2", rightx2);
        telemetry.addData("a2", a2);
        telemetry.addData("b2", b2);
        telemetry.addData("x2", x2);
        telemetry.addData("y2", y2);
        telemetry.addData("lb2", lb2);
        telemetry.addData("rb2", rb2);

        // emergency stop
        if (b1 && y1) {
            stop();
        }


        if (Math.abs(lefty2) > 0.1) {
            verticalArm.setPower(armPow * lefty2);
        }
        else if (buttonup2 ){
            verticalArm.setPower(armPow*0.5);
        }
        else if (buttondown2) {
            verticalArm.setPower(armPow*-0.5);

        }
        else {
            // no movement
            verticalArm.setPower(0); // just enough to keep from falling was 0.05 changed to see if we need it after robot adjustments
        }



        if (Math.abs(righty2) > 0.1) {
            // open claw
            intakeClawLeft.setPower(1*righty2);
            intakeClawRight.setPower(-1*righty2);
        }
        else {
            intakeClawLeft.setPower(0);
            intakeClawRight.setPower(0);
        }

    }
    public void stop() {
        // stop code
    }
}