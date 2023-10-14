package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "OneMotor" , group = "Iterative Opmode")
public class OneMotor extends OpMode {

    // declare motors
    private DcMotor motor;

    public void init() {
        // init
        motor  = hardwareMap.get(DcMotor.class, "motor");
    }

    public void loop(){
        // main code

        // gamepad 1
        double lefty1 = -(gamepad1.left_stick_y); // this is the value of gamepad1's left joystick y value

        if (lefty1 >= 0.1) {
            // up
            motor.setPower(0.9);
        }
        else if (lefty1 <= -0.1) {
            // down
            motor.setPower(-0.9);
        }
        else {
            // no move
            motor.setPower(0);
        }
    }

    public void stop() {
        // stop code
    }
}
