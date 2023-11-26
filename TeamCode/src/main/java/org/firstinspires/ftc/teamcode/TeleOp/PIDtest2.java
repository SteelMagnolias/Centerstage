/* package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


    @TeleOp(name = "PIDtest2", group="Linear OpMode")
    public class PIDtest2 extends LinearOpMode {
        DcMotorEx motor;

        double integralSum = 0;
        double Kp = 0;
        double Ki = 0;
        double Kd = 0;
        //could add another controler if needed

        ElapsedTime timer = new ElapsedTime();
        private double Lasterror = 0;

        @Override
        while True:
        current_time = get_current_time()
        current_error = desire_position-current_position

                p = k_p * current_error

        i += k_i * (current_error * (current_time - previous_time))

                if i > max_i:
        i = max_i
        elif i < -max_i:;
        i = -max_i

                D = k_d * (current_error - previous_error) / (current_time - previous_time)

        output = p + i + d

                previous_error = current_error
        previous_time = current_time
        }




 */