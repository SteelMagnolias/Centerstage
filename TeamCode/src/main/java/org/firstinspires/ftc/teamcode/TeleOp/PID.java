package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

    @TeleOp(name = "PID", group="Iterative OpMode")
    public class PID extends OpMode {
        DcMotorEx motor;

        double integralSum = 0;
        double Kp = .03;
        double Ki= 0;
        double Kd = 0;
        //could add another controler if needed
        double reference = 0.8;


        ElapsedTime timer = new ElapsedTime();
        private double lasterror = 0;

        boolean a1;

        @Override
        public void init() {
            motor =  hardwareMap.get(DcMotorEx.class, "motor");
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        @Override
        public void loop() {
            double power = PIDcontrol(-3000, motor.getCurrentPosition());
            motor.setPower(power);

            double actualPower = motor.getPower();



            telemetry.addData("Status: Timer Stop", 0);
            telemetry.addData("Position", motor.getCurrentPosition());
            telemetry.addData("Motor Power", power);
            telemetry.addData("Actual Power", actualPower);
            telemetry.addData("Status: Running",0);
            telemetry.update();

//            a1 = gamepad1.b;
//
//            while (!a1) {
//                // stop arm
//                motor.setPower(0);
//                telemetry.addData("Status: Arm Stop", a1);
//                telemetry.update();
//            }
//            double actualPower = motor.getPower();
//            if (motor.getCurrentPosition() < -480 && motor.getCurrentPosition() > -520) {
//                while(true){}
//            }
            // reset timer
            /*
            timer.reset();

            while(timer.seconds() < 2) {
                telemetry.addData("Status: Timer Stop", 0);
                telemetry.addData("Position", motor.getCurrentPosition());
                telemetry.addData("Motor Power", power);
                telemetry.addData("Actual Power", actualPower);
                telemetry.addData("Status: Running",0);
                telemetry.update();
                motor.setPower(0);
           //     telemetry.update();
            }
            */


        }
        public double PIDcontrol(double reference, double state){
            double error = reference - state;
            integralSum += error * timer.seconds();
            double derivative = (error - lasterror ) / timer.seconds();
            lasterror = error;

            timer.reset();

            double output = (error * Kp) + (derivative * Kd) + (integralSum *Ki);
            return output;


        }
    }


