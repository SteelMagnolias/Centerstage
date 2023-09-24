//Idk if this code will even work but its my test ig, pray for meeeeeeeeee
package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
public class testPID extends LinearOpMode {
    DcMotorEx motor;

    double integralSum = 0;
    double Kp = 0;
    double Ki= 0;
    double Kd = 0;
    //could add another controler if needed

    ElapsedTime timer = new ElapsedTime();
    private double Lasterror = 0;



    @Override
    public void runOPmode () throws InterruptedException {
        motor =  hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            double power = PIDcontrol(100, motor.getCurrentPosition());
            motor.setPower(power);


        }
    }
    public double PIDcontrol(double reference, double state){
        double error =reference-state;
        integralSum += error * timer.seconds();
        double derivative = (error - Lasterror ) / timer.seconds();
        Lasterror = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum *Ki);
        return output;


    }
}
 */