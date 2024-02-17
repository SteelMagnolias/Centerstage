package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp (name = "Drive" , group = "Iterative Opmode")
public class Drive extends OpMode {


    // declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor verticalArm;
    private DcMotor verticalArm2;

    private DcMotor wrist;
    //private DcMotor hangArm;

    //private CRServo hangBolts;



    // servos
    private CRServo intakeClawRight;
    private CRServo intakeClawLeft;


    // sensors
    private AnalogInput potentiometer;
    double potentiometerVoltage;

    // PID
    double integralSum = 0;
    double integralSum2 = 0;
    double integralSum3 = 0;
    double integralSum4 = 0;
    double lasterror = 0;
    double lasterror2 = 0;
    double lasterror3 = 0;
    double lasterror4 = 0;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime timer3 = new ElapsedTime();
    ElapsedTime timer4 = new ElapsedTime();

    // PID Arm
    double KpArmDown = 0;
    double KiArmDown= 0;
    double KdArmDown = 0;
    double KpArmUp = 0;
    double KiArmUp= 0;
    double KdArmUp = 0;
    double referenceArmDown = 0;
    double referenceArmUp = 5353;
    double referenceArmCurled = 10;

    double KpWristDown = 0;
    double KiWristDown = 0;
    double KdWristDown = 0;

    double KpWristTuck = 0;
    double KiWristTuck = 0;
    double KdWristTuck = 0;

    double KpWristUp = -2.5;
    double KiWristUp = 0;
    double KdWristUp = 0;
    // volts
    double referenceWristDown = 3.307;
    double referenceWristUp = 0.19;
    double referenceWristTuck = 1.17;

    // volts

    // cameras


    // other variables
    double pow; // motor power for wheels
    double armPow = 0.6; // arm power
    double theta; // angle of wheels joystick
    double wristPower = 0.8;
    double wristPowerPID = 0;

    enum ArmState {
        ARM_UP,
        ARM_DOWN,
        EMERGENCY,
        MANUAL,
        ARM_TUCK,
    }

    ArmState armPos = ArmState.MANUAL;


    public void init() {
        // init
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftBack.setDirection(DcMotor.Direction.REVERSE );

        // reverse motors
        verticalArm = hardwareMap.get(DcMotor.class, "verticalArm");
        //verticalArm.setDirection(DcMotor.Direction.REVERSE);
        verticalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // hold position
        verticalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset encoder
        verticalArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // run without encoder

        verticalArm2 = hardwareMap.get(DcMotor.class, "verticalArm2");
        //verticalArm2.setDirection(DcMotor.Direction.REVERSE);
        verticalArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // hold position

        intakeClawRight = hardwareMap.get(CRServo.class, "intakeClawRight");
        intakeClawLeft = hardwareMap.get(CRServo.class, "intakeClawLeft");

        wrist = hardwareMap.get(DcMotor.class, "wrist");

        //sensors
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
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
        double r_trig2 = gamepad2.right_trigger;
        double l_trig2 = gamepad2.left_trigger;
        boolean back2 = gamepad2.back;

        // sensor readings
        potentiometerVoltage = potentiometer.getVoltage();

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

        telemetry.addData("Other:", "Sensors");
        telemetry.addData("potentiometerVoltage", potentiometerVoltage);
        telemetry.addData("armEncoder", verticalArm.getCurrentPosition());



        // wheels
        // if in turbo mode, full power, otherwise half
        if (a1) pow = 1; // turbo mode
        else pow =0.9;
        double c = Math.hypot(leftx1, lefty1); // find length of hypot using tan of triangle made by x and y
        double perct = pow * c; // scale by max power
        if (c <= .1) perct = 0; // if we are less than .1 power, than just don't move since we are in dead zone


        // determine quandrant
        if (leftx1 <= 0 && lefty1 >= 0) {
            theta = Math.atan(Math.abs(leftx1) / Math.abs(lefty1));
            theta += (Math.PI / 2);
        } else if (leftx1 < 0 && lefty1 <= 0) {
            theta = Math.atan(Math.abs(lefty1) / Math.abs(leftx1));
            theta += (Math.PI);
        } else if (leftx1 >= 0 && lefty1 < 0) {
            theta = Math.atan(Math.abs(leftx1) / Math.abs(lefty1));
            theta += (3 * Math.PI / 2);
        } else {
            theta = Math.atan(Math.abs(lefty1) / Math.abs(leftx1));
        }


        double dir = 1; // default of direction being forward
        if (theta >= Math.PI) { // if we have an angle other 180 degrees on unit circle, then direction is backward
            theta -= Math.PI;
            dir = -1;
        }
        //if (leftx1 <= 0 && lefty1 >= 0 || leftx1 >= 0 && lefty1 <= 0){
        //   theta += (Math.PI/2);
        //}


        telemetry.addData("pow", pow);
        telemetry.addData("dir", dir);
        telemetry.addData("c", c);
        telemetry.addData("theta", theta);


        // calculate power of front right wheel
        double fr = dir * ((theta - (Math.PI / 4)) / (Math.PI / 4)); // wheels move on a 45 degree angle, find the ratio of where we want to drive to where we need to be
        if (fr > 1) fr = 1; // cap speeds at 1 and -1
        if (fr < -1) fr = -1;
        fr = (perct * fr); // scale by power
        if (leftx1 == 0 && lefty1 == 0) fr = 0; // if no joystick movement stop


        // calculate power of back left wheel, wheels move on 45 degree angles, find the ratio between where we are and where we should be
        double bl = dir * ((theta - (Math.PI / 4)) / (Math.PI / 4));
        if (bl > 1) bl = 1; // cap speeds at 1 and -1
        if (bl < -1) bl = -1;
        bl = (perct * bl); // scale by power
        if (leftx1 < .1 && leftx1 > -.1 && lefty1 < .1 && lefty1 > -.1) bl = 0; // if no joystick movement, stop wheel


        // calculate power of front left wheel, wheels move on 45 degree angles, find the ratio between where we are and where we should be
        double fl = -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4));
        if (fl > 1) fl = 1; // cap powers at 1 and -1
        if (fl < -1) fl = -1;
        fl = (perct * fl); // scale by power
        if (leftx1 < .1 && leftx1 > -.1 && lefty1 < .1 && lefty1 > -.1) fl = 0; // if no joystick movement, stop wheel


        // calculate power of back right wheel, wheels move on 45 degree angles, find the ratio between where we are and where we should be
        double br = -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4));
        if (br > 1) br = 1; // cap powers at 1 and -1
        if (br < -1) br = -1;
        br = (perct * br); // scale by power
        if (leftx1 < .1 && leftx1 > -.1 && lefty1 < .1 && lefty1 > -.1) br = 0; // if no joystick movement, stop


        // add power for each wheel
        telemetry.addData("fl", fl);
        telemetry.addData("fr", fr);
        telemetry.addData("bl", bl);
        telemetry.addData("br", br);


        telemetry.addData("rlf", -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rrf", dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rbl", dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rbr", -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));




        // set power of wheels and apply any rotation
        leftFront.setPower(fl + rightx1);
        leftBack.setPower(bl + rightx1);
        rightFront.setPower(fr - rightx1);
        rightBack.setPower(br - rightx1);


        // Below: precision (slower) movement
        pow *= 0.3;
        if (buttonUp1) {
            // slowly moves forwards
            leftFront.setPower(pow);
            leftBack.setPower(pow);
            rightFront.setPower(pow);
            rightBack.setPower(pow);
        } else if (buttonDown1) {
            // slowly moves backwards
            leftFront.setPower(-pow);
            leftBack.setPower(-pow);
            rightFront.setPower(-pow);
            rightBack.setPower(-pow);
        } else if (buttonRight1) {
            // slowly moves right
            leftFront.setPower(pow);
            leftBack.setPower(-pow);
            rightFront.setPower(-pow);
            rightBack.setPower(pow);
        } else if (buttonLeft1) {
            // slowly moves left
            leftFront.setPower(-pow);
            leftBack.setPower(pow);
            rightFront.setPower(pow);
            rightBack.setPower(-pow);
        } else if (rb1){
            // rotate slowly right (clockwise)
            leftFront.setPower(pow);
            leftBack.setPower(pow);
            rightFront.setPower(-pow);
            rightBack.setPower(-pow);
        }
        else if (lb1) {
            // rotate slowly left (counter-clockwise)
            leftFront.setPower(-pow);
            leftBack.setPower(-pow);
            rightFront.setPower(pow);
            rightBack.setPower(pow);
        }



        // emergency stop
        if (b1 && y1) {
            stop();
        }

        // arm control

        switch(armPos) {
            case ARM_DOWN:
                wrist.setPower(PIDControlWristDown(KpWristDown, KiWristDown, KdWristDown, referenceWristDown, potentiometerVoltage));
                //armPow = 0.6 +  PIDControlArmDown(KpArmDown, KiArmDown, KdArmDown, referenceArmDown, potentiometerVoltage);
                //verticalArm.setPower(armPow);
                //verticalArm2.setPower(armPow);

                if (y2) {
                    // move arm up
                    armPos = ArmState.ARM_UP;
                }
                else if (x2) {
                    // STOP!
                    armPos = ArmState.EMERGENCY;
                }
                else if (b2) {
                    // TUCK!
                    armPos = ArmState.ARM_TUCK;
                }
                else if (back2) {
                    // change to manual mode
                    armPos = ArmState.MANUAL;
                    verticalArm.setPower(0);
                    verticalArm2.setPower(0);
                    wrist.setPower(0);
                }

                break;
            case ARM_UP:
                wristPowerPID = PIDControlWristUp(KpWristUp, KiWristUp, KdWristUp, referenceWristUp, potentiometerVoltage);

                wrist.setPower(wristPowerPID);
                //armPow = 0.6 + PIDControlArmUp(KpArmUp, KiArmUp, KdArmUp, referenceArmUp, potentiometerVoltage);
                //verticalArm.setPower(armPow);
                //verticalArm2.setPower(armPow);

                if (a2) {
                    // move to down position
                    armPos = ArmState.ARM_DOWN;
                }
                else if (b2) {
                    // move to tuck position
                    armPos = ArmState.ARM_TUCK;
                }
                else if (x2) {
                    armPos = ArmState.EMERGENCY;
                }
                else if (back2) {
                    // change to manual mode
                    armPos = ArmState.MANUAL;
                    verticalArm.setPower(0);
                    verticalArm2.setPower(0);
                    wrist.setPower(0);
                }
                break;
            case ARM_TUCK:
                //wrist.setPower(PIDControlWristTuck(KpWristTuck, KiWristTuck, KdWristTuck, referenceWristTuck, potentiometerVoltage));

                if (a2) {
                    // move to down position
                    armPos = ArmState.ARM_DOWN;
                }
                else if (y2) {
                    // move to tuck position
                    armPos = ArmState.ARM_UP;
                }
                else if (x2) {
                    armPos = ArmState.EMERGENCY;
                }
                else if (back2) {
                    // change to manual mode
                    armPos = ArmState.MANUAL;
                    verticalArm.setPower(0);
                    verticalArm2.setPower(0);
                    wrist.setPower(0);
                }
                break;
            case EMERGENCY:
                wrist.setPower(0);
                verticalArm.setPower(0);
                verticalArm2.setPower(0);

                if (a2) {
                    armPos = ArmState.ARM_DOWN;
                }
                else if (y2) {
                    armPos = ArmState.ARM_UP;
                }
                else if (back2) {
                    armPos = ArmState.MANUAL;
                }
                else if (b2) {
                    // TUCK!
                    armPos = ArmState.ARM_TUCK;
                }
                break;
            case MANUAL:
                // manual modes

                // wrist joystick
                if (Math.abs(righty2) > 0.1) {
                    wrist.setPower(righty2 * 0.6);
                }
                else {
                    wrist.setPower(0);
                }


                // into another case
                if (a2) {
                    // move to down position
                    armPos = ArmState.ARM_DOWN;
                }
                else if (x2) {
                    armPos = ArmState.EMERGENCY;
                }
                else if (y2) {
                    // move arm up
                    armPos = ArmState.ARM_UP;
                }
                else if (b2) {
                    // TUCK!
                    armPos = ArmState.ARM_TUCK;
                }

                /*
                if (b2) {
                    // reset encoders
                    resetArmEncoders();
                }*/
                break;
        }

        if (Math.abs(lefty2) > 0.1) {
            verticalArm.setPower(lefty2 * 0.5);
            verticalArm2.setPower(lefty2 * 0.5);
        }
        else if (buttonup2) {
            verticalArm.setPower(-0.9);
            verticalArm2.setPower(-0.9);
        }
        else {
            verticalArm.setPower(0);
            verticalArm2.setPower(0);
        }


        if (l_trig2 > 0.1) {
            intakeClawRight.setPower(1);
        }
        else if (lb2) {
            intakeClawRight.setPower(-1);
        }
        else {
            intakeClawRight.setPower(0);
        }
        if (r_trig2 > 0.1) {
            intakeClawLeft.setPower(-1);
        }
        else if (rb2) {
            intakeClawLeft.setPower(1);
        }
        else {
            intakeClawLeft.setPower(0);
        }


    }
    public void stop() {
        // stop code
    }


    public void releaseLift() {
        // lower lift (in case of error)
    }


    public void throwPlane() {
        // throw plane from behind truss
    }

    public double PIDControlWristDown(double Kp, double Ki, double Kd, double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lasterror ) / timer.seconds();
        lasterror = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum *Ki);
        telemetry.addData("PIDOutPutWristDown", output);
        return output;

    }

    public double PIDControlWristUp(double Kp, double Ki, double Kd, double reference, double state){
        double error = reference - state;
        integralSum2 += error * timer2.seconds();
        double derivative = (error - lasterror2 ) / timer2.seconds();
        lasterror2 = error;

        timer2.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum2 *Ki);
        telemetry.addData("PIDOutputWristUp", output);
        return output;

    }

    public double PIDControlArmUp(double Kp, double Ki, double Kd, double reference, double state){
        double error = reference - state;
        integralSum3 += error * timer3.seconds();
        double derivative = (error - lasterror3 ) / timer3.seconds();
        lasterror3 = error;

        timer3.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum3 *Ki);
        telemetry.addData("PIDOutPutArmUp", output);
        return output;

    }

    public double PIDControlArmDown(double Kp, double Ki, double Kd, double reference, double state){
        double error = reference - state;
        integralSum4 += error * timer4.seconds();
        double derivative = (error - lasterror4 ) / timer4.seconds();
        lasterror4 = error;

        timer4.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum4 *Ki);
        telemetry.addData("PIDOutPutArmDown", output);
        return output;

    }
}