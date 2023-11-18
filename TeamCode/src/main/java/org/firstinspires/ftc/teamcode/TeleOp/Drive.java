package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp (name = "Drive" , group = "Iterative Opmode")
public class Drive extends OpMode {


    // declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    //private DcMotor hook;
    private DcMotor verticalArm;



    // servos
    private CRServo intakeClaw;


    // sensors
    //private DistanceSensor distanceSensor;


    // cameras


    // bot constraints:
    double trackWidth = 21.5; //(cm)
    double yOffSet = 10.5; //(cm)
    double wheelRadius = 4.8; // centimeters
    double cpr = 8192; // counts per rotation
    double wheelCircumference = 2 * Math.PI * wheelRadius;


    // other variables
    double pow; // motor power for wheels
    double armPow = 0.5; // arm power
    double theta; // angle of wheels joystick
    double prevLeftEncoder = 0;
    double prevRightEncoder = 0;
    double currentLeftEncoder = 0;
    double currentRightEncoder = 0;


    // rotated x and y values for field-centric drive
    double rotX;
    double rotY;
    int x3;




    public void init() {
        // init
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        // reverse motors
        leftBack.setDirection(DcMotor.Direction.REVERSE );
        //hook = hardwareMap.get(DcMotor.class, "hook");
        verticalArm = hardwareMap.get(DcMotor.class, "verticalArm");
        verticalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // hold position

        intakeClaw = hardwareMap.get(CRServo.class, "intakeClaw");

        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
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



        // wheels
        // if in turbo mode, full power, otherwise half
        if (a1) pow = 1; // turbo mode
        else pow =0.5;
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
        pow *= 0.5;
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
        } else {
            // stops movement
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }


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


    /* // climbing
      if (rb2) {
          // lift go up
          automatedLift();
      }


      if(lb2) {
          // release lift
          releaseLift();
      }*/

      if (rb2) {
        // open claw
        intakeClaw.setPower(1);
      }
      else if (lb2) {
          // close claw
          intakeClaw.setPower(-1);
      }
      else {
          intakeClaw.setPower(0);
      }


    /* if (a2) {
          // throw plane
          throwPlane();
      } */

        /* switch(x3){
            case 0:
                //raise lift
                break;
            case 1:
                //drive forward
                break;
            default:
                //raise bot
                break;
        } */


    }
    public void stop() {
        // stop code
    }


    public void automatedLift() {
        // bring lift up for hang

    }


    public void releaseLift() {
        // lower lift (in case of error)
    }


    public void throwPlane() {
        // throw plane from behind truss
    }
}