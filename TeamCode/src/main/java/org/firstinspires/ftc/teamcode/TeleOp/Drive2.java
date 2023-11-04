package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "Drive2" , group = "Iterative Opmode")
public class Drive2 extends OpMode {


    // declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    //private DcMotor intake;
    //private DcMotor hook;
    //private DcMotor horizontalArm;
    //private DcMotor verticalArm;



    // servos
    private Servo intakeClaw;
    private Servo wrist;


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
    double theta; // angle of wheels joystick
    boolean clawClosed; // tells whether the claw is closed or not
    boolean wristOut;
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
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        //intake = hardwareMap.get(DcMotor.class, "intake");
        //hook = hardwareMap.get(DcMotor.class, "hook");
        //horizontalArm = hardwareMap.get(DcMotor.class, "horizontalArm");
        //verticalArm = hardwareMap.get(DcMotor.class, "verticalArm");
/*

        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeClaw.setDirection(Servo.Direction.REVERSE);
        intakeClaw.setPosition(0); // closed
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(0);
        clawClosed = true;
        wristOut = false;
*/

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

/*
        // gamepad 2
        double lefty2 = -(gamepad2.left_stick_y); // this is the value of gamepad2's left joystick y value
        double leftx2 = gamepad2.left_stick_x; // this is the value of gamepad2's left joystick x value
        double rightx2 = gamepad2.right_stick_x; // this the value of gamepad2's right joystick x value
        double righty2 = -(gamepad2.right_stick_y); // this is the value of gamepad2's right joystick y value
        boolean a2 = gamepad2.a; // this is the value of the a button on gamepad2
        boolean x2 = gamepad2.x; // this is the value of the x button on gamepad2
        boolean y2 = gamepad2.y; // this is the value of the y button on gamepad2
        boolean b2 = gamepad2.b; // this is the value of the b button on gamepad2

*/
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
/*

        telemetry.addData("Gamepad:", 2);
        telemetry.addData("lefty2", lefty2);
        telemetry.addData("leftx2", leftx2);
        telemetry.addData("righty2", righty2);
        telemetry.addData("rightx2", rightx2);
        telemetry.addData("a2", a2);
        telemetry.addData("b2", b2);
        telemetry.addData("x2", x2);
        telemetry.addData("y2", y2);

*/
        double pow;
        if (a1) pow = 1; // turbo mode
        else pow =0.5;
        double c = Math.hypot(leftx1, lefty1);
        double perct = pow * c;
        if (c <= .1) perct = 0;
        //
        double theta;

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

        double dir = 1;
        if (theta >= Math.PI) {
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

        double fr = dir * ((theta - (Math.PI / 4)) / (Math.PI / 4));
        if (fr > 1) fr = 1;
        if (fr < -1) fr = -1;
        fr = (perct * fr);
        if (leftx1 == 0 && lefty1 == 0) fr = 0;

        double bl = dir * ((theta - (Math.PI / 4)) / (Math.PI / 4));
        if (bl > 1) bl = 1;
        if (bl < -1) bl = -1;
        bl = (perct * bl);
        if (leftx1 < .1 && leftx1 > -.1 && lefty1 < .1 && lefty1 > -.1) bl = 0;

        double fl = -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4));
        if (fl > 1) fl = 1;
        if (fl < -1) fl = -1;
        fl = (perct * fl);
        if (leftx1 < .1 && leftx1 > -.1 && lefty1 < .1 && lefty1 > -.1) fl = 0;

        double br = -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4));
        if (br > 1) br = 1;
        if (br < -1) br = -1;
        br = (perct * br);
        if (leftx1 < .1 && leftx1 > -.1 && lefty1 < .1 && lefty1 > -.1) br = 0;

        telemetry.addData("fl", fl);
        telemetry.addData("fr", fr);
        telemetry.addData("bl", bl);
        telemetry.addData("br", br);

        telemetry.addData("rlf", -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rrf", dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rbl", dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rbr", -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));


        leftFront.setPower(fl + rightx1);
        leftBack.setPower(bl + rightx1);
        rightFront.setPower(fr - rightx1);
        rightBack.setPower(br - rightx1);

        telemetry.addData("rightBack", rightBack.getPower());

        // emergency stop
        if (b1 && y1) {
            stop();
        }
        /*
      pow = 0.4;
      // ball and socket movement (horizontal)
      if (Math.abs(leftx2) > 0.1) {
          horizontalArm.setPower(pow * leftx2);
      }
      else {
          // no movement
          horizontalArm.setPower(0);
      }

         */

        /*
      // ball and socket movement (vertical)
      if (Math.abs(lefty2) > 0.1) {
          verticalArm.setPower(pow * lefty2);
      }
      else {
          // no movement
          verticalArm.setPower(0);
      }


      /*
      pow = 0.9;
      // intake in out controls
      if (Math.abs(righty2) > 0.1) {
          // intake or outtake
          intake.setPower(pow * righty2);
      }
      else {
          intake.setPower(0);
      }




      // climbing
      if (rb2) {
          // lift go up
          automatedLift();
      }


      if(lb2) {
          // release lift
          releaseLift();
      }




      if(b2) {
          // claw open / close
          if (clawClosed) {
              // open claw
              intakeClaw.setPosition(1);
              clawClosed = true;
          }
          else if (!clawClosed) {
              // close claw
              intakeClaw.setPosition(0);
              clawClosed = true;
          }
      }
*/
        /*

      if (x2) {
          if (wristOut) {
              // wrist out, let's tuck it in
              wrist.setPosition(0);

              // change status
              wristOut = false;
          }
          else {
              // wrist tucked in, let's bring it out
              wrist.setPosition(1);

              // change status
              wristOut = true;
          }
      }

*/
/*
      if (a2) {
          // throw plane
          throwPlane();
      }


       */

        /*
        switch(x3){
            case 0:
                //raise lift
                break;
            case 1:
                //drive forward
                break;
            default:
                //raise bot
                break;
        }

         */


    }
    public void stop() {
        // stop code
    }
/*

    public void automatedLift() {
        // bring lift up for hang
    }


    public void releaseLift() {
        // lower lift (in case of error)
    }


    public void throwPlane() {
        // throw plane from behind truss
    }

 */
}