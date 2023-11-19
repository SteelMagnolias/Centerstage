package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp (name = "FieldOrientedDrive" , group = "Iterative Opmode")
public class FieldOrientedDrive extends OpMode {


    // declare motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    //private DcMotor hook;
    private DcMotor verticalArm;


    // encoders (really motors), but clarity
    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor backEncoder;


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
    double theta = 0; // angle of wheels joystick
    double botHeading; // angle of robot on field according to encoders
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
        leftFront.setDirection(DcMotor.Direction.REVERSE );
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        //hook = hardwareMap.get(DcMotor.class, "hook");
        verticalArm = hardwareMap.get(DcMotor.class, "verticalArm");

        intakeClaw = hardwareMap.get(CRServo.class, "intakeClaw");

        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");


        // copy values from the attached motor port.  better for readability
        leftEncoder = leftBack;
        rightEncoder = rightBack;
        backEncoder = rightFront;
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


        // wheels
        // if in turbo mode, full power, otherwise half
        if (a1) pow = 1; // turbo mode
        else pow =0.5;

        if (leftx1 > 0.1 || lefty1 > 0.1 || rightx1 > 0.1 ) {
            // find bot heading
            botHeading = -1 * getAngle();
            //turning variable
            double rx = rightx1;

            // find rotated x and y using rotation matrix
            rotX = leftx1 * Math.cos(botHeading) - lefty1 * Math.sin(botHeading);
            rotY = leftx1 * Math.sin(botHeading) + lefty1 * Math.cos(botHeading);

            // denominator: scales to the ratio of the sides to determine powers (max of 1)
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            // set powers of wheels
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);
        }
        else{
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }

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
          verticalArm.setPower(pow * lefty2);
      }
      else {
          // no movement
          verticalArm.setPower(0);
      }


      pow = 0.9;
      // intake in out controls
      if (Math.abs(righty2) > 0.1) {
          // intake or outtake
          intakeClaw.setPower(pow * righty2);
      }
      else {
          intakeClaw.setPower(0);
      }


/*

      // climbing
      if (y2) {
          // lift go up
          automatedLift();
      }


      if(x2) {
          // release lift
          releaseLift();
      }


      if(b2) {

          }
      }


      if (a2) {
          // throw plane
          throwPlane();
      }


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


    public void automatedLift() {
        // bring lift up for hang
    }


    public void releaseLift() {
        // lower lift (in case of error)
    }


    public void throwPlane() {
        // throw plane from behind truss
    }

  public double getAngle() {


      double phi = 0; // angle we will be calculating
      double changeLeft = 0;
      double changeRight = 0;


      // get current positions
      currentLeftEncoder = leftEncoder.getCurrentPosition();
      currentRightEncoder = rightEncoder.getCurrentPosition();


      // calculate change in encoder positions
      changeLeft = currentLeftEncoder - prevLeftEncoder;
      changeRight = currentRightEncoder - prevRightEncoder;


      changeLeft = (changeLeft / cpr) * wheelCircumference; // centimeters
      changeRight = (changeRight / cpr) * wheelCircumference; // centimeters


      phi = (changeLeft - changeRight) / trackWidth; // angle changed

      prevLeftEncoder = currentLeftEncoder;
      prevRightEncoder = currentRightEncoder;

      return phi;
  }
}