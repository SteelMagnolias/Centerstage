package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;



@Autonomous(name = "OdoRed", group="Iterative OpMode")
public class OdoRed extends OpMode {

    // declare motors!
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private DcMotor wrist; // core hex on wrist

    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor backEncoder;

    private DcMotor verticalArm;
    private DcMotor verticalArm2;



    // servos
    private CRServo intakeClawRight;
    private CRServo intakeClawLeft;


    /// bot constraints:
    double TRACK_WIDTH = 20; //(cm)!
    double TRACK_WIDTH_DELTA = 0;
    double Y_OFFSET = -13.5; //(cm)!
    double Y_OFFSET_DELTA = 0;
    double WHEEL_LEFT_DIAMETER = 3.469; // centimeters!
    double WHEEL_RIGHT_DIAMETER = 3.315;
    double WHEEL_BACK_DIAMETER = 3.471;
    double CPR = 8192; // counts per rotation!
    double WHEEL_CIRCUMFERENCE_LEFT = Math.PI * WHEEL_LEFT_DIAMETER;
    double WHEEL_CIRCUMFERENCE_RIGHT = Math.PI * WHEEL_RIGHT_DIAMETER;
    double WHEEL_CIRCUMFERENCE_BACK = Math.PI * WHEEL_BACK_DIAMETER;

    // current pose!
    double[] pose = {0,0,Math.toRadians(270)};

    // previous encoder positions!
    double prevLeftEncoder = 0;
    double prevRightEncoder = 0;
    double prevBackEncoder = 0;

    // auton step / action!
    int step = 0;
    int stackedStep = 0;
    int spikeMark = 3;
    int location = 1;

    int logCount = 0;

    double adjustablePow; // used to slow bot down when we get close to position in certain movements that need to be extra precise.

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        wrist = hardwareMap.get(DcMotor.class, "wrist");

        leftBack.setDirection(DcMotor.Direction.REVERSE );


        verticalArm = hardwareMap.get(DcMotor.class, "verticalArm");
        verticalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // hold position
        verticalArm2 = hardwareMap.get(DcMotor.class, "verticalArm2");
        verticalArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // hold position

        verticalArm.setDirection(DcMotor.Direction.REVERSE);
        verticalArm2.setDirection(DcMotor.Direction.REVERSE);


        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intakeClawRight = hardwareMap.get(CRServo.class, "intakeClawRight");
        intakeClawLeft = hardwareMap.get(CRServo.class, "intakeClawLeft");

        // fix encoders resets - wrong motors reset might be issue!
        leftEncoder = rightFront;
        rightEncoder = verticalArm2;
        backEncoder = rightBack;

        prevLeftEncoder = -leftEncoder.getCurrentPosition();
        prevRightEncoder = -rightEncoder.getCurrentPosition();
        prevBackEncoder = -backEncoder.getCurrentPosition();
    }


    @Override
    public void loop() {
        // repeating code - contains state machine!

        runOdometry();

        logCount++;
        //RobotLog.d("LogCount: " + logCount + "    Coordinates: (" + pose[0] + ", " + pose[1] + ")");


        switch(step) {
            case 0: // drive backward to give space for rotation!
                drive(-0.3);
                if (pose[1] >= 5) {
                    drive(0);
                    step++;
                }

                //setPIDSettings(1,0,0); // set the kp, ki, and kd for forward movemement
                //drivePID(0); // keep angle at 0 (moving forward in straight line)
                break;
            case 1:
                if (spikeMark == 1){
                    position1();
                }
                else if (spikeMark == 2) {
                    position2();
                }
                else{
                    position3();
                }
                break;
            case 2: // strafe left towards airplanes
                if (pose[0] <= -34) {
                    adjustablePow =  -0.2;
                }
                else {
                    adjustablePow = -0.3;
                }
                strafe(adjustablePow);
                if (pose[0] <= -44){
                    strafe(0);
                    step++;
                }
                else{

                }
                break;
            case 3: // rotate until forward to avoid stacks
                rotate(0.1);
                if (pose[2] <= Math.toRadians(100)) {
                    rotate(0);
                    step++;
                }
                break;
            case 4: // drive forward to middle but stop around first spike mark for a readjust
                drive (0.5);
                if (pose[1] >= 60) {
                    drive(0);
                    step++;
                }
                break;
            case 5: // rotate until forward to avoid stacks
                rotate(0.1);
                if (pose[2] <= Math.toRadians(100)) {
                    rotate(0);
                    step++;
                }
                break;
            case 6: // drive forward to middle
                drive(0.5);
                if (pose[1] >= 115) {
                    drive(0);
                    step++;
                }

                break;
            case 7: // strafe to not hit pixel stacks
                strafe(0.3);
                if(pose[0] >=  0){
                    drive(0);
                    step++;
                }
                break;
            case 8: // turn to go through truss
                rotate(0.3);
                if(pose[2] <= Math.toRadians(0)){
                    rotate(0);
                    step++;
                }
                break;
            case 9: // through stage door
                drive(0.3);
                if (pose [0] >= 180){
                    drive(0);
                    step++;
                }
                break;
            case 10: // turn to face board
                rotate(0.3);
                if (pose[2] <= Math.toRadians(-190)){
                    rotate (0);
                    step++;
                }
                break;
            case 11://move in front of board
                strafe (-0.3);
                if (pose[1] <= 30){
                    strafe(0);
                    step++;
                }
                break;
            case 12: // apriltag read
                step++;
                break;
            case 13: //arm up
                step++;
                break;
            case 14: // move in to board
                drive(-0.3);
                if(pose [0] >= 220){
                    drive (0);
                    step++;
                }
                break;
            case 15:// place pixel
                step++;
                break;
            case 16:
                drive(0.3);
                if(pose [0] <= 180){
                    drive (0);
                    step++;
                }
                break;
            case 17: //arm down
                step++;
                break;
            case 18:
                strafe(-0.3);
                if  (pose[1] <= 102){
                    strafe(0);
                    step++;
                }
                break;
            case 19:
                drive(0.3);
                if (pose[0] >= 220){
                    drive(0);
                    step++;
                }
                break;
            default: // do nothing!
                drive(0);
                stop();
        }

        telemetry.addData("Pose0", pose[0]);
        telemetry.addData("Pose1", pose[1]);
        telemetry.addData("Pose2", Math.toDegrees(pose[2]));
        telemetry.addData("Case", step);

        telemetry.update();
    }


    @Override
    public void stop() {
        // stops code
    }

    public void runOdometry() {
        // runs odometry!
        // robot front faces 90 degrees
        // x direction is horizontal is parallel to driver station
        // y direction is perpendicular to driver station

        // distance wheel turns in cm!
        double rawLeftEncoder = -leftEncoder.getCurrentPosition();
        double rawRightEncoder = -rightEncoder.getCurrentPosition();
        double rawBackEncoder = backEncoder.getCurrentPosition();

        /*telemetry.addData("Raw Left", rawLeftEncoder);
        telemetry.addData("Raw Right", rawRightEncoder);
        telemetry.addData("Raw Back", rawBackEncoder);*/

        double rawChangeLeft = ((rawLeftEncoder - prevLeftEncoder) / CPR) * WHEEL_CIRCUMFERENCE_LEFT;
        double rawChangeRight = ((rawRightEncoder - prevRightEncoder) / CPR) * WHEEL_CIRCUMFERENCE_RIGHT;
        double rawChangeBack = ((rawBackEncoder - prevBackEncoder) / CPR) * WHEEL_CIRCUMFERENCE_BACK;

        /*telemetry.addData("Raw Left Change", rawChangeLeft);
        telemetry.addData("Raw Right Change", rawChangeRight);
        telemetry.addData("Raw Back Change", rawChangeBack);*/

        // find change in theta!
        double deltaTheta = -(rawChangeLeft - rawChangeRight) / (TRACK_WIDTH + TRACK_WIDTH_DELTA);
        telemetry.addData("deltaTheta", deltaTheta);

        // find change of x (center)!
        double xCenter = (rawChangeLeft + rawChangeRight) / 2;
        telemetry.addData("xCenter", xCenter);

        // find change in x perpendicular!
        double xPerp = rawChangeBack - ((Y_OFFSET + Y_OFFSET_DELTA) * deltaTheta);
        telemetry.addData("xPerp", xPerp);

        /*
        NOTICE - yChange and xChange (below) are swapped from GM0
        This is because in GM0, they swap the x and y axis
        I wanted it to be like a normal graph
         */

        //find change in x!
        double xChange = xCenter * Math.cos(pose[2]) - xPerp * Math.sin(pose[2]);
        telemetry.addData("xChange", xChange);

        // find changein y!
        double yChange = xCenter * Math.sin(pose[2]) + xPerp * Math.cos(pose[2]);
        telemetry.addData("yChange", yChange);

        pose[0] += xChange;
        pose[1] += yChange;
        pose[2] += deltaTheta;

        prevLeftEncoder = rawLeftEncoder;
        prevRightEncoder = rawRightEncoder;
        prevBackEncoder = rawBackEncoder;

        /*telemetry.addData("prevLeftEncoder", rawChangeLeft);
        telemetry.addData("Raw Left Change", rawChangeLeft);
        telemetry.addData("Raw Left Change", rawChangeLeft);*/

       // RobotLog.d("values: " + rawLeftEncoder + ", " + rawRightEncoder + ", " + rawBackEncoder + ", " + pose[0] + ", " + pose[1] + ", " + pose[2]);
    }


    public void drive(double pow) {
        // drive forward or backward
        leftFront.setPower(pow);
        rightFront.setPower(pow);
        leftBack.setPower(pow);
        rightBack.setPower(pow);

    }

    public void strafe(double pow) {
        // strafe left
        leftFront.setPower(pow);
        rightFront.setPower(-pow);
        leftBack.setPower(-pow);
        rightBack.setPower(pow);
    }

    public void rotate(double pow) {
        // rotate left or right counter clockwise
        leftFront.setPower(pow);
        rightFront.setPower(-pow);
        leftBack.setPower(pow);
        rightBack.setPower(-pow);
    }

    public void position2() {
        telemetry.addData("stackedStep", stackedStep);
        RobotLog.d("Pose:" + pose[0] + "," + pose[1] + "," +  pose[2] + "Stacked Step: " + stackedStep);
        switch(stackedStep) {
            case 0:
                rotate(0.3); // rotate 180 degrees to face spike mark 2
                if (pose[2] <= Math.toRadians(100)) {
                    rotate(0);
                    stackedStep++;
                }
                break;
            case 1:
                //arm stuff
                stackedStep++;
                break;
            case 2:
                drive(0.3);
                if (pose[1] >= 63){
                    drive(0);
                    stackedStep++;
                }
                break;
            case 3:
                //arm stuff
                stackedStep++;
                break;
            case 4:
                drive(-0.3);
                if(pose[1] <= 10){
                    drive(0);
                    step++;
                }
                break;
        }
    }

    public void position1() {
        telemetry.addData("stackedStep", stackedStep);
        RobotLog.d("Pose:" + pose[0] + "," + pose[1] + "," + pose[2] + "Stacked Step: " + stackedStep);
        switch (stackedStep) {
            case 0:
                rotate(0.3); // rotate 90 degrees to face spike mark 1
                if (pose[2] <= Math.toRadians(190)) {
                    rotate(0);
                    stackedStep++;
                }
                break;
            case 1:
                //arm stuff
                stackedStep++;
                break;
            case 2:
                strafe(0.3);
                if (pose[1] >= 63) {
                    drive(0);
                    stackedStep++;
                }
                break;
            case 3:
                drive(-0.2);
                if (pose[0] >= -3) {
                    drive(0);
                    stackedStep++;
                }
                break;
            case 4:
                //arm stuff
                stackedStep++;
                break;
            case 5:
                strafe(-0.3);
                if (pose[1] <= 10) {
                    drive(0);
                    stackedStep++;
                }
                break;
            case 6:
                rotate(0.3);
                if (pose[2] <= Math.toRadians(100)) {
                    rotate(0);
                    step++;
                }
                break;
        }
    }

    public void position3() {
        telemetry.addData("stackedStep", stackedStep);
        RobotLog.d("Pose:" + pose[0] + "," + pose[1] + "," +  pose[2] + "Stacked Step: " + stackedStep);
        switch(stackedStep) {
            case 0:
                rotate(-0.3); // rotate 90 degrees to face spike mark 1
                if (pose[2] >= Math.toRadians(350)) {
                    rotate(0);
                    stackedStep++;
                }
                break;
            case 1:
                //arm stuff
                stackedStep++;
                break;
            case 2:
                strafe(-0.3);
                if (pose[1] >= 63){
                    drive(0);
                    stackedStep++;
                }
                break;
            case 3:
                //arm stuff
                stackedStep++;
                break;
            case 4:
                strafe(0.3);
                if(pose[1] <= 10){
                    drive(0);
                    stackedStep++;
                }
                break;
            case 5:
                rotate(-0.3);
                if (pose[2] >= Math.toRadians(440)) {
                    rotate(0);
                    step++;
                    pose[2] -= (2 * Math.PI); // to get back to the normal circle
                }
                break;
        }
    }
}