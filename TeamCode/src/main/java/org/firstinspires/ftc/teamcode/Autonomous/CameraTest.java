// group of code/location
package org.firstinspires.ftc.teamcode.Autonomous;


//import important files from sdk
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


import java.util.List;
import android.util.Size;


//define class and name for driver hub
@Autonomous(name = "CameraTest", group="Iterative OpMode")
public class CameraTest extends LinearOpMode {


    //define camera
    private WebcamName camera;


    //define processor variable
    private TfodProcessor tfod;
    private  TfodProcessor tfod2;
    //define custom vision portal variable
    private VisionPortal visionPortal;
    private VisionPortal visionPortal2;
    //pick a custom model to use
    private static final String TFOD_MODEL_ASSET = "CenterstageHeartTraining2.tflite";
    //name options of objects identified
    private static final String[] LABELS = {
            "heartRed",
            "heartBlue"
    };


    //spike marker variable
    int spikeMark=3;


    //alliance color variable 1 red - 1 blue
    int alliance=1;
    //cord x variable
    double x;
    double x2;
    //cord y variable
    double y;
    double y2;

    @Override
    public void runOpMode() throws InterruptedException {
        //see function
        initTfod();
        //if we have started and have not stopped
        while(!opModeIsActive() && !isStopRequested()) {
            //if tfod has been initialized
            if (tfod != null) {
                //see function
                tfodtelemetry();
                //see function
                setSpikeMark();
            }
            // if tfod hasn't been initialized
            else {
                //say that it doesn't see anything and spike marker is default 3
                telemetry.addLine("Spike Marker Default (Didn't Load Properly) 3");
            }
        }


        //wait for start
        waitForStart();

    }


    //function to initialize tensorflow and vision portal with preferred settings
    private void initTfod() {

        //webcam 1 pink tape cam
        // start building custom tfod
        TfodProcessor.Builder builder = new TfodProcessor.Builder();
        //custom training model file import
        builder.setModelAssetName(TFOD_MODEL_ASSET);
        //get names of objects
        builder.setModelLabels(LABELS);
        // verifies the type of tensor flow model???? not really sure
        builder.setIsModelTensorFlow2(true);
        // no clue
        builder.setIsModelQuantized(true);
        // how many minutes of model will be used.
        builder.setModelInputSize(300);
        //set camera aspect ratio
        builder.setModelAspectRatio(16.0 / 9.0);
        //make custom tfod
        tfod = builder.build();


        //start building custom vision portal
        VisionPortal.Builder builder2 = new VisionPortal.Builder();
        //pick a camera
        builder2.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        // set camera resolution
        //this one already has pre calibrated apriltag info
        builder2.setCameraResolution(new Size(640, 480));
        //allow you to see the camera feed on driver hub
        builder2.enableLiveView(true);
        // streaming bandwidth
        builder2.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        // don't auto stop camera feed
        builder2.setAutoStopLiveView(false);
        //add custom tfod to vision portal
        builder2.addProcessor(tfod);
        //build vision portal
        visionPortal = builder2.build();


        //must be at least 75% confident to display data
        tfod.setMinResultConfidence(0.75f);


        //processor is on
        visionPortal.setProcessorEnabled(tfod, true);

        //webcam 2 no tape camera

        // start building custom tfod
        TfodProcessor.Builder builder3 = new TfodProcessor.Builder();
        //custom training model file import
        builder3.setModelAssetName(TFOD_MODEL_ASSET);
        //get names of objects
        builder3.setModelLabels(LABELS);
        // verifies the type of tensor flow model???? not really sure
        builder3.setIsModelTensorFlow2(true);
        // no clue
        builder3.setIsModelQuantized(true);
        // how many minutes of model will be used.
        builder3.setModelInputSize(300);
        //set camera aspect ratio
        builder3.setModelAspectRatio(16.0 / 9.0);
        //make custom tfod
        tfod2 = builder3.build();


        //start building custom vision portal
        VisionPortal.Builder builder4 = new VisionPortal.Builder();
        //pick a camera
        builder4.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        // set camera resolution
        //this one already has pre calibrated apriltag info
        builder4.setCameraResolution(new Size(640, 480));
        //allow you to see the camera feed on driver hub
        builder4.enableLiveView(true);
        // streaming bandwidth
        builder4.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        // don't auto stop camera feed
        builder4.setAutoStopLiveView(false);
        //add custom tfod to vision portal
        builder4.addProcessor(tfod);
        //build vision portal
        visionPortal2 = builder4.build();


        //must be at least 75% confident to display data
        tfod.setMinResultConfidence(0.75f);


        //processor is on
        visionPortal2.setProcessorEnabled(tfod2, true);
    }


    //function for tfod telemetry
    private void tfodtelemetry() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            //coordinates
            //average x coordinates
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            //average y coordinates
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;


            //add telemetry
            //label and recognition %
            telemetry.addData(recognition.getLabel(), "%s (%.0f %% Conf.)", recognition.getConfidence() * 100);
            // label and coordinates
            telemetry.addData("Position of ", recognition.getLabel(), "(", "%.0f / %.0f", x, y, ")");
            //label and size
            telemetry.addData("Size of ", recognition.getLabel(), "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            //update telemetry
            telemetry.update();


            //is the item is a red heart
            if (recognition.getLabel() .equals("heartRed")) {
                //set alliance 1 (red)
                alliance = 1;
                //add red alliance to telemetry
                telemetry.addLine("red alliance (1)");
            }
            //is the item is a blue heart
            else if (recognition.getLabel() .equals("heartBlue")) {
                //set alliance 1 (red)
                alliance = -1;
                //add blue alliance to telemetry
                telemetry.addLine("blue alliance (-1)");
            }
        }

        //tfod 2 other camera
        List<Recognition> currentRecognitions2 = tfod2.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions2.size());

        for (Recognition recognition : currentRecognitions2) {
            //coordinates
            //average x coordinates
            x2 = (recognition.getLeft() + recognition.getRight()) / 2 ;
            //average y coordinates
            y2 = (recognition.getTop()  + recognition.getBottom()) / 2 ;


            //add telemetry
            //label and recognition %
            telemetry.addData(recognition.getLabel(), "%s (%.0f %% Conf.)", recognition.getConfidence() * 100);
            // label and coordinates
            telemetry.addData("Position of ", recognition.getLabel(), "(", "%.0f / %.0f", x2, y2, ")");
            //label and size
            telemetry.addData("Size of ", recognition.getLabel(), "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            //update telemetry
            telemetry.update();


            //is the item is a red heart
            if (recognition.getLabel() .equals("heartRed")) {
                //set alliance 1 (red)
                alliance = 1;
                //add red alliance to telemetry
                telemetry.addLine("red alliance (1)");
            }
            //is the item is a blue heart
            else if (recognition.getLabel() .equals("heartBlue")) {
                //set alliance 1 (red)
                alliance = -1;
                //add blue alliance to telemetry
                telemetry.addLine("blue alliance (-1)");
            }
        }
    }


    //function for setting the spike marker
    private void setSpikeMark(){
        // determine spike mark
        //if pink cam sees left spike mark
        if (x<500){
            //set left
            spikeMark = 1;
            //say set left
            telemetry.addLine("Spike Marker Left 1");
        }
        //if pink cam sees middle spike mark
        else if (x>499){
            //set right
            spikeMark = 2;
            // say set right
            telemetry.addLine("Spike Marker middle (left) 2");
        }
        //if no tape cam sees middle spike mark
        else if (x2<550){
            spikeMark = 2;
            telemetry.addLine("Spike Marker middle (right) 2");
        }
        //if no tape cam sees right spoke mark
        else if (x2>549){
            spikeMark = 3;
            telemetry.addLine("Spike Marker right 3");
        }
        //if nothing is seen
        else{
            spikeMark = 3;
            telemetry.addLine("Spike Marker default (not identified) 3");
        }

        telemetry.update();
    }
}