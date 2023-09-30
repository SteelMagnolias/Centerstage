package org.firstinspires.ftc.teamcode.autonmous;

//import importaint files from sdk
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import android.util.Size;


@Autonomous(name = "CameraTest", group="Iterative OpMode")
public class CameraTest extends LinearOpMode {

    //define camera
    private WebcamName camera;

    //define processor variable
    private TfodProcessor tfod;

    private VisionPortal visionPortal;

    //pick a custom model to use
    private static final String TFOD_MODEL_ASSET = "change this when have custom model";

    //name of objects identified
    private static final String[] LABELS = {
            "heart"
    };

    //spike marker variable
    int spikeMark=0;
    //cord x variable
    double x;
    //cord y variable
    double y;
    @Override
    public void runOpMode() throws InterruptedException {
        //see function
        initTfod();
        //see function
        tfodtelemetry();

        // determin spike mark
        //if x is left of set value
        if (x<20){
            //set left
            spikeMark = 1;
        }
        //if x is in between set values
        else if (20<x & x<40) {
            //set center
            spikeMark = 2;
        }
        //if more than set value
        else if (x>40){
            //set right
            spikeMark = 3;
        }
        //if nothing
        else{
            //set right
            spikeMark = 3;
        }

        waitForStart();

    }

    //function to initilize tensorflow with prefered settings
    private void initTfod() {

        // start building custom tfod
        TfodProcessor.Builder builder = new TfodProcessor.Builder();
        //custom training modle file import
        builder.setModelFileName(TFOD_MODEL_ASSET);
        //get names of objects
        builder.setModelLabels(LABELS);
        // verifys the type of tensor flow model???? not really sure
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
        //this one already has precalibrated apriltag info
        builder2.setCameraResolution(new Size(640, 480));
        //allow you to see the camera feed on driver hub
        builder2.enableLiveView(true);
        // streaming bandwith
        builder2.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        // dont auto stop camera feed
        builder2.setAutoStopLiveView(false);
        //add custom tfod to vision portal
        builder2.addProcessor(tfod);
        //build vision portal
        visionPortal = builder2.build();

        //must be at least 75% confident to display data
        tfod.setMinResultConfidence(0.75f);

        //processor is on
        visionPortal.setProcessorEnabled(tfod, true);
    }

    private void tfodtelemetry() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            //cordinents
            //average x cordinents
             x = (recognition.getLeft() + recognition.getRight()) / 2 ;
           //average y cordinents
             y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            //add telemetry
            //label and regignition %
            telemetry.addData(recognition.getLabel(), "%s (%.0f %% Conf.)", recognition.getConfidence() * 100);
            // label and cordinents
            telemetry.addData("Position of ", recognition.getLabel(), "%.0f / %.0f", x, y);
            //label and size
            telemetry.addData("Size of ", recognition.getLabel(), "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            //update telemetry
            telemetry.update();
        }
    }
}