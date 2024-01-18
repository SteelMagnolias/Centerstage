// group of code/location
package org.firstinspires.ftc.teamcode.Autonomous;

//import important files from sdk
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortal.CameraState;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import android.util.Size;

//define class and name for driver hub
@Autonomous(name = "CameraTest", group="Iterative OpMode")
public class CameraTest extends LinearOpMode {

    //define camera
    private WebcamName camera1;
    private WebcamName camera2;

    //define processor variable
    private TfodProcessor tfod;
    //define custom vision portal variable
    private VisionPortal visionPortal;
    //pick a custom model to use
    private static final String TFOD_MODEL_ASSET = "CenterstageHeartTraining2.tflite";
    //name options of objects identified
    private static final String[] LABELS = {
            "heartBlue",
            "heartRed"
    };


    //spike marker variable
    int spikeMark=3;

    //alliance color variable 1 red - 1 blue
    int alliance=1;
    int cam=1;
    //cord x variable
    double x;
    //cord y variable
    double y;
    @Override
    public void runOpMode() throws InterruptedException {
        //if we have started and have not stopped
        camera1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera2 = hardwareMap.get(WebcamName.class, "Webcam 2");

        initTfod();
        while(!opModeIsActive() && !isStopRequested()) {
            //if tfod has been initialized
            if (tfod != null) {
                switchCameras();
                //see function
                tfodtelemetry();
                //see function
                setSpikeMark();
            }
            // if tfod hasn't been initialized
            else {
                //say that it doesn't see anything and spike marker is default 3
                telemetry.addLine("Didn't load properly Spike Marker 3");
            }
            sleep(20);
        }

        //wait for start
        waitForStart();
    }

    //function to initialize tensorflow and vision portal with preferred settings
    private void initTfod() {
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
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(camera1, camera2);
        builder2.setCamera(switchableCamera);
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
        telemetry.addData("camera state", visionPortal.getCameraState());
    }

    //function for tfod telemetry
    private void tfodtelemetry() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            //continents
            //average x coordinates
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            //average y coordinates
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
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
        //if x is left of set value
        if (cam == 1 && x < 300){
                spikeMark = 1;
                telemetry.addLine("Spike Mark Left-1");
        }
        //if x is in between set values
        else if (cam == 2) {
            if (x > 300){
                spikeMark =2;
                telemetry.addLine("Spike Mark Middle-2");
            }
            else if (x < 299){
                spikeMark = 3;
                telemetry.addLine("Spike Mark Right-3");
            }
        }
        //if nothing is seen
        else{
            //set right
            spikeMark = 0;
            //say set right but also default
            telemetry.addLine("Spike Mark None-0");
        }
    }
    private void switchCameras(){
       if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
           if (visionPortal.getActiveCamera().equals((camera1.isWebcam()))){
               cam = 1;
           }
           else {
               cam = 2;
           }
           if (cam == 2 && spikeMark == 0) {
              visionPortal.setActiveCamera(camera1);
           } else if (cam == 1 && spikeMark == 0) {
               visionPortal.setActiveCamera(camera2);
           }else if (cam == 1){
               visionPortal.setActiveCamera(camera1);
           }else{
               visionPortal.setActiveCamera(camera2);
           }
       }
    }
}