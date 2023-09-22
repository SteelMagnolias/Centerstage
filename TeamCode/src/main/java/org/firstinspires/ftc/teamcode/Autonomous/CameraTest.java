package org.firstinspires.ftc.teamcode.Autonomous;

//import importaint files from sdk
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "CameraTest", group="Iterative OpMode")
public class CameraTest extends LinearOpMode {

    //define camera
    private WebcamName camera;

    //create tensor flow processor this will only do default items and settings
    TfodProcessor tfodProcessor = TfodProcessor.easyCreateWithDefaults();

    //create vision portal stream with default settings use camera and send to tfodprocessor
    VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(camera, tfodProcessor);

    @Override
    public void runOpMode() throws InterruptedException {
        // gets webcam info from configuration
        WebcamName camera = hardwareMap.get(WebcamName.class, "webcam 1");

        // recognize things
        List<Recognition> recognitions = tfodProcessor.getRecognitions();

        // list recognitions
        for (Recognition recognition : recognitions){

            //prep label
            String label = recognition.getLabel();

            //prep confidence
            float confidence = recognition.getConfidence();

            //add object to telemetry
            telemetry.addLine("recognized object: " + label);
            telemetry.addLine("confidence: " + confidence);

            //update telemetry
            telemetry.update();
        }
    }
}