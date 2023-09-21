package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "CameraTest", group="Iterative OpMode")
public class CameraTest extends LinearOpMode {
    private WebcamName camera;

    TfodProcessor tfodProcessor = TfodProcessor.easyCreateWithDefaults();

    VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(camera, tfodProcessor);

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName camera = hardwareMap.get(WebcamName.class, "webcam 1");

        List<Recognition> recognitions = tfodProcessor.getRecognitions();

        for (Recognition recognition : recognitions);{

            String label = recognition.getLabel();

            float confidence = recognition.getConfidence();
        }
    }
}
