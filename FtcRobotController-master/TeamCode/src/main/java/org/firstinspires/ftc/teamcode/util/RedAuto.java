package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.PipelineRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Auto")
public class RedAuto extends LinearOpMode {
    public OpenCvCamera webcam;
    PipelineRed pipeline;



    @Override
    public void runOpMode() {

        //while (!opModeIsActive()) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new PipelineRed();
        webcam.setPipeline(pipeline);



        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });


        //}
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("Realtime analysis", PipelineRed.get_element_zone());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        if (pipeline.get_element_zone() == 1) {
            telemetry.addLine("Zone 1 was Detected");
            telemetry.update();
        } else if (pipeline.get_element_zone() == 3) {
            telemetry.addLine("Zone 3 was Detected");
            telemetry.update();
        } else if (pipeline.get_element_zone() == 2){
            telemetry.addLine("Zone 2 Detected");
            telemetry.update();
        } else {
            telemetry.addLine("No sleeve detected, defaulting to 2");
            telemetry.update();
        }
    }



}


