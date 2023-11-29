package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.PipelineBlue;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue Auto")
    public class BlueAuto extends LinearOpMode {
        public OpenCvCamera webcam;
        PipelineBlue pipeline;
        public DcMotorEx left;
        public DcMotorEx right;

        int LeftPosition;
        int RightPosition;

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1/20 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


        @Override
        public void runOpMode() {
            left = hardwareMap.get(DcMotorEx.class, "left");
            right = hardwareMap.get(DcMotorEx.class, "right");
            //while (!opModeIsActive()) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            pipeline = new PipelineBlue();
            webcam.setPipeline(pipeline);

            left.setDirection(DcMotorSimple.Direction.FORWARD);
            right.setDirection(DcMotorSimple.Direction.REVERSE);

            left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            LeftPosition = 0;
            RightPosition = 0;

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
                telemetry.addData("Realtime analysis", PipelineBlue.get_element_zone());
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
            } else {
                telemetry.addLine("Zone 2 Detected");
                telemetry.update();
            }
        }

    public void Forward(int Distance, double Velocity) {
            double DistanceTicks = (int) (Distance * COUNTS_PER_INCH);
            double leftCurrent = left.getCurrentPosition();
            double rightCurrent = right.getCurrentPosition();
            double leftTarget = leftCurrent + DistanceTicks;
            double rightTarget = rightCurrent + DistanceTicks;

            left.setTargetPosition((int) leftTarget);
            right.setTargetPosition((int) rightTarget);

            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            left.setPower(Math.abs(Velocity));
            right.setPower(Math.abs(Velocity));

            while (left.isBusy() && right.isBusy()){
                telemetry.addLine("Running");
                telemetry.update();
            }

            left.setPower(0);
            right.setPower(0);
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(500);

    }

}


