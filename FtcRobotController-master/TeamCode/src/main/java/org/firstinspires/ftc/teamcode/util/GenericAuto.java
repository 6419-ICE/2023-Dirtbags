package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.PipelineRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "Generic")
public class GenericAuto extends LinearOpMode {
    public OpenCvCamera webcam;
    PipelineRed pipeline;
    public DcMotorEx left;
    public DcMotorEx right;
    public DcMotorEx intake;

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

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        LeftPosition = 0;
        RightPosition = 0;

        //}
        waitForStart();
        if (opModeIsActive()) {
            Forward(15, 0.75);
        }
        while (opModeIsActive())
        {
            telemetry.addData("Left Encoder Position: ", left.getCurrentPosition());
            telemetry.addData("Right Encoder Position: ", right.getCurrentPosition());
            telemetry.addData("Left Encoder Target: ", left.getTargetPosition());
            telemetry.addData("Right Encoder Target: ", right.getTargetPosition());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        Forward(15, 0.75);

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

        while (left.getCurrentPosition() < left.getTargetPosition() && right.getCurrentPosition() < right.getTargetPosition()){
            left.setPower(Math.abs(Velocity));
            right.setPower(Math.abs(Velocity));
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


