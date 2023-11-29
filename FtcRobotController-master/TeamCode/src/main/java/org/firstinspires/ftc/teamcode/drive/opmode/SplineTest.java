package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;
    DcMotor ArmHeight;
    ColorSensor color;
    Servo VWrist;
    DcMotorEx H1;
    DcMotor H2;
    DcMotor Turret;
    Servo VRight;
    Servo VLeft;
    Servo HRight;
    Servo HLeft;
    Servo HWrist;

    final double  TicksCount            = 537.7;
    final double  GearRatio             = 1.0;     // No Gearing.
    final double  WheelDiameterIn       = 3.77953;     // For circumference(r2pi)
    double  TickstoInchesWheel          = (TicksCount * GearRatio) / (WheelDiameterIn * 3.1415);
    public double Position = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        ArmHeight = hardwareMap.dcMotor.get("AH");
        VWrist = hardwareMap.servo.get("VWrist");
        VRight = hardwareMap.servo.get("VRight");
        color = hardwareMap.colorSensor.get("color");

        VLeft = hardwareMap.servo.get("VLeft");
        VRight = hardwareMap.servo.get("VRight");

        HLeft = hardwareMap.servo.get("HLeft");
        HRight = hardwareMap.servo.get("HRight");
        HWrist = hardwareMap.servo.get("HWrist");
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");

        ArmHeight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmHeight.setDirection(DcMotorSimple.Direction.REVERSE);
        ArmHeight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        VWrist.setPosition(0.59);
        sleep(1000);
        VRight.setPosition(1);
        VLeft.setPosition(0);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory toCone = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-17.5, 4.5))
                .build();

        Trajectory toPole1 = drive.trajectoryBuilder(new Pose2d(-16, 6.4))
                .lineToLinearHeading(new Pose2d(-54, 6.4, Math.toRadians(-90)))
                .build();

        Trajectory toPole2 = drive.trajectoryBuilder(new Pose2d(-56, 6.4, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-56, -23, Math.toRadians(-45)))
                .addDisplacementMarker(() -> {
                    VWrist.setPosition(1);
                })
                .build();

        Trajectory StraightTurn = drive.trajectoryBuilder(new Pose2d(-49, -22, Math.toRadians(-45)))
                .lineToLinearHeading(new Pose2d(-56, -21))
                .build();

        drive.followTrajectory(toCone);

        double red = color.red();
        double blue = color.blue();
        double green = color.green();
        double color_num = 0;

        if (blue > red && blue > green) {
            color_num = 3;
        }
        if (green > blue && green > red) {
            color_num = 2;
        }
        if (red > blue && red > green) {
            color_num = 1;
        }
        sleep(500);

        drive.followTrajectory(toPole1);

        drive.followTrajectory(toPole2);

        ArmHeight.setTargetPosition(3000);
        ArmHeight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (ArmHeight.getCurrentPosition() < ArmHeight.getTargetPosition()) {
            ArmHeight.setPower(1);
        }
        ArmHeight.setPower(0);

        VWrist.setPosition(0);
        sleep(1000);

        ArmHeight.setTargetPosition(0);
        ArmHeight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (ArmHeight.getCurrentPosition() > ArmHeight.getTargetPosition()) {
            ArmHeight.setPower(-0.8);
        }
        ArmHeight.setPower(0);

        VRight.setPosition(0);
        VLeft.setPosition(1);

        drive.followTrajectory(StraightTurn);

        VWrist.setPosition(0.6);


        if (color_num == 2) {
            LEFT_ENCODERS_AUTONOMOUS(0.8, 35);
            LFMotor.setPower(0.5);
            RFMotor.setPower(0.5);
            LBMotor.setPower(0.5);
            RBMotor.setPower(0.5);
            sleep(500);
            LFMotor.setPower(0);
            RFMotor.setPower(0);
            LBMotor.setPower(0);
            RBMotor.setPower(0);

        } else if (color_num == 3) {
            LEFT_ENCODERS_AUTONOMOUS(0.8, 61.5);
            LFMotor.setPower(0.5);
            RFMotor.setPower(0.5);
            LBMotor.setPower(0.5);
            RBMotor.setPower(0.5);
            sleep(500);
            LFMotor.setPower(0);
            RFMotor.setPower(0);
            LBMotor.setPower(0);
            RBMotor.setPower(0);
        }
    }

    public void LEFT_ENCODERS_AUTONOMOUS(double speed, double inches) {

        int ForwardDriveLFC = LFMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        int ForwardDriveRFC = RFMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        int ForwardDriveLBC = LBMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        int ForwardDriveRBC = RBMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        RFMotor.setTargetPosition(ForwardDriveRFC);
        RBMotor.setTargetPosition(-ForwardDriveRBC);
        LFMotor.setTargetPosition(-ForwardDriveLFC);
        LBMotor.setTargetPosition(ForwardDriveLBC);

        while (RFMotor.getCurrentPosition() < ForwardDriveRFC && LBMotor.getCurrentPosition() < ForwardDriveLBC) {
            RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFMotor.setPower(Math.abs(speed));
            RBMotor.setPower(Math.abs(speed));
            LFMotor.setPower(Math.abs(speed));
            LBMotor.setPower(Math.abs(speed));
        }
        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);
        LFMotor.setPower(0);

        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
