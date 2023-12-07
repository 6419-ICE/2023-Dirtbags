package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name="TeleOp", group = "1")
    public class Teleopp extends OpMode
    {
        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor LeftDrive = null;
        private DcMotor RightDrive = null;

        private DcMotor Intake = null;
        static final double     LEFT_CORRECTION         = 1.0;
        static final double     RIGHT_CORRECTION        = 1.1;

        private DcMotor AcutatorForIntake = null;

        private Servo hangerServo = null;

        private Servo servo = null;





        @Override
        public void init() {
            AcutatorForIntake = hardwareMap.get(DcMotor.class, "actuator");
            Intake = hardwareMap.get(DcMotor.class, "intake");
            LeftDrive  = hardwareMap.get(DcMotor.class, "left");
            RightDrive = hardwareMap.get(DcMotor.class, "right");
            servo = hardwareMap.get(Servo.class, "servo");
            hangerServo = hardwareMap.get(Servo.class, "hang");
            LeftDrive.setDirection(DcMotor.Direction.REVERSE);
            RightDrive.setDirection(DcMotor.Direction.FORWARD);
            LeftDrive.setZeroPowerBehavior(BRAKE);
            RightDrive.setZeroPowerBehavior(BRAKE);

            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Initialized");
        }

        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */
        @Override
        public void init_loop() {
        }

        /*
         * Code to run ONCE when the driver hits PLAY
         */
        @Override
        public void start() {
            runtime.reset();
        }

        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */
        @Override
        public void loop() {
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double Turbo;
            if (gamepad1.right_trigger >  0.5) {
                Turbo = 1;
            } else {
                Turbo = 0.6;
            }

            if(gamepad1.left_trigger > 0.5){
                if(gamepad1.x){
                    hangerServo.setPosition(1);
                }
                else {
                    hangerServo.setPosition(0);
                }

            }
            double drive = gamepad1.left_stick_y * Turbo;
            double turn  =  -gamepad1.right_stick_x * Turbo;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) * LEFT_CORRECTION;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) * RIGHT_CORRECTION;

            if (gamepad1.right_bumper) {
                //servo.setPosition(1);
                Intake.setPower(1);

            } else if (gamepad1.left_bumper) {
                //servo.setPosition(0);
                Intake.setPower(-1);
            } else {
                Intake.setPower(0);
            }

            if(gamepad1.a){
                AcutatorForIntake.setPower(1);
            } else if(gamepad1.b){
                AcutatorForIntake.setPower(-1);

            }
            else {
                AcutatorForIntake.setPower(0);
            }



            // Send calculated power to wheels
            LeftDrive.setPower(leftPower);
            RightDrive.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {

        }
    }


