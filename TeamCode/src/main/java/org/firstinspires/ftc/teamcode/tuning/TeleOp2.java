package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

@TeleOp(name = "RRTOP")
public class TeleOp2 extends LinearOpMode {

    boolean LastGamepad1_dpad_down    = false;
    boolean CurrentGamepad1_dpad_down = false;
    boolean LastGamepad1_B    = false;
    boolean CurrentGamepad1_B = false;
    boolean LastGamepad1_dpad_up    = false;
    boolean CurrentGamepad1_dpad_up = false;
    boolean LastGamepad1_A    = false;
    boolean CurrentGamepad1_A = false;
    boolean LastGamepad1_right_bumper    = false;
    boolean CurrentGamepad1_right_bumper = false;
    boolean CurrentGamepad1_left_bumper = false;
    boolean LastGamepad1_left_bumper = false;
    boolean CurrentGamepad2_X = false;
    boolean LastGamepad2_X = false;
    boolean CurrentGamepad2_Y = false;
    boolean LastGamepad2_Y = false;
    boolean CurrentGamepad2_A = false;
    boolean LastGamepad2_A = false;
    boolean LastGamepad2_dpad_up = false;
    boolean CurrentGamepad2_dpad_up = false;
    boolean CurrentGamepad2_dpad_left = false;
    boolean LastGamepad2_dpad_left = false;
    boolean CurrentGamepad2_dpad_right = false;
    boolean LastGamepad2_dpad_right = false;
    boolean CurrentGamepad1_Y = false;
    boolean LastGamepad1_Y = false;
    boolean LastGamepad2_dpad_down = false;
    boolean CurrentGamepad2_dpad_down = false;
    int    IntakeFlag  = 0;
    int    TiltFlag  = 0;
    int    BoxFlag   = 0;
    int    ClipFlag  = 0;
    int    ClawRotateFlag = 0;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor MOTOR_LR = hardwareMap.get(DcMotor.class, "MOTOR_LR");
        DcMotor ODOMETRY1 = hardwareMap.get(DcMotor.class, "ODOMETRY1");
        DcMotor ODOMETRY3 = hardwareMap.get(DcMotor.class, "ODOMETRY3");


        MOTOR_LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ODOMETRY1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ODOMETRY3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Servo IntakeArm_Servo;
        Servo IntakeTilt_Servo1;
        Servo IntakeTilt_Servo2;
        Servo Box_Servo1; // left
        Servo Box_Servo2; // right
        Servo Clip_Servo;
        Servo IntakeFlip_Servo;
        Servo Claw_Servo;

        IntakeArm_Servo = hardwareMap.servo.get("IntakeArm_Servo");
        IntakeTilt_Servo1 = hardwareMap.servo.get("IntakeTilt_Servo1");
        IntakeTilt_Servo2 = hardwareMap.servo.get("IntakeTilt_SERVO2");
        Box_Servo1 = hardwareMap.servo.get("Box_Servo1");
        Box_Servo2 = hardwareMap.servo.get("Box_Servo2");
        Clip_Servo = hardwareMap.servo.get("Clip_Servo");
        IntakeFlip_Servo = hardwareMap.servo.get("IntakeFlip_SERVO");
        Claw_Servo = hardwareMap.servo.get("Claw_SERVO");


        IntakeArm_Servo.setPosition(0.0);
        IntakeTilt_Servo1.setPosition(0.625);   // 0.3 = Horizontal   0.625 = Vertical
        IntakeTilt_Servo2.setPosition(0.0);  // 0.75 = pickup level  0.0 = init
        Box_Servo1.setPosition(0.25);
        Box_Servo2.setPosition(0.75);
        Clip_Servo.setPosition(0.5);
        IntakeFlip_Servo.setPosition(0.22);    //   0.3 init  0.55 look level  0.61 grab level   0.22 = drop level
        Claw_Servo.setPosition(0.7);  // 0.9 to close 1 for lock
        // Intake_Servo.setPower(-1);
        // sleep(3000);
        // Intake_Servo.setPower(0);

        MOTOR_LR.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {

                CurrentGamepad1_dpad_up = gamepad1.dpad_up;
                CurrentGamepad1_B = gamepad1.b;
                CurrentGamepad1_dpad_down = gamepad1.dpad_down;
                CurrentGamepad1_A = gamepad1.a;
                CurrentGamepad1_right_bumper = gamepad1.right_bumper;
                CurrentGamepad1_left_bumper = gamepad1.left_bumper;
                CurrentGamepad2_X = gamepad2.x;
                CurrentGamepad2_Y = gamepad2.y;
                CurrentGamepad2_A = gamepad2.a;
                CurrentGamepad2_dpad_up = gamepad2.dpad_up;
                CurrentGamepad2_dpad_left = gamepad2.dpad_left;
                CurrentGamepad2_dpad_right = gamepad2.dpad_right;
                CurrentGamepad1_Y = gamepad1.y;
                CurrentGamepad2_dpad_down = gamepad2.dpad_down;
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.right_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                MOTOR_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                ODOMETRY1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                ODOMETRY3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



                if (gamepad2.right_bumper == false && gamepad2.left_bumper == false) {    // no buttons pressed set power to -0.005
                    MOTOR_LR.setPower(-0.05);
                }
                if (gamepad2.dpad_right == false && gamepad2.dpad_left == false) {
                    ODOMETRY1.setPower(0.0);
                    ODOMETRY3.setPower(0.0);
                }

                if (CurrentGamepad1_right_bumper && !LastGamepad1_right_bumper) {
                    if (TiltFlag == 0) {
                        IntakeFlip_Servo.setPosition(0.55);               // to look level
                        IntakeTilt_Servo2.setPosition(0.75);
                        TiltFlag = 1;
                    } else if (TiltFlag == 1) {
                        IntakeFlip_Servo.setPosition(0.22);               // drop to bucket level
                        IntakeTilt_Servo2.setPosition(0.0);
                        IntakeTilt_Servo1.setPosition(0.625);
                        TiltFlag = 0;
                    } else if (TiltFlag == 2) {
                        IntakeFlip_Servo.setPosition(0.22);             // drop into bucket level
                        IntakeTilt_Servo2.setPosition(0.0);
                        IntakeTilt_Servo1.setPosition(0.625);
                        TiltFlag = 1;
                    } else if (TiltFlag == 3) {
                        IntakeFlip_Servo.setPosition(0.55);               // to look level
                        IntakeTilt_Servo2.setPosition(0.75);
                        TiltFlag = 1;
                    }
                }

                if (CurrentGamepad1_left_bumper && !LastGamepad1_left_bumper) {      //pickup level
                    if (TiltFlag == 1) {
                        IntakeFlip_Servo.setPosition(0.61);
                        TiltFlag = 2;
                    } else if (TiltFlag == 2) {
                        IntakeFlip_Servo.setPosition(0.45);
                        TiltFlag = 1;
                    }
                }

                if (gamepad1.dpad_up == true) {         // vertical linear rack out
                    IntakeArm_Servo.setPosition(0.23);
                }


                if (gamepad1.dpad_down == true) {     // vertical linear rack init position
                    IntakeArm_Servo.setPosition(0.0);
                }

                if (CurrentGamepad1_A && !LastGamepad1_A) {   // open and close claw
                    if (IntakeFlag == 0) {
                        Claw_Servo.setPosition(1);
                        IntakeFlag = 1;
                    } else if (IntakeFlag == 1) {
                        Claw_Servo.setPosition(0.7);
                        IntakeFlag = 0;
                    }
                }


                if (CurrentGamepad1_B && !LastGamepad1_B) {     // rotate claw
                    if (ClawRotateFlag == 0) {
                        IntakeTilt_Servo1.setPosition(0.3);
                        ClawRotateFlag = 1;
                    } else if (ClawRotateFlag == 1) {
                        IntakeTilt_Servo1.setPosition(0.625);
                        ClawRotateFlag = 0;
                    }
                    //    else Intake_Servo.setPower(1);
                }

                if (CurrentGamepad2_Y && !LastGamepad2_Y) {     // open and close clip claw
                    if (ClipFlag == 0) {
                        Clip_Servo.setPosition(0.2);
                        ClipFlag = 1;
                    } else if (ClipFlag == 1) {
                        Clip_Servo.setPosition(0.5);
                        ClipFlag = 0;
                    }
                }


                if (CurrentGamepad2_X && !LastGamepad2_X) {    // bucket up and down
                    if (BoxFlag == 1) {
                        IntakeTilt_Servo1.setPosition(0.3);
                        IntakeTilt_Servo2.setPosition(0.75);
                        Claw_Servo.setPosition(0.7);
                        IntakeFlip_Servo.setPosition(0.55);
                        sleep(400);
                        Box_Servo1.setPosition(0.25);
                        Box_Servo2.setPosition(0.75);
                        TiltFlag = 1;
                        BoxFlag = 0;
                    } else if (BoxFlag == 0) {
                        IntakeTilt_Servo1.setPosition(0.3);
                        IntakeTilt_Servo2.setPosition(0.75);
                        Claw_Servo.setPosition(0.7);
                        IntakeFlip_Servo.setPosition(0.55);
                        sleep(400);
                        Box_Servo1.setPosition(0.9);
                        Box_Servo2.setPosition(0.1);
                        TiltFlag = 1;
                        BoxFlag = 2;
                    }
                    else if (BoxFlag == 2) {
                        IntakeTilt_Servo1.setPosition(0.3);
                        IntakeTilt_Servo2.setPosition(0.75);
                        Claw_Servo.setPosition(0.7);
                        IntakeFlip_Servo.setPosition(0.55);
                        sleep(200);
                        Box_Servo1.setPosition(0.25);
                        Box_Servo2.setPosition(0.75);
                        TiltFlag = 1;
                        BoxFlag = 0;
                    }
                }

                if (CurrentGamepad1_Y && !LastGamepad1_Y){
                    IntakeTilt_Servo1.setPosition(0.3);
                    IntakeTilt_Servo2.setPosition(0.75);
                    Claw_Servo.setPosition(0.7);
                    IntakeFlip_Servo.setPosition(0.3);
                    TiltFlag = 3;
                }
/*   IntakeArm_Servo.setPosition(0.0);
                IntakeTilt_Servo1.setPosition(0.625);   // 0.3 = Horizontal   0.625 = Vertical
                IntakeTilt_Servo2.setPosition(0.7);  // 0.7 = pickup level  0.0 = init
                Box_Servo1.setPosition(0.25);
                Box_Servo2.setPosition(0.75);
                Release_Servo.setPosition(0.75);
                Clip_Servo.setPosition(0.5);
                IntakeFlip_Servo.setPosition(0.55);    //   0.3 init  0.55 look level  0.61 grab level   0.22 = drop level
                Claw_Servo.setPosition(0.7);  // 0.9 to close 1 for lock  0.7 = open
              */
                if (gamepad1.dpad_right) {
                    if (IntakeTilt_Servo1.getPosition() >= 1);
                    IntakeTilt_Servo1.setPosition(IntakeTilt_Servo1.getPosition() - 0.005);
                }
                if (gamepad1.dpad_left) {
                    if (IntakeTilt_Servo1.getPosition() <= 0) ;
                    IntakeTilt_Servo1.setPosition(IntakeTilt_Servo1.getPosition() + 0.005);
                }

                if (gamepad2.left_bumper == true) {    // linear rack down
                    MOTOR_LR.setPower(0.65);
                }


                if (gamepad2.right_bumper == true) {     // linear rack up
                    MOTOR_LR.setPower(-0.65);
                }

                if (CurrentGamepad2_dpad_down && !LastGamepad2_dpad_down) {
                    MOTOR_LR.setTargetPosition(-20);
                    MOTOR_LR.setPower(-1);
                    MOTOR_LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);  sleep(2000);
                    MOTOR_LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                }

                if (CurrentGamepad2_dpad_up && !LastGamepad2_dpad_up) {   // auto bucket score
                    IntakeTilt_Servo1.setPosition(0.3);
                    IntakeTilt_Servo2.setPosition(0.75);
                    Claw_Servo.setPosition(0.7);
                    IntakeFlip_Servo.setPosition(0.3);
                    sleep(500);
                    MOTOR_LR.setTargetPosition(-3470);
                    MOTOR_LR.setPower(1);
                    MOTOR_LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);  sleep(2000);
                    MOTOR_LR.setPower(0.01);
                    Box_Servo1.setPosition(0.9);
                    Box_Servo2.setPosition(0.1);  sleep(900);
                    Box_Servo1.setPosition(0.265);
                    Box_Servo2.setPosition(0.735);
                    MOTOR_LR.setPower(0);
                /*
                MOTOR_LR.setTargetPosition(0);
                MOTOR_LR.setPower(-0.5);
                MOTOR_LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);  sleep(3500)
                 */
                    TiltFlag = 3;
                    BoxFlag = 0;
                    MOTOR_LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                if (CurrentGamepad2_dpad_right && !LastGamepad2_dpad_right) {     // hang up
                    ODOMETRY1.setPower(1);
                    ODOMETRY3.setPower(1);
                }
                if (CurrentGamepad2_dpad_left && LastGamepad2_dpad_left) {       // hang down
                    ODOMETRY1.setPower(-1);
                    ODOMETRY3.setPower(-1);
                }

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("MOTOR_LR ", MOTOR_LR.getCurrentPosition());
                //   telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.update();

                LastGamepad1_dpad_down = CurrentGamepad1_dpad_down;
                LastGamepad1_B = CurrentGamepad1_B;
                LastGamepad1_dpad_up = CurrentGamepad1_dpad_up;
                LastGamepad1_A = CurrentGamepad1_A;
                LastGamepad1_right_bumper = CurrentGamepad1_right_bumper;
                LastGamepad1_left_bumper = CurrentGamepad1_left_bumper;
                LastGamepad2_X = CurrentGamepad2_X;
                LastGamepad2_Y = CurrentGamepad2_Y;
                LastGamepad2_A = CurrentGamepad2_A;
                LastGamepad2_dpad_up = CurrentGamepad2_dpad_up;
                LastGamepad2_dpad_left = CurrentGamepad2_dpad_left;
                LastGamepad2_dpad_right = CurrentGamepad2_dpad_right;
                LastGamepad1_Y = CurrentGamepad1_Y;
                LastGamepad2_dpad_down = CurrentGamepad2_dpad_down;



            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
