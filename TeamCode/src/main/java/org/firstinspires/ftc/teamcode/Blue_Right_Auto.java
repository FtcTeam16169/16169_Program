package org.firstinspires.ftc.teamcode;
// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
        import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
        import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="Auto1", group = "Auto_1")

public class Blue_Right_Auto extends LinearOpMode {

    // Initialization - Varibales
    double MotorPower;
    int DELAY_START;
    int Arm_Level_0 = 0;
    public String ALLIANCE_COLOR = "Press X or B";
    public String START_POSITION = "Press Y or A";
    private DcMotor MOTOR_LR = null;
    private Servo IntakeArm_Servo;
    private Servo Box_Servo1;
    private Servo Box_Servo2;
    private Servo Clip_Servo;
    private Servo IntakeTilt_Servo1;
    private Servo IntakeTilt_Servo2;
    private Servo IntakeFlip_Servo;
    private Servo Claw_Servo;


    @Override
    public void runOpMode() {
        // Autonomous Configuration
        // ========================
        DELAY_START = 0;            // Delay start
        MOTOR_LR = hardwareMap.get(DcMotor.class, "MOTOR_LR");
        IntakeArm_Servo = hardwareMap.servo.get("IntakeArm_Servo");
        IntakeTilt_Servo1 = hardwareMap.servo.get("IntakeTilt_Servo1");
        IntakeTilt_Servo2 = hardwareMap.servo.get("IntakeTilt_SERVO2");
        Box_Servo1 = hardwareMap.servo.get("Box_Servo1");
        Box_Servo2 = hardwareMap.servo.get("Box_Servo2");
        Clip_Servo = hardwareMap.servo.get("Clip_Servo");
        IntakeFlip_Servo = hardwareMap.servo.get("IntakeFlip_SERVO");
        Claw_Servo = hardwareMap.servo.get("Claw_SERVO");
        // Initialization - Servo Motors
        IntakeArm_Servo.setPosition(0.0);
        IntakeTilt_Servo1.setPosition(0.625);   // 0.3 = Horizontal   0.625 = Vertical
        IntakeTilt_Servo2.setPosition(0.0);  // 0.7 = pickup level  0.0 = init
        Box_Servo1.setPosition(0.25);
        Box_Servo2.setPosition(0.75);
        Clip_Servo.setPosition(0.5);
        IntakeFlip_Servo.setPosition(0.22);    //   0.3 init  0.55 look level  0.61 grab level   0.22 = drop level
        Claw_Servo.setPosition(0.7);  // 0.9 to close 1 for lock

        // Initialization - DC Motors
        MOTOR_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d initialPose = new Pose2d(-16, 60, Math.toRadians(90)); // starting // DONE
        Pose2d deliver1 = new Pose2d(-16, 40, Math.toRadians(90));
        Pose2d deliver2 = new Pose2d(-16, 36, Math.toRadians(90));
   //     Pose2d Push1 = new Pose2d(-50, 40, Math.toRadians(90));
    //    Pose2d Push2 = new Pose2d(-50, 5, Math.toRadians(90));
     //   Pose2d Push3 = new Pose2d(-65, 5, Math.toRadians(90));
    //    Pose2d Push4 = new Pose2d(-65, 55, Math.toRadians(90));
        Pose2d Grab1 = new Pose2d(-65, 30, Math.toRadians(90));
        Pose2d Grab2 = new Pose2d(-65, 40, Math.toRadians(248));
        Pose2d Grab3 = new Pose2d(-55, 80, Math.toRadians(248));
        Pose2d Score1 = new Pose2d(10, 50, Math.toRadians(248));
        Pose2d Score2 = new Pose2d(10, 45, Math.toRadians(90));
        Pose2d Score3 = new Pose2d(10, 35, Math.toRadians(90));
        Pose2d Score4 = new Pose2d(10, 23, Math.toRadians(90));
        Pose2d Score5 = new Pose2d(-55, 70, Math.toRadians(90));


        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Pose2d pose = drive.localizer.getPose();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(deliver1, Math.toRadians(90));
        tab1.build();

        TrajectoryActionBuilder trb2 = drive.actionBuilder(deliver1)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(deliver2,Math.toRadians(90));
        trb2.build();
/*
        TrajectoryActionBuilder trb3 = drive.actionBuilder(deliver2)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Push1,Math.toRadians(90));
        trb3.build();

        TrajectoryActionBuilder trb4 = drive.actionBuilder(Push1)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Push2,Math.toRadians(90));
        trb4.build();

        TrajectoryActionBuilder trb5 = drive.actionBuilder(Push2)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Push3,Math.toRadians(90));
        trb5.build();

        TrajectoryActionBuilder trb6 = drive.actionBuilder(Push3)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Push4,Math.toRadians(90));
        trb6.build();

        TrajectoryActionBuilder trb7 = drive.actionBuilder(Push4)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Grab1,Math.toRadians(90));
        trb7.build();

 */

        TrajectoryActionBuilder trb8 = drive.actionBuilder(deliver2)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Grab1,Math.toRadians(90));
        trb8.build();

        TrajectoryActionBuilder trb9 = drive.actionBuilder(Grab1)
                .setTangent(Math.toRadians(248))
                .splineToLinearHeading(Grab2,Math.toRadians(248));
        trb9.build();

        TrajectoryActionBuilder trb10 = drive.actionBuilder(Grab2)
                .setTangent(Math.toRadians(248))
                .splineToLinearHeading(Grab3,Math.toRadians(248));
        trb10.build();

        TrajectoryActionBuilder trb11 = drive.actionBuilder(Grab3)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Score1,Math.toRadians(90));
        trb11.build();

        TrajectoryActionBuilder trb12 = drive.actionBuilder(Score1)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Score2,Math.toRadians(90));
        trb12.build();

        TrajectoryActionBuilder trb13 = drive.actionBuilder(Score2)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Score3,Math.toRadians(90));
        trb13.build();

        TrajectoryActionBuilder trb14 = drive.actionBuilder(Score3)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Score4,Math.toRadians(90));
        trb14.build();

        TrajectoryActionBuilder trb15 = drive.actionBuilder(Score4)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Score5,Math.toRadians(90));
        trb15.build();




        waitForStart();

        if (isStopRequested()) return;

        // Define the actions for arm lifting, joint movement, and intake control
        Action LR_Lift = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                IntakeTilt_Servo1.setPosition(0.3);
                IntakeTilt_Servo2.setPosition(0.7);
                Claw_Servo.setPosition(0.7);
                IntakeFlip_Servo.setPosition(0.3);
                MOTOR_LR.setTargetPosition(2500);
                MOTOR_LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                MOTOR_LR.setPower(-0.5);


                if (MOTOR_LR.isBusy()) {
                    Clip_Servo.setPosition(0.5);
                    return true;
                } else {
                    return false;
                }
            }
        };
        Action LR_Pull = new Action() {
            public boolean run(@NonNull TelemetryPacket packet) {

                MOTOR_LR.setTargetPosition(1450);
                MOTOR_LR.setPower(1);
                MOTOR_LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (MOTOR_LR.isBusy()) {
                    Clip_Servo.setPosition(0.5);
                    return true;
                } else {
                    return false;
                }
            }
        };
        Action LR_Down = new Action() {
            public boolean run(@NonNull TelemetryPacket packet) {

                MOTOR_LR.setTargetPosition(0);
                MOTOR_LR.setPower(0.7);
                MOTOR_LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    return false;
                }
        };
        Action Clip_Drop = new Action() {
            public boolean run(@NonNull TelemetryPacket packet) {

                Clip_Servo.setPosition(0.2);
                return false;
            }
        };

        Action Clip_Grab = new Action() {
            public boolean run(@NonNull TelemetryPacket packet) {

                Clip_Servo.setPosition(0.5);
                return false;
            }
        };

// Build and run the autonomous sequence
        Action trajectoryActionChosen1 = tab1.build();
        Action trajectoryActionChosen2 = trb2.build();
      //  Action trajectoryActionChosen3 = trb3.build();
      //  Action trajectoryActionChosen4 = trb4.build();
     //   Action trajectoryActionChosen5 = trb5.build();
     //   Action trajectoryActionChosen6 = trb6.build();
     //   Action trajectoryActionChosen7 = trb7.build();
        Action trajectoryActionChosen8 = trb8.build();
        Action trajectoryActionChosen9 = trb9.build();
        Action trajectoryActionChosen10 = trb10.build();
        Action trajectoryActionChosen11 = trb11.build();
        Action trajectoryActionChosen12 = trb12.build();
        Action trajectoryActionChosen13 = trb13.build();
        Action trajectoryActionChosen14 = trb14.build();
        Action trajectoryActionChosen15 = trb15.build();


        Actions.runBlocking(
                new ParallelAction(
                        trajectoryActionChosen1,
                        LR_Lift
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen2,
                        LR_Pull)
        );

        Actions.runBlocking(
                new ParallelAction(
                        Clip_Drop
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        trajectoryActionChosen8,
                        LR_Down
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                         trajectoryActionChosen9
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen10,
                        Clip_Grab
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        trajectoryActionChosen11
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        trajectoryActionChosen12
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen13,
                        LR_Lift
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen14,
                        LR_Pull
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        Clip_Drop,
                        trajectoryActionChosen15,
                        LR_Down
                )
        );

}}
