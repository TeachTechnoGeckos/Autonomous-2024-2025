package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "autoRoadRunner", preselectTeleOp = "Starterbot2025TeleOpBlocksTwoGamepad (1) ")
public class autoRoadRunner extends LinearOpMode {

    private DcMotor arm;
    private DcMotor wrist;
    private DcMotor rightBack;
    private Servo claw;
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightFront;

    int targetArm;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode()
    {
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        claw = hardwareMap.get(Servo.class, "claw");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        // Put initialization blocks here.
        //initializes arm to encoder settings, wheel directions, and claw position.
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        claw.setPosition(1.5);

        Pose2d initialPose = new Pose2d(-9, 63, 3*Math.PI/2);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Action TrajectoryAction11 = drive.actionBuilder(drive.pose)
                                            // move forward to hang preloaded specimen
                                            .strafeToLinearHeading(new Vector2d(12,45),3*Math.PI/2)
                                            .strafeToLinearHeading(new Vector2d(12,32), 3* Math.PI/2)
                                            .build();


        Action waitToClip = drive.actionBuilder(drive.pose)
                                     // move forward to hang preloaded specimen
                                     .waitSeconds(1)
                                     .build();
      Action TrajectoryAction12 = drive.actionBuilder(drive.pose)
                                            // move forward to hang preloaded specimen
                                            .strafeToLinearHeading(new Vector2d(12,55), 3*Math.PI/2)
                                           .build();
        Action waitToClip2 = drive.actionBuilder(drive.pose)
                                    // move forward to hang preloaded specimen
                                    .waitSeconds(2)
                                    .build();
        Action pushIntoObservation = drive.actionBuilder(drive.pose)
                                             //.setTangent(3*Math.PI/4)
                                             //TODO Crab over away from submersible
                                             //.splineToConstantHeading(new Vector2d(-15, 45),3*Math.PI/4)
                                             //.splineToConstantHeading(new Vector2d(-36,55),3*Math.PI/2)
                                             //TODO Get beside 1 sample
                                             /*.splineToConstantHeading(new Vector2d(-33,22),3*Math.PI/2)
                                             //TODO Get behind 1 sample
                                             .splineToConstantHeading(new Vector2d(-50,22),Math.PI/2)
                                             //TODO push 1 sample into human player area
                                             .splineToConstantHeading(new Vector2d(-50,56),3*Math.PI/2)

                                             //TODO Back up to get 2 sample
                                             .splineToSplineHeading(new Pose2d(-46,20,Math.PI/2),Math.PI)
                                             //TODO Get behind 2 sample
                                             .splineToSplineHeading(new Pose2d(-62,20,5*Math.PI/8),Math.PI/2)
                                             // TODO Push second sample to the wall
                                             .splineToSplineHeading(new Pose2d(-62,54, 5*Math.PI/8),Math.PI/2)
                                             // TODO line up to grab second specimen
                                             .splineToLinearHeading(new Pose2d(-49,65,Math.PI/2),Math.PI/2)*/
                                             .build();

        waitForStart();
        if(isStopRequested()) return;


            moveArm("upToClip");
            Actions.runBlocking(new SequentialAction(TrajectoryAction11));
            moveArm("clip");
            Actions.runBlocking(new SequentialAction(waitToClip));
            openClaw();
            Actions.runBlocking(new SequentialAction(TrajectoryAction12));









    }

    private void moveArm(String position)
    {
        if (position.equals("intake"))
        {
            targetArm = 450;
        }
        else if (position.equals("clip"))
        {
            targetArm = 1330;
        }
        else
        {
            if (position.equals("init"))
            {
                targetArm = 300;
            }
            else if (position.equals("wallGrab"))
            {
                targetArm = 820;
            }
            else
            {
                if (position.equals("unhook"))
                {
                    targetArm = 1300;
                }
                else if (position.equals("lowBasket"))
                {
                    targetArm = 1900;
                }
                else
                {
                    if (position.equals("upToClip"))
                    {
                        targetArm = 1960;
                    }
                }
            }
        }
        arm.setTargetPosition(targetArm);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
    }

    private void openClaw()
    {
        claw.setPosition(0.57);
    }

}
