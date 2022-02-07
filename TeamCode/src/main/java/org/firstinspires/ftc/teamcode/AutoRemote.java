package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utilities.IntakeWatchdog;
import org.firstinspires.ftc.teamcode.Wrappers.DuckMechanism;
import org.firstinspires.ftc.teamcode.Wrappers.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.Lifter;
import org.firstinspires.ftc.teamcode.Wrappers.MeasuringTapeTurret;

import java.util.logging.Level;

@Autonomous
public class AutoRemote extends LinearOpMode {
    MecanumDriveImpl drive;
    Intake intake;
    Lifter lifter;
    IntakeWatchdog intakeWatchdog;
    MeasuringTapeTurret turret;
    DuckMechanism duckMechanism;

    Pose2d startPose = new Pose2d(-40.085, -63.54, radians(270.0));
    Pose2d shippingHubPose = new Pose2d(-9.0, -46.5, radians(265.0));

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDriveImpl(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        turret = new MeasuringTapeTurret(hardwareMap);
        duckMechanism = new DuckMechanism(hardwareMap);
        intakeWatchdog = new IntakeWatchdog(intake, hardwareMap, telemetry, gamepad1, gamepad2);
        intakeWatchdog.enable();

        Thread updater = new Thread(new Updater());

        waitForStart();
        //detect go brr

        drive.setPoseEstimate(startPose);
        updater.start(); //start calling update for watchdog and lifter

//        Lifter.LEVEL result = Lifter.LEVEL.THIRD;
//        TrajectorySequence sequence = null;
//        if (result == Lifter.LEVEL.THIRD) {
//            sequence = buildLevelThree();
//        }
        drive.followTrajectorySequence(levelThree());
    }

    TrajectorySequence levelThree() {
        return drive.trajectorySequenceBuilder(startPose)

                //DUCK
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> duckMechanism.startSpin())
                .lineToLinearHeading(new Pose2d(-55.23, -59.0, radians(270.0)))
                .waitSeconds(1)
                .addDisplacementMarker(() -> duckMechanism.stopSpin())

                //PRELOAD
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> lifter.goToPosTicks(500,1))
                .lineToLinearHeading(shippingHubPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lifter.dumpingBox.setPosition(0.8))
                .waitSeconds(0.7)
                .addDisplacementMarker(() -> {
                    lifter.closeBox();
                    //lifter.goToPosition(0, Lifter.LEVEL.DOWN);
                    lifter.goToPosTicks(0,-1);
                })
//
//                //START OF CYCLE
//                .setVelConstraint(new TranslationalVelocityConstraint(20.0))
//                .lineToLinearHeading(new Pose2d(6.84, -73.0, radians(0.0)))
//                .addDisplacementMarker(() -> {
//                    intake.lowerIntake();
//                    intake.startIntake();
//                })
//                .setVelConstraint(new TranslationalVelocityConstraint(25.0))
//                .lineToLinearHeading(new Pose2d(50.0, -71.0, radians(0.0)))
//                .waitSeconds(1.0)
//
//                //go back now
//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(23.3, -72.0, radians(0.0)), radians(180.0))
//                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> lifter.goToPosition(0, Lifter.LEVEL.THIRD))
//                .resetVelConstraint()
//                .splineToSplineHeading(new Pose2d(-6.0, -46.5, radians(280.0)), radians(80.0))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> lifter.dumpingBox.setPosition(0.8))
//                .waitSeconds(0.8)
//                .addDisplacementMarker(() -> {
//                    lifter.closeBox();
//                    lifter.goToPosition(0, Lifter.LEVEL.DOWN);
//                })
//                .setReversed(false)
//                .resetVelConstraint()
//
//                //END OF CYCLE
//
//                //START CYCLE AGAIN
//                .setVelConstraint(new TranslationalVelocityConstraint(20.0))
//                .lineToLinearHeading(new Pose2d(6.84, -73.0, radians(0.0)))
//                .addDisplacementMarker(() -> {
//                    intake.lowerIntake();
//                    intake.startIntake();
//                })
//                .setVelConstraint(new TranslationalVelocityConstraint(25.0))
//                .lineToLinearHeading(new Pose2d(52.0, -70.5, radians(20.0)))
//                .waitSeconds(1.0)
//
//                //go back now
//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(23.3, -72.0, radians(0.0)), radians(180.0))
//                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> lifter.goToPosition(0, Lifter.LEVEL.THIRD))
//                .resetVelConstraint()
//                .splineToSplineHeading(new Pose2d(-6.0, -46.5, radians(280.0)), radians(80.0))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> lifter.dumpingBox.setPosition(0.8))
//                .waitSeconds(0.8)
//                .addDisplacementMarker(() -> {
//                    lifter.closeBox();
//                    lifter.goToPosition(0, Lifter.LEVEL.DOWN);
//                })
//                .setReversed(false)
//                .resetVelConstraint()

                //PARK
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lifter.dumpingBox.setPosition(0.8))
                .waitSeconds(0.8)
                .addDisplacementMarker(() -> {
                    lifter.closeBox();
                    lifter.goToPosTicks(0,-1);
                })
                .setReversed(false)
                .resetVelConstraint()
                .setVelConstraint(new TranslationalVelocityConstraint(30.0))
                .lineToLinearHeading(new Pose2d(6.84, -73.0, radians(0.0)))
                .setVelConstraint(new TranslationalVelocityConstraint(35.0))
                .lineToLinearHeading(new Pose2d(52.0, -70.5, radians(20.0)))
                .build();
    }

    static double radians(double deg) {
        return Math.toRadians(deg);
    }

    class Updater implements Runnable {
        @Override
        public void run() {
            while (opModeIsActive()) {
                lifter.update();
                intakeWatchdog.update();
            }
        }
    }

    public void autoLifterShit(){
        turret.setBasePos(0.98);
        sleep(200);
        lifter.goThird();
        lifter.intermediateBoxPosition(200);
        lifter.depositMineral(500);

        lifter.closeBox();
        sleep(1200);
        lifter.goDown();
    }
}