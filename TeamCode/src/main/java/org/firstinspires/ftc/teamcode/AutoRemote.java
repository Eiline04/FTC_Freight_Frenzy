package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utilities.IntakeWatchdog;
import org.firstinspires.ftc.teamcode.Wrappers.DuckMechanism;
import org.firstinspires.ftc.teamcode.Wrappers.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.Lifter;
import org.firstinspires.ftc.teamcode.Wrappers.MeasuringTapeTurret;

public class AutoRemote extends LinearOpMode {
    MecanumDriveImpl drive;
    Intake intake;
    Lifter lifter;
    IntakeWatchdog intakeWatchdog;
    MeasuringTapeTurret turret;
    DuckMechanism duckMechanism;

    Pose2d startPose = new Pose2d(-36.0, -65.0, radians(270.0));
    Pose2d shippingHubPose = new Pose2d(-9.0, -40.0, radians(280.0));

    TrajectorySequence cycle;
    TrajectorySequence firstTasks;
    double lowSpeed = 30.0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDriveImpl(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        turret = new MeasuringTapeTurret(hardwareMap);
        duckMechanism = new DuckMechanism(hardwareMap);
        intakeWatchdog = new IntakeWatchdog(intake, hardwareMap, telemetry, gamepad1, gamepad2);
        intakeWatchdog.enable();

        buildCycle();

        waitForStart();
        //detect go brr

        drive.setPoseEstimate(startPose);

        Lifter.LEVEL result = Lifter.LEVEL.THIRD;
        buildFirstTasks(result);

        drive.followTrajectorySequence(firstTasks);
        //drive.followTrajectorySequence(cycle);
    }

    //TODO This depends on detection result
    void buildFirstTasks(Lifter.LEVEL target) {
        firstTasks = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> duckMechanism.startSpin())
                .lineToLinearHeading(new Pose2d(-55.0, -63.0, radians(270.0)))
                .waitSeconds(2.0)
                .addDisplacementMarker(() -> duckMechanism.stopSpin())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> lifter.goToPosition(0, target.ticks))
                .lineToLinearHeading(shippingHubPose)
                .addDisplacementMarker(() -> lifter.depositMineral())
                .waitSeconds(0.8)
                .build();
    }

    void buildCycle() {
        cycle = drive.trajectorySequenceBuilder(shippingHubPose)
                .splineToSplineHeading(new Pose2d(9.0, -64.0, radians(350.0)), 0.0)
                .setVelConstraint(new TranslationalVelocityConstraint(lowSpeed))
                .splineToLinearHeading(new Pose2d(50.0, -65.0, radians(0.0)), 0.0)
                .setReversed(true)
                .waitSeconds(1.2)
                .splineToSplineHeading(new Pose2d(9.0, -64.0, radians(355.0)), radians(185.0))
                .resetVelConstraint()
                .splineToSplineHeading(new Pose2d(-9.0, -40.0, radians(280.0)), radians(80.0))
                .waitSeconds(0.5)
                .build();
    }

    static double radians(double deg) {
        return Math.toRadians(deg);
    }
}
