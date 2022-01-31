package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepViz {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(-36.0, -65.0, Math.toRadians(270.0));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.0)
                .setDimensions(11.96, 13.22)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {

                                }) //start duck motor

                                .lineToLinearHeading(new Pose2d(-55.0, -63.0, radians(270.0)))
                                .waitSeconds(2.0)
                                .addDisplacementMarker(() -> {

                                }) //stop duck motor

                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {

                                }) //lifter

                                .lineToLinearHeading(new Pose2d(-9.0, -40.0, radians(280.0)))
                                .waitSeconds(0.8)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                }) //lower lifter

                                //-START OF CYCLE
                                .splineToSplineHeading(new Pose2d(9.0, -64.0, radians(350.0)), 0.0)
                                .addDisplacementMarker(() -> {
                                }) //start intake
                                .setVelConstraint(new TranslationalVelocityConstraint(30.0))
                                .splineToLinearHeading(new Pose2d(50.0, -65.0, radians(0.0)), 0.0)
                                .setReversed(true)
                                .waitSeconds(1.2)
                                .splineToSplineHeading(new Pose2d(9.0, -64.0, radians(355.0)), radians(175.0))
                                .resetVelConstraint()
                                .splineToSplineHeading(new Pose2d(-9.0, -40.0, radians(280.0)), radians(80.0))
                                .waitSeconds(0.5)
                                //-----END OF CYCLE----

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    static double radians(double deg) {
        return Math.toRadians(deg);
    }
}