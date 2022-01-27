package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
                                        .lineToLinearHeading(new Pose2d(-16.0, -40.0, radians(260.0)))
                                        .waitSeconds(0.5)
                                        .lineToLinearHeading(new Pose2d(-22.0, -51.0, radians(250.0)))
                                        .lineToLinearHeading(new Pose2d(-55.0, -62.0, radians(270.0)))
                                        .waitSeconds(2.0)
                                        .lineToLinearHeading(new Pose2d(-4.0, -54.0, radians(-35.0)))
                                        .splineToSplineHeading(new Pose2d(50.0, -64.0, radians(0.0)), 0.0)
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(-1, -61.0, radians(350.0)), 10.0)
                                        //.splineToSplineHeading(new Pose2d(12.0, -63.0, radians(350.0)), radians(10.0))

//                                .lineToLinearHeading(new Pose2d(-7.0, -40.0, radians(285.0)))
//                                .waitSeconds(0.5)
//
//                                .splineToSplineHeading(new Pose2d(12.0, -63.0, radians(350.0)), radians(350.0))
//                                .splineToSplineHeading(new Pose2d(50.0, -64.0, radians(0.0)), 0.0)
//                                .waitSeconds(1.0)
//
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(12.0, -63.0, radians(350.0)), radians(0.0))
//                                //.splineToSplineHeading(new Pose2d(-7.0, -40.0, radians(285.0)), radians(75.0)).setReversed(true)
//                                .waitSeconds(1.0)

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