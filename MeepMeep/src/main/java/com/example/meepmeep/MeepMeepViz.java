package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepViz {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(-40.085, -63.54, Math.toRadians(270.0));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.0)
                .setDimensions(12.59, 16.14)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)

                                .lineToLinearHeading(new Pose2d(-55.23, -61.0, radians(270.0)))
                                .waitSeconds(0.7)

                                .lineToLinearHeading(new Pose2d(-6.0, -46.5, radians(265.0)))
                                .waitSeconds(1.0)

                                //-START OF CYCLE
                                .setVelConstraint(new TranslationalVelocityConstraint(15.0))
                                .lineToLinearHeading(new Pose2d(6.84, -73.0, radians(0.0)))
                                .setVelConstraint(new TranslationalVelocityConstraint(20.0))
                                .splineToLinearHeading(new Pose2d(50.0, -70.5, radians(0.0)), 0.0)
                                .waitSeconds(2.0)

                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(23.3, -72.0, radians(0.0)), radians(180.0))
                                //.splineToSplineHeading(new Pose2d(9.0, -72.0, radians(355.0)), radians(175.0))
                                .resetVelConstraint()
                                .splineToSplineHeading(new Pose2d(-6.0, -46.5, radians(280.0)), radians(80.0))
                                .waitSeconds(0.5)
                                .setReversed(false)
                                .resetVelConstraint()
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