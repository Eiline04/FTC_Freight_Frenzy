package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp (name = "Driving", group = "teleOp")
public class Diving extends LinearOpMode {

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            double leftStickY = gamepad1.left_stick_y;
            double leftStickX = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;

            telemetry.addData("y = ", leftStickY);
            telemetry.addData("X = ", leftStickX);
            telemetry.addData("rotation = ", rotation);
            telemetry.update();

            drive.setWeightedDrivePower(new Pose2d(leftStickY, leftStickX,rotation ));
        }

    }

}
