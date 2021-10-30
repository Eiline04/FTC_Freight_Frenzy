package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Wrappers.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.Lifter;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp()
public class Driving extends LinearOpMode {
    ControllerInput controller1, controller2;

    SampleMecanumDrive drive;
    Intake intake;
    Lifter lifter;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        lifter = new Lifter(hardwareMap);

        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);
        waitForStart();

        while (opModeIsActive()) {
            controller1.update();
            controller2.update();
            double leftStickY = controller1.left_stick_y;
            double leftStickX = controller1.left_stick_x;
            double rotation = controller1.right_stick_x;

            drive.setWeightedDrivePower(new Pose2d(leftStickY, leftStickX, rotation));

            //Intake servos
            if (controller2.dpadDownOnce()) {
                intake.lowerIntake();
            }
            if (controller2.dpadUpOnce()) {
                intake.raiseIntake();
                intake.stopIntake();
            }

            //Intake Motor
            if (controller2.AOnce()) {
                intake.startIntake();
            }
            if (controller2.BOnce()) {
                intake.stopIntake();
            }
            if (controller2.XOnce()) {
                intake.stopIntake();
                sleep(200);
                intake.reverseIntake();
            }

            //Lifter
            double lifterUp = controller2.right_trigger;
            double lifterDown = controller2.left_trigger;

            if (lifterUp > 0 && lifterDown == 0) {
                //go up
                lifter.setLifterPower(lifterUp * 0.6);
            } else {
                if (lifterDown > 0 && lifterUp == 0) {
                    //go down
                    lifter.setLifterPower(-lifterDown * 0.4);
                } else {
                    //stop
                    lifter.setLifterPower(0.0);
                }
            }

            //Dumping Box
            if (controller1.dpadRightOnce()) {
                lifter.openBox();
            }
            if (controller1.dpadLeftOnce()) {
                lifter.closeBox();
            }

            telemetry.addData("Lifter Position", lifter.getLifterPosition());
            telemetry.update();

        }

    }

}
