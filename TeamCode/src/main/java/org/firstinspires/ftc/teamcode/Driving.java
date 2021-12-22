package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.ControllerInput;
import org.firstinspires.ftc.teamcode.Utilities.IntakeWatchdog;
import org.firstinspires.ftc.teamcode.Wrappers.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.Lifter;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Wrappers.TeamShippingElement;

@TeleOp()
public class Driving extends LinearOpMode {
    ControllerInput controller1, controller2;

    SampleMecanumDrive drive;
    Intake intake;
    Lifter lifter;
    TeamShippingElement tse;
    IntakeWatchdog intakeWatchdog;

    boolean tseGripState = false; //false = closed
    boolean tsePlacingState = false; //false = closed

    @Override
    public void runOpMode() throws InterruptedException {

        tseGripState = false;
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        tse = new TeamShippingElement(hardwareMap, telemetry);
        intakeWatchdog = new IntakeWatchdog(intake, hardwareMap, telemetry, gamepad1, gamepad2);
        intakeWatchdog.enable();

        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            controller1.update();
            controller2.update();
            lifter.update();
            intakeWatchdog.update();

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
            if (controller2.AOnce() && !gamepad2.start) {
                intake.lowerIntake();
                sleep(100);
                intake.startIntake();
            }
            if (controller2.BOnce() && !gamepad2.start) {
                intake.stopIntake();
            }
            if (controller2.XOnce()) {
                intake.stopIntake();
                sleep(200);
                intake.reverseIntake();
            }

            //Lifter
            if (controller2.rightBumperOnce()) {
                lifter.goToPosition(0, 450);
                lifter.intermediateBoxPosition(300);
                sleep(600);
                lifter.depositMineral();
                lifter.goToPosition(800, 0);
            }

            if (controller2.leftBumperOnce()) {
                lifter.closeBox();
                lifter.goToPosition(0, 0);
            }

            //-----------MANUAL CONTROL---------------
//            double lifterUp = controller2.right_trigger;
//            double lifterDown = controller2.left_trigger;
//
//            if (lifterUp > 0 && lifterDown == 0) {
//                //go up
//                lifter.setLifterPower(lifterUp * 0.2);
//            } else {
//                if (lifterDown > 0 && lifterUp == 0) {
//                    //go down
//                    lifter.setLifterPower(-lifterDown * 0.2);
//                } else {
//                    //stop
//                    lifter.setLifterPower(0.0);
//                }
//            }

            //Dumping Box
            /*if (controller1.dpadRightOnce()) {
                lifter.depositMineral();
            }*/

            if (controller1.dpadLeftOnce()) {
                lifter.closeBox();
            }

            //TSE
            if (controller1.dpadUpOnce()) {
                tsePlacingState = !tsePlacingState;
                tse.tsePlacingPos(tsePlacingState);
            }

            if (controller1.AOnce()) {
                tseGripState = !tseGripState;
                tse.tseGrip(tseGripState);
            }

            if (controller1.XOnce()) {
                tse.lowerTSEArm();
            }

            if (controller1.YOnce()) {
                tse.initTSEArm();
            }

            if (controller1.BOnce()) {
                tse.raiseTSEArm();
            }
        }
    }

}