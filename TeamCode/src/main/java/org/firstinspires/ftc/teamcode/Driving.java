package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Utilities.ControllerInput;
import org.firstinspires.ftc.teamcode.Utilities.IntakeWatchdog;
import org.firstinspires.ftc.teamcode.Wrappers.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.Lifter;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.Wrappers.MeasuringTapeTurret;

@TeleOp()
public class Driving extends LinearOpMode {
    ControllerInput controller1, controller2;

    MecanumDriveImpl drive;
    Intake intake;
    Lifter lifter;
    IntakeWatchdog intakeWatchdog;
    MeasuringTapeTurret turret;

    boolean tseGripState = false; //false = closed
    boolean tsePlacingState = false; //false = closed

    DcMotorEx encoderLeft, encoderRight;

    @Override
    public void runOpMode() throws InterruptedException {

        tseGripState = false;
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDriveImpl(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        turret = new MeasuringTapeTurret(hardwareMap);
        intakeWatchdog = new IntakeWatchdog(intake, hardwareMap, telemetry, gamepad1, gamepad2);
        intakeWatchdog.enable();

        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

//        encoderLeft = hardwareMap.get(DcMotorEx.class,"BR");
//        encoderRight = hardwareMap.get(DcMotorEx.class, "FR");
//        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        encoderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        encoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            controller1.update();
            controller2.update();
            lifter.update();
            intakeWatchdog.update();

//            telemetry.addData("Encoder Left", encoderLeft.getCurrentPosition());
//            telemetry.addData("Encoder Right", encoderRight.getCurrentPosition());
//            telemetry.update();

            double leftStickY = controller1.left_stick_y;
            double leftStickX = controller1.left_stick_x;
            double rotation = controller1.right_stick_x;

            drive.setWeightedDrivePower(new Pose2d(leftStickY, leftStickX, rotation));

            if (controller1.rightBumper()) {
                turret.startExtend();
            } else if (controller1.leftBumper()) {
                turret.startRetract();
            } else turret.stop();

            if(controller1.AOnce()) {
                turret.setAngleServoPos(0.5);
            }
            if(controller1.BOnce()) {
                turret.setAngleServoPos(0.2);
            }

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
        }
    }

}