package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.ControllerInput;
import org.firstinspires.ftc.teamcode.Wrappers.Lifter;

@TeleOp()
public class LifterTest extends LinearOpMode {
    Lifter lifter;
    ControllerInput controller1, controller2;

    @Override
    public void runOpMode() throws InterruptedException {

        lifter = new Lifter(hardwareMap, telemetry);

        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            controller1.update();
            controller2.update();
            lifter.update();

            telemetry.addData("Ticks", lifter.getLifterPosition());
            telemetry.update();

            //Lifter
            if (controller2.rightBumperOnce()) {
                lifter.goToPosition(0, Lifter.LEVEL.THIRD);
                lifter.intermediateBoxPosition(200);
                lifter.depositMineral(500);
                lifter.goToPosition(0, Lifter.LEVEL.DOWN);
            }

            if (controller2.leftBumperOnce()) {
                lifter.closeBox();
                lifter.goToPosition(2000, Lifter.LEVEL.DOWN);
            }

            //-----------MANUAL LIFTER CONTROL---------------
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

        }
    }
}
