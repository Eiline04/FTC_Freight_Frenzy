package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lifter {
    private DcMotorEx lifter;  //470 ticks max

    private PIDFCoefficients pidfCoefficients;
    public static volatile int currentPosition = 0;

    public static volatile boolean running = false;

    private Servo dumpingBox;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Lifter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        lifter = hardwareMap.get(DcMotorEx.class, "lifter");
        dumpingBox = hardwareMap.get(Servo.class, "dumpingBox");

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setDirection(DcMotorSimple.Direction.REVERSE);
        lifter.setPower(0.0);

        MotorConfigurationType motorConfigurationType = lifter.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        lifter.setMotorType(motorConfigurationType);

        pidfCoefficients = lifter.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        setPIDFCoefficients(new PIDFCoefficients(10, 0, 0, 0));

        closeBox();
    }


    public void update() {
        currentPosition = lifter.getCurrentPosition();
    }

    public void setLifterPower(double power) {
        lifter.setPower(power);
    }

    public void setPIDFCoefficients(PIDFCoefficients pidfCoefficients) {
        this.pidfCoefficients = pidfCoefficients;
        lifter.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
    }

    public PIDFCoefficients getPidfCoefficients() {
        return pidfCoefficients;
    }

    public int getLifterPosition() {
        return currentPosition;
    }

    public void openBox() {
        dumpingBox.setPosition(0.8);
    }

    public void closeBox() {
        dumpingBox.setPosition(0.0);
    }

    public void openBox_Thread(long wait) {
        DumpingBoxThread dumpingBoxThread = new DumpingBoxThread(wait, 0.8);
        Thread thread = new Thread(dumpingBoxThread);
        thread.start();
    }

    public void closeBox_Thread(long wait) {
        DumpingBoxThread dumpingBoxThread = new DumpingBoxThread(wait, 0.0);
        Thread thread = new Thread(dumpingBoxThread);
        thread.start();
    }

    public void intermediateBoxPosition() {
        dumpingBox.setPosition(0.5);
    }

    public void intermediateBoxPosition_Thread(long wait) {
        DumpingBoxThread dumpingBoxThread = new DumpingBoxThread(wait, 0.5);
        Thread thread = new Thread(dumpingBoxThread);
        thread.start();
    }

    public void depositMineral() {
        openBox();
        closeBox_Thread(500);
        goToPosition(1000, 0);
    }

    public void goToPosition(long waitFor, int targetPosition) {
        LifterThread lifterThread = new LifterThread(waitFor, targetPosition);
        Thread thread = new Thread(lifterThread);
        thread.start();
    }

    class LifterThread implements Runnable {
        long wait;
        int targetPosition;

        public LifterThread(long wait_ms, int targetPosition) {
            this.wait = wait_ms;
            this.targetPosition = targetPosition;
        }

        @Override
        public void run() {
            if (running) {
                //if another thread is running don't use this one
                telemetry.addLine("Already running");
                telemetry.update();
            }

            running = true;

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            //We might already be at the target position
            if (Math.abs(currentPosition - targetPosition) < 10) return;

            //go to targetPosition
            int direction = currentPosition > targetPosition ? -1 : 1;
            lifter.setTargetPosition(targetPosition);
            lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lifter.isBusy(); //this is so stupid

            if (direction == -1) lifter.setPower(-0.4);
            else lifter.setPower(0.8);
            while (lifter.isBusy() && !Thread.currentThread().isInterrupted()) {
                //loop
            }
            lifter.setPower(0.0);
            lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            running = false;
        }
    }


    class DumpingBoxThread implements Runnable {
        long wait;
        double pos;

        public DumpingBoxThread(long wait_ms, double pos) {
            this.wait = wait_ms;
            this.pos = pos;
        }

        @Override
        public void run() {
            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            dumpingBox.setPosition(pos);
            running = false;
        }
    }
}
