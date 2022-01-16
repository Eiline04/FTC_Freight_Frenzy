package org.firstinspires.ftc.teamcode.Wrappers;

import android.os.SystemClock;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lifter {
    private DcMotorEx lifter;  //470 ticks max

    private PIDFCoefficients pidfCoefficients;
    public static volatile int currentPosition = 0;

    public static volatile boolean box_running = false;

    private Servo dumpingBox;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public LifterThread lifterThread;

    public Lifter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        lifter = hardwareMap.get(DcMotorEx.class, "lifter");
        dumpingBox = hardwareMap.get(Servo.class, "dumpingBox");

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setDirection(DcMotorSimple.Direction.REVERSE);

        lifterThread = new LifterThread();
        Thread lifterRunnable = new Thread(lifterThread);

//        MotorConfigurationType motorConfigurationType = lifter.getMotorType().clone();
//        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//        lifter.setMotorType(motorConfigurationType);

        closeBox();
        lifterRunnable.start();

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(12, 0, 0, 0);
        setPIDFCoefficients(pidfCoefficients);
    }

    public PIDFCoefficients getPIDFCoefficients() {
        return pidfCoefficients;
    }

    public void setPIDFCoefficients(PIDFCoefficients pidfs) {
        this.lifter.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfs);
        pidfCoefficients = pidfs;
    }

    public void update() {
        currentPosition = lifter.getCurrentPosition();
    }

    public void setLifterPower(double power) {
        lifter.setPower(power);
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

    public void openBox(long wait) {
        new Thread(() -> {
            if (box_running) return; //if another thread is running don't use this one
            box_running = true;
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            openBox();
            box_running = false;
        }).start();
    }

    public void closeBox(long wait) {
        new Thread(() -> {
            if (box_running) return; //if another thread is running don't use this one
            box_running = true;
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            closeBox();
            box_running = false;
        }).start();
    }

    public void intermediateBoxPosition() {
        dumpingBox.setPosition(0.5);
    }

    public void intermediateBoxPosition(long wait) {
        new Thread(() -> {
            if (box_running) return; //if another thread is running don't use this one
            box_running = true;
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            intermediateBoxPosition();
            box_running = false;
        }).start();
    }

    public void depositMineral() {
        openBox();
        closeBox(500);
    }

    public void goToPosition(long waitFor, int targetPosition) {
        if (waitFor == 0) {
            lifterThread.setTicks(targetPosition);
            return;
        }
        new Thread(() -> {
            try {
                Thread.sleep(waitFor);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            lifterThread.setTicks(targetPosition);
        }).start();
    }

    class LifterThread implements Runnable {
        public volatile boolean kill = false;
        private volatile int currentTicks = -1;
        private volatile int lastTicks = -1;

        //public double maxVel = 2000;

        @Override
        public void run() {
            while (!kill) {
                if (currentTicks == -1) continue;

                lifter.setTargetPosition(currentTicks);
                lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                int direction = currentTicks > lastTicks ? 1 : -1;
                telemetry.update();

                if (direction == -1) lifter.setVelocity(-900);
                else lifter.setVelocity(2500);

                lastTicks = currentTicks;

                while (!Thread.currentThread().isInterrupted()) {
                    //stay here until a new value comes in
                    if (currentTicks != lastTicks) break;
                }
                lifter.setVelocity(0.0);
                lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        public void setTicks(int ticks) {
            if (Math.abs(currentTicks - ticks) < 50) return; //basically already there
            currentTicks = ticks;
        }
    }
}
