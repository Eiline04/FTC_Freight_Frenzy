package org.firstinspires.ftc.teamcode.Wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lifter {
    public DcMotorEx lifterMotor;  //470 ticks max

    private PIDFCoefficients pidfCoefficients;
    public static volatile int currentPosition = 0;

    public Servo dumpingBox;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public LifterThread lifterThread;

    public enum LEVEL {
        DOWN(-10), FIRST(150), SECOND(300), THIRD(500);//down 20; third 450
        public int ticks;

        LEVEL(int ticks) {
            this.ticks = ticks;
        }
    }

    public Lifter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        lifterMotor = hardwareMap.get(DcMotorEx.class, "lifter");
        dumpingBox = hardwareMap.get(Servo.class, "dumpingBox");

        lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

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
        this.lifterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfs);
        pidfCoefficients = pidfs;
    }

    public void update() {
        currentPosition = lifterMotor.getCurrentPosition();
    }

    public void setLifterPower(double power) {
        lifterMotor.setPower(power);
    }

    public int getLifterPosition() {
        return currentPosition;
    }

    public void openBox() {
        dumpingBox.setPosition(0.8);
    }

    public void closeBox() {
        dumpingBox.setPosition(0.1);
    }

    public void openBox(long wait) {
        new Thread(() -> {
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            openBox();
        }).start();
    }

    public void closeBox(long wait) {
        new Thread(() -> {
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            closeBox();
        }).start();
    }


    public void intermediateBoxPosition() {
        dumpingBox.setPosition(0.5);
    }

    public void intermediateBoxPosition(long wait) {
        new Thread(() -> {
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            intermediateBoxPosition();
        }).start();
    }

    public void depositMineral(long wait) {
        new Thread(() -> {
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            openBox();
            closeBox(500);

        }).start();

    }

    public void goToPosition(long waitFor, LEVEL level) {
        if (waitFor == 0) {
            lifterThread.setTicks(level.ticks);
            return;
        }
        new Thread(() -> {
            try {
                Thread.sleep(waitFor);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            lifterThread.setTicks(level.ticks);
        }).start();
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

    public void goDown(){
       lifterMotor.setTargetPosition(LEVEL.DOWN.ticks);
       lifterMotor.setVelocity(-900);
        while(lifterMotor.getCurrentPosition() > 50){

        }
        lifterMotor.setVelocity(0);
    }
    public void goThird(){
        lifterMotor.setTargetPosition(500);
        lifterMotor.setVelocity(1000);
        while(lifterMotor.getCurrentPosition() < 450){

        }
        lifterMotor.setVelocity(0);
    }

    public void goToPosTicks(int ticks, int direction){
        lifterMotor.setTargetPosition(ticks);
        lifterMotor.setVelocity(1000 * direction);

        if(direction<0){
            while(lifterMotor.getCurrentPosition() > ticks + 50){

            }
        lifterMotor.setVelocity(0);
        }
        else {
            while(lifterMotor.getCurrentPosition() < ticks - 50){

            }
            lifterMotor.setVelocity(0);
            
        }


    }

    class LifterThread implements Runnable {
        public volatile boolean kill = false;
        private volatile int currentTicks = -1;
        private volatile int lastTicks = -1;

        @Override
        public void run() {
            while (!kill) { //paranoia
                if (currentTicks == -1) continue;

                lifterMotor.setTargetPosition(currentTicks);
                lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                int direction = currentTicks > lastTicks ? 1 : -1;
                telemetry.update();

                if (direction == -1) lifterMotor.setVelocity(-700);//-900
                else lifterMotor.setVelocity(1500);

                lastTicks = currentTicks;

                if (lastTicks < 50) {
                    lifterMotor.isBusy(); //stupid bug
                    //we are going completely down so don't bother with pid after the fact
                    ElapsedTime timer = new ElapsedTime();
                    timer.reset();
                    while (lifterMotor.isBusy()) {
                        if(timer.seconds() > 1.2) break;
                    }
                    lifterMotor.setVelocity(0.0);

                    while (currentTicks == lastTicks) {
                        //stay here until a new value comes in
                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                } else {
                    while (currentTicks == lastTicks) {
                        //stay here until a new value comes in
                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                    lifterMotor.setVelocity(0.0);
                }
            }
        }

        public void setTicks(int ticks) {
            if (Math.abs(currentTicks - ticks) < 50) return; //basically already there
            currentTicks = ticks;
        }
    }
}
