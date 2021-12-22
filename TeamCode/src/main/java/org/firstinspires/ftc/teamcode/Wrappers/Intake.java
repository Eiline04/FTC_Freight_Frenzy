package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    public DcMotorEx intake;
    private int direction = 1;
    public static volatile boolean running = false;

    public Servo leftServo;
    public Servo rightServo;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftServo = hardwareMap.get(Servo.class, "intakeLeftServo");
        rightServo = hardwareMap.get(Servo.class, "intakeRightServo");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0.0);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        raiseIntake();
    }

    public void startIntake() {
        intake.setPower(0.5 * direction);
    }

    public void stopIntake() {
        intake.setPower(0.0);
    }

    public void stopIntake(long wait) {
        new Thread(() -> {
            if(Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            intake.setPower(0.0);
        }).start();
    }

    public void reverseIntake() {
        direction *= -1;
    }

    public void setIntakePosition(double pos) {
        leftServo.setPosition(pos);
        rightServo.setPosition(1 - pos);
    }

    public void setIntakePosition(long wait, double pos) {
        new Thread(() -> {
            if (running) return; //if another thread is running don't use this one
            running = true;
            if(Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            setIntakePosition(pos);
            running = false;
        }).start();
    }

    public void raiseIntake() {
        setIntakePosition(0.35);
    }

    public void raiseIntake(long wait) {
        new Thread(() -> {
            if (running) return; //if another thread is running don't use this one
            running = true;
            if(Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            raiseIntake();
            running = false;
        }).start();
    }

    public void lowerIntake() {
        setIntakePosition(0.0);
    }

    public void lowerIntake(long wait) {
        new Thread(() -> {
            if (running) return; //if another thread is running don't use this one
            running = true;
            if(Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            lowerIntake();
            running = false;
        }).start();
    }
}
