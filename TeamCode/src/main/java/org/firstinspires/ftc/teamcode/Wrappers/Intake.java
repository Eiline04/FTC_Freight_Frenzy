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

    public void reverseIntake() {
        direction *= -1;
    }

    public void setIntakePosition(double pos) {
        leftServo.setPosition(pos);
        rightServo.setPosition(1 - pos);
    }

    public void raiseIntake() {
        setIntakePosition(0.35);
    }

    public void raiseIntake_Thread(long wait) {
        IntakeServoThread intakeServoThread = new IntakeServoThread(wait, 0.35);
        Thread thread = new Thread(intakeServoThread);
        thread.start();
    }

    public void lowerIntake() {
        setIntakePosition(0.0);
    }

    public void lowerIntake_Thread(long wait) {
        IntakeServoThread intakeServoThread = new IntakeServoThread(wait, 0.0);
        Thread thread = new Thread(intakeServoThread);
        thread.start();
    }

    class IntakeServoThread implements Runnable {
        long wait;
        double position;

        public IntakeServoThread(long wait, double position) {
            this.wait = wait;
            this.position = position;
        }

        @Override
        public void run() {
            if (running) return; //if another thread is running don't use this one

            running = true;

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            setIntakePosition(position);

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
    }
}
