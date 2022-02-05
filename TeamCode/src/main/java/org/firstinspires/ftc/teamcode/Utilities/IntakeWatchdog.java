package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Wrappers.Intake;

@Config
public class IntakeWatchdog {
    private Rev2mDistanceSensor distanceSensor;
    private volatile double rawDistance;
    private volatile boolean enabled = false;
    private Intake intake;

    public static double DISTANCE_THRESHOLD = 4;
    public static long WATCHDOG_DELAY = 2;
    public ElapsedTime timer;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;


    public IntakeWatchdog(Intake intake, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.intake = intake;

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

        timer = new ElapsedTime();
    }

    public void update() {
        if (!enabled) return;
        rawDistance = distanceSensor.getDistance(DistanceUnit.CM);
        if (timer.seconds() < WATCHDOG_DELAY) return;

        if (rawDistance < DISTANCE_THRESHOLD && rawDistance != 0.0) {
            intake.raiseIntake();
            intake.stopIntake(200);
            timer.reset();
        }
    }

    public double getDistance() {
        return rawDistance;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

}
