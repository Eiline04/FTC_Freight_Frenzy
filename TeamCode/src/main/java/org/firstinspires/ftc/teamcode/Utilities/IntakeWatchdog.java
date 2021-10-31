package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Wrappers.Intake;

@Config
public class IntakeWatchdog {
    private Rev2mDistanceSensor distanceSensor;
    private double rawDistance;
    private boolean enabled = false;
    private Intake intake;

    public static double DISTANCE_THRESHOLD = 3;
    public ElapsedTime timer;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;


    public IntakeWatchdog(Intake intake, HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.intake = intake;

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

        timer = new ElapsedTime();
    }

    public void update() {
        if (!enabled) return;
        rawDistance = distanceSensor.getDistance(DistanceUnit.CM);
        if(timer.seconds() < 4)  return;

        if(rawDistance < DISTANCE_THRESHOLD) {
            intake.raiseIntake();
            intake.stopIntake();
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
