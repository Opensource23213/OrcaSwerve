package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.OrcaAuto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

@TeleOp(name="TestAuto", group="ABC Opmode")
public class TestCode extends OpMode {
    public OrcaAuto drive = new OrcaAuto();
    public double color = 1;
    public List<LynxModule> hubs = null;
    public ArrayList<Pose2D> path = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 40, -40,AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.INCH, 60, 0,AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.INCH, 0, 0,AngleUnit.DEGREES, 90)));
    @Override
    public void init() {
        drive.SwerveOrcaInit(hardwareMap, gamepad1, color);
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        if(gamepad1.a){
            drive.currentPath = new ArrayList<>(path);
            drive.auto_drive = true;
        }
        telemetry.addData("angle", Math.toDegrees(drive.real_angle));
        telemetry.addData("x", drive.pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("y", drive.pinpoint.getPosition().getY(DistanceUnit.INCH));
        telemetry.addData("dis", drive.targetDistance);
        telemetry.update();
        drive.runDrive();
    }
}
