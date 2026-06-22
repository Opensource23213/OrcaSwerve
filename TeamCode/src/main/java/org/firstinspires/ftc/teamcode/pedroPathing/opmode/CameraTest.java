package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;
import static java.lang.Math.tan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@TeleOp(name="Camera Test", group="ABC Opmode")

public class CameraTest extends DecodeLibrary {
    public static double test_pos = .5;
    public static String name = "blocker";
    public Servo Servo;
    public double x = 0;
    public double balls = 0;
    public double[] python = new double[0];
    @Override
    public void init(){
        color = 0;
        cameraCode.init();
    }
    @Override
    public void loop(){
        telemetry.addData("hi", cameraCode.limelight.isConnected());
        telemetry.update();
    }

}
