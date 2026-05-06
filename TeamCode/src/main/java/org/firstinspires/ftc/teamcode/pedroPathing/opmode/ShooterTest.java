package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.opmode.DecodeLibrary.intake;

@Config

@TeleOp(name="Shooter Test", group="ABC Opmode")

public class ShooterTest extends OpMode {
    public Servo flap;
    public PIDController controller;
    public DcMotorEx shoot1;
    public DcMotorEx shoot2;
    public DcMotorEx intake;
    public static double test_pos = .5;
    public static double p = 200, i = 1, d = 0;

    public static double f = 8 ;
    public static double speed = 0;
    @Override
    public void init(){
        //initialize();
        controller = new PIDController(p, i, d);
        flap = hardwareMap.get(Servo.class, "flap");
        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
        shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shoot2.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop(){
        shoot1.setVelocity(speed);
        shoot2.setVelocity(speed);
        flap.setPosition(test_pos);
        intake.setPower(gamepad1.right_trigger);
        telemetry.update();
    }

}
