package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@TeleOp(name="ShooterTest", group="ABC Opmode")
@Config
public class ShooterTest extends DecodeLibrary{
    public static double test_speed = 0;
    public static double spindexer_speed = .8;
    public static PIDFCoefficients pidf = new PIDFCoefficients(100, .5, .001, 7);
    public void init() {
        shooter.initialize();
        intake_system.init();
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
        if(shooter.pidf != pidf){
            shooter.pidf = new PIDFCoefficients(pidf);
            shooter.shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooter.pidf);
            shooter.shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooter.pidf);
        }
        if(gamepad1.left_trigger > .4){
            intake_system.spindexer.setPower(-spindexer_speed);
            intake_system.intake.setPower(-1);
            intake_system.flippy_pos = flippy_up;
            intake_system.blocking = false;
        }else if(gamepad1.right_trigger > .4){
            intake_system.spindexer.setPower(1);
            intake_system.intake.setPower(-1);
            intake_system.flippy_pos = flippy_down;
            intake_system.blocking = true;
        }else{
            intake_system.spindexer.setPower(0);
            intake_system.intake.setPower(0);
            intake_system.flippy_pos = flippy_up;
            intake_system.blocking = true;
        }
        if(intake_system.blocking){
            intake_system.blocker.setPosition(intake_system.block);
        }else{
            intake_system.blocker.setPosition(intake_system.unblock);
        }
        intake_system.flippy.setPosition(intake_system.flippy_pos);
        shooter.shoot1.setVelocity(test_speed);
        shooter.shoot2.setVelocity(test_speed);
        shooter.flap.setPosition(.02);
        telemetry.addData("Shooter Speed", shooter.shoot1.getVelocity());
        telemetry.update();
    }
}
