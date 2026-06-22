package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous(name="Partner Close Auto", group="ABC Opmode", preselectTeleOp = "ShortTeleop")
public class PartnerAuto extends DecodeLibrary{
    public double forward = 0.25;
    public ArrayList<Pose2D> first_pick;
    public ArrayList<Pose2D> pick_after_stuff;
    public ArrayList<Pose2D> first_shoot;
    public ArrayList<Pose2D> second_shoot;
    public ArrayList<Pose2D> third_shoot;
    public ArrayList<Pose2D> fourth_shoot;
    public ArrayList<Pose2D> fifth_shoot;
    public ArrayList<Pose2D> second_pick;
    public ArrayList<Pose2D> third_pick;
    public ArrayList<Pose2D> park;
    public ArrayList<Pose2D> gate;
    public double shot = 0;
    public double old_turns = 0;
    public double steps = 0;
    public ElapsedTime gate_open = new ElapsedTime();
    public ElapsedTime shooting_time = new ElapsedTime();
    public double sort_pos = 0;
    public double old_pattern = 2;
    public double old_color = 1;
    public boolean shooting = false;
    public double index_steps = 0;
    public static double first_dump_time = 2200;
    public static double second_dump_time = 2500;
    @Override
    public void init(){
        color = 1;
        teleop = false;
        if(color == 1){
            auto_pose = new Pose2D(DistanceUnit.INCH, 122,-25, AngleUnit.DEGREES, -90);
        }else{
            auto_pose = new Pose2D(DistanceUnit.INCH, 122,25, AngleUnit.DEGREES, 90);
        }
        initialize();
        drive.pinpoint.setPosition(auto_pose);
        if(color == 0) {
            blue_init();
        }else{
            red_init();
        }
        short_match();
        intake_system.flippy_pos = flippy_hold;
    }
    @Override
    public void init_loop(){
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        drive.auto_drive = true;
        drive.stop_and_point = true;
        drive.runDrive();

        turret.zero = true;
        /*cam.look();
        if(cam.tags.contains(21.0)){
            pattern = 1;
        }else if(cam.tags.contains(22.0)){
            pattern = 2;
        }else if(cam.tags.contains(23.0)){
            pattern = 3;
        }*/
        turret.turret_move();
        if(gamepad1.right_bumper) {
            color = 0;
        }else if(gamepad1.left_bumper){
            color = 1;
        }
        if(old_color != color || gamepad1.a) {
            initialize();
            if(color == 1){
                auto_pose = new Pose2D(DistanceUnit.INCH, 122,-25, AngleUnit.DEGREES, -90);
            }else{
                auto_pose = new Pose2D(DistanceUnit.INCH, 122,25, AngleUnit.DEGREES, 90);
            }
            drive.pinpoint.setPosition(auto_pose);
            if (color == 0) {
                blue_init();
                old_color = 0;
            } else {
                red_init();
                old_color = 1;
            }
            drive.currentPath = new ArrayList<>(first_shoot);
            short_match();
        }
        if(drive.pinpoint.getPosition() != auto_pose){
            drive.pinpoint.setPosition(auto_pose);
        }
        telemetry.addData("pattern", pattern);
        telemetry.addData("color", color);
        telemetry.addData("X", auto_pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y", auto_pose.getY(DistanceUnit.INCH));
        telemetry.update();
    }

    @Override
    public void loop() {
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        drive.runDrive();
        turret.turret_move();
        shooter.shooting();
        intake_system.auto();
        auto_pose = drive.position;
        if(forward >= 6 && drive.position.getX(DistanceUnit.INCH) < 20){
            turret.zero = true;
        }
        if(drive.currentPath == null) {
            if(forward == .25){
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    drive.currentPath = new ArrayList<>(first_pick);
                    forward = .75;
                    steps = 0;
                }
            }else if(forward == .75){
                if(steps == 0){
                    intake_system.stop_intake = true;
                    gate_open.reset();
                    steps = 1;
                }else if(steps == 1 && gate_open.milliseconds() > first_dump_time) {
                    intake_system.stop_intake = false;
                    drive.currentPath = new ArrayList<>(second_shoot);
                    forward = .8;
                    steps = 0;
                }
            }else if (forward == .8) {
                shoot();
                shooting = true;
                if(steps == 2){
                    shooting = false;
                    drive.currentPath = new ArrayList<>(second_pick);
                    forward = 2;
                    steps = 0;
                }
            }else if(forward == 2){
                if(steps == 0){
                    intake_system.stop_intake = true;
                    gate_open.reset();
                    steps = 1;
                }else if(steps == 1 && gate_open.milliseconds() > first_dump_time) {
                    intake_system.stop_intake = false;
                    drive.currentPath = new ArrayList<>(third_shoot);
                    forward = 3;
                    steps = 0;
                }
            }else if(forward == 3){
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    drive.currentPath = new ArrayList<>(third_pick);
                    forward = 5;
                    steps = 0;
                }
            }else if(forward == 5){
                if(steps == 0){
                    intake_system.stop_intake = true;
                    gate_open.reset();
                    steps = 1;
                }else if(steps == 1 && gate_open.milliseconds() > first_dump_time) {
                    intake_system.stop_intake = false;
                    drive.currentPath = new ArrayList<>(fourth_shoot);
                    forward = 7;
                    steps = 0;
                }
            }else if(forward == 7){
                shooting_time.reset();
                forward = 7.1;
            }else if(forward == 7.1 && shooting_time.milliseconds() > 500){
                intake_system.spindexer.setPower(-.5);
                intake_system.spindex_shooting = true;
                intake_system.flippy_pos = flippy_up;
                intake_system.blocking = false;
            }
        }
    }
    public ArrayList<Pose2D> gate2;
    public ArrayList<Pose2D> gate3;
    public ArrayList<Pose2D> to_gate;

    public void red_init(){
        first_shoot = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 70, -4,AngleUnit.DEGREES, 180)));
        first_pick = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 52, -5,AngleUnit.DEGREES, 180), new Pose2D(DistanceUnit.INCH, 46, -48,AngleUnit.DEGREES, 180), new Pose2D(DistanceUnit.INCH, 46, -35,AngleUnit.DEGREES, 180), new Pose2D(DistanceUnit.INCH, 55, -42,AngleUnit.DEGREES, 180)));
        second_shoot = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 55, -25,AngleUnit.DEGREES, 180),new Pose2D(DistanceUnit.INCH, 70, -12,AngleUnit.DEGREES, 180)));
        second_pick = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 70, -40,AngleUnit.DEGREES, 180), new Pose2D(DistanceUnit.INCH, 61, -34,AngleUnit.DEGREES, 180), new Pose2D(DistanceUnit.INCH, 60, -41,AngleUnit.DEGREES, 180)));
        third_shoot = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 72, -8,AngleUnit.DEGREES, 180)));
        third_pick = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 32, 0,AngleUnit.DEGREES, 180), new Pose2D(DistanceUnit.INCH, 27, -44,AngleUnit.DEGREES, 180), new Pose2D(DistanceUnit.INCH, 53, -35,AngleUnit.DEGREES, 180), new Pose2D(DistanceUnit.INCH, 55, -42,AngleUnit.DEGREES, 180)));
        fourth_shoot = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 92, -5,AngleUnit.DEGREES, 180)));
    }
    public void blue_init(){
        first_shoot = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 70, 4,AngleUnit.DEGREES, 0)));
        first_pick = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 52, 5,AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.INCH, 46, 48,AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.INCH, 46, 35,AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.INCH, 55, 42,AngleUnit.DEGREES, 0)));
        second_shoot = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 55, 25,AngleUnit.DEGREES, 0),new Pose2D(DistanceUnit.INCH, 70, 12,AngleUnit.DEGREES, 0)));
        second_pick = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 70, 40,AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.INCH, 61, 34,AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.INCH, 60, 41,AngleUnit.DEGREES, 0)));
        third_shoot = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 72, 8,AngleUnit.DEGREES, 0)));
        third_pick = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 32, 0,AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.INCH, 27, 44,AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.INCH, 53, 35,AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.INCH, 55, 42,AngleUnit.DEGREES, 0)));
        fourth_shoot = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 92, 5,AngleUnit.DEGREES, 0)));
    }
    public void shoot(){
        if(shooting) {
            if (steps == 0) {
                shooting_time.reset();
                steps = .5;
            } else if (steps == .5) {
                shooting_time.reset();
                intake_system.spindexer.setPower(-.5);
                intake_system.spindex_shooting = true;
                intake_system.flippy_pos = flippy_up;
                intake_system.blocking = false;
                steps = 1;
            } else if (steps == 1 && shooting_time.milliseconds() > 1000) {
                intake_system.flippy_pos = flippy_down;
                intake_system.spindexer.setPower(1);
                intake_system.spindex_shooting = false;
                intake_system.blocking = true;
                steps = 2;
            }
        }
    }




}
