package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
@Autonomous(name="Moment Maker", group="ABC Opmode", preselectTeleOp = "DecodeTeleop")
public class MM_Auto extends DecodeLibrary{
    public double forward = 0.25;
    public Path first_pick;
    public Path pick_after_stuff;
    public Path first_shoot;
    public Path second_shoot;
    public Path third_shoot;
    public Path fourth_shoot;
    public Path fifth_shoot;
    public Path second_pick;
    public Path third_pick;
    public Path park;
    public Path gate;
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
    public static double first_dump_time = 1000;
    public static double second_dump_time = 3000;
    @Override
    public void init(){
        color = 1;
        teleop = false;
        if(color == 1){
            auto_pose =new Pose(122,-25, Math.toRadians(-90));
        }else{
            auto_pose = new Pose(122,25, Math.toRadians(90));
        }
        initialize();
        follower.setPose(auto_pose);
        if(color == 0) {
            blue_init();
        }else{
            red_init();
        }
        intake_system.balls.add("Purple");
        intake_system.balls.add("Purple");
        intake_system.balls.add("Purple");
        follower.setMaxPower(1);
        follower.setMaxPowerScaling(1);
        follower.followPath(first_shoot);
        intake_system.flippy_pos = flippy_hold;
    }
    @Override
    public void init_loop(){
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        x_mod = -8;
        turret.zero = true;
        cam.look();
        if(cam.tags.contains(21.0)){
            pattern = 1;
        }else if(cam.tags.contains(22.0)){
            pattern = 2;
        }else if(cam.tags.contains(23.0)){
            pattern = 3;
        }
        turret.turret_move();
        follower.drivetrain.setYVelocity(0);
        follower.update();
        if(gamepad1.right_bumper) {
            color = 0;
        }else if(gamepad1.left_bumper){
            color = 1;
        }
        if(old_color != color || gamepad1.a) {
            initialize();
            if(color == 1){
                auto_pose =new Pose(122,-25, Math.toRadians(-90));
            }else{
                auto_pose = new Pose(122,25, Math.toRadians(90));
            }
            follower.setMaxPower(1);
            follower.setMaxPowerScaling(1);
            follower.setPose(auto_pose);
            if (color == 0) {
                blue_init();
                old_color = 0;
            } else {
                red_init();
                old_color = 1;
            }

            follower.followPath(first_shoot);
        }
        auto_pose = follower.getPose();
        telemetry.addData("pattern", pattern);
        telemetry.addData("color", color);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void loop() {
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        follower.update();
        turret.turret_move();
        shooter.set_speed = shoot_multiplier * ((dead_distance * .0254) - 1.6) + shoot_power_offset + (speed_increase * abs(turret.target_angle - .5));
        shooter.set_speed = Math.round(shooter.set_speed / 20) * 20 + 60;
        shooter.shooting();
        intake_system.auto();
        auto_sort();
        intake_system.sort();
        auto_pose = follower.getPose();
        if(forward == 6 || forward == 7 || (forward == 8 && follower.getPose().getX() < 70)){
            turret.zero = true;
        }
        if(follower.atParametricEnd() || !follower.isBusy() || follower.isRobotStuck()) {
            if (forward == .25) {
                shoot();
                shooting = true;
                if(steps == 2){
                    shooting = false;
                    follower.followPath(second_pick);
                    forward = .5;
                    steps = 0;
                }
            }else if (forward == .5) {
                sort_balls = true;
                follower.followPath(to_gate);
                forward = 1.1;
            }else if (forward == 1.1) {
                follower.followPath(gate);
                forward = 2;
            }else if(forward == 2){
                if(steps == 0){
                    intake_system.stop_intake = true;
                    gate_open.reset();
                    steps = 1;
                }else if(steps == 1 && gate_open.milliseconds() > first_dump_time) {
                    intake_system.stop_intake = false;
                    follower.followPath(third_shoot);
                    forward = 3;
                    steps = 0;
                }
            }else if(forward == 3){
                intake_system.outtake = false;
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    follower.followPath(first_pick);
                    forward = 4;
                    steps = 0;
                }
            }else if(forward == 4) {
                follower.followPath(second_shoot);
                forward = 5;
                steps = 0;
            }else if(forward == 5){
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    follower.followPath(gate3);
                    forward = 6;
                    steps = 0;
                }
            }else if(forward == 6){
                if(steps == 0){
                    gate_open.reset();
                    steps = 1;
                }else if(steps == 1 && gate_open.milliseconds() > second_dump_time) {
                    follower.followPath(pick_after_stuff);
                    forward = 7;
                    steps = 0;
                }
            }else if(forward == 7){
                follower.followPath(fifth_shoot);
                sort_balls = true;
                forward = 8;
            }else if(forward == 8){
                shooting_time.reset();
                forward = 8.1;
            }else if(forward == 8.1 && shooting_time.milliseconds() > 1000){
                intake_system.spindexer.setPower(-.7);
                intake_system.spindex_shooting = true;
                intake_system.flippy_pos = flippy_up;
                intake_system.blocking = false;
            }
        }
    }
    public Path gate2;
    public Path gate3;
    public Path to_gate;

    public void red_init(){
        first_shoot = new Path(new BezierLine(new Pose(123,-24), new Pose(80, -4)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
        second_pick = new Path(new BezierCurve(first_shoot.getLastControlPoint(), new Pose(70, -39)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        to_gate = new Path(new BezierCurve(second_pick.getLastControlPoint(), new Pose(67, -35)));
        to_gate.setConstantHeadingInterpolation(Math.toRadians(0));
        gate = new Path(new BezierCurve(to_gate.getLastControlPoint(), new Pose(64, -40)));
        gate.setConstantHeadingInterpolation(Math.toRadians(0));
        third_shoot = new Path(new BezierLine(gate.getLastControlPoint(), new Pose(72, -8)));
        third_shoot.setConstantHeadingInterpolation(0);
        first_pick = new Path(new BezierCurve(third_shoot.getLastControlPoint(), new Pose(50, -3), new Pose(47, -49)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        second_shoot = new Path(new BezierCurve(first_pick.getLastControlPoint(),new Pose(49, -30), new Pose(70, -12)));
        second_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
        gate3 = new Path(new BezierCurve(second_shoot.getLastControlPoint(), new Pose(58, -44)));
        gate3.setConstantHeadingInterpolation(Math.toRadians(-180));
        pick_after_stuff = new Path(new BezierCurve(gate3.getLastControlPoint(), new Pose(53,-40), new Pose(45,-47), new Pose(21, -50), new Pose(2, -50)));
        pick_after_stuff.setConstantHeadingInterpolation(Math.toRadians(-180));
        fifth_shoot = new Path(new BezierLine(pick_after_stuff.getLastControlPoint(), new Pose(90, -10)));
        fifth_shoot.setConstantHeadingInterpolation(Math.toRadians(-180));
    }
    public void blue_init(){
        first_shoot = new Path(new BezierLine(new Pose(123,24), new Pose(78, 4)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        second_pick = new Path(new BezierCurve(first_shoot.getLastControlPoint(), new Pose(70, 10), new Pose(70, 40)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        to_gate = new Path(new BezierCurve(second_pick.getLastControlPoint(), new Pose(67, 35)));
        to_gate.setConstantHeadingInterpolation(Math.toRadians(0));
        gate = new Path(new BezierCurve(to_gate.getLastControlPoint(), new Pose(64, 42)));
        gate.setConstantHeadingInterpolation(Math.toRadians(0));
        third_shoot = new Path(new BezierLine(gate.getLastControlPoint(), new Pose(72, 8)));
        third_shoot.setConstantHeadingInterpolation(0);
        first_pick = new Path(new BezierCurve(third_shoot.getLastControlPoint(), new Pose(52, 3), new Pose(49, 49)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        second_shoot = new Path(new BezierCurve(first_pick.getLastControlPoint(),new Pose(49, 20), new Pose(70, 12)));
        second_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        gate3 = new Path(new BezierCurve(second_shoot.getLastControlPoint(), new Pose(58, 45)));
        gate3.setConstantHeadingInterpolation(Math.toRadians(180));
        pick_after_stuff = new Path(new BezierCurve(gate3.getLastControlPoint(), new Pose(53,40), new Pose(45,47), new Pose(21, 50), new Pose(2, 50)));
        pick_after_stuff.setConstantHeadingInterpolation(Math.toRadians(180));
        fifth_shoot = new Path(new BezierLine(pick_after_stuff.getLastControlPoint(), new Pose(95, 0)));
        fifth_shoot.setConstantHeadingInterpolation(Math.toRadians(180));
    }
    public void shoot(){
        if(shooting) {
            if (steps == 0 && abs(shooter.shoot1.getVelocity() - shooter.speed) <= 20 && intake_system.sort_time == 0 && !intake_system.sorting) {
                shooting_time.reset();
                steps = .5;
            } else if(steps == .5){
                shooting_time.reset();
                intake_system.spindexer.setPower(-.7);
                intake_system.spindex_shooting = true;
                intake_system.flippy_pos = flippy_up;
                intake_system.blocking = false;
                steps = 1;
            }else if (steps == 1 && (shooting_time.milliseconds() > 2000 || shooter.balls_shot >= intake_system.balls.size() )) {
                intake_system.flippy_pos = flippy_down;
                intake_system.spindexer.setPower(1);
                intake_system.spindex_shooting = false;
                intake_system.blocking = true;
                steps = 2;
            }
        }
    }
    public int sort_count(){
        intake_system.recount();
        List<String> unsort = new ArrayList<>();
        if(intake_system.balls.size() == 1){
            unsort.add(intake_system.balls.get(0));
        }else if(intake_system.balls.size() == 2){
            unsort.add(intake_system.balls.get(1));
            unsort.add(intake_system.balls.get(0));
        }else if(intake_system.balls.size() == 3){
            unsort.add(intake_system.balls.get(2));
            unsort.add(intake_system.balls.get(1));
            unsort.add(intake_system.balls.get(0));
        }
        List<String> sort = new ArrayList<>();
        int num = 0;
        if(pattern == 1){
            sort.add("Green");
            sort.add("Purple");
            sort.add("Purple");
        }else if(pattern == 2){
            sort.add("Purple");
            sort.add("Green");
            sort.add("Purple");
        }else if(pattern == 3){
            sort.add("Purple");
            sort.add("Purple");
            sort.add("Green");
        }
        if(unsort.size() == 3) {
            if (unsort == sort) {
                num = 0;
            } else if (!unsort.contains("Green")) {
                num = 0;
            } else if (!unsort.contains("Purple")) {
                num = 0;
            } else {
                if (Collections.frequency(unsort, "Green") > 1) {
                    num = sort.indexOf("Purple") - unsort.indexOf("Purple");
                    if (num < 0) {
                        num += 3;
                    }
                } else {
                    num = sort.indexOf("Green") - unsort.indexOf("Green");
                    if (num < 0) {
                        num += 3;
                    }
                }
            }
        }else{
            if(unsort.contains("Green")){
                if(sort.indexOf("Green") != unsort.indexOf("Green")){
                    num = 1;
                }else {
                    num = 0;
                }
            }else{
                num = 0;
            }
        }
        return num;
    }
    public boolean sorted = false;
    public double sort_steps = 0;
    public ElapsedTime sort_timer = new ElapsedTime();
    public boolean sort_balls = false;
    public boolean override = false;
    public void auto_sort(){
        if(!sorted && intake_system.blocking && sort_balls) {
            if (intake_system.balls.size() == 3 || override) {
                if(sort_steps == 0){
                    sort_timer.reset();
                    sort_steps = 1;
                }else if(sort_steps == 1 && sort_timer.milliseconds() > 500){
                    sorted = true;
                    override = false;
                    intake_system.sort_time = sort_count();
                    sort_balls = false;
                    sort_steps = 0;
                }
            }

        }else if(!intake_system.blocking){
            sorted = false;
        }
    }



}
