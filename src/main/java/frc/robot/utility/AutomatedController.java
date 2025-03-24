package frc.robot.utility;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class AutomatedController {
    public final CommandXboxController controller;
    public final SendableChooser<Runnable> selector = new SendableChooser<Runnable>();
    public int mode = 3;
    public GenericHID rumble = new GenericHID(0);

    IO io;

    public AutomatedController(int port, IO io){
        this.io = io;


        selector.setDefaultOption("Automated", () -> {mode = 0;});
        selector.addOption("Manual", () -> {mode = 1;});
        selector.addOption("Characterise", () -> {mode = 2;});
        selector.addOption("Debug", () -> {mode = 3;});

        selector.onChange((x) -> {x.run();});

        controller = new CommandXboxController(port);

        controller.rightStick().onTrue(new InstantCommand(() -> io.chassis.field_oritented = !io.chassis.field_oritented));
        controller.leftStick().debounce(2).onTrue(new InstantCommand(io.chassis::resetAngle));
        configure();

    }

    public BooleanSupplier mode(int targetMode){
        return () -> {return mode == targetMode;};
    }

    public BooleanSupplier automated(){
        return mode(0);
    }
    
    public BooleanSupplier manual(){
        return mode(1);
    }

    public BooleanSupplier characterise(){
        return mode(2);
    }

    public BooleanSupplier debug(){
        return mode(3);
    }

    public void switchMode(){
        mode = (mode + 1) % 3;
    }

    public void configure(){
        controller.start().and(controller.getHID()::getBackButtonPressed).onTrue(Util.Do(this::switchMode));
        configureAutomated();
        configureManual();
        configureCharacterisaton();
        configureDebug();
    }

    void configureAutomated(){
        controller.back().onTrue(Util.Do(io.chassis::resetAngle, io.chassis));
    }

    public void configureManual(){
        controller.back().and(manual()).onTrue(Util.Do(io.chassis::resetOdometry, io.chassis));
        
        controller.povDown().and( manual() ).onTrue(Util.Do(io.chassis::toggle));
        controller.povLeft().and( manual() ).onTrue(Util.Do(io.chassis::syncEncoders));
        controller.povRight().and( manual() ).and(() -> {return !io.chassis.active;}).onTrue(new InstantCommand(io.chassis::zeroAbsolute));
    }

    void configureCharacterisaton(){
        controller.x().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kForward));
        controller.a().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kReverse));
        controller.y().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kForward));
        controller.b().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kReverse));

        controller.povUp().and(characterise()).toggleOnTrue(io.chassis.steerRoutine.quasistatic(Direction.kForward));
        controller.povDown().and(characterise()).toggleOnTrue(io.chassis.steerRoutine.quasistatic(Direction.kReverse));
        controller.povRight().and(characterise()).toggleOnTrue(io.chassis.steerRoutine.dynamic(Direction.kForward));
        controller.povLeft().and(characterise()).toggleOnTrue(io.chassis.steerRoutine.dynamic(Direction.kReverse));
    }

    void configureDebug(){
        controller.back().and(debug()).onTrue(Util.Do(io.chassis::resetOdometry, io.chassis));
    }

}