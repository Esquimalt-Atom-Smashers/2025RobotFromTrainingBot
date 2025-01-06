package frc.Controllers;

import edu.wpi.first.wpilibj.XboxController;
import java.util.Map;
import java.util.HashMap;
//TODO Change this to fit the interface for the current robot
public class xBoxBrandon extends XboxController implements PersonalizedController {
    
    // Define an enumeration for the actions to make the mapping clear and readable.
    public enum Action {
        ENABLE_FINE_CONTROL,
        GO_TO_HIGH_POS,
        GO_TO_MED_POS,
        GO_TO_GROUND_PICKUP_POS,
        GO_TO_SHELF_PICKUP,
        GO_TO_HOME,
        SHIFT_LOW,
        SHIFT_HIGH,
        OPEN_CLAW,
        CLOSE_CLAW_CUBE,
        CLOSE_CLAW_CONE
    }

    // Create a lookup table that maps each action to its corresponding button value.
    //TODO change these mappings in this table to remap functions in one place
    private final Map<Action, Integer> buttonMappings = new HashMap<>() {{
        put(Action.ENABLE_FINE_CONTROL, XboxController.Button.kLeftBumper.value);
        put(Action.GO_TO_HIGH_POS, XboxController.Button.kY.value);
        put(Action.GO_TO_MED_POS, XboxController.Button.kX.value);
        put(Action.GO_TO_GROUND_PICKUP_POS, XboxController.Button.kA.value);
        put(Action.GO_TO_SHELF_PICKUP, XboxController.Button.kB.value);
        put(Action.GO_TO_HOME, XboxController.Button.kStart.value);
        put(Action.SHIFT_LOW, XboxController.Button.kLeftStick.value);
        put(Action.SHIFT_HIGH, XboxController.Button.kRightStick.value);
        put(Action.OPEN_CLAW, XboxController.Button.kRightBumper.value);
        put(Action.CLOSE_CLAW_CUBE, XboxController.Button.kRightStick.value);
        put(Action.CLOSE_CLAW_CONE, XboxController.Button.kLeftStick.value);
    }};

    //-1 is the value returned by the controller when the POV buttons are not pressed
    private int m_lastPovValue = -1;

    public xBoxBrandon(int port) {
        super(port);
    }

    @Override
    public PersonalizedController getcontroller() {
        return this;
    }

    // Each button method uses the lookup table to get the appropriate button value.
    @Override
    public int zeroGyroButton() {
        return buttonMappings.get(Action.ENABLE_FINE_CONTROL);
    }

    @Override
    public int goToHighPosButton() {
        return buttonMappings.get(Action.GO_TO_HIGH_POS);
    }

    @Override
    public int goToMedPosButton() {
        return buttonMappings.get(Action.GO_TO_MED_POS);
    }

    @Override
    public int goToGroundPickupPosButton() {
        return buttonMappings.get(Action.GO_TO_GROUND_PICKUP_POS);
    }

    @Override
    public int goToShelfPickupButton() {
        return buttonMappings.get(Action.GO_TO_SHELF_PICKUP);
    }

    @Override
    public int goToHomeButton() {
        return buttonMappings.get(Action.GO_TO_HOME);
    }

    @Override
    public int shiftLowButton() {
        return buttonMappings.get(Action.SHIFT_LOW);
    }

    @Override
    public int shiftHighButton() {
        return buttonMappings.get(Action.SHIFT_HIGH);
    }

    public int openClawButton() {
        return buttonMappings.get(Action.OPEN_CLAW);
    }

    public int closeClawCubeButton() {
        return buttonMappings.get(Action.CLOSE_CLAW_CUBE);
    }

    public int closeClawConeButton() {
        return buttonMappings.get(Action.CLOSE_CLAW_CONE);
    }

    @Override
    public boolean povPressed() {
        int l_pov = getPOV();
        if (l_pov != -1 && l_pov != m_lastPovValue) {
            m_lastPovValue = l_pov;
            return true;
        } else {
            m_lastPovValue = l_pov;
            return false;
        }
    }

    public int getPOVValue() {
        return getPOV();
    }

    public double ArmUpDownAxis() {
        return -getRightY();
    }

    public double ArmLeftRightAxis() {
        return getRightX();
    }

    public double ArmInOutAxis() {
        return -getLeftY();
    }
}

