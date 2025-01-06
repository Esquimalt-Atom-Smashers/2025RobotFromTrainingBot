package frc.Controllers;
//TODO Change this to fit the interface for the current robot
public interface PersonalizedController {
    PersonalizedController getcontroller();
    int zeroGyroButton();
    int goToHighPosButton();
    int goToMedPosButton();
    int goToGroundPickupPosButton();
    int goToShelfPickupButton();
    int goToHomeButton();
    int shiftLowButton();
    int shiftHighButton();
    boolean povPressed();
    int getPOVValue();
    double ArmUpDownAxis() ;
    double ArmLeftRightAxis();
    double ArmInOutAxis();

    
}
