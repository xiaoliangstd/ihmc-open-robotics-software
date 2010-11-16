package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.controllers.PDController;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.trajectory.YoMinimumJerkTrajectory;

public class FootOrientationControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("FootOrientationControlModule");

   private final PDController pitchController = new PDController("footPitch", registry);
   private final PDController rollController = new PDController("footRoll", registry);

   private final DoubleYoVariable desiredFootPitch = new DoubleYoVariable("desiredFootPitch", "", registry);
   private final DoubleYoVariable desiredFootRoll = new DoubleYoVariable("desiredFootRoll", "", registry);

   private final DoubleYoVariable currentFootPitch = new DoubleYoVariable("currentFootPitch", "", registry);
   private final DoubleYoVariable currentFootRoll = new DoubleYoVariable("currentFootRoll", "", registry);


   private final YoMinimumJerkTrajectory minimumJerkTrajectory;
   private final ProcessedSensorsInterface processedSensors;
   private final CommonWalkingReferenceFrames referenceFrames;

   private final ReferenceFrame desiredHeadingFrame;

   private final YoFrameOrientation initialOrientation;
   private final YoFrameOrientation finalOrientation;

   private final Orientation desiredFootOrientation;


   public FootOrientationControlModule(ProcessedSensorsInterface processedSensors, CommonWalkingReferenceFrames referenceFrames,
           DesiredHeadingControlModule desiredHeadingControlModule, YoVariableRegistry parentRegistry)
   {
      minimumJerkTrajectory = new YoMinimumJerkTrajectory("footOrientation", registry);
      this.processedSensors = processedSensors;
      this.referenceFrames = referenceFrames;
      this.desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
      desiredFootOrientation = new Orientation(desiredHeadingFrame);
      initialOrientation = new YoFrameOrientation("footInitialOrientation", "", desiredHeadingFrame, registry);
      finalOrientation = new YoFrameOrientation("footFinalOrientation", "", desiredHeadingFrame, registry);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }

      pitchController.setProportionalGain(10.0);    // 500.0); //200.0); //0.0);
      rollController.setProportionalGain(10.0);    // 0.0);

      pitchController.setDerivativeGain(1.0);    // 0.0);
      rollController.setDerivativeGain(1.0);    // 0.0);
   }
   
   public void setParametersForR2()
   {
      pitchController.setProportionalGain(10.0);    // 500.0); //200.0); //0.0);
      rollController.setProportionalGain(10.0);    // 0.0);

      pitchController.setDerivativeGain(1.0);    // 0.0);
      rollController.setDerivativeGain(1.0);    // 0.0);
   }
   
   public void setParametersForM2V2()
   {
      pitchController.setProportionalGain(10.0);    
      rollController.setProportionalGain(10.0);   

      pitchController.setDerivativeGain(1.0);   
      rollController.setDerivativeGain(1.0);   
   }

   public void initializeFootOrientationMove(double moveDuration, Orientation finalOrientation, RobotSide supportFoot)
   {
      minimumJerkTrajectory.setParams(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, moveDuration);
      this.finalOrientation.set(finalOrientation);
      this.initialOrientation.set(getFootOrientationInFrame(desiredHeadingFrame, supportFoot));
   }

   public void addAdditionalTorqueForFootOrientationControl(LegTorques legTorquesToAddAddionalTorque, double timeInMove)
   {
      minimumJerkTrajectory.computeTrajectory(timeInMove);
      double alpha = minimumJerkTrajectory.getPosition();

      RobotSide supportLeg = legTorquesToAddAddionalTorque.getRobotSide();

      Orientation currentFootOrientation = getFootOrientationInFrame(desiredHeadingFrame, supportLeg);

      desiredFootOrientation.interpolate(initialOrientation.getFrameOrientationCopy(), finalOrientation.getFrameOrientationCopy(), alpha);

      double[] currentFootYawPitchRoll = currentFootOrientation.getYawPitchRoll();
      currentFootPitch.set(currentFootYawPitchRoll[1]);
      currentFootRoll.set(currentFootYawPitchRoll[2]);

      double[] desiredFootYawPitchRoll = desiredFootOrientation.getYawPitchRoll();
      desiredFootPitch.set(desiredFootYawPitchRoll[1]);
      desiredFootRoll.set(desiredFootYawPitchRoll[2]);

      double currentPitchVelocity = processedSensors.getLegJointVelocity(supportLeg, LegJointName.ANKLE_PITCH);
      double currentRollVelocity = processedSensors.getLegJointVelocity(supportLeg, LegJointName.ANKLE_ROLL);

      double desiredPitchVelocity = 0.0;
      double desiredRollVelocity = 0.0;

      double pitchTorque = pitchController.compute(currentFootPitch.getDoubleValue(), desiredFootPitch.getDoubleValue(), currentPitchVelocity,
                              desiredPitchVelocity);
      double rollTorque = rollController.compute(currentFootRoll.getDoubleValue(), desiredFootRoll.getDoubleValue(), currentRollVelocity, desiredRollVelocity);

      // legTorquesToAddAddionalTorque.addTorque(LegJointName.ANKLE_PITCH, pitchTorque);
      legTorquesToAddAddionalTorque.addTorque(LegJointName.ANKLE_ROLL, rollTorque);
   }
   
   private Orientation getFootOrientationInFrame(ReferenceFrame referenceFrame, RobotSide supportFoot)
   {
      ReferenceFrame supportFootFrame = referenceFrames.getFootFrame(supportFoot);
      Orientation orientation = new Orientation(referenceFrame, supportFootFrame.getTransformToDesiredFrame(referenceFrame));
      return orientation;
   }
}
