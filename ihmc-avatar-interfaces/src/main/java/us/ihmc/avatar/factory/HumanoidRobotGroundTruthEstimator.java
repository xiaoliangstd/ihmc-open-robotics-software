package us.ihmc.avatar.factory;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;

public class HumanoidRobotGroundTruthEstimator extends SimpleRobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final DRCPerfectSensorReader sensorReader;
   private final ForceSensorDataHolder forceSensorDataHolder;
   private final FullHumanoidRobotModel fullRobotModel;

   private final SideDependentList<ForceSensorDefinition> forceSensorDefinitions = new SideDependentList<>();

   private final SideDependentList<YoFramePoint3D> copInWorld = new SideDependentList<>();

   private final SideDependentList<YoFrameVector3D> footAngularVelocities = new SideDependentList<>();
   private final SideDependentList<YoFrameVector3D> footCoPLinearVelocities = new SideDependentList<>();

   public HumanoidRobotGroundTruthEstimator(FullHumanoidRobotModelFactory fullRobotModelFactory, FloatingRootJointRobot robot)
   {
      fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      forceSensorDataHolder = new ForceSensorDataHolder(fullRobotModel.getForceSensorDefinitions());

      sensorReader = buildSensorReader(robot);

      for (RobotSide robotSide : RobotSide.values)
      {
         forceSensorDefinitions.put(robotSide,
                                    forceSensorDataHolder.getForceSensorDefinitions().stream()
                                                         .filter(definition -> definition.getRigidBody() == fullRobotModel.getFoot(robotSide)).findFirst()
                                                         .get());

         copInWorld.put(robotSide, new YoFramePoint3D(robotSide.getCamelCaseName() + "GroundTruthCoP", worldFrame, registry));

         footAngularVelocities.put(robotSide, new YoFrameVector3D(robotSide.getCamelCaseName() + "FootGroundTruthAngularVelocity", worldFrame, registry));
         footCoPLinearVelocities.put(robotSide, new YoFrameVector3D(robotSide.getCamelCaseName() + "GroundTruthCoPLinearVelocity", worldFrame, registry));
      }
   }

   private DRCPerfectSensorReader buildSensorReader(FloatingRootJointRobot robot)
   {
      DRCPerfectSensorReaderFactory factory = new DRCPerfectSensorReaderFactory(robot, forceSensorDataHolder);
      factory.build(fullRobotModel.getRootJoint(), fullRobotModel.getIMUDefinitions(), fullRobotModel.getForceSensorDefinitions(), null, registry);
      return factory.getSensorReader();
   }

   private final Wrench wrench = new Wrench();
   private final FramePoint3D cop = new FramePoint3D();
   private final FrameVector3D vector = new FrameVector3D();
   private final CenterOfPressureResolver copResolver = new CenterOfPressureResolver();

   @Override
   public void doControl()
   {
      sensorReader.read(null);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);
         MovingReferenceFrame footBodyFixedFrame = foot.getBodyFixedFrame();

         forceSensorDataHolder.get(forceSensorDefinitions.get(robotSide)).getWrench(wrench);
         copResolver.resolveCenterOfPressureAndNormalTorque(cop, wrench, fullRobotModel.getSoleFrame(robotSide));
         if (cop.containsNaN())
            cop.setToZero();
         copInWorld.get(robotSide).setMatchingFrame(cop);

         TwistReadOnly footTwist = footBodyFixedFrame.getTwistOfFrame();
         footAngularVelocities.get(robotSide).setMatchingFrame(footTwist.getAngularPart());
         cop.changeFrame(footBodyFixedFrame);
         footTwist.getLinearVelocityAt(cop, vector);
         footCoPLinearVelocities.get(robotSide).setMatchingFrame(vector);
      }
   }
}
