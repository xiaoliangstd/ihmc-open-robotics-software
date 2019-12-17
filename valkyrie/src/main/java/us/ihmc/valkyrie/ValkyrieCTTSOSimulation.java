package us.ihmc.valkyrie;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.robotEnvironmentAwareness.tools.ConstantPlanarRegionsPublisher;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;

public class ValkyrieCTTSOSimulation
{
   private static final DataSetName DATA_SET_TO_USE = DataSetName._20190220_172417_EOD_Cinders;

   public static void main(String[] args)
   {
      DataSet dataSet = DataSetIOTools.loadDataSet(DATA_SET_TO_USE);

      DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      PlanarRegionsListDefinedEnvironment simEnvironment = new PlanarRegionsListDefinedEnvironment(dataSet.getPlanarRegionsList(), 0.02, true);

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, simEnvironment);
      simulationStarter.setRunMultiThreaded(true);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.setStartingLocation(() -> new OffsetAndYawRobotInitialSetup(dataSet.getPlannerInput().getStartPosition(), dataSet.getPlannerInput().getStartYaw()));

      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();

      // talk to controller and footstep planner
      networkProcessorParameters.enableFootstepPlanningToolbox(true);
      networkProcessorParameters.enableWalkingPreviewToolbox(true);
      networkProcessorParameters.enableBipedalSupportPlanarRegionPublisher(true);

      // disable everything else
      networkProcessorParameters.enableBehaviorModule(false);
      networkProcessorParameters.enableBehaviorVisualizer(false);
      networkProcessorParameters.enableSensorModule(true);
      networkProcessorParameters.enableZeroPoseRobotConfigurationPublisherModule(false);
      networkProcessorParameters.enableRosModule(false);
      networkProcessorParameters.enableKinematicsToolbox(false);
      networkProcessorParameters.enableWholeBodyTrajectoryToolbox(false);
      networkProcessorParameters.enableKinematicsPlanningToolbox(false);
      networkProcessorParameters.enableRobotEnvironmentAwerenessModule(false);
      networkProcessorParameters.enableMocapModule(false);
      networkProcessorParameters.enableAutoREAStateUpdater(true);

      // start sim
      simulationStarter.startSimulation(networkProcessorParameters, false);

      // spoof and publish planar regions
      ConstantPlanarRegionsPublisher constantPlanarRegionsPublisher = new ConstantPlanarRegionsPublisher(dataSet.getPlanarRegionsList());
      constantPlanarRegionsPublisher.start(2000);
   }
}
