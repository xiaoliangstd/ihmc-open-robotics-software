package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoFrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLongTime;

public class FootTrajectoryPredictor
{
   private static final double defaultSwingHeight = 0.15;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final MultipleWaypointsPositionTrajectoryGenerator leftFootTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("leftFootPredictedTrajectory", ReferenceFrame.getWorldFrame(), registry);
   private final MultipleWaypointsPositionTrajectoryGenerator rightFootTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("rightFootPredictedTrajectory", ReferenceFrame.getWorldFrame(), registry);
   private final SideDependentList<MultipleWaypointsPositionTrajectoryGenerator> footTrajectories = new SideDependentList<>(leftFootTrajectory, rightFootTrajectory);

   private MultipleWaypointsPoseTrajectoryGenerator swingTrajectory;

   private final SwingTrajectoryParameters swingTrajectoryParameters;

   public FootTrajectoryPredictor(SwingTrajectoryParameters swingTrajectoryParameters, YoRegistry parentRegistry)
   {
      this.swingTrajectoryParameters = swingTrajectoryParameters;
      parentRegistry.addChild(registry);
   }

   public void setSwingTrajectory(MultipleWaypointsPoseTrajectoryGenerator swingTrajectory)
   {
      this.swingTrajectory = swingTrajectory;
   }

   public void compute(CoPTrajectoryGeneratorState state, Footstep footstep)
   {
      if (state.getNumberOfFootstep() > 0 || footstep == null)
         computeWalking(state, footstep);
      else
         computeStanding(state);
   }

   private final FrameVector3DReadOnly zeroVector = new FrameVector3D();

   private void computeStanding(CoPTrajectoryGeneratorState state)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         MultipleWaypointsPositionTrajectoryGenerator footTrajectory = footTrajectories.get(robotSide);
         footTrajectory.clear();
         footTrajectory.appendWaypoint(0.0, state.getFootPose(robotSide).getPosition(), zeroVector);
         footTrajectory.appendWaypoint(sufficientlyLongTime, state.getFootPose(robotSide).getPosition(), zeroVector);
         footTrajectory.initialize();
      }
   }

   private void computeWalking(CoPTrajectoryGeneratorState state, Footstep footstep)
   {
      PlanningTiming timing = state.getTiming(0);

      double transferDuration = Math.min(timing.getTransferTime(), sufficientlyLongTime);
      double swingDuration = Math.min(timing.getSwingTime(), sufficientlyLongTime);

      for (RobotSide robotSide : RobotSide.values)
      {
         MultipleWaypointsPositionTrajectoryGenerator footTrajectory = footTrajectories.get(robotSide);
         footTrajectory.clear();
         footTrajectory.appendWaypoint(0.0, state.getFootPose(robotSide).getPosition(), zeroVector);
         footTrajectory.appendWaypoint(transferDuration, state.getFootPose(robotSide).getPosition(), zeroVector);
      }

      RobotSide swingSide = footstep.getRobotSide();
      RobotSide stanceSide = swingSide.getOppositeSide();

      footTrajectories.get(stanceSide)
                      .appendWaypoint(transferDuration + swingDuration, state.getFootPose(stanceSide).getPosition(), zeroVector);

      if (swingTrajectory == null)
      {
         predictSwingFootTrajectory(transferDuration,
                                    transferDuration + swingDuration,
                                    state.getFootPose(swingSide).getPosition(),
                                    footstep,
                                    footTrajectories.get(swingSide));
      }
      else
      {
         setSwingFootTrajectory(swingTrajectory.getPositionTrajectory(), footTrajectories.get(swingSide));
      }

      leftFootTrajectory.initialize();
      rightFootTrajectory.initialize();
   }

   public MultipleWaypointsPositionTrajectoryGenerator getPredictedLeftFootTrajectories()
   {
      return leftFootTrajectory;
   }

   public MultipleWaypointsPositionTrajectoryGenerator getPredictedRightFootTrajectories()
   {
      return rightFootTrajectory;
   }

   private final FramePoint3D midpoint1 = new FramePoint3D();
   private final FramePoint3D midpoint2 = new FramePoint3D();

   private final FrameVector3D velocityVector1 = new FrameVector3D();
   private final FrameVector3D velocityVector2 = new FrameVector3D();

   private final double[] waypointProportions = new double[2];

   void predictSwingFootTrajectory(double startTime,
                                   double endTime,
                                   FramePoint3DReadOnly startPosition,
                                   Footstep footstep,
                                   MultipleWaypointsPositionTrajectoryGenerator trajectoryToPack)
   {
      FramePoint3DReadOnly endPosition = footstep.getFootstepPose().getPosition();

      // FIXME handle sending down custom waypoints.

      if (footstep.getCustomWaypointProportions().size() == 2)
      {
         waypointProportions[0] = footstep.getCustomWaypointProportions().get(0).getValue();
         waypointProportions[1] = footstep.getCustomWaypointProportions().get(1).getValue();
      }
      else
      {
         waypointProportions[0] = swingTrajectoryParameters.getSwingWaypointProportions()[0];
         waypointProportions[1] = swingTrajectoryParameters.getSwingWaypointProportions()[1];
      }

      double swingHeight = footstep.getSwingHeight() > 0.0 ? footstep.getSwingHeight() : defaultSwingHeight;

      double time1 = InterpolationTools.linearInterpolate(startTime, endTime, waypointProportions[0]);
      midpoint1.interpolate(startPosition, endPosition, waypointProportions[0]);
      midpoint1.addZ(swingHeight);

      double time2 = InterpolationTools.linearInterpolate(startTime, endTime, waypointProportions[1]);
      midpoint2.interpolate(startPosition, endPosition, waypointProportions[1]);
      midpoint2.addZ(swingHeight);

      velocityVector1.sub(midpoint2, startPosition);
      velocityVector1.scale(1.0 / (time2 - startTime));

      velocityVector2.sub(endPosition, midpoint1);
      velocityVector2.scale(1.0 / (endTime - time1));

      trajectoryToPack.appendWaypoint(time1, midpoint1, velocityVector1);
      trajectoryToPack.appendWaypoint(time2, midpoint2, velocityVector2);
      trajectoryToPack.appendWaypoint(endTime, endPosition, zeroVector);
   }

   private static void setSwingFootTrajectory(MultipleWaypointsPositionTrajectoryGenerator swingTrajectory,
                                              MultipleWaypointsPositionTrajectoryGenerator trajectoriesToPack)
   {

      double timeShift = trajectoriesToPack.getLastWaypointTime() - swingTrajectory.getWaypoint(0).getTime();
      for (int waypointIdx = 1; waypointIdx < swingTrajectory.getCurrentNumberOfWaypoints(); waypointIdx++)
      {
         YoFrameEuclideanTrajectoryPoint waypoint = swingTrajectory.getWaypoint(waypointIdx);
         trajectoriesToPack.appendWaypoint(waypoint.getTime() + timeShift, waypoint.getPosition(), waypoint.getLinearVelocity());
      }
   }
}
