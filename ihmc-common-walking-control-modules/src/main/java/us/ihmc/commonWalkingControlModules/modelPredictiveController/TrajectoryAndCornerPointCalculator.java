package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CornerPointViewer;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCQPInputCalculator.sufficientlyLongTime;
import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class TrajectoryAndCornerPointCalculator
{
   private CornerPointViewer viewer = null;

   private final FramePoint3D comPositionToThrowAway = new FramePoint3D();
   private final FramePoint3D dcmPositionToThrowAway = new FramePoint3D();

   private final FrameVector3D comVelocityToThrowAway = new FrameVector3D();
   private final FrameVector3D comAccelerationToThrowAway = new FrameVector3D();
   private final FrameVector3D dcmVelocityToThrowAway = new FrameVector3D();
   private final FramePoint3D vrpStartPosition = new FramePoint3D();
   private final FrameVector3D vrpStartVelocity = new FrameVector3D();
   private final FramePoint3D vrpEndPosition = new FramePoint3D();
   private final FrameVector3D vrpEndVelocity = new FrameVector3D();
   private final FramePoint3D ecmpPositionToThrowAway = new FramePoint3D();

   private final RecyclingArrayList<FramePoint3D> dcmCornerPoints = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> comCornerPoints = new RecyclingArrayList<>(FramePoint3D::new);

   private final RecyclingArrayList<Trajectory3D> vrpTrajectoryPool = new RecyclingArrayList<>(() -> new Trajectory3D(4));
   private final RecyclingArrayList<LineSegment3D> vrpSegments = new RecyclingArrayList<>(LineSegment3D::new);
   private final List<Trajectory3D> vrpTrajectories = new ArrayList<>();

   public void setViewer(CornerPointViewer viewer)
   {
      this.viewer = viewer;
   }

   public void updateCornerPoints(CoMTrajectoryModelPredictiveController mpc, List<? extends ContactStateProvider> contactSequence, int maxCapacity)
   {
      vrpTrajectoryPool.clear();
      vrpTrajectories.clear();

      comCornerPoints.clear();
      dcmCornerPoints.clear();
      vrpSegments.clear();

      for (int segmentId = 0; segmentId < Math.min(contactSequence.size(), maxCapacity + 1); segmentId++)
      {
         double duration = contactSequence.get(segmentId).getTimeInterval().getDuration();

         duration = Math.min(duration, sufficientlyLongTime);
         mpc.compute(segmentId,
                     0.0,
                     comCornerPoints.add(),
                     comVelocityToThrowAway,
                     comAccelerationToThrowAway,
                     dcmCornerPoints.add(),
                     dcmVelocityToThrowAway,
                     vrpStartPosition,
                     vrpStartVelocity,
                     ecmpPositionToThrowAway);
         mpc.compute(segmentId,
                     duration,
                     comPositionToThrowAway,
                     comVelocityToThrowAway,
                     comAccelerationToThrowAway,
                     dcmPositionToThrowAway,
                     dcmVelocityToThrowAway,
                     vrpEndPosition,
                     vrpEndVelocity,
                     ecmpPositionToThrowAway);

         Trajectory3D trajectory3D = vrpTrajectoryPool.add();
         trajectory3D.setCubic(0.0, duration, vrpStartPosition, vrpStartVelocity, vrpEndPosition, vrpEndVelocity);
         //         trajectory3D.setLinear(0.0, duration, vrpStartPosition, vrpEndPosition);
         vrpTrajectories.add(trajectory3D);

         vrpSegments.add().set(vrpStartPosition, vrpEndPosition);
      }

      if (viewer != null)
      {
         viewer.updateDCMCornerPoints(dcmCornerPoints);
         viewer.updateCoMCornerPoints(comCornerPoints);
         viewer.updateVRPWaypoints(vrpSegments);
      }
   }

   public List<Trajectory3D> getVrpTrajectories()
   {
      return vrpTrajectories;
   }
}
