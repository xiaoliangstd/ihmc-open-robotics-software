package us.ihmc.robotics.math.trajectories.generators;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.robotics.time.TimeIntervalProvider;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator.defaultMaximumNumberOfWaypoints;

public class MultipleSegmentPositionTrajectoryGenerator<T extends FixedFramePositionTrajectoryGenerator & TimeIntervalProvider> implements FixedFramePositionTrajectoryGenerator
{
   private final String namePrefix;

   private final int maximumNumberOfSegments;

   private final YoRegistry registry;

   private final YoDouble currentSegmentTime;

   private final YoInteger numberOfSegments;
   private final YoInteger currentSegmentIndex;
   private final List<T> segments;

   private final FixedFramePoint3DBasics currentPosition;
   private final FixedFrameVector3DBasics currentVelocity;
   private final FixedFrameVector3DBasics currentAcceleration;

   private T activeSubTrajectory;

   public MultipleSegmentPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoRegistry parentRegistry)
   {
      this(namePrefix, defaultMaximumNumberOfWaypoints, referenceFrame, parentRegistry);
   }

   public MultipleSegmentPositionTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, ReferenceFrame referenceFrame,
                                                     YoRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.maximumNumberOfSegments = maximumNumberOfWaypoints;

      registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      numberOfSegments = new YoInteger(namePrefix + "NumberOfSegments", registry);
      numberOfSegments.set(0);

      segments = new ArrayList<>();

      currentSegmentTime = new YoDouble(namePrefix + "CurrentTrajectoryTime", registry);
      currentSegmentIndex = new YoInteger(namePrefix + "CurrentSegmentIndex", registry);

      String currentPositionName = namePrefix + "CurrentPosition";
      String currentVelocityName = namePrefix + "CurrentVelocity";
      String currentAccelerationName = namePrefix + "CurrentAcceleration";

      currentPosition = new YoFramePoint3D(currentPositionName, referenceFrame, registry);
      currentVelocity = new YoFrameVector3D(currentVelocityName, referenceFrame, registry);
      currentAcceleration = new YoFrameVector3D(currentAccelerationName, referenceFrame, registry);

      clear();

      parentRegistry.addChild(registry);
   }

   public void clear()
   {
      numberOfSegments.set(0);
      currentSegmentIndex.set(0);
      segments.clear();
   }

   public void appendSegment(T segment)
   {
      checkReferenceFrameMatch(segment);
      checkNumberOfSegments(numberOfSegments.getIntegerValue() + 1);
      checkNextSegmentIsContinuous(segment);
      appendSegmentsUnsafe(segment);
   }

   private void appendSegmentsUnsafe(T segment)
   {
      segments.add(segment);
      numberOfSegments.increment();
   }

   public void appendSegments(T[] segments)
   {
      checkNumberOfSegments(numberOfSegments.getIntegerValue() + segments.length);

      for (int i = 0; i < segments.length; i++)
         appendSegment(segments[i]);
   }

   public void appendSegments(List<T> segments)
   {
      checkNumberOfSegments(numberOfSegments.getIntegerValue() + segments.size());

      for (int i = 0; i < segments.size(); i++)
         appendSegment(segments.get(i));
   }

   private void checkNumberOfSegments(int length)
   {
      if (length > maximumNumberOfSegments)
         throw new RuntimeException("Cannot exceed the maximum number of segments. Number of segments provided: " + length);
   }

   private void checkNextSegmentIsContinuous(T segment)
   {
      if (getCurrentNumberOfSegments() == 0)
         return;

      if (!MathTools.epsilonEquals(segments.get(getCurrentNumberOfSegments() - 1).getTimeInterval().getEndTime(), segment.getTimeInterval().getStartTime(), 5e-3))
         throw new RuntimeException("The next segment doesn't start where the previous one ended.");
   }

   @Override
   public void initialize()
   {
      if (numberOfSegments.getIntegerValue() == 0)
      {
         throw new RuntimeException("Trajectory has no segments.");
      }

      currentSegmentIndex.set(0);
      activeSubTrajectory = segments.get(0);
   }

   @Override
   public void compute(double time)
   {
      if (isEmpty())
      {
         throw new RuntimeException("Can not call compute on an empty trajectory.");
      }

      currentSegmentTime.set(time);
      boolean changedSubTrajectory = false;

      if (!TimeIntervalTools.isTimeSequenceContinuous(segments))
         throw new RuntimeException("The segments do not represent a continuous time trajectory.");

      if (time < segments.get(currentSegmentIndex.getIntegerValue()).getTimeInterval().getStartTime())
      {
         currentSegmentIndex.set(0);
         changedSubTrajectory = true;
      }

      while (currentSegmentIndex.getIntegerValue() < numberOfSegments.getIntegerValue() - 1
             && time >= segments.get(currentSegmentIndex.getIntegerValue()).getTimeInterval().getEndTime())
      {
         currentSegmentIndex.increment();
         changedSubTrajectory = true;
      }

      if (changedSubTrajectory)
      {
         activeSubTrajectory = segments.get(currentSegmentIndex.getIntegerValue());
      }

      T segment = segments.get(currentSegmentIndex.getValue());
      TimeIntervalReadOnly timeInterval = segment.getTimeInterval();
      time = MathTools.clamp(time, timeInterval.getStartTime(), timeInterval.getEndTime());

      double subTrajectoryTime = MathTools.clamp(time - timeInterval.getStartTime(), 0.0, timeInterval.getDuration());
      activeSubTrajectory.compute(subTrajectoryTime);

      currentPosition.set(activeSubTrajectory.getPosition());
      currentVelocity.set(activeSubTrajectory.getVelocity());
      currentAcceleration.set(activeSubTrajectory.getAcceleration());
   }

   @Override
   public boolean isDone()
   {
      if (isEmpty())
         return true;

      boolean isLastWaypoint = currentSegmentIndex.getIntegerValue() >= numberOfSegments.getIntegerValue() - 2;
      if (!isLastWaypoint)
         return false;
      return currentSegmentTime.getValue() >= getEndTime();
   }

   public boolean isEmpty()
   {
      return numberOfSegments.getIntegerValue() == 0;
   }

   public int getCurrentSegmentIndex()
   {
      return currentSegmentIndex.getIntegerValue();
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return currentPosition;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return currentVelocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return currentAcceleration;
   }

   public int getCurrentNumberOfSegments()
   {
      return numberOfSegments.getIntegerValue();
   }

   public int getMaximumNumberOfSegments()
   {
      return maximumNumberOfSegments;
   }

   public double getEndTime()
   {
      return segments.get(getCurrentNumberOfSegments() - 1).getTimeInterval().getEndTime();
   }

   @Override
   public String toString()
   {
      if (numberOfSegments.getIntegerValue() == 0)
         return namePrefix + ": Has no waypoints.";
      else
         return namePrefix + ": number of waypoints = " + numberOfSegments.getIntegerValue() + ", current waypoint index = "
                + currentSegmentIndex.getIntegerValue();
   }
}
