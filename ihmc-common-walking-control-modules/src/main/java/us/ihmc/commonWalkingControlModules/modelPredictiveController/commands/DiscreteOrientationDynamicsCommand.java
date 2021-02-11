package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneHelper;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

import java.util.ArrayList;
import java.util.List;

public class DiscreteOrientationDynamicsCommand implements MPCCommand<DiscreteOrientationDynamicsCommand>
{
   private int commandId;
   private final Quaternion orientationEstimate = new Quaternion();
   private final Vector3D angularVelocityEstimate = new Vector3D();

   private final Point3D comPositionEstimate = new Point3D();

   private final SpatialInertia bodyInertia = new SpatialInertia(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());

   private final List<List<ContactPlaneHelper>> contactPlaneHelpers = new ArrayList<>();
   private final TDoubleArrayList segmentDurations = new TDoubleArrayList();

   private double omega;
   private double weight;
   private ConstraintType constraintType = ConstraintType.OBJECTIVE;

   public void clear()
   {
      contactPlaneHelpers.clear();
      segmentDurations.reset();
   }

   public void setConstraintType(ConstraintType constraintType)
   {
      this.constraintType = constraintType;
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public void addContactPlaneHelper(List<ContactPlaneHelper> contactPlaneHelper)
   {
      contactPlaneHelpers.add(contactPlaneHelper);
   }

   // FIXME this makes garbage
   public void addContactPlaneHelper(int segment, ContactPlaneHelper contactPlaneHelper)
   {
      if (segment > contactPlaneHelpers.size() - 1)
         contactPlaneHelpers.add(new ArrayList<>());
      this.contactPlaneHelpers.get(segment).add(contactPlaneHelper);
   }

   public void addSegmentDuration(double duration)
   {
      segmentDurations.add(duration);
   }

   public void setOrientationEstimate(Orientation3DReadOnly orientationEstimate)
   {
      this.orientationEstimate.set(orientationEstimate);
   }

   public void setAngularVelocityEstimate(Vector3DReadOnly angularVelocityEstimate)
   {
      this.angularVelocityEstimate.set(angularVelocityEstimate);
   }

   public void setComPositionEstimate(Point3DReadOnly comPositionEstimate)
   {
      this.comPositionEstimate.set(comPositionEstimate);
   }

   public void setBodyInertia(SpatialInertiaReadOnly inertia)
   {
      this.bodyInertia.set(inertia);
   }

   public double getOmega()
   {
      return omega;
   }

   public ContactPlaneHelper getContactPlaneHelper(int segment, int i)
   {
      return contactPlaneHelpers.get(segment).get(i);
   }

   public int getNumberOfSegments()
   {
      return contactPlaneHelpers.size();
   }

   public int getNumberOfContacts(int segment)
   {
      return contactPlaneHelpers.get(segment).size();
   }

   public double getSegmentDuration(int segment)
   {
      return segmentDurations.get(segment);
   }

   public Orientation3DReadOnly getOrientationEstimate()
   {
      return orientationEstimate;
   }

   public Vector3DReadOnly getAngularVelocityEstimate()
   {
      return angularVelocityEstimate;
   }

   public Point3DReadOnly getComPositionEstimate()
   {
      return comPositionEstimate;
   }

   public SpatialInertiaReadOnly getBodyInertia()
   {
      return bodyInertia;
   }

   public ConstraintType getConstraintType()
   {
      return constraintType;
   }

   public double getWeight()
   {
      return weight;
   }

   @Override
   public MPCCommandType getCommandType()
   {
      return MPCCommandType.ORIENTATION_DYNAMICS;
   }

   @Override
   public void set(DiscreteOrientationDynamicsCommand other)
   {
      clear();
      setCommandId(other.getCommandId());
      setOrientationEstimate(other.getOrientationEstimate());
      setAngularVelocityEstimate(other.getAngularVelocityEstimate());
      setComPositionEstimate(other.getComPositionEstimate());
      setBodyInertia(other.getBodyInertia());
      setOmega(other.getOmega());
      setWeight(other.getWeight());
      setConstraintType(other.getConstraintType());
      for (int segment = 0; segment < other.getNumberOfSegments(); segment++)
      {
         addSegmentDuration(other.getSegmentDuration(segment));
         for (int i = 0; i < other.getNumberOfContacts(segment); i++)
            addContactPlaneHelper(segment, other.getContactPlaneHelper(segment, i));
      }
   }

   @Override
   public void setCommandId(int id)
   {
      commandId = id;
   }

   @Override
   public int getCommandId()
   {
      return commandId;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof DiscreteOrientationDynamicsCommand)
      {
         DiscreteOrientationDynamicsCommand other = (DiscreteOrientationDynamicsCommand) object;
         if (commandId != other.commandId)
            return false;
         if (constraintType != other.constraintType)
            return false;
         if (omega != other.omega)
            return false;
         if (weight != other.weight)
            return false;
         if (!orientationEstimate.equals(other.orientationEstimate))
            return false;
         if (!angularVelocityEstimate.equals(other.angularVelocityEstimate))
            return false;
         if (!comPositionEstimate.equals(other.comPositionEstimate))
            return false;
         if (!bodyInertia.equals(other.bodyInertia))
            return false;
         if (!segmentDurations.equals(other.segmentDurations))
            return false;
         if (contactPlaneHelpers.size() != other.contactPlaneHelpers.size())
            return false;
         for (int i = 0; i < contactPlaneHelpers.size(); i++)
         {
            if (!contactPlaneHelpers.get(i).equals(other.contactPlaneHelpers.get(i)))
               return false;
         }
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      String string = getClass().getSimpleName() + ": orientation estimate: " + orientationEstimate + ", angular velocity estimate: " + angularVelocityEstimate
                      + ", com position estimate: " + comPositionEstimate + ", body inertia: " + bodyInertia +  ", omega: " + omega + ", weight: "
                      + weight + ", segment durations: " + segmentDurations + ", number of segments: " + getNumberOfSegments() + ".";
      for (int segment = 0; segment < getNumberOfSegments(); segment++)
      {
         for (int i = 0; i < getNumberOfContacts(segment); i++)
         {
            string += "\ncontact " + i + ": " + contactPlaneHelpers.get(segment).get(i);
         }
      }
      return string;
   }
}
