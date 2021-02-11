package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

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

public class OrientationDynamicsCommand implements MPCCommand<OrientationDynamicsCommand>
{
   private int commandId;

   private final Quaternion orientationEstimate = new Quaternion();
   private final Vector3D angularVelocityEstimate = new Vector3D();

   private final Point3D comPositionEstimate = new Point3D();

   private final SpatialInertia bodyInertia = new SpatialInertia(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());

   private final List<ContactPlaneHelper> contactPlaneHelpers = new ArrayList<>();

   private int segmentNumber;
   private double timeOfObjective;
   private double omega;
   private double weight;
   private ConstraintType constraintType = ConstraintType.OBJECTIVE;

   public void clear()
   {
      contactPlaneHelpers.clear();
   }

   public void setConstraintType(ConstraintType constraintType)
   {
      this.constraintType = constraintType;
   }

   public void setSegmentNumber(int segmentNumber)
   {
      this.segmentNumber = segmentNumber;
   }

   public void setTimeOfObjective(double timeOfObjective)
   {
      this.timeOfObjective = timeOfObjective;
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public void addContactPlaneHelper(ContactPlaneHelper contactPlaneHelper)
   {
      this.contactPlaneHelpers.add(contactPlaneHelper);
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

   public double getTimeOfCommand()
   {
      return timeOfObjective;
   }

   public double getOmega()
   {
      return omega;
   }

   public int getSegmentNumber()
   {
      return segmentNumber;
   }

   public double getWeight()
   {
      return weight;
   }

   public int getNumberOfContacts()
   {
      return contactPlaneHelpers.size();
   }

   public ContactPlaneHelper getContactPlaneHelper(int i)
   {
      return contactPlaneHelpers.get(i);
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

   @Override
   public MPCCommandType getCommandType()
   {
      return MPCCommandType.ORIENTATION_DYNAMICS;
   }

   @Override
   public void set(OrientationDynamicsCommand other)
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
      setSegmentNumber(other.getSegmentNumber());
      setTimeOfObjective(other.getTimeOfCommand());
      for (int i = 0; i < other.getNumberOfContacts(); i++)
         addContactPlaneHelper(other.getContactPlaneHelper(i));
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
      else if (object instanceof OrientationDynamicsCommand)
      {
         OrientationDynamicsCommand other = (OrientationDynamicsCommand) object;
         if (commandId != other.commandId)
            return false;
         if (constraintType != other.constraintType)
            return false;
         if (omega != other.omega)
            return false;
         if (weight != other.weight)
            return false;
         if (timeOfObjective != other.timeOfObjective)
            return false;
         if (segmentNumber != other.segmentNumber)
            return false;
         if (!orientationEstimate.equals(other.orientationEstimate))
            return false;
         if (!angularVelocityEstimate.equals(other.angularVelocityEstimate))
            return false;
         if (!comPositionEstimate.equals(other.comPositionEstimate))
            return false;
         if (!bodyInertia.equals(other.bodyInertia))
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
                      + weight + ", time of objective: " + timeOfObjective + ", segment number: " + segmentNumber + ".";
      for (int i = 0; i < getNumberOfContacts(); i++)
      {
         string += "\ncontact " + i + ": " + contactPlaneHelpers.get(i);
      }
      return string;
   }
}
