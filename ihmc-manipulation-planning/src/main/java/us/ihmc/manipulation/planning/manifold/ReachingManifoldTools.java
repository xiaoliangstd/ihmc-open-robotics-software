package us.ihmc.manipulation.planning.manifold;

import javax.vecmath.MismatchedSizeException;

import controller_msgs.msg.dds.ReachingManifoldMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.geometry.Cylinder3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.Sphere3D;
import us.ihmc.euclid.geometry.Torus3D;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.SegmentedLine3DMeshDataGenerator;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.FunctionTrajectory;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.idl.IDLSequence.Byte;
import us.ihmc.manipulation.planning.gradientDescent.GradientDescentModule;
import us.ihmc.manipulation.planning.gradientDescent.SingleQueryFunction;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ReachingManifoldTools
{
   public static Graphics3DObject createManifoldMessageStaticGraphic(ReachingManifoldMessage reachingManifoldMessage, double radius,
                                                                     int resolutionForSingleSpace)
   {
      Pose3D originPose = new Pose3D(reachingManifoldMessage.getManifoldOriginPosition(), reachingManifoldMessage.getManifoldOriginOrientation());

      int numberOfPoints = (int) Math.pow(resolutionForSingleSpace, reachingManifoldMessage.getManifoldConfigurationSpaceNames().size());

      SegmentedLine3DMeshDataGenerator segmentedLine3DMeshGenerator = new SegmentedLine3DMeshDataGenerator(numberOfPoints, resolutionForSingleSpace, radius);

      Point3D[] points = new Point3D[numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         Pose3D pose = new Pose3D(originPose);
         double[] configurationValues = new double[reachingManifoldMessage.getManifoldConfigurationSpaceNames().size()];
         int[] configurationIndex = new int[reachingManifoldMessage.getManifoldConfigurationSpaceNames().size()];

         int tempIndex = i;
         for (int j = reachingManifoldMessage.getManifoldConfigurationSpaceNames().size(); j > 0; j--)
         {
            configurationIndex[j - 1] = (int) (tempIndex / Math.pow(resolutionForSingleSpace, j - 1));
            tempIndex = (int) (tempIndex % Math.pow(resolutionForSingleSpace, j - 1));
         }

         for (int j = 0; j < reachingManifoldMessage.getManifoldConfigurationSpaceNames().size(); j++)
         {
            configurationValues[j] = (reachingManifoldMessage.getManifoldUpperLimits().get(j) - reachingManifoldMessage.getManifoldLowerLimits().get(j))
                  / (resolutionForSingleSpace - 1) * configurationIndex[j] + reachingManifoldMessage.getManifoldLowerLimits().get(j);

            ConfigurationSpaceName configurationSpaceName = ConfigurationSpaceName.fromByte(reachingManifoldMessage.getManifoldConfigurationSpaceNames()
                                                                                                                   .get(j));

            switch (configurationSpaceName)
            {
            case X:
               pose.appendTranslation(configurationValues[j], 0.0, 0.0);
               break;
            case Y:
               pose.appendTranslation(0.0, configurationValues[j], 0.0);
               break;
            case Z:
               pose.appendTranslation(0.0, 0.0, configurationValues[j]);
               break;
            case ROLL:
               pose.appendRollRotation(configurationValues[j]);
               break;
            case PITCH:
               pose.appendPitchRotation(configurationValues[j]);
               break;
            case YAW:
               pose.appendYawRotation(configurationValues[j]);
               break;
            default:
               break;
            }
         }

         points[i] = new Point3D(pose.getPosition());
      }

      segmentedLine3DMeshGenerator.compute(points);

      Graphics3DObject graphics = new Graphics3DObject();
      for (MeshDataHolder mesh : segmentedLine3DMeshGenerator.getMeshDataHolders())
      {
         graphics.addMeshData(mesh, YoAppearance.AliceBlue());
      }

      return graphics;
   }

   public static void packRigidBodyTransformOnManifold(ReachingManifoldMessage reachingManifoldMessage, TDoubleArrayList configurations,
                                                       RigidBodyTransform rigidBodyTransformToPack)
   {
      Byte manifoldConfigurationSpaceNames = reachingManifoldMessage.getManifoldConfigurationSpaceNames();
      if (manifoldConfigurationSpaceNames.size() != configurations.size())
         throw new MismatchedSizeException("configuration space size and name size are not matched.");

      rigidBodyTransformToPack.setIdentity();
      rigidBodyTransformToPack.appendTranslation(reachingManifoldMessage.getManifoldOriginPosition());
      rigidBodyTransformToPack.setRotation(reachingManifoldMessage.getManifoldOriginOrientation());

      for (int i = 0; i < configurations.size(); i++)
         rigidBodyTransformToPack.multiply(ConfigurationSpaceName.fromByte(manifoldConfigurationSpaceNames.get(i))
                                                                 .getLocalRigidBodyTransform(configurations.get(i)));

   }

   public static void packClosestRigidBodyTransformOnManifold(ReachingManifoldMessage reachingManifoldMessage, RigidBodyTransform rigidBodyTransform,
                                                              RigidBodyTransform rigidBodyTransformToPack)
   {
      double[] manifoldUpperLimits = reachingManifoldMessage.getManifoldUpperLimits().toArray();
      double[] manifoldLowerLimits = reachingManifoldMessage.getManifoldLowerLimits().toArray();

      TDoubleArrayList initialInput = new TDoubleArrayList();
      TDoubleArrayList upperLimits = new TDoubleArrayList();
      TDoubleArrayList lowerLimits = new TDoubleArrayList();
      for (int i = 0; i < manifoldLowerLimits.length; i++)
      {
         initialInput.add((manifoldUpperLimits[i] + manifoldLowerLimits[i]) / 2);
         upperLimits.add(manifoldUpperLimits[i]);
         lowerLimits.add(manifoldLowerLimits[i]);
      }

      SingleQueryFunction function = new SingleQueryFunction()
      {
         @Override
         public double getQuery(TDoubleArrayList values)
         {
            RigidBodyTransform closestTransform = new RigidBodyTransform();
            packRigidBodyTransformOnManifold(reachingManifoldMessage, values, closestTransform);

            double x = rigidBodyTransform.getTranslationX();
            double y = rigidBodyTransform.getTranslationY();
            double z = rigidBodyTransform.getTranslationZ();

            double cx = closestTransform.getTranslationX();
            double cy = closestTransform.getTranslationY();
            double cz = closestTransform.getTranslationZ();

            return Math.sqrt(EuclidCoreTools.normSquared(x - cx, y - cy, z - cz));
         }
      };
      GradientDescentModule solver = new GradientDescentModule(function, initialInput);

      solver.setMaximumIterations(200);
      solver.setInputLowerLimit(lowerLimits);
      solver.setInputUpperLimit(upperLimits);

      solver.run();
      TDoubleArrayList optimalSolution = solver.getOptimalInput();

      rigidBodyTransformToPack.setIdentity();
      rigidBodyTransformToPack.appendTranslation(reachingManifoldMessage.getManifoldOriginPosition());
      rigidBodyTransformToPack.setRotation(reachingManifoldMessage.getManifoldOriginOrientation());

      for (int i = 0; i < reachingManifoldMessage.getManifoldConfigurationSpaceNames().size(); i++)
         rigidBodyTransformToPack.multiply(ConfigurationSpaceName.fromByte(reachingManifoldMessage.getManifoldConfigurationSpaceNames().get(i))
                                                                 .getLocalRigidBodyTransform(optimalSolution.get(i)));
   }

   public static void packRigidBodyTransformOnManifold(ReachingManifoldCommand reachingManifoldCommand, TDoubleArrayList configurations,
                                                       RigidBodyTransform rigidBodyTransformToPack)
   {
      int dimensionOfManifold = reachingManifoldCommand.getDimensionOfManifold();
      if (dimensionOfManifold != configurations.size())
         throw new MismatchedSizeException("configuration space size and name size are not matched.");

      rigidBodyTransformToPack.setIdentity();
      rigidBodyTransformToPack.appendTranslation(reachingManifoldCommand.getManifoldOriginPosition());
      rigidBodyTransformToPack.setRotation(reachingManifoldCommand.getManifoldOriginOrientation());

      for (int i = 0; i < configurations.size(); i++)
         rigidBodyTransformToPack.multiply(reachingManifoldCommand.getDegreeOfManifold(i).getLocalRigidBodyTransform(configurations.get(i)));

   }

   public static void packClosestRigidBodyTransformOnManifold(ReachingManifoldCommand reachingManifoldCommand, RigidBodyTransform rigidBodyTransform,
                                                              RigidBodyTransform rigidBodyTransformToPack)
   {
      packClosestRigidBodyTransformOnManifold(reachingManifoldCommand, rigidBodyTransform, rigidBodyTransformToPack, 1.0, 1.0);
   }
   
   public static void packClosestRigidBodyTransformOnManifold(ReachingManifoldCommand reachingManifoldCommand, RigidBodyTransform rigidBodyTransform,
                                                              RigidBodyTransform rigidBodyTransformToPack, double positionWeight, double orientationWeight)
   {
      double[] manifoldUpperLimits = reachingManifoldCommand.getManifoldUpperLimits().toArray();
      double[] manifoldLowerLimits = reachingManifoldCommand.getManifoldLowerLimits().toArray();

      TDoubleArrayList initialInput = new TDoubleArrayList();
      TDoubleArrayList upperLimits = new TDoubleArrayList();
      TDoubleArrayList lowerLimits = new TDoubleArrayList();
      for (int i = 0; i < manifoldLowerLimits.length; i++)
      {
         initialInput.add((manifoldUpperLimits[i] + manifoldLowerLimits[i]) / 2);
         upperLimits.add(manifoldUpperLimits[i]);
         lowerLimits.add(manifoldLowerLimits[i]);
      }

      SingleQueryFunction function = new SingleQueryFunction()
      {
         @Override
         public double getQuery(TDoubleArrayList values)
         {
            RigidBodyTransform closestTransform = new RigidBodyTransform();
            packRigidBodyTransformOnManifold(reachingManifoldCommand, values, closestTransform);

            return getDistance(rigidBodyTransform, closestTransform, positionWeight, orientationWeight);
         }
      };
      GradientDescentModule solver = new GradientDescentModule(function, initialInput);

      solver.setMaximumIterations(200);
      solver.setInputLowerLimit(lowerLimits);
      solver.setInputUpperLimit(upperLimits);

      solver.run();
      TDoubleArrayList optimalSolution = solver.getOptimalInput();

      rigidBodyTransformToPack.setIdentity();
      rigidBodyTransformToPack.appendTranslation(reachingManifoldCommand.getManifoldOriginPosition());
      rigidBodyTransformToPack.setRotation(reachingManifoldCommand.getManifoldOriginOrientation());

      for (int i = 0; i < reachingManifoldCommand.getDimensionOfManifold(); i++)
         rigidBodyTransformToPack.multiply(reachingManifoldCommand.getDegreeOfManifold(i).getLocalRigidBodyTransform(optimalSolution.get(i)));
   }

   public static ReachingManifoldMessage createSphereManifoldMessage(RigidBody hand, Sphere3D sphere3D)
   {
      return createSphereManifoldMessage(hand, sphere3D.getPosition(), sphere3D.getRadius());
   }

   public static ReachingManifoldMessage createCylinderManifoldMessage(RigidBody hand, Cylinder3D cylinder3D)
   {
      return createCylinderManifoldMessage(hand, cylinder3D.getPosition(), cylinder3D.getOrientation(), cylinder3D.getRadius(), cylinder3D.getHeight());
   }

   public static ReachingManifoldMessage createBoxManifoldMessage(RigidBody hand, Box3D box3D)
   {
      return createBoxManifoldMessage(hand, box3D.getPosition(), box3D.getOrientation(), box3D.getLength(), box3D.getWidth(), box3D.getHeight());
   }

   public static ReachingManifoldMessage createTorusManifoldMessage(RigidBody hand, Torus3D torus3D)
   {
      return createTorusManifoldMessage(hand, torus3D.getPosition(), torus3D.getOrientation(), torus3D.getRadius(), torus3D.getTubeRadius());
   }

   public static ReachingManifoldMessage createSphereManifoldMessage(RigidBody hand, Tuple3DReadOnly tuple3dReadOnly, double radius)
   {
      ConfigurationSpaceName[] manifoldSpaces = {ConfigurationSpaceName.YAW, ConfigurationSpaceName.PITCH, ConfigurationSpaceName.X};
      double[] lowerLimits = new double[] {-Math.PI, -Math.PI / 2, -radius};
      double[] upperLimits = new double[] {Math.PI, Math.PI / 2, -radius};

      ReachingManifoldMessage reachingManifoldMessage = HumanoidMessageTools.createReachingManifoldMessage(hand);

      reachingManifoldMessage.getManifoldOriginPosition().set(tuple3dReadOnly);
      reachingManifoldMessage.getManifoldOriginOrientation().set(new Quaternion());

      HumanoidMessageTools.packManifold(ConfigurationSpaceName.toBytes(manifoldSpaces), lowerLimits, upperLimits, reachingManifoldMessage);
      return reachingManifoldMessage;
   }

   public static ReachingManifoldMessage createCylinderManifoldMessage(RigidBody hand, Tuple3DReadOnly tuple3dReadOnly,
                                                                       RotationMatrixReadOnly rotationMatrixReadOnly, double radius, double height)
   {
      ConfigurationSpaceName[] manifoldSpaces = {ConfigurationSpaceName.YAW, ConfigurationSpaceName.Z, ConfigurationSpaceName.X};
      double[] lowerLimits = new double[] {-Math.PI / 2, -height / 2, -radius};
      double[] upperLimits = new double[] {Math.PI / 2, height / 2, radius};

      ReachingManifoldMessage reachingManifoldMessage = HumanoidMessageTools.createReachingManifoldMessage(hand);

      reachingManifoldMessage.getManifoldOriginPosition().set(tuple3dReadOnly);
      reachingManifoldMessage.getManifoldOriginOrientation().set(rotationMatrixReadOnly);

      HumanoidMessageTools.packManifold(ConfigurationSpaceName.toBytes(manifoldSpaces), lowerLimits, upperLimits, reachingManifoldMessage);
      return reachingManifoldMessage;
   }

   public static ReachingManifoldMessage createBoxManifoldMessage(RigidBody hand, Tuple3DReadOnly tuple3dReadOnly,
                                                                  RotationMatrixReadOnly rotationMatrixReadOnly, double length, double width, double height)
   {
      ConfigurationSpaceName[] manifoldSpaces = {ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z};
      double[] lowerLimits = new double[] {-length / 2, -width / 2, -height / 2};
      double[] upperLimits = new double[] {length / 2, width / 2, height / 2};

      ReachingManifoldMessage reachingManifoldMessage = HumanoidMessageTools.createReachingManifoldMessage(hand);

      reachingManifoldMessage.getManifoldOriginPosition().set(tuple3dReadOnly);
      reachingManifoldMessage.getManifoldOriginOrientation().set(rotationMatrixReadOnly);

      HumanoidMessageTools.packManifold(ConfigurationSpaceName.toBytes(manifoldSpaces), lowerLimits, upperLimits, reachingManifoldMessage);
      return reachingManifoldMessage;
   }

   public static ReachingManifoldMessage createTorusManifoldMessage(RigidBody hand, Tuple3DReadOnly tuple3dReadOnly,
                                                                    RotationMatrixReadOnly rotationMatrixReadOnly, double radius, double thickness)
   {
      ConfigurationSpaceName[] manifoldSpaces = {ConfigurationSpaceName.YAW, ConfigurationSpaceName.X, ConfigurationSpaceName.PITCH, ConfigurationSpaceName.X};
      double[] lowerLimits = new double[] {-Math.PI, -radius, -Math.PI, -thickness};
      double[] upperLimits = new double[] {Math.PI, -radius, Math.PI, -thickness};

      ReachingManifoldMessage reachingManifoldMessage = HumanoidMessageTools.createReachingManifoldMessage(hand);

      reachingManifoldMessage.getManifoldOriginPosition().set(tuple3dReadOnly);
      reachingManifoldMessage.getManifoldOriginOrientation().set(rotationMatrixReadOnly);

      HumanoidMessageTools.packManifold(ConfigurationSpaceName.toBytes(manifoldSpaces), lowerLimits, upperLimits, reachingManifoldMessage);
      return reachingManifoldMessage;
   }

   public static ReachingManifoldMessage createGoalManifoldMessage(RigidBody hand, FunctionTrajectory handFunction, double trajectoryTime,
                                                                   ConfigurationSpaceName[] manifoldSpaces)
   {
      ReachingManifoldMessage reachingManifoldMessage = HumanoidMessageTools.createReachingManifoldMessage(hand);

      reachingManifoldMessage.getManifoldOriginPosition().set(handFunction.compute(trajectoryTime).getPosition());
      reachingManifoldMessage.getManifoldOriginOrientation().set(handFunction.compute(trajectoryTime).getOrientation());

      double[] lowerLimits = new double[manifoldSpaces.length];
      double[] upperLimits = new double[manifoldSpaces.length];
      for (int i = 0; i < lowerLimits.length; i++)
      {
         lowerLimits[i] = manifoldSpaces[i].getDefaultExplorationLowerLimit();
         upperLimits[i] = manifoldSpaces[i].getDefaultExplorationUpperLimit();
      }

      HumanoidMessageTools.packManifold(ConfigurationSpaceName.toBytes(manifoldSpaces), lowerLimits, upperLimits, reachingManifoldMessage);

      return reachingManifoldMessage;
   }

   public static ReachingManifoldMessage createGoalManifoldMessage(RigidBody hand, FunctionTrajectory handFunction, double trajectoryTime,
                                                                   ConfigurationSpaceName[] manifoldSpaces, double[] upperLimits, double[] lowerLimits)
   {
      ReachingManifoldMessage reachingManifoldMessage = HumanoidMessageTools.createReachingManifoldMessage(hand);

      reachingManifoldMessage.getManifoldOriginPosition().set(handFunction.compute(trajectoryTime).getPosition());
      reachingManifoldMessage.getManifoldOriginOrientation().set(handFunction.compute(trajectoryTime).getOrientation());

      HumanoidMessageTools.packManifold(ConfigurationSpaceName.toBytes(manifoldSpaces), lowerLimits, upperLimits, reachingManifoldMessage);

      return reachingManifoldMessage;
   }
   
   public static double getDistance(RigidBodyTransform from, RigidBodyTransform to, double positionWeight, double orientationWeight)
   {
      Point3D pointFrom = new Point3D(from.getTranslationVector());
      Quaternion orientationFrom = new Quaternion(from.getRotationMatrix());

      Point3D pointTo = new Point3D(to.getTranslationVector());
      Quaternion orientationTo = new Quaternion(to.getRotationMatrix());

      double positionDistance = positionWeight * pointFrom.distance(pointTo);
      double orientationDistance = orientationWeight * orientationFrom.distance(orientationTo);

      double distance = positionDistance + orientationDistance;

      return distance;
   }
}
