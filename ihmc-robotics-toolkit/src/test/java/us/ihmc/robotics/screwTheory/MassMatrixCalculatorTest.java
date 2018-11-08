package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.simple.SimpleMatrix;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;

public abstract class MassMatrixCalculatorTest
{
   protected static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   protected static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   protected static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

   protected final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   protected ArrayList<RevoluteJoint> joints;
   protected RigidBodyBasics elevator;

   private final Random random = new Random(1776L);

   @Before
   public void setUp()
   {
      elevator = new RigidBody("elevator", worldFrame);
   }

   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   protected void setUpRandomChainRobot()
   {
      Random random = new Random();
      joints = new ArrayList<RevoluteJoint>();
      Vector3D[] jointAxes = {X, Y, Z, X, Z, Z, X, Y, Z, X};
      joints.addAll(MultiBodySystemRandomTools.nextRevoluteJointChain(random, "", elevator, jointAxes));
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
      elevator.updateFramesRecursively();
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
   }

   protected double computeKineticEnergy(ArrayList<RevoluteJoint> joints)
   {
      double ret = 0.0;
      ReferenceFrame elevatorFrame = elevator.getBodyFixedFrame();
      Twist twistWithRespectToWorld = new Twist(elevatorFrame, elevatorFrame, elevatorFrame);
      Twist successorTwist = new Twist();
      for (RevoluteJoint joint : joints)
      {
         joint.getSuccessorTwist(successorTwist);
         successorTwist.changeFrame(elevatorFrame);
         twistWithRespectToWorld.add(successorTwist);

         SpatialInertia inertia = new SpatialInertia(joint.getSuccessor().getInertia());
         inertia.changeFrame(elevatorFrame);

         ret += inertia.computeKineticCoEnergy(twistWithRespectToWorld);
      }

      return ret;
   }

   protected double computeKineticEnergy(ArrayList<RevoluteJoint> joints, DenseMatrix64F massMatrix)
   {
      SimpleMatrix jointVelocities = new SimpleMatrix(joints.size(), 1);
      for (int i = 0; i < joints.size(); i++)
      {
         jointVelocities.set(i, 0, joints.get(i).getQd());
      }

      SimpleMatrix massMatrix_ = SimpleMatrix.wrap(massMatrix);

      SimpleMatrix kineticEnergy = jointVelocities.transpose().mult(massMatrix_).mult(jointVelocities);

      return 0.5 * kineticEnergy.get(0, 0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.7)
   @Test(timeout = 30000)
   public void compareMassMatrixCalculators()
   {
      double eps = 1e-10;
      setUpRandomChainRobot();
      ArrayList<MassMatrixCalculator> massMatrixCalculators = new ArrayList<MassMatrixCalculator>();
      massMatrixCalculators.add(new DifferentialIDMassMatrixCalculator(worldFrame, elevator));
      massMatrixCalculators.add(new MassMatrixCalculator()
      {
         CompositeRigidBodyMassMatrixCalculator crbmmc = new CompositeRigidBodyMassMatrixCalculator(elevator);

         @Override
         public void getMassMatrix(DenseMatrix64F massMatrixToPack)
         {
            massMatrixToPack.set(getMassMatrix());
         }

         @Override
         public DenseMatrix64F getMassMatrix()
         {
            return crbmmc.getMassMatrix();
         }

         @Override
         public JointBasics[] getJointsInOrder()
         {
            return crbmmc.getInput().getJointsToConsider().toArray(new JointBasics[0]);
         }

         @Override
         public void compute()
         {
            crbmmc.reset();
         }
      });
      ArrayList<DenseMatrix64F> massMatrices = new ArrayList<DenseMatrix64F>();
      int nDoFs = ScrewTools.computeDegreesOfFreedom(joints);
      for (int i = 0; i < massMatrixCalculators.size(); i++)
      {
         massMatrices.add(new DenseMatrix64F(nDoFs, nDoFs));
      }
      DenseMatrix64F diffMassMatrix = new DenseMatrix64F(nDoFs, nDoFs);

      int nIterations = 10000;
      for (int i = 0; i < nIterations; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);
         elevator.updateFramesRecursively();

         for (int j = 0; j < massMatrixCalculators.size(); j++)
         {
            massMatrixCalculators.get(j).compute();
            massMatrices.set(j, massMatrixCalculators.get(j).getMassMatrix());

            if (j > 0)
            {
               CommonOps.subtract(massMatrices.get(j), massMatrices.get(j - 1), diffMassMatrix);

               double[] data = diffMassMatrix.getData();
               for (int k = 0; k < data.length; k++)
               {
                  assertEquals(0.0, data[k], eps);
               }
            }
         }
      }

   }

}
