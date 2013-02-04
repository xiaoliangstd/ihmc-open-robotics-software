package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolver;

import us.ihmc.commonWalkingControlModules.WrenchDistributorTools;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.FootPolygonVisualizer;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCoPAndCMPControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionMomentControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributor;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributorInputData;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributorOutputData;
import us.ihmc.commonWalkingControlModules.controlModules.NoLungingDesiredCoPAndCMPControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.SacrificeDeltaCMPDesiredCoPAndCMPControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.CapturabilityBasedDesiredCoPVisualizer;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.Updatable;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumSolver;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAccelerationData;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.DampedLeastSquaresSolver;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.math.geometry.OriginAndPointFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.MomentumCalculator;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TotalWrenchCalculator;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.AxisAngleOrientationController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoFrameVector;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;

public abstract class ICPAndMomentumBasedController implements RobotController
{
   private static final long serialVersionUID = -7013956504623280825L;

   private final String name = getClass().getSimpleName();
   protected final YoVariableRegistry registry = new YoVariableRegistry(name);

   protected final MomentumSolver solver;

   protected final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   protected final ReferenceFrame elevatorFrame;
   private final ReferenceFrame centerOfMassFrame;

   protected final FullRobotModel fullRobotModel;
   private final CenterOfMassJacobian centerOfMassJacobian;
   protected final CommonWalkingReferenceFrames referenceFrames;
   protected final TwistCalculator twistCalculator;
   protected final SpatialAccelerationCalculator spatialAccelerationCalculator;
   protected final List<ContactablePlaneBody> contactablePlaneBodies;
   protected final SideDependentList<? extends ContactablePlaneBody> bipedFeet;
   private final HashMap<ContactablePlaneBody, YoFramePoint> centersOfPressure = new HashMap<ContactablePlaneBody, YoFramePoint>();
   protected final HashMap<ContactablePlaneBody, YoFramePoint2d> centersOfPressure2d = new HashMap<ContactablePlaneBody, YoFramePoint2d>();
   protected final LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, YoPlaneContactState>();
   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();


   protected final DoubleYoVariable yoTime;
   protected final double controlDT;
   protected final double gravity;


   // TODO: move to subclasses:
   protected final YoFrameOrientation desiredPelvisOrientation;
   private final AxisAngleOrientationController pelvisOrientationController;
   private final YoFrameVector desiredPelvisAngularAcceleration;

   protected final SideDependentList<FootSwitchInterface> footSwitches;
   protected final BipedSupportPolygons bipedSupportPolygons;
   protected final YoFramePoint2d desiredICP;
   protected final YoFrameVector2d desiredICPVelocity;
   protected final EnumYoVariable<RobotSide> supportLeg;
   protected final EnumYoVariable<RobotSide> upcomingSupportLeg;
   protected final DoubleYoVariable desiredCoMHeightAcceleration;
   protected final YoFramePoint capturePoint;
   protected final DoubleYoVariable omega0;
   private final DesiredCoPAndCMPControlModule desiredCoPAndCMPControlModule;
   private final DoubleYoVariable alphaFz = new DoubleYoVariable("alphaFz", registry);
   private final AlphaFilteredYoVariable fZ = new AlphaFilteredYoVariable("fZ", registry, alphaFz);
   private final GroundReactionMomentControlModule groundReactionMomentControlModule;
   private final CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();
   private final OriginAndPointFrame copToCoPFrame = new OriginAndPointFrame("copToCoP", worldFrame);
   private final MomentumCalculator momentumCalculator;


   private final YoFrameVector finalDesiredPelvisLinearAcceleration;
   private final YoFrameVector finalDesiredPelvisAngularAcceleration;
   private final YoFrameVector desiredPelvisForce;
   private final YoFrameVector desiredPelvisTorque;

   private final DoubleYoVariable alphaGroundReactionWrench = new DoubleYoVariable("alphaGroundReactionWrench", registry);
   private final YoFrameVector unfilteredDesiredGroundReactionTorque;
   private final YoFrameVector unfilteredDesiredGroundReactionForce;
   private final AlphaFilteredYoFrameVector desiredGroundReactionTorque;
   private final AlphaFilteredYoFrameVector desiredGroundReactionForce;
   private final YoFrameVector admissibleDesiredGroundReactionTorque;
   private final YoFrameVector admissibleDesiredGroundReactionForce;
   private final YoFrameVector groundReactionTorqueCheck;
   private final YoFrameVector groundReactionForceCheck;

   private final HashMap<RevoluteJoint, DoubleYoVariable> desiredAccelerationYoVariables = new HashMap<RevoluteJoint, DoubleYoVariable>();
   private final SpatialForceVector gravitationalWrench;

   private final ProcessedOutputsInterface processedOutputs;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final double gravityZ;
   private final double totalMass;

   private final GroundReactionWrenchDistributor groundReactionWrenchDistributor;

   private final boolean doStrictPelvisControl;    // TODO: remove, just have momentumSubspace and accelerationSubspace
   private final DenseMatrix64F momentumSubspace;
   private final DenseMatrix64F accelerationSubspace;

   public ICPAndMomentumBasedController(FullRobotModel fullRobotModel, CenterOfMassJacobian centerOfMassJacobian, CommonWalkingReferenceFrames referenceFrames,
           DoubleYoVariable yoTime, double gravityZ, TwistCalculator twistCalculator, SideDependentList<? extends ContactablePlaneBody> bipedFeet,
           BipedSupportPolygons bipedSupportPolygons, double controlDT, ProcessedOutputsInterface processedOutputs,
           SideDependentList<FootSwitchInterface> footSwitches, GroundReactionWrenchDistributor groundReactionWrenchDistributor,
           ArrayList<Updatable> updatables, boolean doStrictPelvisControl, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      LinearSolver<DenseMatrix64F> jacobianSolver = createJacobianSolver();
      this.solver = new MomentumSolver(rootJoint, rootJoint.getPredecessor(), centerOfMassFrame, twistCalculator, jacobianSolver, controlDT, registry);

      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.fullRobotModel = fullRobotModel;
      this.centerOfMassJacobian = centerOfMassJacobian;
      this.referenceFrames = referenceFrames;
      this.twistCalculator = twistCalculator;
      this.bipedFeet = bipedFeet;
      this.contactablePlaneBodies = new ArrayList<ContactablePlaneBody>(bipedFeet.values());
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.controlDT = controlDT;
      this.footSwitches = footSwitches;
      this.gravity = gravityZ;
      this.yoTime = yoTime;

      RigidBody elevator = fullRobotModel.getElevator();
      this.spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, twistCalculator, gravity, true);

      this.processedOutputs = processedOutputs;
      this.gravityZ = gravityZ;
      this.momentumCalculator = new MomentumCalculator(twistCalculator);
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);

      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      this.groundReactionMomentControlModule = new GroundReactionMomentControlModule(pelvisFrame, registry);
      this.groundReactionMomentControlModule.setGains(10.0, 100.0);    // kPelvisYaw was 0.0 for M3 movie
      this.totalMass = TotalMassCalculator.computeSubTreeMass(elevator);

      this.groundReactionWrenchDistributor = groundReactionWrenchDistributor;
      this.desiredPelvisOrientation = new YoFrameOrientation("desiredPelvis", worldFrame, registry);

      this.doStrictPelvisControl = doStrictPelvisControl;

      if (doStrictPelvisControl)
      {
         this.pelvisOrientationController = new AxisAngleOrientationController("pelvis", pelvisFrame, registry);
         pelvisOrientationController.setProportionalGains(100.0, 100.0, 100.0);    // 100.0, 100.0, 100.0);
         pelvisOrientationController.setDerivativeGains(20.0, 20.0, 20.0);    // 20.0, 20.0, 20.0);
      }
      else
      {
         this.pelvisOrientationController = null;
      }

      omega0 = new DoubleYoVariable("omega0", registry);
      capturePoint = new YoFramePoint("capturePoint", worldFrame, registry);

      this.desiredPelvisAngularAcceleration = new YoFrameVector("desiredPelvisAngularAcceleration", pelvisFrame, registry);
      this.finalDesiredPelvisLinearAcceleration = new YoFrameVector("finalDesiredPelvisLinearAcceleration", "", pelvisFrame, registry);
      this.finalDesiredPelvisAngularAcceleration = new YoFrameVector("finalDesiredPelvisAngularAcceleration", "", pelvisFrame, registry);
      this.desiredPelvisForce = new YoFrameVector("desiredPelvisForce", "", centerOfMassFrame, registry);
      this.desiredPelvisTorque = new YoFrameVector("desiredPelvisTorque", "", centerOfMassFrame, registry);

      this.unfilteredDesiredGroundReactionTorque = new YoFrameVector("unfilteredDesiredGroundReactionTorque", centerOfMassFrame, registry);
      this.unfilteredDesiredGroundReactionForce = new YoFrameVector("unfilteredDesiredGroundReactionForce", centerOfMassFrame, registry);

      if (doStrictPelvisControl)
      {
//       alphaFz.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(7.0, controlDT));
         alphaGroundReactionWrench.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(7.0, controlDT));
      }

      this.desiredGroundReactionTorque = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("desiredGroundReactionTorque", "", registry,
              alphaGroundReactionWrench, unfilteredDesiredGroundReactionTorque);
      this.desiredGroundReactionForce = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("desiredGroundReactionForce", "", registry,
              alphaGroundReactionWrench, unfilteredDesiredGroundReactionForce);

      this.admissibleDesiredGroundReactionTorque = new YoFrameVector("admissibleDesiredGroundReactionTorque", centerOfMassFrame, registry);
      this.admissibleDesiredGroundReactionForce = new YoFrameVector("admissibleDesiredGroundReactionForce", centerOfMassFrame, registry);

      this.groundReactionTorqueCheck = new YoFrameVector("groundReactionTorqueCheck", centerOfMassFrame, registry);
      this.groundReactionForceCheck = new YoFrameVector("groundReactionForceCheck", centerOfMassFrame, registry);

      for (ContactablePlaneBody contactableBody : bipedFeet)
      {
         String copName = contactableBody.getRigidBody().getName() + "CoP";
         String listName = "cops";

         YoFramePoint cop = new YoFramePoint(copName, worldFrame, registry);
         centersOfPressure.put(contactableBody, cop);

         YoFramePoint2d cop2d = new YoFramePoint2d(copName + "2d", "", contactableBody.getPlaneFrame(), registry);
         centersOfPressure2d.put(contactableBody, cop2d);

         DynamicGraphicPosition copViz = cop.createDynamicGraphicPosition(copName, 0.005, YoAppearance.Navy(), GraphicType.BALL);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(listName, copViz);
         dynamicGraphicObjectsListRegistry.registerArtifact(listName, copViz.createArtifact());
      }

      DynamicGraphicPosition capturePointViz = capturePoint.createDynamicGraphicPosition("Capture Point", 0.01, YoAppearance.Blue(), GraphicType.ROTATED_CROSS);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("Capture Point", capturePointViz);
      dynamicGraphicObjectsListRegistry.registerArtifact("Capture Point", capturePointViz.createArtifact());

      if (updatables != null)
      {
         this.updatables.addAll(updatables);
      }

      this.desiredCoMHeightAcceleration = new DoubleYoVariable("desiredCoMHeightAcceleration", registry);

      desiredICP = new YoFramePoint2d("desiredICP", "", ReferenceFrame.getWorldFrame(), registry);
      desiredICPVelocity = new YoFrameVector2d("desiredICPVelocity", "", ReferenceFrame.getWorldFrame(), registry);

      supportLeg = EnumYoVariable.create("supportLeg", "", RobotSide.class, registry, true);
      upcomingSupportLeg = EnumYoVariable.create("upcomingSupportLeg", "", RobotSide.class, registry, true);

      elevatorFrame = fullRobotModel.getElevatorFrame();

      for (ContactablePlaneBody contactablePlaneBody : bipedFeet)
      {
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();
         YoPlaneContactState contactState = new YoPlaneContactState(rigidBody.getName(), contactablePlaneBody.getPlaneFrame(), registry);
         contactState.setContactPoints(contactablePlaneBody.getContactPoints2d());    // initialize with flat 'feet'
         contactStates.put(contactablePlaneBody, contactState);
      }

      InverseDynamicsJoint[] joints = ScrewTools.computeJointsInOrder(elevator);
      for (InverseDynamicsJoint joint : joints)
      {
         if (joint instanceof RevoluteJoint)
         {
            desiredAccelerationYoVariables.put((RevoluteJoint) joint, new DoubleYoVariable(joint.getName() + "qdd_d", registry));
         }
      }

      updateBipedSupportPolygons(bipedSupportPolygons);

      CapturabilityBasedDesiredCoPVisualizer visualizer = new CapturabilityBasedDesiredCoPVisualizer(registry, dynamicGraphicObjectsListRegistry);

      if (doStrictPelvisControl)
      {
         NoLungingDesiredCoPAndCMPControlModule desiredCoPAndCMPControlModule = new NoLungingDesiredCoPAndCMPControlModule(visualizer, bipedSupportPolygons,
                                                                                   controlDT, registry);
         desiredCoPAndCMPControlModule.setGains(1.5, 1000.0);    // 7.0);
         this.desiredCoPAndCMPControlModule = desiredCoPAndCMPControlModule;
      }
      else
      {
         SacrificeDeltaCMPDesiredCoPAndCMPControlModule desiredCoPAndCMPControlModule = new SacrificeDeltaCMPDesiredCoPAndCMPControlModule(visualizer,
                                                                                           bipedSupportPolygons,
                                                                                           fullRobotModel.getPelvis().getBodyFixedFrame(), controlDT, registry);
         desiredCoPAndCMPControlModule.setGains(3e-2, 1.0, 1.5, 15.0);
         this.desiredCoPAndCMPControlModule = desiredCoPAndCMPControlModule;
      }

      gravitationalWrench = new SpatialForceVector(centerOfMassFrame, new Vector3d(0.0, 0.0, totalMass * gravityZ), new Vector3d());

      omega0.set(3.39);    // FIXME: hack to resolve circularity
      computeCapturePoint();


      this.updatables.add(new FootPolygonVisualizer(contactStates.values(), dynamicGraphicObjectsListRegistry, registry));

      momentumSubspace = new DenseMatrix64F(SpatialForceVector.SIZE, 3);
      momentumSubspace.set(3, 0, 1.0);
      momentumSubspace.set(4, 1, 1.0);
      momentumSubspace.set(5, 2, 1.0);

      accelerationSubspace = new DenseMatrix64F(SpatialMotionVector.SIZE, 3);
      accelerationSubspace.set(0, 0, 1.0);
      accelerationSubspace.set(1, 1, 1.0);
      accelerationSubspace.set(2, 2, 1.0);
   }


   private static LinearSolver<DenseMatrix64F> createJacobianSolver()
   {
      DampedLeastSquaresSolver jacobianSolver = new DampedLeastSquaresSolver(SpatialMotionVector.SIZE);
      jacobianSolver.setAlpha(5e-2);

      return jacobianSolver;
   }

   protected void setExternalHandWrench(RobotSide robotSide, Wrench handWrench)
   {
      inverseDynamicsCalculator.setExternalWrench(fullRobotModel.getHand(robotSide), handWrench);
   }

   public final void doControl()
   {
      updateBipedSupportPolygons(bipedSupportPolygons);

      callUpdatables();

      inverseDynamicsCalculator.reset();
      solver.reset();

      doMotionControl();

      solver.compute();

      SpatialForceVector totalGroundReactionWrench = computeDesiredTotalGroundReactionWrench();

      SpatialForceVector desiredCentroidalMomentumRate = new SpatialForceVector(totalGroundReactionWrench);
      desiredCentroidalMomentumRate.sub(gravitationalWrench);


      if (doStrictPelvisControl)
      {
         DenseMatrix64F momentumMultipliers = new DenseMatrix64F(3, 1);
         MatrixTools.setDenseMatrixFromTuple3d(momentumMultipliers, desiredCentroidalMomentumRate.getLinearPartCopy(), 0, 0);

         DenseMatrix64F accelerationMultipliers = new DenseMatrix64F(3, 1);
         FrameVector pelvisAngularAcceleration = computeDesiredPelvisAngularAcceleration();
         this.desiredPelvisAngularAcceleration.set(pelvisAngularAcceleration);
         pelvisAngularAcceleration.changeFrame(fullRobotModel.getPelvis().getBodyFixedFrame());
         MatrixTools.setDenseMatrixFromTuple3d(accelerationMultipliers, pelvisAngularAcceleration.getVector(), 0, 0);

         RootJointAccelerationData rootJointAccelerationData = new RootJointAccelerationData();
         rootJointAccelerationData.setAccelerationSubspace(accelerationSubspace);
         rootJointAccelerationData.setAccelerationMultipliers(accelerationMultipliers);

         MomentumRateOfChangeData momentumRateOfChangeData = new MomentumRateOfChangeData();
         momentumRateOfChangeData.setMomentumSubspace(momentumSubspace);
         momentumRateOfChangeData.setMomentumMultipliers(momentumMultipliers);

         solver.solve(accelerationSubspace, accelerationMultipliers, momentumSubspace, momentumMultipliers);
         solver.getRateOfChangeOfMomentum(totalGroundReactionWrench);

         totalGroundReactionWrench.add(gravitationalWrench);
      }

      unfilteredDesiredGroundReactionTorque.set(totalGroundReactionWrench.getAngularPartCopy());
      unfilteredDesiredGroundReactionForce.set(totalGroundReactionWrench.getLinearPartCopy());
      desiredGroundReactionTorque.update();
      desiredGroundReactionForce.update();
      totalGroundReactionWrench.setAngularPart(desiredGroundReactionTorque.getFrameVectorCopy().getVector());
      totalGroundReactionWrench.setLinearPart(desiredGroundReactionForce.getFrameVectorCopy().getVector());

      GroundReactionWrenchDistributorInputData groundReactionWrenchDistributorInputData = new GroundReactionWrenchDistributorInputData();

      double coefficientOfFriction = 1.0;    // 0.5;    // TODO
      double rotationalCoefficientOfFriction = 0.5;    // TODO

      groundReactionWrenchDistributorInputData.reset();

      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
      {
         PlaneContactState contactState = contactStates.get(contactablePlaneBody);

         List<FramePoint> footContactPoints = contactState.getContactPoints();

         if (footContactPoints.size() > 0)
         {
            groundReactionWrenchDistributorInputData.addContact(contactState, coefficientOfFriction, rotationalCoefficientOfFriction);
         }
      }

      groundReactionWrenchDistributorInputData.setSpatialForceVectorAndUpcomingSupportSide(totalGroundReactionWrench, upcomingSupportLeg.getEnumValue());

      groundReactionWrenchDistributor.solve(groundReactionWrenchDistributorInputData);
      GroundReactionWrenchDistributorOutputData distributedWrench = groundReactionWrenchDistributor.getSolution();

      List<Wrench> wrenches = new ArrayList<Wrench>();
      List<FramePoint2d> cops = new ArrayList<FramePoint2d>();

      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
      {
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();
         PlaneContactState contactState = contactStates.get(contactablePlaneBody);
         List<FramePoint> footContactPoints = contactState.getContactPoints();

         if (footContactPoints.size() > 0)
         {
            FrameVector force = distributedWrench.getForce(contactState);
            FramePoint2d cop = distributedWrench.getCenterOfPressure(contactState);
            double normalTorque = distributedWrench.getNormalTorque(contactState);
           
            centersOfPressure2d.get(contactablePlaneBody).set(cop);

            cops.add(cop);
            FramePoint cop3d = cop.toFramePoint();
            cop3d.changeFrame(worldFrame);
            centersOfPressure.get(contactablePlaneBody).set(cop3d);
            Wrench groundReactionWrench = new Wrench(rigidBody.getBodyFixedFrame(), contactState.getPlaneFrame());
            WrenchDistributorTools.computeWrench(groundReactionWrench, force, cop, normalTorque);
            groundReactionWrench.changeFrame(rigidBody.getBodyFixedFrame());
            wrenches.add(groundReactionWrench);
            inverseDynamicsCalculator.setExternalWrench(rigidBody, groundReactionWrench);
         }
         else
         {
            centersOfPressure.get(contactablePlaneBody).setToNaN();
         }
      }

      Wrench admissibleGroundReactionWrench = TotalWrenchCalculator.computeTotalWrench(wrenches, totalGroundReactionWrench.getExpressedInFrame());
      admissibleDesiredGroundReactionTorque.set(admissibleGroundReactionWrench.getAngularPartCopy());
      admissibleDesiredGroundReactionForce.set(admissibleGroundReactionWrench.getLinearPartCopy());
      this.omega0.set(computeOmega0(cops, admissibleGroundReactionWrench));

      desiredCentroidalMomentumRate.set(admissibleGroundReactionWrench);
      desiredCentroidalMomentumRate.sub(gravitationalWrench);

      solver.solve(desiredCentroidalMomentumRate);

      SpatialForceVector groundReactionWrenchCheck = inverseDynamicsCalculator.computeTotalExternalWrench(centerOfMassFrame);
      groundReactionTorqueCheck.set(groundReactionWrenchCheck.getAngularPartCopy());
      groundReactionForceCheck.set(groundReactionWrenchCheck.getLinearPartCopy());

      inverseDynamicsCalculator.compute();
      if (processedOutputs != null)
         fullRobotModel.setTorques(processedOutputs);
      updateYoVariables();
   }


   private SpatialForceVector computeDesiredTotalGroundReactionWrench()
   {
      Momentum momentum = computeCentroidalMomentum();
      ReferenceFrame frame = worldFrame;
      RobotSide supportLeg = this.supportLeg.getEnumValue();

      FrameOrientation desiredPelvisOrientation = this.desiredPelvisOrientation.getFrameOrientationCopy();
      desiredCoPAndCMPControlModule.compute(capturePoint.getFramePoint2dCopy(), supportLeg, desiredICP.getFramePoint2dCopy(),
              desiredICPVelocity.getFrameVector2dCopy(), desiredPelvisOrientation, omega0.getDoubleValue(), momentum);
      FramePoint2d desiredCoP2d = new FramePoint2d(worldFrame);
      desiredCoPAndCMPControlModule.packCoP(desiredCoP2d);
      FramePoint2d desiredCMP2d = new FramePoint2d(worldFrame);
      desiredCoPAndCMPControlModule.packCMP(desiredCMP2d);

      desiredCoP2d.changeFrame(frame);
      GeometryTools.projectOntoPolygonAndCheckDistance(desiredCoP2d, bipedSupportPolygons.getSupportPolygonInMidFeetZUp(), 1e-10);    // fix numerical roundoff

      desiredCMP2d.changeFrame(frame);
      FrameVector2d desiredDeltaCMP = new FrameVector2d(desiredCMP2d);
      desiredDeltaCMP.sub(desiredCoP2d);

      this.fZ.update(computeFz());
      FrameVector totalgroundReactionMoment = groundReactionMomentControlModule.determineGroundReactionMoment(momentum,
                                                 desiredPelvisOrientation.getYawPitchRoll()[0]);

      SpatialForceVector totalGroundReactionWrench = computeTotalGroundReactionWrench(desiredCoP2d, desiredCMP2d, totalgroundReactionMoment,
                                                        fZ.getDoubleValue(), omega0.getDoubleValue());
      totalGroundReactionWrench.changeFrame(centerOfMassFrame);

      return totalGroundReactionWrench;
   }


   private FrameVector computeDesiredPelvisAngularAcceleration()
   {
      Twist pelvisTwist = new Twist();
      twistCalculator.packTwistOfBody(pelvisTwist, fullRobotModel.getPelvis());
      ReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      FrameOrientation desiredPelvisOrientation = this.desiredPelvisOrientation.getFrameOrientationCopy();
      FrameVector desiredPelvisAngularVelocity = new FrameVector(pelvisFrame);
      FrameVector currentPelvisAngularVelocity = new FrameVector(pelvisTwist.getExpressedInFrame(), pelvisTwist.getAngularPartCopy());
      FrameVector feedForwardPelvisAcceleration = new FrameVector(pelvisFrame);
      FrameVector pelvisAngularAcceleration = new FrameVector(pelvisFrame);
      pelvisOrientationController.compute(pelvisAngularAcceleration, desiredPelvisOrientation, desiredPelvisAngularVelocity, currentPelvisAngularVelocity,
              feedForwardPelvisAcceleration);

      return pelvisAngularAcceleration;
   }


   private double computeOmega0(List<FramePoint2d> cop2ds, Wrench totalGroundReactionWrenchAfterProjection)
   {
      totalGroundReactionWrenchAfterProjection.changeFrame(centerOfMassFrame);
      double fz = totalGroundReactionWrenchAfterProjection.getLinearPartCopy().getZ();

      double deltaZ;
      if (cop2ds.size() == 1)
      {
         FramePoint cop = cop2ds.get(0).toFramePoint();
         cop.changeFrame(centerOfMassFrame);
         deltaZ = -cop.getZ();
      }
      else    // assume 2 CoPs
      {
         List<FramePoint> cops = new ArrayList<FramePoint>(cop2ds.size());
         for (FramePoint2d cop2d : cop2ds)
         {
            FramePoint cop = cop2d.toFramePoint();
            cop.changeFrame(copToCoPFrame.getParent());
            cops.add(cop);
         }

         copToCoPFrame.setOriginAndPositionToPointAt(cops.get(0), cops.get(1));
         copToCoPFrame.update();
         FramePoint2d pseudoCoP2d = new FramePoint2d(copToCoPFrame);
         centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(pseudoCoP2d, totalGroundReactionWrenchAfterProjection, copToCoPFrame);
         FramePoint pseudoCoP = pseudoCoP2d.toFramePoint();
         pseudoCoP.changeFrame(centerOfMassFrame);
         deltaZ = -pseudoCoP.getZ();
      }

      return Math.sqrt(fz / (totalMass * deltaZ));
   }

   private SpatialForceVector computeTotalGroundReactionWrench(FramePoint2d desiredCoP2d, FramePoint2d desiredCMP2d, FrameVector totalgroundReactionMoment,
           double fZ, double omega0)
   {
      FramePoint com = new FramePoint(centerOfMassFrame);
      double zCoP = com.getZ() - fZ / (totalMass * MathTools.square(omega0));    // FIXME: hack to resolve circularity

      FramePoint desiredCoP = desiredCoP2d.toFramePoint();
      desiredCoP.changeFrame(com.getReferenceFrame());
      desiredCoP.setZ(zCoP);

      FramePoint desiredCMP = desiredCMP2d.toFramePoint();
      desiredCMP.changeFrame(com.getReferenceFrame());
      desiredCMP.setZ(zCoP);

      FrameVector force = new FrameVector(com);
      force.sub(desiredCMP);
      force.scale(fZ / force.getZ());
      FrameVector momentArm = new FrameVector(desiredCoP);
      momentArm.sub(com);

      SpatialForceVector totalGroundReactionWrench = SpatialForceVector.createUsingArm(centerOfMassFrame, force.getVector(), momentArm.getVector());
      totalGroundReactionWrench.addAngularPart(totalgroundReactionMoment.getVector());

      return totalGroundReactionWrench;
   }

   public abstract void doMotionControl();

   private Momentum computeCentroidalMomentum()
   {
      Momentum momentum = new Momentum(centerOfMassFrame);
      momentumCalculator.computeAndPack(momentum);

      return momentum;
   }

   private void callUpdatables()
   {
      double time = yoTime.getDoubleValue();
      for (Updatable updatable : updatables)
      {
         updatable.update(time);
      }
   }

   protected ReferenceFrame getHandFrame(RobotSide robotSide)
   {
      return fullRobotModel.getHand(robotSide).getBodyFixedFrame();
   }

   public void addUpdatable(Updatable updatable)
   {
      updatables.add(updatable);
   }

   protected void doPDControl(OneDoFJoint[] joints, double k, double d)
   {
      for (OneDoFJoint joint : joints)
      {
         doPDControl(joint, k, d);
      }
   }

   protected void doPDControl(OneDoFJoint joint, double k, double d)
   {
      double desiredAcceleration = computeDesiredAcceleration(k, d, 0.0, 0.0, joint);
      DenseMatrix64F jointAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
      jointAcceleration.set(0, 0, desiredAcceleration);
      solver.setDesiredJointAcceleration(joint, jointAcceleration);
   }

   protected static double computeDesiredAcceleration(double k, double d, double qDesired, double qdDesired, OneDoFJoint joint)
   {
      return k * (qDesired - joint.getQ()) + d * (qdDesired - joint.getQd());
   }

   protected void computeCapturePoint()
   {
      FramePoint centerOfMass = computeCenterOfMass();
      FrameVector centerOfMassVelocity = computeCenterOfMassVelocity();
      FramePoint2d capturePoint = CapturePointCalculator.computeCapturePoint(centerOfMass, centerOfMassVelocity, omega0.getDoubleValue());
      capturePoint.changeFrame(this.capturePoint.getReferenceFrame());
      this.capturePoint.set(capturePoint.getX(), capturePoint.getY(), 0.0);
   }

   private FramePoint computeCenterOfMass()
   {
      return new FramePoint(referenceFrames.getCenterOfMassFrame());
   }

   private FrameVector computeCenterOfMassVelocity()
   {
      centerOfMassJacobian.compute();
      FrameVector ret = new FrameVector(ReferenceFrame.getWorldFrame());
      centerOfMassJacobian.packCenterOfMassVelocity(ret);

      return ret;
   }

   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }

   public ContactState getContactState(RigidBody rigidBody)
   {
      return contactStates.get(rigidBody);
   }

   private double computeFz()
   {
      double fZ = totalMass * (gravityZ + desiredCoMHeightAcceleration.getDoubleValue());    // TODO: don't rely on DoubleYoVariable being set

      return fZ;
   }

   private void updateYoVariables()
   {
      SpatialAccelerationVector pelvisAcceleration = new SpatialAccelerationVector();
      fullRobotModel.getRootJoint().packDesiredJointAcceleration(pelvisAcceleration);

      finalDesiredPelvisAngularAcceleration.checkReferenceFrameMatch(pelvisAcceleration.getExpressedInFrame());
      finalDesiredPelvisAngularAcceleration.set(pelvisAcceleration.getAngularPartCopy());

      finalDesiredPelvisLinearAcceleration.checkReferenceFrameMatch(pelvisAcceleration.getExpressedInFrame());
      finalDesiredPelvisLinearAcceleration.set(pelvisAcceleration.getLinearPartCopy());

      Wrench pelvisJointWrench = new Wrench();
      fullRobotModel.getRootJoint().packWrench(pelvisJointWrench);
      pelvisJointWrench.changeFrame(referenceFrames.getCenterOfMassFrame());
      desiredPelvisForce.set(pelvisJointWrench.getLinearPartCopy());
      desiredPelvisTorque.set(pelvisJointWrench.getAngularPartCopy());

      for (RevoluteJoint joint : desiredAccelerationYoVariables.keySet())
      {
         desiredAccelerationYoVariables.get(joint).set(joint.getQddDesired());
      }
   }

   protected void updateBipedSupportPolygons(BipedSupportPolygons bipedSupportPolygons)
   {
      SideDependentList<List<FramePoint>> footContactPoints = new SideDependentList<List<FramePoint>>();
      for (RobotSide robotSide : RobotSide.values())
      {
         footContactPoints.put(robotSide, contactStates.get(bipedFeet.get(robotSide)).getContactPoints());
      }

      bipedSupportPolygons.update(footContactPoints);
   }

   public void initialize()
   {
      solver.initialize();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }
}
