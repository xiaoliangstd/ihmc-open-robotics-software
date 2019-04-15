package us.ihmc.humanoidBehaviors.tools;

import controller_msgs.msg.dds.FlatGroundPlanarRegionParametersMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import org.apache.poi.ss.formula.functions.T;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2ModuleIdentifier;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidBehaviors.tools.thread.ExceptionPrintingThreadScheduler;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.TimeUnit;

/**
 * Supply flat ground regions to REA. TODO: Add reset functionality and/or sizing and positioning
 */
public class FlatGroundPlanarRegionPublisher
{
   public static final ROS2ModuleIdentifier ROS2_ID = new ROS2ModuleIdentifier("flat_ground_planar_region_publisher",
                                                                               ROS2Tools.FLAT_GROUND_REGION_PUBLISHER);
   public static final int FLAT_GROUND_REGION_ID = 1996; // 7 = yellow, 6 = green

   private final ExceptionPrintingThreadScheduler scheduler;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> publisher;

   private final PlanarRegionsListMessage flatRegionMessage;
   private final PlanarRegionsListMessage emptyRegionMessage; // empty region required to clear
   private PlanarRegionsListMessage regionMessageToPublish;

   public FlatGroundPlanarRegionPublisher()
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2_ID.getNodeName());

      flatRegionMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(new PlanarRegionsList(createFlatGroundRegion()));
      emptyRegionMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(new PlanarRegionsList(createEmptyRegion()));
      regionMessageToPublish = emptyRegionMessage;

      scheduler = new ExceptionPrintingThreadScheduler(getClass().getSimpleName());

      new ROS2Callback<>(ros2Node, FlatGroundPlanarRegionParametersMessage.class, null, ROS2_ID, this::acceptParameters);

      publisher = new IHMCROS2Publisher<>(ros2Node,
                                          PlanarRegionsListMessage.class,
                                          null,
                                          LIDARBasedREAModule.ROS2_ID.qualifyMore(LIDARBasedREAModule.CUSTOM_REGION_QUALIFIER));
      scheduler.schedule(this::publish, 1, TimeUnit.SECONDS);
   }

   private void acceptParameters(FlatGroundPlanarRegionParametersMessage message)
   {
      if (message.getEnable())
      {
         regionMessageToPublish = flatRegionMessage;
      }
      else
      {
         regionMessageToPublish = emptyRegionMessage;
      }
   }

   private void publish()
   {
      publisher.publish(regionMessageToPublish);
   }

   private PlanarRegion createFlatGroundRegion()
   {
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();  // start with a flat ground region
      convexPolygon.addVertex(10.0, 10.0);
      convexPolygon.addVertex(-10.0, 10.0);
      convexPolygon.addVertex(-10.0, -10.0);
      convexPolygon.addVertex(10.0, -10.0);
      convexPolygon.update();
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      transformToWorld.setTranslationZ(-0.0001);
      PlanarRegion planarRegion = new PlanarRegion(transformToWorld, convexPolygon);
      planarRegion.setRegionId(FLAT_GROUND_REGION_ID);
      return planarRegion;
   }

   private PlanarRegion createEmptyRegion()
   {
      PlanarRegion planarRegion = new PlanarRegion();
      planarRegion.setRegionId(FLAT_GROUND_REGION_ID);
      return planarRegion;
   }

   public void shutdown()
   {
      scheduler.shutdown();
   }
}