package us.ihmc.atlas;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2NodeInterface;

import java.io.File;

public class AtlasLineDetectionDemo
{
   private ROS2NodeInterface ros2Node;

   public void execFootstepPlan()
   {
      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      FootstepPlannerLogLoader.LoadResult loadresult = logLoader.load(new File("/home/quantum/.ihmc/logs/20200821_172044151_FootstepPlannerLog/"));
      if (!(loadresult == FootstepPlannerLogLoader.LoadResult.LOADED))
      {
         LogTools.error("Couldn't find file");
         return;
      }
      FootstepPlannerLog log = logLoader.getLog();
      FootstepPlanningToolboxOutputStatus statusPacket = log.getStatusPacket();
      FootstepDataListMessage footstepDataList = statusPacket.getFootstepDataList();

      String robotName = "Atlas";

      IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher
            = ROS2Tools.createPublisherTypeNamed(ros2Node, FootstepDataListMessage.class, ROS2Tools.getControllerInputTopic(robotName));
      footstepDataListPublisher.publish(footstepDataList);
   }
}