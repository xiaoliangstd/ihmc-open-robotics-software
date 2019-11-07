package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepEdge;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;

public class StagePlannerListener implements BipedalFootstepPlannerListener
{
   private final FootstepNodeSnapper snapper;

   private final HashSet<PlannerCell> occupancyMapCellsThisTick = new HashSet<>();
   private final ConcurrentSet<PlannerCell> occupancyMapCellsSinceLastReportReference = new ConcurrentSet<>();
   private final ConcurrentSet<FootstepNode> expandedNodesSinceLastReportReference = new ConcurrentSet<>();

   private final ConcurrentList<PlannerNodeData> fullGraphToReportReference = new ConcurrentList<>();

   private final List<FootstepNode> lowestCostPlan = new ArrayList<>();

   private final HashMap<FootstepNode, List<PlannerNodeData>> rejectedNodeData = new HashMap<>();

   private final ConcurrentCopier<PlannerNodeDataList> concurrentLowestCostNodeDataList = new ConcurrentCopier<>(PlannerNodeDataList::new);

   private final AtomicBoolean hasOccupiedCells = new AtomicBoolean(false);
   private final AtomicBoolean hasExpandedNodes = new AtomicBoolean(false);
   private final AtomicBoolean hasFullGraph = new AtomicBoolean(false);
   private final AtomicBoolean hasLowestCostPlan = new AtomicBoolean(false);

   private final long occupancyMapUpdateDt;
   private long lastUpdateTime = -1;
   private final EnumMap<BipedalFootstepPlannerNodeRejectionReason, MutableInt> rejectionCount = new EnumMap<>(BipedalFootstepPlannerNodeRejectionReason.class);
   private int totalNodeCount = 0;

   public StagePlannerListener(FootstepNodeSnapper snapper, long occupancyMapUpdateDt)
   {
      this.snapper = snapper;
      this.occupancyMapUpdateDt = occupancyMapUpdateDt;

      for(BipedalFootstepPlannerNodeRejectionReason rejectionReason : BipedalFootstepPlannerNodeRejectionReason.values)
      {
         rejectionCount.put(rejectionReason, new MutableInt(0));
      }
   }

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {
      if (previousNode == null)
         reset();

      node.setNodeIndex(totalNodeCount);

      occupancyMapCellsThisTick.add(new PlannerCell(node.getXIndex(), node.getYIndex()));
      totalNodeCount++;
   }

   public void reset()
   {
      rejectedNodeData.clear();

      occupancyMapCellsSinceLastReportReference.clear();
      expandedNodesSinceLastReportReference.clear();
      fullGraphToReportReference.clear();

      lowestCostPlan.clear();
      totalNodeCount = 0;
      rejectionCount.values().forEach(count -> count.setValue(0));
   }

   @Override
   public void reportLowestCostNodeList(List<FootstepNode> plan)
   {
      lowestCostPlan.clear();
      lowestCostPlan.addAll(plan);
   }

   @Override
   public void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      RigidBodyTransform nodePose = snapper.snapFootstepNode(rejectedNode).getOrComputeSnappedNodeTransform(rejectedNode);
      PlannerNodeData nodeData = new PlannerNodeData(parentNode.getNodeIndex(), rejectedNode, nodePose, reason);

      if (rejectedNodeData.get(parentNode) == null)
         rejectedNodeData.put(parentNode, new ArrayList<>());
      rejectedNodeData.get(parentNode).add(nodeData);
      rejectionCount.get(reason).increment();
   }

   @Override
   public void tickAndUpdate()
   {
      long currentTime = System.currentTimeMillis();

      if (lastUpdateTime == -1)
         lastUpdateTime = currentTime;

      boolean isTimeForUpdate = currentTime - lastUpdateTime > occupancyMapUpdateDt;
      if (!isTimeForUpdate)
         return;

      updateOccupiedCells();
//      updateLowestCostPlan();

      lastUpdateTime = currentTime;
   }

   @Override
   public void plannerFinished(List<FootstepNode> plan, Collection<FootstepNode> expandedNodes, FootstepGraph footstepGraph)
   {
      updateOccupiedCells();

      Collection<FootstepNode> expandedNodesToSet = expandedNodesSinceLastReportReference.getCopyForWriting();
      expandedNodesToSet.clear();
      expandedNodesToSet.addAll(expandedNodes);
      expandedNodesSinceLastReportReference.commit();

      computeFullGraphData(footstepGraph);
   }

   private void updateOccupiedCells()
   {
      occupancyMapCellsSinceLastReportReference.addAll(occupancyMapCellsThisTick);
      hasOccupiedCells.set(hasOccupiedCells.get() || !occupancyMapCellsThisTick.isEmpty());
      occupancyMapCellsThisTick.clear();
   }

   private void updateLowestCostPlan()
   {
      PlannerNodeDataList concurrentNodeDataList = this.concurrentLowestCostNodeDataList.getCopyForWriting();
      concurrentNodeDataList.clear();
      for (int i = 0; i < lowestCostPlan.size(); i++)
      {
         FootstepNode node = lowestCostPlan.get(i);
         RigidBodyTransform nodePose = snapper.snapFootstepNode(node).getOrComputeSnappedNodeTransform(node);
         int parentNodeIndex = i > 0 ? lowestCostPlan.get(i - 1).getNodeIndex() : -1;
         concurrentNodeDataList.addNode(parentNodeIndex, node, nodePose, null);
      }
      this.concurrentLowestCostNodeDataList.commit();
      lowestCostPlan.clear();

      hasLowestCostPlan.set(!concurrentNodeDataList.getNodeData().isEmpty());
   }

   private void computeFullGraphData(FootstepGraph footstepGraph)
   {
      List<PlannerNodeData> fullGraphToReport = this.fullGraphToReportReference.getCopyForWriting();
      HashMap<FootstepNode, HashSet<FootstepEdge>> outgoingEdges = footstepGraph.getOutgoingEdges();
      for (FootstepNode footstepNode : outgoingEdges.keySet())
      {
         for (FootstepEdge outgoingEdge : outgoingEdges.get(footstepNode))
         {
            FootstepNode childNode = outgoingEdge.getEndNode();
            RigidBodyTransform nodePose = snapper.snapFootstepNode(childNode).getOrComputeSnappedNodeTransform(childNode);
            PlannerNodeData nodeData = new PlannerNodeData(footstepNode.getNodeIndex(), childNode, nodePose, null);
            fullGraphToReport.add(nodeData);
         }
         List<PlannerNodeData> rejectedData = rejectedNodeData.get(footstepNode);
         if (rejectedData != null)
            fullGraphToReport.addAll(rejectedData);
      }
      this.fullGraphToReportReference.commit();

      hasFullGraph.set(true);
   }


   public boolean hasOccupiedCells()
   {
      return hasOccupiedCells.get();
   }

   public boolean hasExpandedNodes()
   {
      return hasExpandedNodes.get();
   }

   public boolean hasLowestCostPlan()
   {
      return hasLowestCostPlan.get();
   }

   public boolean hasFullGraph()
   {
      return hasFullGraph.get();
   }

   PlannerOccupancyMap getOccupancyMap()
   {
      PlannerOccupancyMap occupancyMap = new PlannerOccupancyMap();
      for (PlannerCell plannerCell : occupancyMapCellsSinceLastReportReference.getCopyForReading())
         occupancyMap.addOccupiedCell(plannerCell);

      hasOccupiedCells.set(false);
      occupancyMapCellsSinceLastReportReference.clear();

      return occupancyMap;
   }

   PlannerLatticeMap getExpandedNodes()
   {
      PlannerLatticeMap latticeMap = new PlannerLatticeMap();
      for (FootstepNode latticeNode : expandedNodesSinceLastReportReference.getCopyForReading())
         latticeMap.addFootstepNode(latticeNode);

      hasExpandedNodes.set(false);
      expandedNodesSinceLastReportReference.clear();

      return latticeMap;
   }

   PlannerNodeDataList getLowestCostPlan()
   {
      hasLowestCostPlan.set(false);

      return concurrentLowestCostNodeDataList.getCopyForReading();
   }

   PlannerNodeDataList getFullGraph()
   {
      if (!hasFullGraph.get())
         return null;

      PlannerNodeDataList plannerNodeDataList = new PlannerNodeDataList();
      plannerNodeDataList.setIsFootstepGraph(true);

      for (PlannerNodeData nodeData : fullGraphToReportReference.getCopyForReading())
         plannerNodeDataList.addNode(nodeData);

      hasFullGraph.set(false);
      fullGraphToReportReference.clear();

      return plannerNodeDataList;
   }

   public int getTotalNodeCount()
   {
      return totalNodeCount;
   }

   public int getRejectionReasonCount(BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      return rejectionCount.get(rejectionReason).getValue();
   }

   private class ConcurrentSet<T> extends ConcurrentCopier<Set<T>>
   {
      public ConcurrentSet()
      {
         super(HashSet::new);
      }

      public void clear()
      {
         getCopyForWriting().clear();
         commit();
      }

      public void addAll(Collection<? extends T> collection)
      {
         Set<T> currentSet = getCopyForReading();
         Set<T> updatedSet = getCopyForWriting();
         updatedSet.clear();
         if (currentSet != null)
            updatedSet.addAll(currentSet);
         updatedSet.addAll(collection);
         commit();
      }

      public T[] toArray(T[] ts)
      {
         Set<T> currentSet = getCopyForReading();
         return currentSet.toArray(ts);
      }

      public boolean isEmpty()
      {
         Set<T> currentList = getCopyForReading();
         if (currentList == null)
            return true;

         return currentList.isEmpty();
      }

      public int size()
      {
         Set<T> currentList = getCopyForReading();
         if (currentList == null)
            return 0;

         return currentList.size();
      }
   }

   private class ConcurrentList<T> extends ConcurrentCopier<List<T>>
   {
      public ConcurrentList()
      {
         super(ArrayList::new);
      }

      public void clear()
      {
         getCopyForWriting().clear();
         commit();
      }

      public void addAll(Collection<? extends T> collection)
      {
         List<T> currentSet = getCopyForReading();
         List<T> updatedSet = getCopyForWriting();
         updatedSet.clear();
         if (currentSet != null)
            updatedSet.addAll(currentSet);
         updatedSet.addAll(collection);
         commit();
      }

      public T[] toArray(T[] ts)
      {
         List<T> currentSet = getCopyForReading();
         return currentSet.toArray(ts);
      }

      public boolean isEmpty()
      {
         List<T> currentList = getCopyForReading();
         if (currentList == null)
            return true;

         return currentList.isEmpty();
      }

      public int size()
      {
         List<T> currentList = getCopyForReading();
         if (currentList == null)
            return 0;

         return currentList.size();
      }
   }
}
