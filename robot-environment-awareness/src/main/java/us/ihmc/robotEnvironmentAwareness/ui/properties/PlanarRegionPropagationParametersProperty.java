package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.PlanarRegionPropagationParameters;

public class PlanarRegionPropagationParametersProperty extends ParametersProperty<PlanarRegionPropagationParameters>
{
   private final DoubleField proximityThreshold = new DoubleField(PlanarRegionPropagationParameters::getProximityThreshold,
                                                                  (p, v) -> p.setProximityThreshold(v));
   private final DoubleField planarityThreshold = new DoubleField(PlanarRegionPropagationParameters::getPlanarityThreshold,
                                                                  (p, v) -> p.setPlanarityThreshold(v));

   private final BooleanField enableExtending = new BooleanField(PlanarRegionPropagationParameters::isExtendingEnabled,
                                                                    (p, v) -> p.setEnableExtending(v));
   private final BooleanField updateExtendedData = new BooleanField(PlanarRegionPropagationParameters::isUpdateExtendedData,
                                                                    (p, v) -> p.setUpdateExtendedData(v));
   private final DoubleField extendingDistanceThreshold = new DoubleField(PlanarRegionPropagationParameters::getExtendingDistanceThreshold,
                                                                          (p, v) -> p.setExtendingDistanceThreshold(v));
   private final DoubleField extendingRadiusThreshold = new DoubleField(PlanarRegionPropagationParameters::getExtendingRadiusThreshold,
                                                                        (p, v) -> p.setExtendingRadiusThreshold(v));

   public PlanarRegionPropagationParametersProperty(Object bean, String name)
   {
      super(bean, name, new PlanarRegionPropagationParameters());
   }


   public void bindBidirectionalProximityThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, proximityThreshold);
   }

   public void bindBidirectionalPlanarityThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, planarityThreshold);
   }
   
   public void bindBidirectionalEnableExtending(BooleanProperty property)
   {
      bindFieldBidirectionalToBooleanProperty(property, enableExtending);
   }

   public void bindBidirectionalUpdateExtendedData(BooleanProperty property)
   {
      bindFieldBidirectionalToBooleanProperty(property, updateExtendedData);
   }

   public void bindBidirectionalExtendingDistanceThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, extendingDistanceThreshold);
   }

   public void bindBidirectionalExtendingRadiusThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, extendingRadiusThreshold);
   }

   @Override
   protected PlanarRegionPropagationParameters getValueCopy(PlanarRegionPropagationParameters valueToCopy)
   {
      return new PlanarRegionPropagationParameters(valueToCopy);
   }
}
