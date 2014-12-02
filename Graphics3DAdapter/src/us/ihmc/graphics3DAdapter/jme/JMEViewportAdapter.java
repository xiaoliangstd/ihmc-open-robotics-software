package us.ihmc.graphics3DAdapter.jme;

import java.awt.Canvas;
import java.awt.Toolkit;
import java.util.ArrayList;

import javax.vecmath.Point3d;

import us.ihmc.graphics3DAdapter.ContextSwitchedListener;
import us.ihmc.graphics3DAdapter.Graphics3DFrameListener;
import us.ihmc.graphics3DAdapter.camera.CameraController;
import us.ihmc.graphics3DAdapter.camera.CaptureDevice;
import us.ihmc.graphics3DAdapter.camera.ViewportAdapter;
import us.ihmc.graphics3DAdapter.jme.context.PBOAwtPanel;
import us.ihmc.graphics3DAdapter.jme.context.PBOAwtPanelsContext;
import us.ihmc.graphics3DAdapter.jme.input.JMESelectedListener;
import us.ihmc.graphics3DAdapter.jme.util.JMEDataTypeUtils;
import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import us.ihmc.utilities.time.Timer;

import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.light.DirectionalLight;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.post.FilterPostProcessor;
import com.jme3.post.SceneProcessor;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.shadow.EdgeFilteringMode;
import com.jme3.system.JmeContext;
import com.jme3.system.awt.AwtPanel;
import com.jme3.system.awt.AwtPanelsContext;
import com.jme3.system.awt.PaintMode;
import com.jme3.texture.FrameBuffer;
import com.jme3.texture.Image.Format;

public class JMEViewportAdapter extends ViewportAdapter implements InputMapSetter, SceneProcessor
{
   public enum ViewportType {OFFSCREEN, CANVAS, MULTICAM}

   private final JMECamera jmeCamera;
   private final JMESelectedListener selectedListener;
   private final boolean isMainViewport;

   private final JmeContext context;

   private Canvas panel = null;
   private FrameBuffer frameBuffer = null;
   private ViewportType viewportType;
   private final ViewPort viewPort;
   private final AssetManager assetManager;
   private final ArrayList<ContextSwitchedListener> contextSwitchedListeners = new ArrayList<ContextSwitchedListener>();

   private final RenderManager renderManager;

   private final JMEFastCaptureDevice screenShotHelper;
   private final AppStateManager stateManager;

   private final DirectionalLight primaryLight;
   private FilterPostProcessor fpp;

   private Timer frameTimer = new Timer().start();

   public JMEViewportAdapter(JMERenderer jmeRenderer, Node rootNode, boolean isMainViewport, ViewportType viewportType)
   {
      this(jmeRenderer, rootNode, isMainViewport, viewportType, false);
   }

   public JMEViewportAdapter(JMERenderer jmeRenderer, Node rootNode, boolean isMainViewport, ViewportType viewportType, boolean addExtraVisuals)
   {
      this.assetManager = jmeRenderer.getAssetManager();
      this.stateManager = jmeRenderer.getStateManager();
      this.context = jmeRenderer.getContext();
      this.primaryLight = jmeRenderer.getPrimaryLight();
      this.isMainViewport = isMainViewport;
      renderManager = jmeRenderer.getRenderManager();
      jmeCamera = new JMECamera(jmeRenderer.getCamera());
      viewPort = renderManager.createMainView("JMEViewport", jmeCamera);
      viewPort.attachScene(rootNode);
      viewPort.setClearFlags(true, true, true);
      viewPort.setBackgroundColor(new ColorRGBA(0.7f, 0.8f, 0.8f, 1f));
      viewPort.addProcessor(this);
      screenShotHelper = new JMEFastCaptureDevice(viewPort);
      stateManager.attach(screenShotHelper);




      if (addExtraVisuals)
      {
         setupShadows(viewPort);

         setUpPostProcesses();
      }


      this.viewportType = viewportType;


      switch (viewportType)
      {
         case CANVAS :
         case MULTICAM :
            jmeRenderer.getContextManager().addJMEViewportAdapter(this);
            this.selectedListener = new JMESelectedListener(jmeRenderer.getSelectedListenerHolder(), jmeRenderer.getKeyListenerHolder(),
                    jmeRenderer.getMouseListenerHolder(), jmeRenderer.getJmeGraphicsNodes(), rootNode, jmeRenderer.getInputManager(), jmeCamera);

            break;

         default :
            this.selectedListener = null;

            break;
      }

      jmeRenderer.registerViewport(this);
   }



   public void setupViewport(double left, double right, double bottom, double top)
   {
      if (viewportType != ViewportType.MULTICAM)
      {
         throw new RuntimeException("Viewport is not setup for multicam rendering");
      }

      viewPort.setClearFlags(true, true, true);
      jmeCamera.setViewPort((float) left, (float) right, (float) bottom, (float) top);

   }

   public Canvas getCanvas()
   {
      if (viewportType != ViewportType.CANVAS)
      {
         throw new RuntimeException("Viewport is not setup for canvas rendering");
      }

      /*
       * Only make panel when it is actually needed
       */
      if (panel == null)
      {
         if (context instanceof AwtPanelsContext)
         {
            AwtPanel awtPanel = ((AwtPanelsContext) context).createPanel(PaintMode.Accelerated);
            awtPanel.attachTo(isMainViewport, viewPort);
            panel = awtPanel;
         }
         else if (context instanceof PBOAwtPanelsContext)
         {
            PBOAwtPanel awtPanel = ((PBOAwtPanelsContext) context).createPanel();
            awtPanel.attachTo(isMainViewport, viewPort);
            panel = awtPanel;
         }
      }

      return panel;
   }

   public void setupOffscreenView(int width, int height)
   {
      if (viewportType == ViewportType.CANVAS)
      {
         throw new RuntimeException("Viewport is setup for canvas rendering");
      }

      frameBuffer = new FrameBuffer(width, height, 1);
      viewPort.setOutputFrameBuffer(frameBuffer);
      viewPort.getCamera().resize(width, height, true);
      frameBuffer.setDepthBuffer(Format.Depth);
      frameBuffer.setColorBuffer(Format.RGBA8);
   }

   public void closeAndDispose()
   {
      viewPort.setEnabled(false);
      renderManager.removeMainView(viewPort);
   }



   boolean addedPostProcessors = false;

   private void setupShadows(ViewPort viewPort)
   {
      DirectionalLightShadowRenderer dlsr = new DirectionalLightShadowRenderer(assetManager, 4096, 2);
      dlsr.setLight(primaryLight);
      dlsr.setLambda(0.8f);
      dlsr.setShadowIntensity(0.5f);
      dlsr.setEdgeFilteringMode(EdgeFilteringMode.PCF4);
      viewPort.addProcessor(dlsr);
   }

   public void setUpPostProcesses()
   {
      if (fpp == null)
      {
//         fpp = new FilterPostProcessor(assetManager);
//
//         CartoonEdgeFilter f = new CartoonEdgeFilter();
//
//         f.setEdgeWidth(0.5f);
//         f.setEdgeIntensity(0.5f);
//
////
//         fpp.addFilter(f);

//       SSAOFilter ssaoFilter = new SSAOFilter(12.940201f, 43.928635f, 0.32999992f, 0.6059958f);
//       fpp.addFilter(ssaoFilter);
//         getViewPort().addProcessor(fpp);


      }



   }

   public void setDefaultInputMappings()
   {
      selectedListener.registerWithInputManager();
   }

   public void reset()
   {
      selectedListener.reset();
   }

   public void setClipDistances(double near, double far)
   {
      // TODO Auto-generated method stub

   }

   public void setFieldOfView(double fieldOfView)
   {
   }

   public CameraController getCameraController()
   {
      return jmeCamera.getController();
   }

   public CaptureDevice getCaptureDevice()
   {
      return screenShotHelper;
   }

   public int getHeight()
   {
      return panel.getHeight();
   }

   public int getWidth()
   {
      return panel.getWidth();
   }

   public void setSize(int width, int height)
   {
      if (viewportType == ViewportType.CANVAS)
      {
         panel.setSize(width, height);
      }
      else
      {
         System.err.println("Resizing offscreen renderer, might crash!");
         setupOffscreenView(width, height);
      }
   }

   public double getFieldOfView()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   private double convertToPhysicalDimension(int pixels)
   {
      double dPixels = pixels;
      double dpi = Toolkit.getDefaultToolkit().getScreenResolution();

      return (dPixels / dpi) * 0.0254;

   }

   public double getPhysicalWidth()
   {
      return convertToPhysicalDimension(getWidth());
   }

   public double getPhysicalHeight()
   {
      return convertToPhysicalDimension(getHeight());
   }

   public void updateCamera()
   {
      jmeCamera.updateCamera();
   }

   public void setCameraController(CameraController cameraController)
   {
      jmeCamera.setCameraController(cameraController);
   }

   public JMECamera getCamera()
   {
      return jmeCamera;
   }

   public boolean isMainViewport()
   {
      return isMainViewport;
   }

   /**
    * Calculate real world coordinates from x and y position on the screen
    *
    * @param x X coordinate on screen
    * @param y Y coordinate on screen
    * @param z Depth as relative value between near and far frustrum
    */
   public Point3d getWorldCoordinatesFromScreenCoordinates(float x, float y, double z)
   {
      Vector3f worldCoordinates = jmeCamera.getWorldCoordinates(new Vector2f(x, y), (float) z);
      JMEGeometryUtils.transformFromJMECoordinatesToZup(worldCoordinates);

      return JMEDataTypeUtils.jmeVector3fToJ3DPoint3d(worldCoordinates);
   }

   public void addContextSwitchedListener(ContextSwitchedListener contextSwitchedListener)
   {
      contextSwitchedListeners.add(contextSwitchedListener);
   }

   public ViewPort getViewPort()
   {
      return viewPort;
   }

   public void attachScene(Spatial scene)
   {
      viewPort.attachScene(scene);
   }

   public void detachScene(Spatial scene)
   {
      viewPort.detachScene(scene);
   }

   public void initialize(RenderManager rm, ViewPort vp)
   {
      // TODO Auto-generated method stub

   }

   public void reshape(ViewPort vp, int w, int h)
   {
      // TODO Auto-generated method stub

   }

   public boolean isInitialized()
   {
      // TODO Auto-generated method stub
      return false;
   }

   public void preFrame(float tpf)
   {
      // TODO Auto-generated method stub

   }

   public void postQueue(RenderQueue rq)
   {
      // TODO Auto-generated method stub

   }

   public void postFrame(FrameBuffer out)
   {
      double timePerFrame = frameTimer.lap();
      for (Graphics3DFrameListener listener : getFrameListeners())
      {
         listener.postFrame(timePerFrame);
      }
   }

   public void cleanup()
   {
      // TODO Auto-generated method stub

   }
}
