package artisynth.VSP.reconstruction;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayDeque;
import java.util.Deque;

import javax.swing.JFrame;
import javax.swing.JMenu;

import artisynth.core.driver.Main;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.AxialSpringList;
import artisynth.core.modelbase.ComponentChangeEvent;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.modelbase.ScanWriteUtils;
import artisynth.core.util.ArtisynthIO;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.ScanToken;
import artisynth.core.workspace.RootModel;
import maspack.geometry.MeshBase;
import maspack.matrix.AxisAlignedRotation;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.properties.PropertyUtils;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.PointStyle;
import maspack.util.IndentingPrintWriter;
import maspack.util.PathFinder;
import maspack.util.ReaderTokenizer;
import maspack.util.Write;

/**
 * Base RootModel for ArtiSynth reconstruction applications.
 */
public class ReconAppRoot extends RootModel {

   public double DENSITY = 1e-6;
   public double DTOR = Math.PI/180;
   
   // describes the type of donor bone
   public enum DonorType {
      FIBULA, 
      SCAPULA
   };

   // colors for reconstruction objects
   //public static Color BONE_COLOR = new Color (1f, 1f, 0.8f);
   // darker color for GL2:
   public static Color BONE_COLOR = new Color (1f, 0.94f, 0.7f);
   public static Color MARKER_COLOR = new Color (.5f, .5f, 1f);
   public static Color LANDMARK_COLOR = new Color (1f, 0f, 0f);
   public static Color LINE_COLOR = new Color (.4f, .4f, .8f);
   public static Color RDP_COLOR = new Color (0f, 1f, 1f);
   public static Color PLANE_COLOR = new Color (0f, .8f, .8f);
   public static Color PLANE_BACK_COLOR = new Color (0f, 0.4f, 0.4f);
   public static Color PLATE_COLOR = new Color (0.98f, 0.58f, 0f);
   public static Color GUIDE_COLOR = new Color (0.6f, 0.8f, 1.0f);

   MechModel myMech; // host reconstruction objects and worker classes

   // worker classes:
   MeshManager myMeshManager;

   // properties definitions and accessors:

   static boolean DEFAULT_WRITE_STL_AS_BINARY = true;
   boolean myWriteStlAsBinary = DEFAULT_WRITE_STL_AS_BINARY;

   static DonorType DEFAULT_DONOR_TYPE = DonorType.FIBULA;
   DonorType myDonorType = DEFAULT_DONOR_TYPE;

   protected static double DEFAULT_RENDER_PLANE_WIDTH = 50;
   protected double myRenderPlaneWidth = DEFAULT_RENDER_PLANE_WIDTH;
   protected PropertyMode myRenderPlaneWidthMode = PropertyMode.Inherited;

   protected static double DEFAULT_CUT_PLANE_WIDTH = 95;
   protected double myCutPlaneWidth = DEFAULT_CUT_PLANE_WIDTH;

   protected static int DEFAULT_PLANE_RESOLUTION = 1;
   protected int myPlaneResolution = DEFAULT_PLANE_RESOLUTION;

   static double DEFAULT_DONOR_SET_BACK = 0;
   double myDonorSetBack = DEFAULT_DONOR_SET_BACK;

   static boolean DEFAULT_USE_2D_BONY_CONTACT = true;
   boolean myUse2DBonyContact = DEFAULT_USE_2D_BONY_CONTACT;

   public static PropertyList myProps =
      new PropertyList (ReconAppRoot.class, RootModel.class);

   static VectorNd myZeroVec = new VectorNd(7);

   static {
      myProps.add (
         "donorType", "describes the type of the donor bone",
         DEFAULT_DONOR_TYPE);     
      myProps.addInheritable (
         "renderPlaneWidth",
         "size with which to render the cut planes",
         DEFAULT_RENDER_PLANE_WIDTH);
      myProps.add (
         "cutPlaneWidth",
         "width (in each direction) of a cut plane mesh",
         DEFAULT_CUT_PLANE_WIDTH);
      myProps.add (
         "planeResolution",
         "resolution (along each direction) of a cut plane mesh",
         DEFAULT_PLANE_RESOLUTION);
      myProps.add (
         "donorSetBack",
         "amount to set back the donor in the viewer",
         DEFAULT_DONOR_SET_BACK);         
      myProps.add (
         "writeStlAsBinary",
         "write STL files as binary",
         DEFAULT_WRITE_STL_AS_BINARY);         
      myProps.add (
         "use2DBonyContact",
         "use 2D polygon computations to determine bony contact",
         DEFAULT_USE_2D_BONY_CONTACT);
   }
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public DonorType getDonorType() {
      return myDonorType;
   }

   public void setDonorType (DonorType donorType) {
      if (donorType != myDonorType) {
         myDonorType = donorType;
         setDonorSetBack (myDonorType == DonorType.SCAPULA ? 50 : 0);         
      }
   }

   public double getRenderPlaneWidth () {
      return myRenderPlaneWidth;
   }

   public void setRenderPlaneWidth (double size) {
      myRenderPlaneWidth = size;
      myRenderPlaneWidthMode =
         PropertyUtils.propagateValue (
            this, "renderPlaneWidth", myRenderPlaneWidth, myRenderPlaneWidthMode);
   }

   public void setRenderPlaneWidthMode (PropertyMode mode) {
      myRenderPlaneWidthMode =
         PropertyUtils.setModeAndUpdate (
            this, "renderPlaneWidth", myRenderPlaneWidthMode, mode);
   }

   public PropertyMode getRenderPlaneWidthMode() {
      return myRenderPlaneWidthMode;
   }

   public double getCutPlaneWidth () {
      return myCutPlaneWidth;
   }

   public void setCutPlaneWidth (double size) {
      myCutPlaneWidth = size;
   }

   public int getPlaneResolution () {
      return myPlaneResolution;
   }

   public void setPlaneResolution (int res) {
      myPlaneResolution = res;
   }

   public void setDonorSetBack (double d) {
      if (d != myDonorSetBack) {
         if (myMeshManager != null) {
            myMeshManager.adjustDonorSetback (d - myDonorSetBack);
         }
         myDonorSetBack = d;
         rerender();
      }
   }

   void removeDonorSetBack (MeshBase mesh) {
      if (myDonorSetBack != 0) {
         RigidTransform3d TSB = new RigidTransform3d (0, myDonorSetBack, 0);
         mesh.inverseTransform (TSB);
      }
   }

   public double getDonorSetBack () {
      return myDonorSetBack;
   }

   public boolean getWriteStlAsBinary() {
      return myWriteStlAsBinary;
   }

   public void setWriteStlAsBinary (boolean enable) {
      myWriteStlAsBinary = enable; 
   }

   public boolean getUse2DBonyContact() {
      return myUse2DBonyContact;
   }

   public void setUse2DBonyContact (boolean enable) {
      myUse2DBonyContact = enable; 
   }

   // other accessors:

   public MechModel getMechModel() {
      return myMech;
   }
   
   public MeshManager getMeshManager() {
      return myMeshManager;
   }

}
