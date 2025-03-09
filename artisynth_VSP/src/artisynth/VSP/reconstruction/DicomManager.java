package artisynth.VSP.reconstruction;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Deque;

import artisynth.VSP.reconstruction.MandibleRecon.*;
import artisynth.VSP.reconstruction.MeshManager.MarkerFormat;
import artisynth.VSP.reconstruction.ReconAppRoot.DonorType;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.modelbase.*;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.util.ScanToken;
import artisynth.core.renderables.DicomViewer;
import maspack.collision.IntersectionContour;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.MeshFactory;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Plane;
import maspack.geometry.Vertex3d;
import maspack.image.dicom.DicomImage;
import maspack.interpolation.CubicHermiteSpline3d;
import maspack.matrix.NumericalException;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.Vector2d;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.properties.PropertyUtils;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.util.InternalErrorException;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;

/**
 * Worker component for managing the dicom viewer.
 */
public class DicomManager extends WorkerComponentBase {

   // properties:

   static double DEFAULT_X_POSITION = 0.5;
   double myXPosition = DEFAULT_X_POSITION;

   static double DEFAULT_Y_POSITION = 0.5;
   double myYPosition = DEFAULT_Y_POSITION;

   static double DEFAULT_Z_POSITION = 0.5;
   double myZPosition = DEFAULT_Z_POSITION;

   static boolean DEFAULT_YZ_VISIBLE = true;
   boolean myYZVisible = DEFAULT_YZ_VISIBLE;

   static boolean DEFAULT_XZ_VISIBLE = true;
   boolean myXZVisible = DEFAULT_XZ_VISIBLE;

   static boolean DEFAULT_XY_VISIBLE = true;
   boolean myXYVisible = DEFAULT_XY_VISIBLE;

   DicomViewer myDicomViewer;

   public static PropertyList myProps =
      new PropertyList (
         DicomManager.class, WorkerComponentBase.class);

   static {
      myProps.add (
         "viewerVisible isViewerVisible",
         "visibility of the dicom viewer",
         false);
      myProps.add (
         "xPosition",
         "x position of the yz dicom view plane",
         DEFAULT_X_POSITION);
      myProps.add (
         "yPosition",
         "y position of the xz dicom view plane",
         DEFAULT_Y_POSITION);
      myProps.add (
         "zPosition",
         "z position of the xy dicom view plane",
         DEFAULT_Z_POSITION);
      myProps.add (
         "YZVisible",
         "visibility of the yz dicom view plane",
         DEFAULT_YZ_VISIBLE);
      myProps.add (
         "XZVisible",
         "visibility of the xz dicom view plane",
         DEFAULT_XZ_VISIBLE);
      myProps.add (
         "XYVisible",
         "visibility of the xy dicom view plane",
         DEFAULT_XY_VISIBLE);
   }
   

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   // property accessors

   public double getXPosition() {
      return myXPosition;
   }

   public void setXPosition(double x) {
      DicomViewer dviewer = getViewer();
      if (dviewer != null) {
         dviewer.setX (x);
      }
      myXPosition = x;
   }

   public double getYPosition() {
      return myYPosition;
   }

   public void setYPosition(double y) {
      DicomViewer dviewer = getViewer();
      if (dviewer != null) {
         dviewer.setY (y);
      }
      myYPosition = y;
   }

   public double getZPosition() {
      return myZPosition;
   }

   public void setZPosition(double z) {
      DicomViewer dviewer = getViewer();
      if (dviewer != null) {
         dviewer.setZ (z);
      }
      myZPosition = z;
   }

   public boolean getYZVisible() {
      return myYZVisible;
   }

   public void setYZVisible(boolean visible) {
      DicomViewer dviewer = getViewer();
      if (dviewer != null) {
         dviewer.setDrawYZ (visible);
      }
      myYZVisible = visible;
   }

   public boolean getXZVisible() {
      return myXZVisible;
   }

   public void setXZVisible(boolean visible) {
      DicomViewer dviewer = getViewer();
      if (dviewer != null) {
         dviewer.setDrawXZ (visible);
      }
      myXZVisible = visible;
   }

   public boolean getXYVisible() {
      return myXYVisible;
   }

   public void setXYVisible(boolean visible) {
      DicomViewer dviewer = getViewer();
      if (dviewer != null) {
         dviewer.setDrawXY (visible);
      }
      myXYVisible = visible;
   }

   // constructors

   /**
    * Note: need a public no-args constructor for ArtiSynth save/load.
    */
   public DicomManager () {
      setRenderProps (defaultRenderProps(this));
   }

   public DicomManager (String name) {
      this();
      setName (name);
   }

   // manage viewer

   public boolean hasViewer() {
      return getViewer() != null;
   }

   public DicomViewer getViewer() {
      MechModel mech = getRoot().getMechModel();
      return (DicomViewer)mech.renderables().get("DicomViewer");
   }

   public boolean isViewerVisible() {
      return RenderableComponentBase.isVisible(getViewer());
   }

   public void setViewerVisible(boolean enable) {
      RenderableComponentBase.setVisible(getViewer(), enable);      
   }

   /**
    * Clear the viewer.
    */
   boolean clearViewer () {
      DicomViewer viewer = getViewer();
      if (viewer != null) {
         MechModel mech = getRoot().getMechModel();
         mech.renderables().remove(viewer);
         return true;
      }
      else {
         return false;
      }
   }  

   // actions

   /**
    * Create the dicom viewer from a file.
    */
   void createViewer (File dicomFile) {
      MechModel mech = getRoot().getMechModel();

      clearViewer();

      DicomViewer viewer = new DicomViewer(
         "DicomViewer", dicomFile,
         /*pattern=*/null, /*check subdirectories*/true);
      viewer.setIgnoreUpdateBounds(true);

      DicomImage image = viewer.getImage();
      System.out.println(image.toString());
      
      Point3d cent = getRoot().getMeshManager().getMandibleCentroid();
      if (cent != null) {
         RigidTransform3d X = new RigidTransform3d();
         X.p.set (cent);
         X.invert();
         viewer.transformGeometry (X);
      }
      mech.addRenderable(viewer);
      RenderProps.setVisible (viewer, false);
   }

   

}

