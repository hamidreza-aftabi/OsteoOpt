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
import artisynth.core.modelbase.CompositeComponent;
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
 * Worker component for managing the implants and their placement.
 */
public class ImplantsManager extends WorkerComponentBase {

   // properties:

   static double DEFAULT_UPPER_IMPLANT_RADIUS = 2.05;
   double myUpperImplantRadius = DEFAULT_UPPER_IMPLANT_RADIUS;

   static double DEFAULT_LOWER_IMPLANT_RADIUS = 2.05;
   double myLowerImplantRadius = DEFAULT_LOWER_IMPLANT_RADIUS;

   static double DEFAULT_IMPLANT_LENGTH = 10;
   double myImplantLength = DEFAULT_IMPLANT_LENGTH;

   static double DEFAULT_OCCLUSAL_OFFSET = 12;
   double myOcclusalOffset = DEFAULT_OCCLUSAL_OFFSET;

   static double DEFAULT_IMPLANT_TOP_EXTENSION = DEFAULT_OCCLUSAL_OFFSET;
   double myImplantTopExtension = DEFAULT_IMPLANT_TOP_EXTENSION;

   static double DEFAULT_PLANE_SLICE_SCALE = 1.1;
   double myPlaneSliceScale = DEFAULT_PLANE_SLICE_SCALE;

   public enum SliceContours {
      MANDIBLE,
      MAXILLA,
      NONE,
   }

   static SliceContours DEFAULT_OCCLUSAL_SLICE_CONTOURS = SliceContours.MANDIBLE;
   SliceContours myOcclusalSliceContours = DEFAULT_OCCLUSAL_SLICE_CONTOURS;

   public static PropertyList myProps =
      new PropertyList (
         ImplantsManager.class, WorkerComponentBase.class);

   static {
      myProps.add (
         "upperImplantRadius",
         "upper radius of the next implant to be added",
         DEFAULT_UPPER_IMPLANT_RADIUS);
      myProps.add (
         "lowerImplantRadius",
         "lower radius of the next implant to be added",
         DEFAULT_LOWER_IMPLANT_RADIUS);
      myProps.add (
         "implantLength",
         "length of the next implant to be added",
         DEFAULT_IMPLANT_LENGTH);
      myProps.add (
         "implantTopExtension",
         "length of renderable top extension for next implant",
         DEFAULT_IMPLANT_TOP_EXTENSION);
      myProps.add (
         "occlusalOffset",
         "distance between implant top and occlusal plane",
         DEFAULT_OCCLUSAL_OFFSET);
      myProps.add (
         "occlusalSliceContours",
         "mesh for creating contours on the occlusalSlice",
         DEFAULT_OCCLUSAL_SLICE_CONTOURS);
      myProps.add (
         "planeSliceScale",
         "scale factor for occlusal plane and slices",
         DEFAULT_PLANE_SLICE_SCALE);
   }
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   // property accessors

   public double getUpperImplantRadius() {
      return myUpperImplantRadius;
   }

   public void setUpperImplantRadius(double value) {
      myUpperImplantRadius = value;
   }

   public double getLowerImplantRadius() {
      return myLowerImplantRadius;
   }

   public void setLowerImplantRadius(double value) {
      myLowerImplantRadius = value;
   }

   public double getImplantLength() {
      return myImplantLength;
   }

   public void setImplantLength(double value) {
      myImplantLength = value;
   }

   public double getImplantTopExtension() {
      return myImplantTopExtension;
   }

   public void setImplantTopExtension(double value) {
      myImplantTopExtension = value;
   }

   public double getOcclusalOffset() {
      return myOcclusalOffset;
   }

   public void setOcclusalOffset(double value) {
      myOcclusalOffset = value;
   }

   public double getPlaneSliceScale() {
      return myPlaneSliceScale;
   }

   public void setPlaneSliceScale(double value) {
      if (myPlaneSliceScale != value && value != 0) {
         TaskFrame taskFrame = ((MandibleRecon)getRoot()).getTaskFrame();
         double s = value/myPlaneSliceScale;
         SlicePlane oslice = taskFrame.getOcclusalSlice();
         if (oslice != null) {
            oslice.scaleMesh (s);
         }
         SlicePlane sslice = taskFrame.getSagittalSlice();
         if (sslice != null) {
            sslice.scaleMesh (s);
         }
         MeshManager meshManager = getRoot().getMeshManager();
         OcclusalPlane oplane = meshManager.getOcclusalPlane();
         if (oplane != null) {
            oplane.scaleMesh (s);
         }
         myPlaneSliceScale = value;
      }
   }

   public SliceContours getOcclusalSliceContours() {
      return myOcclusalSliceContours;
   }

   public void setOcclusalSliceContours(SliceContours value) {
      if (value != myOcclusalSliceContours) {
         myOcclusalSliceContours = value;
         TaskFrame taskFrame = ((MandibleRecon)getRoot()).getTaskFrame();
         if (taskFrame.hasOcclusalSlice()) {
            SlicePlane oslice = taskFrame.getOcclusalSlice();
            setSliceContours (oslice);
         }
      }
   }

   // constructors

   /**
    * Note: need a public no-args constructor for ArtiSynth save/load.
    */
   public ImplantsManager () {
      setRenderProps (defaultRenderProps(this));
   }

   public ImplantsManager (String name) {
      this();
      setName (name);
   }

   // manage implants

   public boolean hasImplants() {
      return getImplants().size() > 0;
   }

   public void addImplant (Implant implant) {
      getImplants().add (implant);
   }

   public boolean removeImplant (Implant implant) {
      return getImplants().remove (implant);
   }

   public boolean clearImplants() {
      RenderableComponentList<Implant> implants = getImplants();
      if (implants.size() > 0) {
         implants.removeAll();
         return true;
      }
      else {
         return false;
      }
   }

   public RenderableComponentList<Implant> getImplants() {
      return (RenderableComponentList<Implant>)myMech.get ("implants");
   }

   // actions

   Implant createImplant () {
      Implant implant = new Implant (
         getUpperImplantRadius(),
         getLowerImplantRadius(),
         getImplantLength(),
         getImplantTopExtension());
      return implant;
   }

   void setTopExtension (RigidBody implant, double topExtLen) {
      implant.removeMeshComp ("topext");
      if (topExtLen > 0) {
         // add top extension mesh
         PolygonalMesh mesh = MeshFactory.createCylinder (
            getUpperImplantRadius(),
            topExtLen,
            /*nsides=*/32);
         mesh.translate (new Vector3d (0, 0, (getImplantLength()+topExtLen)/2));
         MeshComponent mcomp = implant.addMesh (mesh);
         mcomp.setName ("topext");
         RenderProps.setFaceColor (mcomp, new Color (0.7f, 1f, 0.7f));
      }
   }

   DicomViewer getDicomViewer() {
      MechModel mech = getRoot().getMechModel();
      return (DicomViewer)mech.renderables().get("DicomViewer");
   }

   void setSliceContours (SlicePlane slice) {
      RigidBody contourBody = null;
      slice.removeAllContourBodies();
      MeshManager meshManager = getRoot().getMeshManager();
      switch (getOcclusalSliceContours()) {
         case MANDIBLE: {
            contourBody = meshManager.getMandible();
            break;
         }
         case MAXILLA: {
            contourBody = meshManager.getMaxilla();            
         }
      }
      if (contourBody != null) {
         slice.addContourBody (contourBody);
      }
   }

   OcclusalPlane createOcclusalPlane (Point3d p0, Point3d p1, Point3d p2) {
      MeshManager meshManager = getRoot().getMeshManager();
      Vector2d xysize = meshManager.getMandibleXYSize();
      xysize.scale (getPlaneSliceScale());
      OcclusalPlane oplane = new OcclusalPlane (p1, p0, p2, xysize.x, xysize.y);
      return oplane;
   }

   SlicePlane createOcclusalSlice (RigidTransform3d TSW) {
      MeshManager meshManager = getRoot().getMeshManager();

      Point3d cent = meshManager.getMandibleCentroid();
      RigidTransform3d TVI = null;
      if (cent != null) {
         TVI = new RigidTransform3d();
         TVI.p.set (cent);
         TVI.invert();
      }
      
      Vector2d xysize = meshManager.getMandibleXYSize();
      xysize.scale (getPlaneSliceScale());
      double w = xysize.x;
      double h = xysize.y;
      PolygonalMesh mesh = MeshFactory.createCircularSector (w/2, Math.PI, 1, 30);
      mesh.scale (h/(w/2), 1.0, 0);
      mesh.transform (new RigidTransform3d (0, h/2, 0, -Math.PI/2, 0, 0));

      SlicePlane slice =
         new SlicePlane ("occlusalSlice", getDicomViewer(), mesh, TSW, TVI);
      setSliceContours (slice);

      RenderProps.setFaceStyle (slice, FaceStyle.FRONT_AND_BACK);
      RenderProps.setLineColor (slice, Color.RED);
      RenderProps.setLineWidth (slice, 3);
      return slice;
   }

   SlicePlane createSagittalSlice (RigidTransform3d TSW) {
      MeshManager meshManager = getRoot().getMeshManager();

      Point3d cent = meshManager.getMandibleCentroid();
      RigidTransform3d TVI = null;
      if (cent != null) {
         TVI = new RigidTransform3d();
         TVI.p.set (cent);
         TVI.invert();
      }

      Vector2d xysize = meshManager.getMandibleXYSize();
      xysize.x = 100;
      xysize.scale (getPlaneSliceScale());
      double w = xysize.x;
      double h = xysize.y;
      PolygonalMesh mesh =
         MeshFactory.createRectangle (w, h, /*texture=*/false);
      RigidTransform3d TPW = new RigidTransform3d();
      TPW.p.set (TSW.p);
      TPW.mulRotY (-Math.PI/2);

      SlicePlane slice =
         new SlicePlane ("sagittalSlice", getDicomViewer(), mesh, TPW, TVI);

      RenderProps.setFaceStyle (slice, FaceStyle.FRONT_AND_BACK);
      RenderProps.setLineColor (slice, Color.RED);
      RenderProps.setLineWidth (slice, 3);
      return slice;
   }
}

