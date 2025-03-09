package artisynth.VSP.reconstruction;

import java.awt.Color;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Deque;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.HasCoordinateFrame;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableCompositeBase;
import artisynth.core.modelbase.TransformGeometryContext;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.core.modelbase.ScanWriteUtils;
import artisynth.core.workspace.RootModel;
import artisynth.core.util.ScanToken;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.GeometryTransformer;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.AxisAngle;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.HasProperties;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.properties.PropertyUtils;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.AxisDrawStyle;
import maspack.render.Renderer.FaceStyle;
import maspack.util.IndentingPrintWriter;
import maspack.util.FunctionTimer;
import maspack.util.InternalErrorException;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;

/**
 * Custom component representing the cut planes for creating a single segment
 * from a donor, together with a mesh showing what the resulting segment looks
 * like within the reconstruction, and a coordinate frame that allows the
 * sgement to be adjusted by transforming its position in mandible space.
 */
public class DonorSegmentBody extends RenderableCompositeBase 
   implements HasCoordinateFrame, TransformableGeometry {

   static private double INF = Double.POSITIVE_INFINITY;
   static private double DTOR = Math.PI/180.0;

   static private final int POSE_FORMAT = RigidTransform3d.MATRIX_3X4_STRING;

   // setting the pose in mandible space allows the segment pose
   // to be adjusted in its final location within the reconstruction
   static boolean myPoseInMandibleSpace = true;
   
   RigidTransform3d myTSW;   // primary pose
   RigidTransform3d myTDW0;  // reference pose in donor space
   RigidTransform3d myTSM;   // offset frame from M to primary pose
   RigidBody myClippedDonor;        
   FixedMeshBody myMeshBody;

   ArrayList<RigidTransform3d> myPlaneTPS;
   ArrayList<ModelComponent> myPlaneBodies;
   ArrayList<PolygonalMesh> myDonorPlanes;
   ArrayList<PolygonalMesh> myDonorRenderPlanes;

   protected static double DEFAULT_AXIS_LENGTH = 20.0;
   double myAxisLength = DEFAULT_AXIS_LENGTH;

   PolygonalMesh myClipMesh;
   PolygonalMesh myClipRenderMesh;
   PolygonalMesh myClipRenderMeshRef;

   boolean myClipMeshValid = false;

   // property defintions

   protected static final AxisDrawStyle DEFAULT_AXIS_RENDER_STYLE =
      AxisDrawStyle.ARROW;
   protected AxisDrawStyle myAxisDrawStyle = DEFAULT_AXIS_RENDER_STYLE;

   protected static boolean DEFAULT_LIVE_UPDATING = true;
   protected boolean myLiveUpdating = DEFAULT_LIVE_UPDATING;

   protected static double DEFAULT_RENDER_PLANE_WIDTH = 50;
   protected double myRenderPlaneWidth = DEFAULT_RENDER_PLANE_WIDTH;
   protected PropertyMode myRenderPlaneWidthMode = PropertyMode.Inherited;
   
   protected static int DEFAULT_RENDER_PLANE_RES = 30;
   protected int myRenderPlaneRes = DEFAULT_RENDER_PLANE_RES;

   private RigidTransform3d myRenderFrame = new RigidTransform3d();

   protected static RenderProps defaultRenderProps (HasProperties host) {
      RenderProps props = RenderProps.createRenderProps (host);
      return props;
   }

   public static PropertyList myProps =
      new PropertyList (DonorSegmentBody.class, RenderableCompositeBase.class);

   static {
      myProps.add (
         "renderProps", "render properties", defaultRenderProps(null));
      myProps.add (
         "position", "position of the body coordinate frame",null,"NW");
      myProps.add (
         "orientation", "orientation of the body coordinate frame", null, "NW");
      myProps.add (
         "axisLength", "length of rendered frame axes", 1f);
      myProps.add (
         "axisDrawStyle", "rendering style for the frame axes",
         DEFAULT_AXIS_RENDER_STYLE);
      myProps.add (
         "liveUpdating",
         "update mesh as pose changes", DEFAULT_LIVE_UPDATING);
      myProps.addInheritable (
         "renderPlaneWidth",
         "size with which to render the cut planes",
         DEFAULT_RENDER_PLANE_WIDTH);         
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }  


   public Point3d getPosition() {
      return new Point3d (getPose().p);
   }

   public void setPosition (Point3d pos) {
      RigidTransform3d TSW = new RigidTransform3d (getPose());
      TSW.p.set (pos);
      setPose (TSW);
   }

   public AxisAngle getOrientation() {
      return getPose().R.getAxisAngle();
   }

   public void setOrientation (AxisAngle axisAng) {
      RigidTransform3d TSW = new RigidTransform3d (getPose());
      TSW.R.setAxisAngle (axisAng);
      setPose (TSW);
   }

   public double getAxisLength() {
      return myAxisLength;
   }

   public void setAxisLength (double len) {
      myAxisLength = Math.max (0, len);
   }

   public AxisDrawStyle getAxisDrawStyle() {
      return myAxisDrawStyle;
   }

   public void setAxisDrawStyle (AxisDrawStyle style) {
      myAxisDrawStyle = style;
   }

   public boolean getLiveUpdating () {
      return myLiveUpdating;
   }

   public void setLiveUpdating (boolean enable) {
      myLiveUpdating = enable;
   }

   public double getRenderPlaneWidth () {
      return myRenderPlaneWidth;
   }

   public void setRenderPlaneWidth (double size) {
      if (myRenderPlaneWidth != size) {
         myRenderPlaneWidth = size;
         updatePlaneSizes();
      }
      myRenderPlaneWidthMode =
         PropertyUtils.propagateValue (
            this, "renderPlaneWidth", myRenderPlaneWidth, myRenderPlaneWidthMode);
   }

   public void setRenderPlaneWidthMode (PropertyMode mode) {
      double prev = myRenderPlaneWidth;
      myRenderPlaneWidthMode =
         PropertyUtils.setModeAndUpdate (
            this, "renderPlaneWidth", myRenderPlaneWidthMode, mode);
      if (prev != myRenderPlaneWidth) {
         updatePlaneSizes();
      }
   }

   public PropertyMode getRenderPlaneWidthMode() {
      return myRenderPlaneWidthMode;
   }

   // other accessors

   public FixedMeshBody getMeshBody() {
      return myMeshBody;
   }

   public PolygonalMesh getMesh() {
      return (PolygonalMesh)myMeshBody.getMesh();
   }

   public ArrayList<PolygonalMesh> getDonorPlaneMeshes() {
      return myDonorPlanes;
   }

   // constructors

   /**
    * Note: need a public no-args constructor for ArtiSynth save/load.
    */
   public DonorSegmentBody() {
      myTSW = new RigidTransform3d();
      myTSM = new RigidTransform3d();
      myTDW0 = new RigidTransform3d();
      myPlaneTPS = new ArrayList<>();
      myPlaneBodies = new ArrayList<>();
      myDonorPlanes = new ArrayList<>();
      setRenderProps (defaultRenderProps(null));
      myMeshBody = new FixedMeshBody ("meshBody");
      RenderProps.setFaceColor (myMeshBody, ReconAppRoot.BONE_COLOR);
      RenderProps.setBackColor (myMeshBody, Color.GRAY);
      RenderProps.setFaceStyle (myMeshBody, FaceStyle.FRONT);
      add (myMeshBody);      
   }

   /**
    * Creates a DonorSegmentBody given its defining transformations and donor
    * mesh.
    *
    * @param TSW_M transform from segment to world, in mandible space
    * @param TSW_D transform from segment to world, in donor spac
    * @param TPS0 transform plane 0 to the segment
    * @param TPS1 transform plane 1 to the segment
    * @param clippedDonor clipped donor mesh
    */
   public DonorSegmentBody (
      RigidTransform3d TSW_M, RigidTransform3d TSW_D,
      RigidBody clippedDonor, Object... planeInfo) {

      this();
      myTDW0.set (TSW_D);
      if (myPoseInMandibleSpace) {
         setPose (TSW_M);
      }
      else {
         setPoseD (TSW_D);
      }
      // set mesh body pose now since this stores PoseM
      myMeshBody.setPose (TSW_M);
      myClippedDonor = clippedDonor;
      int pbodyCnt = 0;
      for (int i=0; i<planeInfo.length; i++) {
         Object pinfo = planeInfo[i];
         if (pinfo instanceof RigidTransform3d) {
            addCutPlane ((RigidTransform3d)pinfo);
            myPlaneBodies.add (null);
         }
         else if (pinfo instanceof ModelComponent &&
                  pinfo instanceof HasCoordinateFrame) {
            myPlaneBodies.add ((ModelComponent)pinfo);
            addCutPlane (computeTPS ((ModelComponent)pinfo));
            pbodyCnt++;
         }
         else {
            throw new InternalErrorException (
               "Unexpecting plane info object "+i+": "+pinfo);
         }
      }
      if (pbodyCnt == 0) {
         myPlaneBodies.clear();
      }
   }

   // pose control and accessors for plane and coordinate frames

   /*
     Note on poses. Let TSW give the pose of this segment.

     1) If the pose is given in mandible space, then the pose in donor space
     TDW is given by

       TDW = TDW0 inv(TMW) TMW0
     
     where TDW0 and TMW0 are the reference poses in donor and mandible space,
     and TMW is the current mandible frame which is related to TSW by

       TMW = TSW inv(TSM)

     Solving the top equation for TMW, we get

       TMW = TMW0 inv(TDW) TDW0

     and hence
     
       TSW = TMW0 inv(TDW) TDW0 TSM
  
     2) If the pose is not given in mandible space, then TSW equals
     the pose in donor space, so that

       TDW = TSW

     and the reference pose TDW0 is simply the initial value of TSW.
   */

   /**
    * Returns the segment's reference pose in donor space. The value of this
    * will remain fixed unless it is set using {@link #setPoseD0}.
    *
    * @return reference pose in donor space. Should not be modified.
    */
   public RigidTransform3d getPoseD0 () {
      return new RigidTransform3d(myTDW0);
   }

   /**
    * Sets the segment's reference pose in donor space, and also
    * resets poseD to this value (so that the current and reference
    * poses are the same). 
    *
    * @param TDW0 new value of the reference pose
    */
   public void setPoseD0 (RigidTransform3d TDW0) {
      if (myPoseInMandibleSpace) {
         // see note on poses above
         RigidTransform3d TSW = new RigidTransform3d();
//         TSW.mulInverseRight (getPoseM0(), getPoseD());
//         TSW.mul (TDW0);
//         TSW.mul (myTSM);
         TSW.mul (getPoseM0(), myTSM);
         myTDW0.set (TDW0);
         setPose (TSW);
      }
      else {
         myTDW0.set (TDW0);
         setPose (TDW0);
      }
   }

   /**
    * Returns the segment's current pose in donor space.
    *
    * @return current pose in donor space. Should not be modified.
    */
   public RigidTransform3d getPoseD () {
      if (myPoseInMandibleSpace) {
         // see note on poses above
         RigidTransform3d TDW = new RigidTransform3d();
         RigidTransform3d TMW = new RigidTransform3d();
         TMW.mulInverseRight (getPose(), myTSM);
         TDW.mulInverseRight (myTDW0, TMW);
         TDW.mul (getPoseM0());
         return TDW;
      }
      else {
         return getPose();
      }
   }

   /**
    * Sets the segment's current pose in donor space.
    *
    * @param TDW new pose value in donor space
    */
   public void setPoseD (RigidTransform3d TDW) {
      if (myPoseInMandibleSpace) {
         // see note on poses above
         RigidTransform3d TSW = new RigidTransform3d();
         TSW.mulInverseRight (getPoseM0(), TDW);
         TSW.mul (getPoseD0());
         TSW.mul (myTSM);
         setPose (TSW);
      }
      else {
         setPose (TDW);
      }
   }

   /**
    * Adjust's both TDW and TDW0 to account for a change in the donor setback
    * (in which both transforms are shifted by {@code inc} along the y axis in
    * world space.
    *
    * @param inc
    */
   public void adjustDonorSetback (double inc) {
      RigidTransform3d TDY = new RigidTransform3d (0, inc, 0);
      myTDW0.mul (TDY, myTDW0);
      if (!myPoseInMandibleSpace) {
         myTSW.mul (TDY, myTSW);
      }
      updateWorldPlanePoses();
   }

   /**
    * Returns the segment's reference pose in mandible space. The value of this
    * should remain constant as the segment moves about.
    *
    * @return reference pose in mandible space. Should not be modified.
    */
   public RigidTransform3d getPoseM0() {
      return myMeshBody.getPose();
   }


   /**
    * Returns the segments's current pose.
    *
    * @return current pose. Should not be modified.
    */
   public RigidTransform3d getPose() {
      return myTSW;
   }

   /**
    * Returns the segments's current pose.
    *
    * @param TSW returns the current pose
    */
   public void getPose (RigidTransform3d TSW) {
      TSW.set (myTSW);
   }

   /**
    * Sets the segment's current pose.
    *
    * @param TSW new segment pose
    */
   public void setPose (RigidTransform3d TSW) {
      myTSW.set (TSW);
      updateWorldPlanePoses();
      updateDonorMesh();
   }

   /**
    * Sets the segment's current pose.
    *
    * @param TSW new segment pose
    */
   public void setPose (
      double px, double py, double pz,
      double ux, double uy, double uz, double deg) {
      RigidTransform3d TSW = new RigidTransform3d();
      TSW.p.set (px, py, pz);
      TSW.R.setAxisAngle (ux, uy, uz, DTOR*deg);;
      setPose (TSW);
   }

   /**
    * Returns the segments's pose offset.
    *
    * @return pose offset from S to M
    */
   public RigidTransform3d getPoseOffset() {
      return myTSM;
   }

   /**
    * Sets the segments's pose offset. Used only when the pose is in mandible
    * space, this a transform from the segment pose S to the mandible frame
    * M. It is provided to make it easier to manipulate the segment using
    * dragger fixtures.
    *
    * @param TSW new pose offset
    */
   public void setPoseOffset (RigidTransform3d TSM) {

      if (myPoseInMandibleSpace) {
         // reset TSW to account for the new TSM'.
         // Assume current pose is given by TSW = TSW0 X, with TSW0 = TMW0 TSM.
         // Then: X = inv(TSM) inv(TMW0) TSW, and so the new pose is
         // TSW = TMW0 TSM' inv(TSM) inv(TMW0) TSW
         RigidTransform3d TMW0 = getPoseM0();
         myTSW.mulInverseLeft (TMW0, myTSW);
         myTSW.mulInverseLeft (myTSM, myTSW);
         myTSW.mul (TSM, myTSW);
         myTSW.mul (TMW0, myTSW);
      }
      myTSM.set (TSM);
   }

   /**
    * Computes a plane-to-world transform, given segment-to-world and
    * plane-to-segment transforms.
    *
    * @param TSW segment-to-world transform
    * @param TPS plane-to-segment transform
    * @return plane-to-world transform
    */
   static RigidTransform3d getTPW (
      RigidTransform3d TSW, RigidTransform3d TPS) { 
      RigidTransform3d TPW = new RigidTransform3d();
      TPW.mul (TSW, TPS);
      return TPW;
   }
   
   public RigidTransform3d getTPW_D (int idx) {
      RigidTransform3d TPW = new RigidTransform3d();
      TPW.mul (getPoseD(), myPlaneTPS.get(idx));
      return TPW;
   }

   public RigidTransform3d getTPW_M (int idx) {
      RigidTransform3d TPW = new RigidTransform3d();
      TPW.mul (getPoseM0(), myPlaneTPS.get(idx));
      return TPW;
   }

   public Plane getPlaneD (int idx) {
      return new Plane (getTPW_D (idx));
   }

   public Plane getPlaneM (int idx) {
      return new Plane (getTPW_M (idx));
   }

   public void updateDonorMesh() {
      if (myClippedDonor != null && myLiveUpdating) {
         myMeshBody.setMesh (createDonorMesh());
      }
   }

   public void clearDonorMesh() {
      myMeshBody.setMesh (null);
   }

   public ReconAppRoot getRoot() {
      RootModel root = RootModel.getRoot (this);
      if (root instanceof ReconAppRoot) {
         return (ReconAppRoot)root;
      }
      else {
         return null;
      }
   }

   /**
    * Creates a mesh to implement a cut plane.
    */
   PolygonalMesh createCutPlaneMesh () {
      ReconAppRoot root = getRoot();
      double width = root.getCutPlaneWidth();
      int res = root.getPlaneResolution();
      return MeshFactory.createPlane (width, width, res, res);
   }

   /**
    * Creates a mesh to render a plane
    */
   PolygonalMesh createRenderPlaneMesh () {
      double width = getRenderPlaneWidth();
      int res = myRenderPlaneRes;
      return MeshFactory.createPlane (width, width, res, res);
   }

   private void updateClipMesh() {
      if (myPlaneTPS.size()==3) {
         myClipMesh =
            createScapulaClipMesh(getRoot().getCutPlaneWidth());
         myClipRenderMesh =
            createScapulaClipMesh(getRoot().getRenderPlaneWidth());
      }
      else {
         myClipMesh =
            createFibulaClipMesh(getRoot().getCutPlaneWidth());
         myClipRenderMesh =
            createFibulaClipMesh(getRoot().getRenderPlaneWidth());
      }
      myClipMeshValid = true;
   }

   public PolygonalMesh createDonorMesh() {
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();
      PolygonalMesh mesh = myClippedDonor.getSurfaceMesh();;
      FunctionTimer timer = new FunctionTimer();
      if (myPlaneTPS.size() > 0) {
         if (!myClipMeshValid) {
            updateClipMesh();
         }
         //System.out.println ("createDonorMesh");
         if (myClipMesh != null) {
            myClipMesh.setMeshToWorld (getPoseD());
            timer.start();
            mesh = intersector.findDifference01 (mesh, myClipMesh);
            timer.stop();
            //System.out.println (" clipMesh "+timer.result(1));
         }
         else {
            PolygonalMesh cutPlane = createCutPlaneMesh();
            int k = 0;
            for (RigidTransform3d TPS : myPlaneTPS) {
               cutPlane.setMeshToWorld (getTPW (getPoseD(), TPS));
               timer.start();
               mesh = intersector.findDifference01 (mesh, cutPlane);
               timer.stop();
               //System.out.println (" "+k+" "+timer.result(1));
            } 
         }
         
      }
      else {
         // no cut planes. May happend if we are debugging. Need to copy mesh
         // and patch up transforms.
         mesh = mesh.clone();
         mesh.transform (mesh.getMeshToWorld());
         mesh.setMeshToWorld (new RigidTransform3d());
      }
      mesh.inverseTransform (getPoseD());
      // Hack. myMeshBody.setMesh() will set the render props to those of the
      // mesh, so we preset those here
      mesh.setRenderProps (myMeshBody.getRenderProps());
      return mesh;      
   }

   public PolygonalMesh createComplementMesh() {
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();
      PolygonalMesh mesh = myClippedDonor.getSurfaceMesh();;
      if (myPlaneTPS.size() > 0) {
         if (!myClipMeshValid) {
            updateClipMesh();
         }
         if (myClipMesh != null) {
            myClipMesh.setMeshToWorld (getPoseD());
            mesh = intersector.findIntersection (mesh, myClipMesh);
         }
         else {
            return null;
         }
      }
      else {
         // no cut planes. May happend if we are debugging. Need to copy mesh
         // and patch up transforms.
         mesh = mesh.clone();
         mesh.transform (mesh.getMeshToWorld());
         mesh.setMeshToWorld (new RigidTransform3d());
      }
      return mesh;      
   }

   /**
    * Create a version of the donor mesh that is extended by a small
    * amount in the opposite direction of each cut plane. This
    * is to enable CSG operations involving the union of the
    * segments.
    *
    * @param ext amount to extend the mesh by near each cut plane
    * @param tol tolerance by which to decide if a vertex is on a cut plane
    */
   public PolygonalMesh createExtendedMesh (double ext, double tol) {
      //PolygonalMesh mesh = (PolygonalMesh)myMeshBody.getMesh().clone();
      PolygonalMesh mesh = new PolygonalMesh((PolygonalMesh)myMeshBody.getMesh());
      Point3d pos = new Point3d();
      for (RigidTransform3d TPS : myPlaneTPS) {
         Plane plane = new Plane(TPS);
         // assume cut planes are directed *into* the segment, so we want to
         // project in the negative direction relative to the normal
         ComputeUtils.extendVerticesNearPlane (mesh, plane, -ext, tol);
      }
      return mesh;
   }

   protected void addCutPlane (RigidTransform3d TPS) {
      myPlaneTPS.add (TPS);
      PolygonalMesh plane = createRenderPlaneMesh();
      plane.setMeshToWorld (getTPW (getPoseD(), TPS));
      myDonorPlanes.add (plane);
   }

   void updateWorldPlanePoses() {
      for (int i=0; i<myDonorPlanes.size(); i++) {
         PolygonalMesh plane = myDonorPlanes.get(i);
         plane.setMeshToWorld (getTPW (getPoseD(), myPlaneTPS.get(i)));
      }
      if (myClipMesh != null) {
         myClipMesh.setMeshToWorld (getPoseD());
         myClipRenderMesh.setMeshToWorld (getPoseD());
      }
   }

   private RigidTransform3d computeTPS (ModelComponent pbody) {
      RigidTransform3d TPS = new RigidTransform3d();
      ((HasCoordinateFrame)pbody).getPose (TPS);
      TPS.mulInverseLeft (getPoseM0(), TPS);
      return TPS;
   }

   /**
    * Queries whether this body contains a reference to a specified
    * plane body.
    *
    * @return {@code true} if this body refers to planeBody
    */
   boolean containsPlaneBody (ModelComponent pbody) {
      return myPlaneBodies.contains (pbody);
   }

   void updateLocalPlanePoses() {
      for (int i=0; i<myPlaneBodies.size(); i++) {
         ModelComponent pbody = myPlaneBodies.get(i);
         if (pbody != null) {
            myPlaneTPS.get(i).set (computeTPS (myPlaneBodies.get(i)));
         }
      }
      myClipMeshValid = false;
   }

   private void updatePlaneSizes() {
      ArrayList<PolygonalMesh> planes = new ArrayList<>();
      for (RigidTransform3d TPS : myPlaneTPS) {
         PolygonalMesh plane = createRenderPlaneMesh();
         plane.setMeshToWorld (getTPW (getPoseD(), TPS));
         planes.add (plane);
      }
      if (myClipMeshValid && myClipRenderMesh != null) {
         if (myPlaneTPS.size()==3) {
            myClipRenderMesh =
               createScapulaClipMesh(getRoot().getRenderPlaneWidth());
         }
         else {
            myClipRenderMesh =
               createFibulaClipMesh(getRoot().getRenderPlaneWidth());
         }
         myClipRenderMesh.setMeshToWorld (getPoseD());
      }
      myDonorPlanes = planes;
   }

   // transform geometry methods

   public void addTransformableDependencies (
      TransformGeometryContext context, int flags) {
   }

   public void transformGeometry(AffineTransform3dBase X) {
      TransformGeometryContext.transform (this, X, 0);
   }

   public void transformGeometry (
      GeometryTransformer gtr, TransformGeometryContext context, int flags) {

      // transform the pose
      RigidTransform3d TSW = new RigidTransform3d();
      TSW.set (getPose());
      gtr.transform (TSW);
      setPose (TSW);
   } 

   // renderable methods

   public RenderProps createRenderProps() {
      return defaultRenderProps (this);
   }

   public void render (Renderer renderer, int flags) {
      if (isSelected()) {
         flags |= Renderer.HIGHLIGHT;
      }
      if (myClipRenderMeshRef != null) {
         RenderProps props = new RenderProps (myRenderProps);
         myClipRenderMeshRef.render (renderer, myRenderProps, flags);
      }
      else if (myDonorRenderPlanes != null) {
         ArrayList<PolygonalMesh> planes = myDonorRenderPlanes;
         for (PolygonalMesh plane : planes) {
            plane.render (renderer, myRenderProps, flags);
         }
      }
      if (myAxisLength > 0) {
         renderer.drawAxes (
            myRenderFrame, myAxisDrawStyle, 
            myAxisLength, myRenderProps.getLineWidth(), 0, isSelected());
      }
      // render mesh body directly so we can't select it
      FixedMeshBody mcomp = myMeshBody;
      RenderProps props = mcomp.getRenderProps();
      if (props.isVisible()) {
         if (mcomp.isSelected()) {
            flags |= Renderer.HIGHLIGHT;
         }
         mcomp.render (renderer, props, flags);
         if (mcomp.getAxisLength() > 0) {
            renderer.drawAxes (
               getPoseM0(), mcomp.getAxisDrawStyle(), mcomp.getAxisLength(),
               props.getLineWidth(), 0, mcomp.isSelected());
         }
      }     
   }

   public void prerender (RenderList list) {
      myDonorRenderPlanes = myDonorPlanes;
      myClipRenderMeshRef = myClipRenderMesh;
      if (myClipRenderMeshRef != null) {
         myClipRenderMeshRef.prerender (myRenderProps);
      }
      else if (myDonorRenderPlanes != null) {
         for (PolygonalMesh plane : myDonorRenderPlanes) {
            plane.prerender (myRenderProps);
         }
      }
      MeshComponent mcomp = myMeshBody;
      if (mcomp.getRenderProps().isVisible()) {
         mcomp.prerenderMesh();
      }
      myRenderFrame.set (getPoseD());
   }

   public void updateBounds (Vector3d pmin, Vector3d pmax) {
      // if (myDonorPlanes != null) {
      //    for (PolygonalMesh plane : myDonorPlanes) {
      //       plane.updateBounds (pmin, pmax);
      //    }
      // }
      myMeshBody.updateBounds (pmin, pmax);
   }


   /**
    * Write out the transforms for this segment.
    */
   protected void writeTransforms (
      PrintWriter pw, NumberFormat fmt) 
      throws IOException {
      pw.print ("[ ");
      IndentingPrintWriter.addIndentation (pw, 2);
      pw.println ("TSW_M0=\n"+getPoseM0().toString (fmt, POSE_FORMAT));
      RigidTransform3d TSW_D = new RigidTransform3d (getPoseD());
      double setback = getRoot().getDonorSetBack();
      if (setback != 0) {
         TSW_D.mul (new RigidTransform3d (0, -setback, 0), TSW_D);
      }
      pw.println ("TSW_D=\n" + TSW_D.toString (fmt, POSE_FORMAT));
      pw.println ("planes=[");
      IndentingPrintWriter.addIndentation (pw, 2);
      for (RigidTransform3d TPS : myPlaneTPS) {
         pw.println (TPS.toString (fmt, POSE_FORMAT));
      }
      IndentingPrintWriter.addIndentation (pw, -2);
      pw.println ("]");      
      IndentingPrintWriter.addIndentation (pw, -2);
      pw.println ("]");      
   }

   // I/O method overrides to write and scan quantities not stored as
   // properties


   /**
    * {@inheritDoc}
    */
   @Override
   protected void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor) 
      throws IOException {
      super.writeItems (pw, fmt, ancestor);
      pw.println ("TDW0=" + myTDW0.toString (fmt, POSE_FORMAT));
      pw.println ("TSW=" + myTSW.toString (fmt, POSE_FORMAT));
      if (!myTSM.isIdentity()) {
         pw.println ("TSM=" + myTSM.toString (fmt, POSE_FORMAT));
      }
      pw.println ("planes=[");
      IndentingPrintWriter.addIndentation (pw, 2);
      for (RigidTransform3d TPS : myPlaneTPS) {
         pw.println (TPS.toString (fmt, POSE_FORMAT));
      }
      IndentingPrintWriter.addIndentation (pw, -2);
      pw.println ("]");      
      if (myPlaneBodies.size() > 0) {
         pw.println ("planeBodies=");
         ScanWriteUtils.writeBracketedReferences (
            pw, myPlaneBodies, ancestor);
      }     
      if (myClippedDonor != null) {
         pw.println (
            "clippedDonor=" +
            ComponentUtils.getWritePathName (ancestor, myClippedDonor));
      }
   }

   /**
    * {@inheritDoc}
    */
   @Override
   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {
      rtok.nextToken();
      if (scanAndStoreReference (rtok, "clippedDonor", tokens)) {
         return true;
      }
      else if (scanAttributeName (rtok, "TDW0")) {
         myTDW0.scan (rtok);
         return true;
      }
      else if (scanAttributeName (rtok, "TSW")) {
         myTSW.scan (rtok);
         return true;
      }
      else if (scanAttributeName (rtok, "TSM")) {
         myTSM.scan (rtok);
         return true;
      }
      else if (scanAttributeName (rtok, "planes")) {
         myPlaneTPS.clear();
         myDonorPlanes.clear();
         rtok.scanToken ('[');
         while (rtok.nextToken() != ']') {
            rtok.pushBack();
            RigidTransform3d TPS = new RigidTransform3d();
            TPS.scan (rtok);
            addCutPlane (TPS);
         }
         return true;
      }
      else if (scanAndStoreReferences (rtok, "planeBodies", tokens) >= 0) {
         return true;
      }
      rtok.pushBack();
      return super.scanItem (rtok, tokens);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   protected boolean postscanItem (
   Deque<ScanToken> tokens, CompositeComponent ancestor) throws IOException {
      if (postscanAttributeName (tokens, "clippedDonor")) {
         myClippedDonor = postscanReference (tokens, RigidBody.class, ancestor);
         return true;
      }   
      else if (postscanAttributeName (tokens, "planeBodies")) {
         myPlaneBodies.clear();
         ScanWriteUtils.postscanReferences (
            tokens, myPlaneBodies, ModelComponent.class, ancestor);
         return true;
      }
      return super.postscanItem (tokens, ancestor);
   }
   
   void alignYZ (DonorSegmentBody prev) {
      RigidTransform3d TSW1 = new RigidTransform3d(getPoseD());
      SegmentGenerator.alignYZ (TSW1, prev.getPoseD());
      setPoseD (TSW1);
   }

   /**
    * Create a specialized clipping mesh for use with scapula donors.  It
    * combines the right and left resection planes, bridged by the trim
    * plane. These three planes are collectively intersected by the planes y =
    * w/2, y = -w/2, and x = -w/2, where w is the nominal width of the mesh.
    *
    * This intersection produces three faces of a distorted cube with 8
    * vertices. We first compute the vertex coordinates, and the use
    * interpolation create the intermediate vertices which are mapped onto the
    * vertices of a rectangular triangular mesh.
    */
   private PolygonalMesh createScapulaClipMesh (double w) {

      Plane planeL = new Plane(myPlaneTPS.get(0));
      Plane planeR = new Plane(myPlaneTPS.get(1));
      Plane planeT = new Plane(myPlaneTPS.get(2));

      PolygonalMesh clipMesh = 
         ComputeUtils.createThreeSegmentClipMesh (
            new Plane[] { planeL, planeR, planeT },
            w, getRoot().getPlaneResolution(), 
            /*trimPlaneOnTop=*/myPlaneTPS.get(2).p.x > 0,
            /*fillBack=*/false);
      clipMesh.setMeshToWorld (getPoseD());
      return clipMesh;
   }

   /**
    * Create a specialized clipping mesh for use with fibula donors.  It
    * simply combines the right and left resection planes.
    */
   private PolygonalMesh createFibulaClipMesh (double w) {

      Plane planeL = new Plane(myPlaneTPS.get(0));
      Plane planeR = new Plane(myPlaneTPS.get(1));
      PolygonalMesh clipMesh = new PolygonalMesh();
      PolygonalMesh mesh;

      int res = getRoot().getPlaneResolution();
      mesh = MeshFactory.createPlane (w, w, res, res);
      mesh.transform (myPlaneTPS.get(0));
      clipMesh.addMesh (mesh);

      mesh = MeshFactory.createPlane (w, w, res, res);
      mesh.transform (myPlaneTPS.get(1));
      clipMesh.addMesh (mesh);

      clipMesh.setMeshToWorld (getPoseD());
      return clipMesh;
   }
}
