package artisynth.VSP.reconstruction;

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
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.util.ScanToken;
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
 * Worker component for managing the meshes, resection planes, clipped meshes
 * and some marker locations associated with the reconstruction objects.
 */
public class MeshManager extends WorkerComponentBase {

   public enum PlaneFormat {
      ARTISYNTH,
      SLICER,
      BLENDER
   }

   public enum MarkerFormat {
      ARTISYNTH,
      SLICER
   }

   public static String myDefaultMeshFileExtension = "stl";

   private static double INF = Double.POSITIVE_INFINITY;
   private static double DTOR = Math.PI/180; // degrees to radians

   // offset in X,Y directions for new resection planes
   double myNewResectPlaneOff = 25.0;

   Point3d myMandibleCentroid = null;
   Point3d myMaxillaCentroid = null;
   Vector2d myMandibleXYSize = null;

   File myMandibleMarkerFile = null;
   MeshManager.MarkerFormat myMarkerFormat = MeshManager.MarkerFormat.ARTISYNTH;

   PolygonalMesh myResectionMesh;

   // properties:

   static double DEFAULT_PROX_TRIM_DISTANCE = 80;
   double myProxTrimDistance = DEFAULT_PROX_TRIM_DISTANCE;

   static double DEFAULT_DISTAL_TRIM_DISTANCE = 80;
   double myDistalTrimDistance = DEFAULT_DISTAL_TRIM_DISTANCE;

   static double DEFAULT_MAX_DONOR_RADIUS = 0;
   double myMaxDonorRadius = DEFAULT_MAX_DONOR_RADIUS;

   protected static double DEFAULT_RENDER_PLANE_WIDTH = 50;
   protected double myRenderPlaneWidth = DEFAULT_RENDER_PLANE_WIDTH;
   protected PropertyMode myRenderPlaneWidthMode = PropertyMode.Inherited;

   public static PropertyList myProps =
      new PropertyList (
         MeshManager.class, WorkerComponentBase.class);

   static {
      myProps.add (
         "proxTrimDistance",
         "amount to trim for the proxmal end of the fibula",
         DEFAULT_PROX_TRIM_DISTANCE);
      myProps.add (
         "distalTrimDistance",
         "amount to trim for the distal end of the fibula",
         DEFAULT_DISTAL_TRIM_DISTANCE);
      myProps.add (
         "maxDonorRadius",
         "max radius of donor about the centerline",
         DEFAULT_MAX_DONOR_RADIUS, "NS");
      myProps.addInheritable (
         "renderPlaneWidth",
         "size with which to render the cut planes",
         DEFAULT_RENDER_PLANE_WIDTH);
     myProps.add (
         "expandedMandibleVisible isExpandedMandibleVisible",
         "make expanded mandible visible", false);
   }
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public double getProxTrimDistance () {
      return myProxTrimDistance;
   }

   public void setProxTrimDistance (double dist) {
      this.myProxTrimDistance = dist;
   }

   public double getDistalTrimDistance () {
      return myDistalTrimDistance;
   }

   public void setDistalTrimDistance (double dist) {
      this.myDistalTrimDistance = dist;
   }

   public void setMaxDonorRadius (double rad) {
      this.myMaxDonorRadius = rad;
   }

   public double getMaxDonorRadius () {
      return myMaxDonorRadius;
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

   /**
    * Queries whether or not the expanded mandible is visible.
    */
   public boolean isExpandedMandibleVisible () {
      return isVisible (getExpandedMandible());
   }

   /**
    * Sets whether or not the expanded mandible is visible.
    */
   public void setExpandedMandibleVisible (boolean enable) {
      if (setVisible (getExpandedMandible(), enable)) {
         getRoot().rerender();
      }
   }

   // other accessors

   public String getDefaultMeshFileExtension() {
      return myDefaultMeshFileExtension;
   }

   public Point3d getMandibleCentroid() {
      return myMandibleCentroid;
   }

   public Vector2d getMandibleXYSize() {
      Vector2d size = new Vector2d();
      if (myMandibleXYSize == null) {
         PolygonalMesh mesh = getMandibleMesh();
         if (mesh != null) {
            Point3d min = new Point3d (INF, INF, INF);
            Point3d max = new Point3d (-INF, -INF, -INF);
            mesh.updateBounds (min, max);
            myMandibleXYSize = new Vector2d (
               2*Math.max(Math.abs(min.x), Math.abs(max.x)),
               2*Math.max(Math.abs(min.y), Math.abs(max.y)));
         }
      }
      if (myMandibleXYSize != null) {
         size.set(myMandibleXYSize);
      }
      return size;      
   }

   public void setMandibleCentroid (Point3d cent) {
      myMandibleCentroid = cent;
   }

   public Point3d getMaxillaCentroid() {
      return myMaxillaCentroid;
   }

   public void setMaxillaCentroid (Point3d cent) {
      myMaxillaCentroid = cent;
   }

   // constructors

   /**
    * Note: need a public no-args constructor for ArtiSynth save/load.
    */
   public MeshManager () {
      setRenderProps (defaultRenderProps(this));
   }

   public MeshManager (String name) {
      this();
      setName (name);
   }

   // support for the mandible

   public boolean hasMandible() {
      return getMandible() != null;
   }

   public RigidBody getMandible() {
      return getRigidBody ("mandible");
   }

   public boolean hasExpandedMandible() {
      return getExpandedMandible() != null;
   }

   public RigidBody getExpandedMandible() {
      return getRigidBody ("expandedMandible");
   }

   public PolygonalMesh getMandibleMesh() {
      return getRigidBodyMesh ("mandible");
   }

   public RigidBody setMandible (PolygonalMesh mesh, String fileName) {
      Point3d cent = new Point3d();
      mesh.computeCentroid (cent);
      RigidBody body = setRigidBody ("mandible", mesh, fileName);
      clearAngleMarkers();
      setMandibleCentroid (cent);
      myMandibleXYSize = null;
      return body;
   }

   public boolean removeMandible() {
      if (removeRigidBody ("mandible")) {
         removeRigidBody ("expandedMandible");
         clearAngleMarkers();
         myMandibleXYSize = null;
         return true;
      }
      else {
         return false;
      }
   }

   // support for the maxilla

   public boolean hasMaxilla() {
      return getMaxilla() != null;
   }

   public RigidBody getMaxilla() {
      return getRigidBody ("maxilla");
   }

   public PolygonalMesh getMaxillaMesh() {
      return getRigidBodyMesh ("maxilla");
   }

   public RigidBody setMaxilla (PolygonalMesh mesh, String fileName) {
      return setMaxilla (mesh, fileName, true);
   }

   public RigidBody setMaxilla (
      PolygonalMesh mesh, String fileName, boolean centerPose) {
      Point3d cent = new Point3d();
      mesh.computeCentroid (cent);
      RigidBody body = setRigidBody ("maxilla", mesh, fileName, centerPose);
      setMaxillaCentroid (cent);
      return body;
   }

   public boolean removeMaxilla() {
      if (removeRigidBody ("maxilla")) {
         clearAngleMarkers();
         return true;
      }
      else {
         return false;
      }
   }

   // support for angle markers

   public boolean hasAngleMarkers() {
      return getAngleMarkers().size() > 0;
   }

   public void addAngleMarker (Point3d pworld) {
      RigidBody mandible = getRigidBody ("mandible");
      if (mandible == null) {
         throw new InternalErrorException ("mandible not present");
      }
      Point3d loc = new Point3d();
      loc.inverseTransform (mandible.getPose(), pworld);
      FrameMarker mkr = new FrameMarker (mandible, loc);
      getAngleMarkers().add (mkr);
   }

   public boolean removeAngleMarker (FrameMarker mkr) {
      return getAngleMarkers().remove (mkr);
   }

   public boolean clearAngleMarkers() {
      PointList<FrameMarker> markers = getAngleMarkers();
      if (markers.size() > 0) {
         markers.removeAll();
         return true;
      }
      else {
         return false;
      }
   }

   public PointList<FrameMarker> getAngleMarkers() {
      return (PointList<FrameMarker>)myMech.get ("angleMarkers");
   }

   // support for occlusal plane

   public boolean hasOcclusalPlane() {
      return getOcclusalPlane() != null;
   }

   public OcclusalPlane getOcclusalPlane() {
      return (OcclusalPlane)getRigidBody ("occlusalPlane");
   }

   public void addOcclusalPlane (OcclusalPlane oplane) {
      setRigidBody ("occlusalPlane", oplane);
   }

   public boolean clearOcclusalPlane () {
      return removeRigidBody ("occlusalPlane");
   }

   // support for implants

   public boolean hasImplants() {
      return getImplants().size() > 0;
   }

   public void addImplant (RigidBody implant) {
      getImplants().add (implant);
   }

   public boolean removeImplant (RigidBody implant) {
      return getImplants().remove (implant);
   }

   public boolean clearImplants() {
      RenderableComponentList<RigidBody> implants = getImplants();
      if (implants.size() > 0) {
         implants.removeAll();
         return true;
      }
      else {
         return false;
      }
   }

   public RenderableComponentList<RigidBody> getImplants() {
      return (RenderableComponentList<RigidBody>)myMech.get ("implants");
   }

   // support for the donor 

   public boolean hasDonor() {
      return getDonor() != null;
   }

   public RigidBody getDonor() {
      return getRigidBody ("donor");
   }

   public PolygonalMesh getDonorMesh() {
      return getRigidBodyMesh ("donor");
   }

   public RigidBody setDonor (PolygonalMesh mesh, String fileName) {
      RigidBody body = setRigidBody ("donor", mesh, fileName);
      if (getRoot().getDonorType() == DonorType.SCAPULA) {
         fitScapulaFrame (body);
      }
      double setb = getRoot().getDonorSetBack();
      if (setb != 0) {
         adjustDonorSetback (setb);
      }
      return body;
   }

   public void adjustDonorSetback (double inc) {
      RigidBody body = getDonor();
      if (body != null) {
         RigidTransform3d TDW = new RigidTransform3d (body.getPose());
         TDW.mul (new RigidTransform3d (0, inc, 0), TDW);
         body.setPose (TDW);
      }
      body = getClippedDonor();
      if (body != null) {
         RigidTransform3d TDW = new RigidTransform3d (body.getPose());
         TDW.mul (new RigidTransform3d (0, inc, 0), TDW);
         body.setPose (TDW);
      }
      Spline3dBody curve = getCurve ("donorCurve");
      if (curve != null) {
         RigidTransform3d TDW = new RigidTransform3d (curve.getPose());
         TDW.mul (new RigidTransform3d (0, inc, 0), TDW);
         curve.setPose (TDW);
      }
   }

   public boolean removeDonor() {
      if (removeRigidBody ("donor")) {
         clearAngleMarkers();
         return true;
      }
      else {
         return false;
      }
   }

   // support for donor markers

   public boolean hasDonorMarkers() {
      return getDonorMarkers().size() > 0;
   }

   private RigidBody getDonorMarkerBody() {
      RigidBody donor;
      String donorName;
      if (getRoot().getDonorType() == DonorType.SCAPULA) {
         donorName = "donor";
      }
      else {
         donorName = "clippedDonor";
      }
      donor = getRigidBody (donorName);
      if (donor == null) {
         throw new InternalErrorException (donorName + " not present");
      }
      return donor;
   }

   public void addDonorMarker (Point3d pworld) {
      RigidBody donor = getDonorMarkerBody();
      Point3d loc = new Point3d();
      loc.inverseTransform (donor.getPose(), pworld);
      FrameMarker mkr = new FrameMarker (donor, loc);
      getDonorMarkers().add (mkr);
   }

   public void addDonorMarkerLoc (Point3d ploc) {
      RigidBody donor = getDonorMarkerBody();
      FrameMarker mkr = new FrameMarker (donor, ploc);
      getDonorMarkers().add (mkr);
   }

   public boolean removeDonorMarker (FrameMarker mkr) {
      return getDonorMarkers().remove (mkr);
   }

   public boolean clearDonorMarkers() {
      PointList<FrameMarker> markers = getDonorMarkers();
      if (markers.size() > 0) {
         markers.removeAll();
         return true;
      }
      else {
         return false;
      }
   }

   public PointList<FrameMarker> getDonorMarkers() {
      return (PointList<FrameMarker>)myMech.get ("donorMarkers");
   }

   // support for the clipped donor

   /**
    * Find the donor center curve as follows: in donor coordinates, find the
    * intersection contours between the clipped mesh and a plane perpendicular
    * to the z axis at {@code npnts} points along the z axis. Then fit a single
    * segment cubic spline to these points, with the spline parameterized using
    * the z coordinate value.
    */
   public double findDonorCenterCurve (
      CubicHermiteSpline3d curve, PolygonalMesh clippedMesh, int npnts) {

      double maxz = -INF;
      double minz =  INF;
      for (Vertex3d vtx : clippedMesh.getVertices()) {
         double z = vtx.pnt.z;
         if (z > maxz) {
            maxz = z;
         }
         if (z < minz) {
            minz = z;
         }
      }
      double lenz = maxz-minz;
      if (lenz == 0) {
         return 0;
      }
      // temporarily remove any meshToWorld transform in clippedMesh so all
      // calculations are performed in the local clippedMesh coordinate frame
      RigidTransform3d TCW = new RigidTransform3d();
      clippedMesh.getMeshToWorld (TCW);
      clippedMesh.setMeshToWorld (RigidTransform3d.IDENTITY);

      PolygonalMesh cutPlane = createCutPlaneMesh();
      ArrayList<Point3d> cpnts = new ArrayList<>();
      RigidTransform3d TPW = new RigidTransform3d(); // cut plane pose wrt world
      double endTol = 0.001;
      double maxr = 0;
      for (int i=0; i<=npnts; i++) {
         double z = minz + i*lenz/npnts;
         // shrink cut plane z from the end points to ensure a good
         // intersection
         if (i == 0) {
            TPW.p.z = z + endTol;
         }
         else if (i == npnts) {
            TPW.p.z = z - endTol;
         }
         else {
            TPW.p.z = z;
         }
         cutPlane.setMeshToWorld (TPW);
         IntersectionContour contour =
            ComputeUtils.findPrimaryIsectContour (clippedMesh, cutPlane);
         if (contour != null) {
            Point3d cent = new Point3d();
            contour.computeCentroid (cent);
            double r = ComputeUtils.computeContourRadius (contour, cent);
            if (r > maxr) {
               maxr = r;
            }
            cent.z = z; // reset z in case it was skrunk at the end points
            cpnts.add (cent);
         }
         else {
            System.out.println (
               "WARNING: findDonorCenterCurve: no centroid at z=" + z);
         }
      }
      clippedMesh.setMeshToWorld (TCW);
      curve.setSingleSegment (cpnts, minz, maxz);
      return maxr;
   }      

   /**
    * Create the clipped donor by removing the top and bottom parts of the
    * donor. This is done by intersecting the donor (in donor coordinates) with
    * cut planes perpendicular to the z axis.
    */
   private PolygonalMesh prepareFibula (
      RigidTransform3d TCW, PolygonalMesh donorMesh) {

      double proxDist = getProxTrimDistance();
      double distalDist = getDistalTrimDistance();

      PolygonalMesh clippedMesh;
      RigidTransform3d TDW = donorMesh.getMeshToWorld(); // donor pose wrt world
      OBB boundingBox = donorMesh.computeOBB();
      Vector3d halfWidths = boundingBox.getHalfWidths();
      double halfLen = halfWidths.maxElement();
      boundingBox.getTransform (TCW); 
      TCW.mul (TDW, TCW); // set TCW to pose of clipped donor wrt world
      // if necessary, adjust TCW so that z points along the longest box axis
      if (halfWidths.maxAbsIndex() == 0) {
         TCW.R.mulRotY (Math.PI/2); // longest axis is x; rotate about y
      }
      else if (halfWidths.maxAbsIndex() == 1) {
         TCW.R.mulRotX (-Math.PI/2); // longest axis is y; rotate about x
      }
      PolygonalMesh cutPlane = createCutPlaneMesh();
      RigidTransform3d TPW = new RigidTransform3d(); // cut plane pose wrt world
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();
      double clippedLen = 2*halfLen-proxDist-distalDist; // clipped donor length

      TPW.set (TCW);
      TPW.mulXyz (0, 0, halfLen-proxDist);
      cutPlane.setMeshToWorld (TPW);
      clippedMesh = intersector.findIntersection (donorMesh, cutPlane);
      TPW.mulXyz (0, 0, -clippedLen);
      cutPlane.setMeshToWorld (TPW);
      clippedMesh = intersector.findDifference01 (clippedMesh, cutPlane);

      clippedMesh.inverseTransform (TCW);
      return clippedMesh;
   }

   /**
    * Repositions the scapula frame so that the z axis is perpendiclar to the
    * thin plane.
    */
   void fitScapulaFrame (RigidBody scapula) {
      // TDW is the initial donor pose, just in case it's not the identity
      PolygonalMesh mesh = scapula.getSurfaceMesh();
      RigidTransform3d TDW = scapula.getPose();
      RigidTransform3d TBW = new RigidTransform3d(); // bounding box to world
      OBB boundingBox = mesh.computeOBB();
      Vector3d halfWidths = boundingBox.getHalfWidths();
      double halfLen = halfWidths.maxElement();

      boundingBox.getTransform (TBW); 
      TBW.mul (TDW, TBW); // apply TDW in case it's not the identity
      Vector3d zdir = new Vector3d();
      // get z direction corresponding to shortest length
      TBW.R.getColumn (halfWidths.minAbsIndex(), zdir);
      if (zdir.y > 0) {
         zdir.negate(); // want z to face viewer
      }
      Vector3d ydir = new Vector3d (0, 0, 1);
      ydir.scaledAdd (-ydir.dot(zdir), zdir);

      RigidTransform3d TDWnew = new RigidTransform3d();
      TDWnew.p.set (TBW.p);
      TDWnew.R.setYZDirections (ydir, zdir);
      //TDWnew.R.setZDirection (zdir);
      RigidTransform3d TNB = new RigidTransform3d();
      TNB.mulInverseLeft (TDW, TDWnew);
      scapula.transformCoordinateFrame (TNB);  

      // now rotate the scapula
      TDWnew.R.setZDirection (new Vector3d (0, -1, 0));
      //TDWnew.mul (new RigidTransform3d (0, 50, 0), TDWnew);
      scapula.setPose (TDWnew);
   }

   boolean hasClippedDonor() {
      return (getRigidBody ("clippedDonor") != null &&
              getCurve ("donorCurve") != null);
   }

   RigidBody getClippedDonor() {
      return getRigidBody ("clippedDonor");
   }

   boolean isClippedDonorVisible () {
      boolean visible = 
         (isVisible (getRigidBody ("clippedDonor")) &&
          isVisible (getCurve ("donorCurve")));
      if (getDonorMarkers().size() > 0) {
         visible &= isVisible(getDonorMarkers());
      }
      return visible;
   }

   boolean setClippedDonorVisible (boolean enable) {
      boolean changed = false;
      changed |= setVisible (getRigidBody ("clippedDonor"), enable);
      changed |= setVisible (getCurve ("donorCurve"), enable);
      if (getDonorMarkers().size() > 0) {
         changed |= setVisible(getDonorMarkers(), enable);
      }
      return changed;
   }

   void createClippedDonor (RigidBody donor) {
      RigidTransform3d TCW = new RigidTransform3d();
      PolygonalMesh clippedDonor = 
         prepareFibula (TCW, donor.getSurfaceMesh());
      CubicHermiteSpline3d clcurve = new CubicHermiteSpline3d();
      double maxradius = findDonorCenterCurve (clcurve, clippedDonor, 50);
      if (maxradius == 0) {
         throw new NumericalException ("clipped donor has length 0");
      }
      setCurve ("donorCurve", clcurve, TCW);
      setMaxDonorRadius (maxradius);
      RigidBody body = setRigidBody ("clippedDonor", clippedDonor);
      body.setPose (TCW);
   }

   boolean clearClippedDonor() {
      boolean changed = false;
      changed |= removeRigidBody ("clippedDonor");
      changed |= removeCurve ("donorCurve");
      return changed;
   }

   // support for the clipped mandible

   boolean hasClippedMandible() {
      return (getRigidBody ("mandibleL") != null &&
              getRigidBody ("mandibleR") != null &&
              getRigidBody ("resection") != null);
   }

   boolean isClippedMandibleVisible () {
      return (isVisible (getRigidBody ("mandibleL")) &&
              isVisible (getRigidBody ("mandibleR")));
   }

   boolean setClippedMandibleVisible (boolean enable) {
      boolean changed = false;
      changed |= setVisible (getRigidBody ("mandibleL"), enable);
      changed |= setVisible (getRigidBody ("mandibleR"), enable);
      return changed;
   }

   public PolygonalMesh clipMesh (
      PolygonalMesh mesh, RigidTransform3d TPW) {

      PolygonalMesh cutPlane = createCutPlaneMesh();
      cutPlane.setMeshToWorld (TPW);

      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();

      PolygonalMesh clippedMesh =
         intersector.findIntersection (mesh, cutPlane);

      // filter out small intersection meshes
      PolygonalMesh[] submeshes = clippedMesh.partitionIntoConnectedMeshes();
      if (submeshes != null) {
         clippedMesh = submeshes[0];
         clippedMesh.removeDisconnectedFaces();
         clippedMesh.removeDisconnectedVertices();
      }
      MeshFactory.fillHoles (clippedMesh); // just in case      
      return clippedMesh;
   }


   public PolygonalMesh[] clipMandible (
      PolygonalMesh mandible, RigidTransform3d TLW, RigidTransform3d TRW) {

      PolygonalMesh resectMesh = getResectionMesh();

      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();
      PolygonalMesh mandibleL = null;
      PolygonalMesh mandibleR = null;

      PolygonalMesh resection =
         intersector.findDifference01 (mandible, resectMesh);
      MeshFactory.fillHoles (resection); // just in case

      // filter out small intersection meshes
      PolygonalMesh[] submeshes = resection.partitionIntoConnectedMeshes();
      if (submeshes != null) {
         resection = submeshes[0];
         resection.removeDisconnectedFaces();
         resection.removeDisconnectedVertices();
      }

      PolygonalMesh complement = 
         intersector.findIntersection (mandible, resectMesh);

      // take the two largest meshes
      submeshes = complement.partitionIntoConnectedMeshes();
      if (submeshes == null || submeshes.length < 2) {
         throw new NumericalException (
            "Resection operation does not split mandible into two meshes");
      }
      PolygonalMesh mesh0 = submeshes[0];
      PolygonalMesh mesh1 = submeshes[1];
      // need to determine which is left and which is right
      Plane planeL = new Plane (TLW);
      int num0OutsideL = 0;
      int num1OutsideL = 0;
      for (Vertex3d vtx : mesh0.getVertices()) {
         if (planeL.distance (vtx.getPosition()) < 0) {
            num0OutsideL++;
         }
      }
      for (Vertex3d vtx : mesh1.getVertices()) {
         if (planeL.distance (vtx.getPosition()) < 0) {
            num1OutsideL++;
         }
      }
      if (num0OutsideL > num1OutsideL) {
         mandibleL = mesh0;
         mandibleR = mesh1;
      }
      else {
         mandibleR = mesh0;
         mandibleL = mesh1;
      }
      return new PolygonalMesh[] {resection, mandibleL, mandibleR};
   }

   PolygonalMesh getResectionMesh() {
      if (myResectionMesh == null) {
         
         RigidTransform3d[] TPWs = getResectionPlanes();
         myResectionMesh = new PolygonalMesh();
         PolygonalMesh mesh = createCutPlaneMesh();

         mesh.setMeshToWorld (TPWs[0]);
         myResectionMesh.addMesh (mesh, /*respect transforms=*/true);

         mesh.setMeshToWorld (TPWs[1]);
         myResectionMesh.addMesh (mesh, /*respect transforms=*/true);
      }
      return myResectionMesh;
   }

   void invalidateResectionMesh() {
      myResectionMesh = null;
   }

   void createClippedMandible () {
      RigidBody mandible = getRigidBody ("mandible");
      RigidTransform3d[] planes = getResectionPlanes();
      PolygonalMesh[] meshes = clipMandible (
         mandible.getSurfaceMesh(), planes[0], planes[1]);

      RigidBody body;
      body = setRigidBody ("resection", meshes[0]);
      RenderProps.setVisible (body, false);      
      body = setRigidBody ("mandibleL", meshes[1]);
      body = setRigidBody ("mandibleR", meshes[2]);
   }

   boolean clearClippedMandible() {
      boolean changed = false;
      changed |= removeRigidBody ("resection");
      changed |= removeRigidBody ("mandibleL");
      changed |= removeRigidBody ("mandibleR");

      RigidBody mandible = getRigidBody ("mandible");
      if (mandible != null) {
         changed |= setVisible (mandible, true);
      }
      changed |= setResectionPlanesVisible (true);
      return changed;
   }

   // support for resection planes

   public RenderableComponentList<FixedMeshBody> getResectionPlaneBodies() {
      return (RenderableComponentList<FixedMeshBody>)myMech.get (
         "resectionPlanes");
   }

   int numResectionPlanes() {
      return getResectionPlaneBodies().size();
   }

   boolean hasResectionPlanes() {
      return (numResectionPlanes() >= 2);
   }

   boolean getResectionPlanesVisible () {
      return (isVisible (getResectionPlaneBodies()));
   }

   boolean setResectionPlanesVisible (boolean enable) {
      return setVisible (getResectionPlaneBodies(), enable);
   }

//   void setResectionPlane (String name, RigidTransform3d TPW) {
//      PolygonalMesh mesh = createRenderPlaneMesh();
//      FixedMeshBody mbody = setMeshBody (name, mesh);
//      mbody.setPose (TPW);
//      invalidateResectionMesh();
//   }

   void setResectionPlanes (RigidTransform3d[] planes) {
      clearResectionPlanes();
      for (int i=0; i<planes.length; i++) {
         RigidTransform3d TPW = planes[i];
         PolygonalMesh mesh = createRenderPlaneMesh();
         String name;
         if (i == 0) {
            name = "planeL";
         }
         else if (i == 1) {
            name = "planeR";
         }
         else {
            name = "plane" + i;
         }
         FixedMeshBody mbody = new FixedMeshBody (name, mesh);
         mbody.setPose (TPW);
         getResectionPlaneBodies().add (mbody);
      }
      invalidateResectionMesh();
   }

   void createResectionPlanes() {
      double off = myNewResectPlaneOff;
      RigidTransform3d TLW = new RigidTransform3d(); // left plane to world
      TLW.p.set ( off, -off, -off/2);
      TLW.R.setRpy (0, -90*DTOR, 45*DTOR);
      RigidTransform3d TRW = new RigidTransform3d(); // right plane to world
      TRW.p.set (-off, -off, -off/2);
      TRW.R.setRpy (0,  90*DTOR, 45*DTOR);
      setResectionPlanes (new RigidTransform3d[] { TLW, TRW });
   }

   RigidTransform3d[] getResectionPlanes() {
      int num = numResectionPlanes();
      RigidTransform3d[] planes = new RigidTransform3d[num];
      RenderableComponentList<FixedMeshBody> bodies = getResectionPlaneBodies();
      for (int i=0; i<num; i++) {
         planes[i] = new RigidTransform3d (bodies.get(i).getPose());
      }
      return planes;
   }

   PolygonalMesh createRenderPlaneMesh() {
      ReconAppRoot root = getRoot();
      double width = getRenderPlaneWidth();
      int res = root.getPlaneResolution();
      PolygonalMesh mesh = MeshFactory.createPlane (width, width, res, res);
      RenderProps.setFaceColor (mesh, ReconAppRoot.PLANE_COLOR);
      RenderProps.setBackColor (mesh, ReconAppRoot.PLANE_BACK_COLOR);
      RenderProps.setFaceStyle (mesh, FaceStyle.FRONT_AND_BACK);
      return mesh;
   }

   void updatePlaneSizes() {
      RenderableComponentList<FixedMeshBody> bodies = getResectionPlaneBodies();
      for (FixedMeshBody body : getResectionPlaneBodies()) {
         body.setMesh (createRenderPlaneMesh());
      }
   }

   boolean clearResectionPlanes() {
      if (numResectionPlanes() > 0) {
         getResectionPlaneBodies().removeAll();
         return true;
      }
      else {
         return false;
      }
   }

   // support for reconstruction

   boolean hasReconstruction() {
      return getRigidBody ("reconstruction") != null;
   }

   boolean clearReconstruction() {
      return removeRigidBody ("reconstruction");
   }

   PolygonalMesh computeReconstruction (
      PolygonalMesh mandibleL, PolygonalMesh mandibleR,
      Collection<DonorSegmentBody> segments) {
      
      PolygonalMesh mesh = mandibleL;
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();
      intersector.setRemoveZeroAreaFaces (false);
      int k = 0;
      for (DonorSegmentBody seg : segments) {
         PolygonalMesh extendedMesh = seg.createExtendedMesh(1e-2, 1e-8);
         if (mesh == null) {
            mesh = extendedMesh;
         }
         else {
            // write out meshes for diagnostic purposes
            // try {
            //    PolygonalMesh tmp = extendedMesh.clone();
            //    tmp.transform (tmp.getMeshToWorld());
            //    tmp.write (new File("extendedMesh"+k+".obj"));
            //    mesh.write (new File("mesh"+k+".obj"));
            // }
            // catch (Exception e) {
            //    e.printStackTrace(); 
            // }
            mesh = intersector.findUnion (extendedMesh, mesh);
         }
         k++;
      }
      if (mandibleR != null) {
         mesh = intersector.findUnion (mandibleR, mesh);
      }
      MeshFactory.fillHoles(mesh); // just in case
      return mesh;
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
      if (myMandibleCentroid != null) {
         pw.println ("mandibleCentroid=["+myMandibleCentroid.toString()+"]");
      }
      super.writeItems (pw, fmt, ancestor);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {
      rtok.nextToken();
      if (scanAttributeName (rtok, "mandibleCentroid")) {
         myMandibleCentroid = new Point3d();
         myMandibleCentroid.scan (rtok, null);
         return true;
      }
      rtok.pushBack();
      return super.scanItem (rtok, tokens);
   }
   
}
