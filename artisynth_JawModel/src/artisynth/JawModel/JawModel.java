package artisynth.JawModel;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix4d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.Property;
import maspack.properties.PropertyList;
import maspack.render.Renderer;
import maspack.render.RenderProps;
import maspack.render.ColorMapProps;
import maspack.render.Renderer.LineStyle;
import maspack.spatialmotion.SpatialInertia;
import maspack.spatialmotion.Wrench;
import maspack.util.ReaderTokenizer;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.RevoluteJoint;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.mechmodels.SegmentedPlanarConnector;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.Traceable;
import artisynth.core.probes.TracingProbe;
import artisynth.core.probes.VectorTracingProbe;
import artisynth.core.util.AmiraLandmarkReader;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.ScalableUnits;

public class JawModel extends MechModel implements ScalableUnits,
      Traceable {

   public boolean debug = false; // set to true for debug printlns

   public static final double CM_TO_MM = 10.0; // conversion factor

   public static final double NEWTONS_TO_FORCEUNITS = 1000.0; // kg*mm*s^-2

   public static final String muscleListFilename = "muscleList.txt";

   public static final String bodyListFilename = "bodyList.txt";

   public static final String muscleListAllFilename = "muscleListAll.txt";

   public static final String bodyListAllFilename = "bodyListAll.txt";

   public static final String muscleInfoFilename = "muscleInfo.txt";

   public static final String muscleGroupInfoFilename = "muscleGroupsInfo.txt";

   protected boolean addAllGeometry = false;

   public static ArrayList<BodyInfo> bodyInfoList = new ArrayList<BodyInfo>();

   public static ArrayList<String> muscleList = new ArrayList<String>();

   public static HashMap<String, MuscleInfo> muscleInfo;

   public static ArrayList<Muscle> myMuscles = new ArrayList<Muscle>();

   public static ArrayList<MuscleGroupInfo> muscleGroupInfo;

   protected HashMap<String, String> muscleGroupNames = new LinkedHashMap<String, String>();

   protected HashMap<String, String> muscleAbbreviations = new LinkedHashMap<String, String>();

   protected boolean myRenderEdgesP;

   protected boolean myRenderFacesP;

   protected boolean myTransparencyP;

   static final double transparentAlpha = 0.5;

   static final double opaqueAlpha = 1.0;

   static final double FIXED_PT_RADIUS = 0.0;

   static final double CONTACT_PT_RADIUS = 1.0;

   static public final Point3d c1Point = new Point3d(0.0, 54.3478, 63.5381);

   public static final Point3d hyoidRefPos = new Point3d(0.0, 1.6876, 7.12046);

   protected FrameMarker hyoidRefMarker;


   protected boolean enableMuscles = true;

   protected static double myMuscleDamping = 0.00;

   protected static double myJawDampingR = 1089.0; // rotational damping for jaw

   protected static double myJawDampingT = 0.403; // translational damping for jaw

   protected static double myHyoidDampingR = 10.0; // rotational damping for hyoid

   protected static double myHyoidDampingT = 0.05; // translational damping for hyoid

   protected double hyoidMass = 0.01;

   protected double thyroCricoidMass = 0.047; // combined thyroid + cricoid mass

   protected double myRestTone = 0.0;

   // XXX - unitConversion must match NEWTONS_TO_FORCE in Muscle model
   // scaling factor for force conversion (1000 = real)
   protected static double unitConversion = 1000;

   protected double gravityVal = 9.8;

   protected ArrayList<Muscle> closerMuscleList;

   Wrench jawMuscleWrench = new Wrench();

   Point3d jawComPosition = new Point3d();

   protected Point3d myTmpPos = new Point3d();

   protected RigidTransform3d XBodyToCom = new RigidTransform3d();

   protected Wrench myTmpWrench = new Wrench();

   protected RigidTransform3d XWorldToCom = new RigidTransform3d();

   RigidBody constrainedBody;

   boolean useComplexJoint = true;

   boolean useCurvJoint = false;

   // indexes of angle arrays
   public static final int LEFT = 0;

   public static final int RIGHT = 1;

   double[] condylarAngle = new double[] { 40.0, 40.0 };// {left, right}

   double[] condylarCant = new double[] { 0.0, -0.0 }; // {left, right}

   double[] medWallAngle = new double[] { -1.0, 1.0 }; // {left, right}

   double[] ltrlWallAngle = new double[] { -1.0, 1.0 }; // {left, right}

   double[] postWallAngle = new double[] { 0.0, 0.0 }; // {left, right}

   // double occlusalAngle = 6.5; // bite plane
   double[] biteAngle = new double[] { 10, 10 };// {left, right}

   double[] biteCant = new double[] { 0.0, 0.0 }; // {left, right}

   // medial wall offset (negative = lateral outward)
   double[] medWallOffset = new double[] { 0, 0 };

   // lateral wall offset (negative = lateral outward)
   double[] ltrlWallOffset = new double[] { -0.1, -0.1 };

   // posterior wall offset (positive = forward);
   double[] postWallOffset = new double[] { -0.0, -0.0 };

   double tmjForceNorm = 0.0;

   double[] curvParams = new double[] { 0.0, // x0
         0.0, // y0
         12.0, // xf
         -5.0, // yf
         -40.0, // initial slope
         -10.0 }; // final slope

   int numSegments = 20;


   public class BodyInfo {
      public String name;

      public String meshName;

      public void scan(ReaderTokenizer rtok) throws IOException {
         name = rtok.sval;
         rtok.nextToken();
         meshName = rtok.sval;
      }
   }

   public class MuscleInfo {
      public String name;

      public String origin;

      public String insertion;

      public String fullName;

      boolean pairedFlag; // true == left-right paired muscle

      public boolean isPaired() {
         return pairedFlag;
      }

      public void scan(ReaderTokenizer rtok) throws IOException {
         name = rtok.sval;
         rtok.nextToken();
         origin = rtok.sval;
         rtok.nextToken();
         insertion = rtok.sval;
         rtok.nextToken();
         pairedFlag = (rtok.sval.compareTo("paired") == 0.0);
         rtok.nextToken();
         fullName = rtok.sval;
      }

   }

   public class MuscleGroupInfo {
      public String name;

      public String fullName;

      public ArrayList<String> coactivators = new ArrayList<String>();

      public void scan(ReaderTokenizer rtok) throws IOException {
         rtok.eolIsSignificant(true);
         name = rtok.sval;
         rtok.nextToken();
         fullName = rtok.sval;
         rtok.nextToken();
         while (rtok.ttype != ReaderTokenizer.TT_EOL
               && rtok.ttype != ReaderTokenizer.TT_EOF) {
            coactivators.add(rtok.sval);
            rtok.nextToken();
         }
      }
   }

   public static PropertyList myProps = new PropertyList(JawModel.class,
         MechModel.class);

   static {
      myProps.add("renderFaces * showMeshFaces",
            "flag for redering mesh faces", false);
      myProps.add("renderEdges * showMeshEdges",
            "flag for redering mesh faces", false);
      myProps
            .add("transparency * *", "transparency of rigid body meshes", true);
      myProps.add("enableMuscles * *", "enables all muscles", true);
      myProps.add("muscleColor * *", "color of closer muscles", Color.CYAN);
      myProps.add("rigidBodyColor * *", "color of jaw and skull", Color.WHITE);
      myProps.addReadOnly("jawMuscleWrench",
            "total wrench applied to jaw by muscles");
      myProps
            .addReadOnly("jawMuscleForce",
                  "translational force component of total wrench applied to jaw by muscles");
      myProps.addReadOnly("jawMuscleMoment",
            "moment component of total wrench applied to jaw by muscles");

      myProps.addReadOnly("jawComPosition",
            "jaw centre-of-mass in world-coordinates");
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public JawModel() throws IOException {
      super();
   }

   public JawModel(String name) throws IOException {
      this(name, true, true, true);
   }

   public JawModel(String name, boolean fixedLaryngeal,
         boolean useComplexJoint, boolean useCurvJoint) throws IOException {
      super(name);

      setGravity(0, 0, -gravityVal * unitConversion);
      setupRenderProps();

      if (!fixedLaryngeal) {
         muscleList = readStringList(ArtisynthPath
            .getSrcRelativePath(JawModel.class, "geometry/" + muscleListAllFilename));
         bodyInfoList = readBodyInfoList(ArtisynthPath
            .getSrcRelativePath(JawModel.class, "geometry/" + bodyListAllFilename));
      } else {
         muscleList = readStringList(ArtisynthPath
            .getSrcRelativePath(JawModel.class, "geometry/" + muscleListFilename));
         bodyInfoList = readBodyInfoList(ArtisynthPath
            .getSrcRelativePath(JawModel.class, "geometry/" +bodyListFilename));
      }

      muscleInfo = readMuscleInfo(ArtisynthPath
         .getSrcRelativePath(JawModel.class, "geometry/" +muscleInfoFilename));
      muscleGroupInfo = readMuscleGroupsInfo(ArtisynthPath.getSrcRelativePath(
            JawModel.class, "geometry/" + muscleGroupInfoFilename));
      // setMuscleAbbreviations();

      assembleRigidBodies();
      attachMarkers();
      assembleMuscles();
      pruneMuscleList(); // removes muscles from myMuscles if not in
      attachMuscles();

      closerMuscleList = createMuscleList(readStringList(ArtisynthPath
         .getSrcRelativePath(JawModel.class, "geometry/" + "closerMuscleList.txt")));

      assembleBilateralExcitors();
      assembleMuscleGroups();

      updateMuscleLengthProps();

      showMasseterMarkers();

      setLaryngealBodiesFixed();
     
      addFixedMarkers();

      showMeshFaces(true);
      showMeshEdges(false);
      setTransparency(false);

      setMaxStepSize(0.0001);
      setIntegrator(Integrator.RungeKutta4);

      this.useComplexJoint = useComplexJoint;

      constrainedBody = myRigidBodies.get("jaw");
      if (constrainedBody == null) // no jaw body - error
      {
         System.err.println("JawModel: unable to get jaw rigidbody");
         return;
      }


      // do not use medial constraints in default jaw model
      bodyConnectors().get("LMED").setEnabled(false);
      bodyConnectors().get("RMED").setEnabled(false);
      RenderProps.setVisible(bodyConnectors().get("LMED"), false);
      RenderProps.setVisible(bodyConnectors().get("RMED"), false);

    

      this.useCurvJoint = useCurvJoint;


   }

   private void setupRenderProps() {
      RenderProps props = createRenderProps();

      // Particle RenderProps
      props.setPointRadius(1.0);
      props.setPointStyle(Renderer.PointStyle.SPHERE);
      props.setPointColor(Color.PINK);

      // Line RenderProps
      props.setLineRadius(2.0);
      props.setLineWidth(3);
      props.setLineStyle(Renderer.LineStyle.LINE);
      props.setLineColor(Color.WHITE);

      // Mesh RenderProps
      props.setShading(Renderer.Shading.SMOOTH);
      props.setFaceColor(new Color(1f, 0.8f, 0.6f));
      props.setFaceStyle(Renderer.FaceStyle.FRONT_AND_BACK);

      ColorMapProps tp = new ColorMapProps();
      tp.setFileName(ArtisynthPath.getSrcRelativePath(JawModel.class,
            "skull.jpg"));
      tp.setColorMixing(Renderer.ColorMixing.MODULATE);
      tp.setEnabled(true);

      props.setColorMap(tp);
      setRenderProps(props);

      // Spring Render Props
      RenderProps.setLineRadius(myAxialSprings, 2.0);
      RenderProps.setLineStyle(myAxialSprings, Renderer.LineStyle.SPINDLE);
      RenderProps.setLineColor(myAxialSprings, Color.RED);
   }



   public HashMap<String, MuscleInfo> readMuscleInfo(String filename)
         throws IOException {
      HashMap<String, MuscleInfo> infoList = new LinkedHashMap<String, MuscleInfo>();
      ReaderTokenizer rtok = new ReaderTokenizer(new FileReader( filename));

      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
         MuscleInfo mi = new MuscleInfo();
         mi.scan(rtok);
         infoList.put(mi.name, mi);
      }
      return infoList;
   }

   protected ArrayList<MuscleGroupInfo> readMuscleGroupsInfo(String filename)
         throws IOException {
      ArrayList<MuscleGroupInfo> infoList = new ArrayList<MuscleGroupInfo>();
      ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(filename));
      rtok.eolIsSignificant(true); // read end of lines
      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
         MuscleGroupInfo mgi = new MuscleGroupInfo();
         mgi.scan(rtok);
         infoList.add(mgi);
      }
      return infoList;
   }

   public  ArrayList<BodyInfo> readBodyInfoList(String filename)
         throws IOException {
      ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(filename));
      rtok.wordChars(".");
      ArrayList<BodyInfo> bodyInfoList = new ArrayList<BodyInfo>();
      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
         BodyInfo bi = new BodyInfo();
         bi.scan(rtok);
         bodyInfoList.add(bi);
      }
      return bodyInfoList;
   }

   public ArrayList<String> readStringList(String filename) throws IOException {
      ArrayList<String> stringList = new ArrayList<String>();
      ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(filename));

      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
         if (rtok.ttype != ReaderTokenizer.TT_WORD) { throw new IOException(
               "readMarkerList Expecting word, got " + rtok.tokenName()); }
         stringList.add(rtok.sval);
      }
      return stringList;
   }

   public ArrayList<Muscle> createMuscleList(
         ArrayList<String> muscleAbbreviations) {
      ArrayList<Muscle> list = new ArrayList<Muscle>();
      for (String name : muscleAbbreviations) {
         if (muscleInfo.get(name).isPaired()) {
            list.add(findMuscle("l" + name));
            list.add(findMuscle("r" + name));
         } else {
            list.add(findMuscle(name));
         }
      }
      return list;
   }



   public void setLaryngealBodiesFixed() {
      String[] larNames = new String[] { "hyoid", "thyroid", "cricoid" };
      RigidBody body;
      for (int i = 0; i < larNames.length; i++) {
         if ((body = rigidBodies().get(larNames[i])) != null) {
            body.setDynamic(false);
         }
      }
   }

   public RigidBody createAndAddBody(String name, String meshName) {

      RigidBody body = myRigidBodies.get (name);
      if (body == null) {
         body = new RigidBody();
         body.setName(name);
         addRigidBody(body);
      }
      body.setInertia(SpatialInertia.createBoxInertia(1, 1, 1, 1));
      setBodyDynamicProps(body);
      if (meshName.compareTo("none") != 0) setBodyMesh(body, meshName);
      RenderProps.setVisible (body, true);
      return body;
   }

   public void setBodyMesh(RigidBody body, String meshName) {
      setBodyMesh(body, meshName, 1.0);
   }

   public void setBodyMesh(RigidBody body, String meshName, double scale) {
      String meshFilename = ArtisynthPath.getSrcRelativePath(JawModel.class,
            "geometry/" + meshName);
      PolygonalMesh mesh = new PolygonalMesh();
      try {
         mesh.read(new BufferedReader(new FileReader(meshFilename)));
      } catch (IOException e) {
         e.printStackTrace();
         return;
      }
      mesh.scale(scale);
      mesh.setFixed(true);
      mesh.triangulate ();
      body.setMesh(mesh, meshFilename);
   }

   public void setBodyDynamicProps(RigidBody body) {
      if (body == null || body.getName() == null) return;

      String name = body.getName();
      if (name.compareTo("jaw") == 0.0) setJawDynamicProps(body);
      else if (name.compareTo("hyoid") == 0.0) setHyoidDynamicProps(body);
      else if (name.compareTo("thyroid") == 0.0) setThyroidDynamicProps(body);
      else
         setBodyFixed(body);

   }

   public void setBodyFixed(RigidBody body) {
      if (body != null) body.setDynamic(false);
   }

   public static void setJawDynamicProps(RigidBody jaw) {
      if (jaw == null) { return; }
      // Ineria Properties from ADAMS model - kg*mm*mm
     
      jaw.setRotationalInertia(new SymmetricMatrix3d(92.19, // (0,0)
            182.2, // (1,1)
            125.2, // (2,2)
            -1.122, // (0,1)
            -10.7, // (0,2)
            1.345)); // (1,2)
      jaw.setCenterOfMass(new Point3d(9.53, 0.0, 36.08)); // mm (uncapped obj)
      // jaw.setCenterOfMass(new Point3d(0.0, -13.73, 31.97)); //mm (capped obj)
      
      jaw.setMass(0.2); // kg
      // rotate to AmiraJaw coordinate frame (Rot about z-axis -PI/2)
      RotationMatrix3d R = new RotationMatrix3d();
      R.setAxisAngle(0, 0, 1, -Math.PI / 2);
      SpatialInertia M = new SpatialInertia();
      jaw.getInertia(M);
      M.transform(R);
      jaw.setInertia(M);
   }

   public void setThyroidDynamicProps(RigidBody thyroid) {
      if (thyroid == null) { return; }
      // Inertia Properties from ADAMS model - kg*mm*mm
      thyroid.setDynamic(true);
      thyroid.setRotationalInertia(new SymmetricMatrix3d(2.1537, // (0,0)
            1.6109, // (1,1)
            7.4320, // (2,2)
            0.0, // (0,1)
            0.0, // (0,2)
            1.9663)); // (1,2)
      thyroid.setCenterOfMass(new Point3d(0.0, 14.92, -14.24)); // mm (from obj)
      thyroid.setMass(thyroCricoidMass); // kg
   }



   public void setHyoidDynamicProps(RigidBody hyoid) {
      if (hyoid == null) { return; }
      // Inertia Properties from ADAMS model - kg*mm*mm
      hyoid.setDynamic(true);
      hyoid.setRotationalInertia(new SymmetricMatrix3d(2.1537, // (0,0)
            1.6109, // (1,1)
            7.4320, // (2,2)
            0.0, // (0,1)
            0.0, // (0,2)
            1.9663)); // (1,2)
      hyoid.setCenterOfMass(new Point3d(0.0, 6.03, 4.48)); // mm (from obj)
      // hyoid.setMass(0.2); //kg
      hyoid.setMass(hyoidMass); // kg
   }

 
   private RigidTransform3d readAnatomyTransformInfo(String name)
         throws IOException {
      // apply tranformations to rigid bodies that require it
      // if "name.transform" exists, read file and apply transform
      Matrix4d M = null;
      RigidTransform3d X;
      ReaderTokenizer rtok;
      try {
         rtok = new ReaderTokenizer(new FileReader(ArtisynthPath
               .getSrcRelativePath(JawModel.class, "geometry/" + name
                     + ".transform")));
      } catch (Exception e) {
         if (debug) {
            System.out.println("Can't find transform file for " + name);
            System.out.println("assuming no transform required");
         }
         return null;
      }

      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
         if (rtok.ttype == ReaderTokenizer.TT_WORD) {
            if (rtok.sval.compareTo("setTransform") == 0) {
               if (debug) {
                  System.out.println(name + " parser found: " + rtok.sval);
               }
               while (rtok.nextToken() != ReaderTokenizer.TT_NUMBER)
                  if (debug) {
                     System.out.println("reading " + rtok.sval);
                  }
               // flush the "setTransform" flag in Amira landmark file
               if (rtok.ttype == ReaderTokenizer.TT_NUMBER) {
                  rtok.pushBack();
                  M = new Matrix4d();
                  M.scan(rtok);
                  M.transpose(); // amira uses opposite row/col convention for
                  // matrices
                  if (true) {
                     System.out.println(name + " pose = " + M.toString());
                  }
               }
               break;
            }
         }
      }
      if (M == null) throw new IOException("Error reading transform info from "
            + name + " file");
      else {
         X = new RigidTransform3d();
         X.set((Matrix) M);
         return X;
      }

   }

   public void assembleRigidBodies() throws IOException {
      for (BodyInfo bodyInfo : bodyInfoList) {
         createAndAddBody(bodyInfo.name, bodyInfo.meshName);
      }

      // transformed vertebrae mesh has been saved to vertebrae_t.obj
      // therefore no need to transform here
      // transformVertebrae();

      // Rigidbody dampers for dynamic components
      setBodyDamping("jaw", myJawDampingT * unitConversion, myJawDampingR
            * unitConversion);
      setBodyDamping("hyoid", myHyoidDampingT * unitConversion, myHyoidDampingR
            * unitConversion);
      setBodyDamping("thyroid", myHyoidDampingT * unitConversion,
            myHyoidDampingR * unitConversion);
      setBodyDamping("cricoid", 0.1, 10);
   }

   public void setBodyDamping(String name, double td, double rd) {
      RigidBody body = myRigidBodies.get(name);
      if (body == null) { return; }
      body.setFrameDamping(td);
      body.setRotaryDamping(rd);
   }

   public void attachMarkers() {
      Point3d[] pts;
      for (int k = 0; k < muscleList.size(); k++) {
         String name = muscleList.get(k);
         MuscleInfo info = muscleInfo.get(name);
         RigidBody origin = myRigidBodies.get(info.origin);
         RigidBody insertion = myRigidBodies.get(info.insertion);

         try {
            if (origin == null || insertion == null) { throw new Exception(
                  "muscle attached to non-existent body"); }
            if (info.isPaired()) {
               pts = AmiraLandmarkReader.read(ArtisynthPath.getSrcRelativePath(
                     JawModel.class, "geometry/l" + name + ".landmarkAscii"),
                     CM_TO_MM);
               checkOriginInsertion(pts);

               addFrameMarker(new FrameMarker("l" + name + "_origin"), origin,
                     pts[0]);
               addFrameMarker(new FrameMarker("l" + name + "_insertion"),
                     insertion, pts[1]);
               // right side muscle is mirrored in x direction
               addFrameMarker(new FrameMarker("r" + name + "_origin"),
                     myRigidBodies.get(info.origin),
                     createRightSidePoint(pts[0]));
               addFrameMarker(new FrameMarker("r" + name + "_insertion"),
                     myRigidBodies.get(info.insertion),
                     createRightSidePoint(pts[1]));
            } else {
               pts = AmiraLandmarkReader.read(ArtisynthPath.getSrcRelativePath(
                     JawModel.class, "geometry/" + name + ".landmarkAscii"),
                     CM_TO_MM);
               addFrameMarker(new FrameMarker(name + "_origin"), origin, pts[0]);
               // right side attachment is mirrored in x direction
               //why is this mirrored if the muscle is not on both sides? Shouldn't this be pts[1] 
               addFrameMarker(new FrameMarker(name + "_insertion"), insertion,
                     createRightSidePoint(pts[0]));

            }
         } catch (Exception e) {
            System.out.println(e.getMessage());
            System.out.println("unable to add markers for muscle, removing: "
                  + info.fullName);
            muscleList.remove(k);
            k--; // update index to reflect purged kth muscle
            continue;
         }
      }
   }

   private static void checkOriginInsertion(Point3d[] markerPts) {
      if (markerPts[0].z < markerPts[1].z) { // origin and insertion point are
         // mixed up, do swap
         Point3d tmp = markerPts[0];
         markerPts[0] = markerPts[1];
         markerPts[1] = tmp;
      }
   }

   private static Muscle createPeckMuscle (
         String name, double maxForce, double optLen, double maxLen, double ratio) {
      Muscle m = new Muscle(name);
      m.setPeckMuscleMaterial(maxForce, optLen, maxLen, ratio);
      return m;
   }
   
   public void assembleMuscles() {
      /*
       * all muscle CSA values and 40 N/cm^2 constant taken from Peck 2000 Arch
       * Oral Biology
       */
      double shlpMaxForce = 66.9 / 0.7 * 0.3; // ihlp reported as 70% of muscle
      // [Peck 2000]
      double mylohyoidMaxForce = 177.0 / 100 / 2.0 * 40.0; // mylohyoid 177 mm^2
      // from
      // Buchilliard2009
      // JASA, divided
      // equally into
      // anterior and
      // posterior parts
      double geniohyoidMaxForce = 80.0 / 100 * 40.0; // geniohyoid 80 mm^2 from
      // Buchilliard2009 JASA
      double postdigMaxForce = 40.0; // same CSA as antdig from vanEijden 1997
      // Anat Rec
      double stylohyoidMaxForce = 0.39 * 40; // 0.39 cm^2 from van Eijden 1997
      // Anat Rec

      // NB - max length and opt length get overwritten in
      // updateMuscleLengthProps()
      // Skull - Jaw Muscles
      myMuscles.add(createPeckMuscle("lat", 158.0, 75.54, 95.92, 0.5)); // lat
      myMuscles.add(createPeckMuscle("ldm", 81.6, 29.07, 44.85, 0.29)); // ldm
      myMuscles.add(createPeckMuscle("lip", 66.9, 31.5, 41.5, 0.0)); // lip
      // (opener)
      myMuscles.add(createPeckMuscle("lmp", 174.8, 40.51, 50.63, 0.64)); // lmp
      myMuscles.add(createPeckMuscle("lmt", 95.6, 65.81, 93.36, 0.48)); // lmt
      myMuscles.add(createPeckMuscle("lpt", 75.6, 77.11, 101.08, 0.51)); // lpt
      myMuscles.add(createPeckMuscle("lsm", 190.4, 51.46, 66.88, 0.46)); // lsm
      myMuscles.add(createPeckMuscle("lsp", shlpMaxForce, 27.7, 37.7, 0.0)); // lsp
      // (opener)
      myMuscles.add(createPeckMuscle("rat", 158.0, 75.54, 95.92, 0.5)); // rat
      myMuscles.add(createPeckMuscle("rdm", 81.6, 29.07, 44.85, 0.29)); // rdm
      myMuscles.add(createPeckMuscle("rip", 66.9, 31.5, 41.5, 0.0)); // rip
      // (opener)
      myMuscles.add(createPeckMuscle("rmp", 174.8, 40.51, 50.63, 0.64)); // rmp
      myMuscles.add(createPeckMuscle("rmt", 95.6, 65.81, 93.36, 0.48)); // rmt
      myMuscles.add(createPeckMuscle("rpt", 75.6, 77.11, 101.08, 0.51)); // rpt
      myMuscles.add(createPeckMuscle("rsm", 190.4, 51.46, 66.88, 0.46)); // rsm
      myMuscles.add(createPeckMuscle("rsp", shlpMaxForce, 27.7, 37.7, 0.0)); // rsp
      // (opener)

      // Laryngeal Muscles (jaw-hyoid, skull-hyoid)
      myMuscles.add(createPeckMuscle("lad", 40.0, 35.1, 45.1, 0.0)); // lad
      // (opener)
      myMuscles.add(createPeckMuscle("lpd", postdigMaxForce, 35.1, 45.1,
            0.0)); // lpd
      myMuscles.add(createPeckMuscle("lam", mylohyoidMaxForce, 35.1, 45.1,
            0.0));// Left Anterior Mylohyoid
      myMuscles.add(createPeckMuscle("lpm", mylohyoidMaxForce, 35.1, 45.1,
            0.0));// Left Posterior Mylohyoid
      myMuscles.add(createPeckMuscle("lgh", geniohyoidMaxForce, 35.1, 45.1,
            0.0));// Left Geniohyoid
      myMuscles.add(createPeckMuscle("lsh", stylohyoidMaxForce, 35.1, 45.1,
            0.0));// Left Stylohyoid
      myMuscles.add(createPeckMuscle("rad", 40.0, 35.1, 45.1, 0.0)); // rad
      // (opener)
      myMuscles.add(createPeckMuscle("rpd", postdigMaxForce, 35.1, 45.1,
            0.0)); // rpd
      myMuscles.add(createPeckMuscle("ram", mylohyoidMaxForce, 35.1, 45.1,
            0.0));// Right Anterior Mylohyoid
      myMuscles.add(createPeckMuscle("rpm", mylohyoidMaxForce, 35.1, 45.1,
            0.0));// Right Posterior Mylohyoid
      myMuscles.add(createPeckMuscle("rgh", geniohyoidMaxForce, 35.1, 45.1,
            0.0));// Right Geniohyoid
      myMuscles.add(createPeckMuscle("rsh", stylohyoidMaxForce, 35.1, 45.1,
            0.0));// Right Stylohyoid

      // hyoid depressors
      myMuscles.add(createPeckMuscle("lth", 20.0, 35.1, 45.1, 0.5));// Left
      // Thyrohyoid
      myMuscles.add(createPeckMuscle("lsteh", 50.0, 35.1, 45.1, 0.5));// Left
      // Sternohyoid
      myMuscles.add(createPeckMuscle("loh", 50.0, 35.1, 45.1, 0.5));// Left
      // Omohyoid
      myMuscles.add(createPeckMuscle("rth", 20.0, 35.1, 45.1, 0.5));// Right
      // Thyrohyoid
      myMuscles.add(createPeckMuscle("rsteh", 50.0, 35.1, 45.1, 0.5));// Right
      // Sternohyoid
      myMuscles.add(createPeckMuscle("roh", 50.0, 35.1, 45.1, 0.5));// Right
      // Omohyoid

      // Laryngeal Muscles (thyroid-cricoid, crico-arytenoid, sternum)
      myMuscles.add(createPeckMuscle("lpc", 20.0, 35.1, 45.1, 0.5));// Left
      // Posterior
      // Cricoarytenoid
      myMuscles.add(createPeckMuscle("llc", 20.0, 35.1, 45.1, 0.5));// Left
      // Lateral
      // Cricoarytenoid
      myMuscles.add(createPeckMuscle("lpct", 20.0, 35.1, 45.1, 0.5));// Left
      // Posterior
      // Cricothyroid
      myMuscles.add(createPeckMuscle("lact", 20.0, 35.1, 45.1, 0.5));// Left
      // Anterior
      // Cricothyroid
      myMuscles.add(createPeckMuscle("lstet", 20.0, 35.1, 45.1, 0.5));// Left
      // Sternothyroid
      myMuscles.add(createPeckMuscle("rpc", 20.0, 35.1, 45.1, 0.5));// Right
      // Posterior
      // Cricoarytenoid
      myMuscles.add(createPeckMuscle("rlc", 20.0, 35.1, 45.1, 0.5));// Right
      // Lateral
      // Cricoarytenoid
      myMuscles.add(createPeckMuscle("rpct", 20.0, 35.1, 45.1, 0.5));// Right
      // Posterior
      // Cricothyroid
      myMuscles.add(createPeckMuscle("ract", 20.0, 35.1, 45.1, 0.5));// Right
      // Anterior
      // Cricothyroid
      myMuscles.add(createPeckMuscle("rstet", 20.0, 35.1, 45.1, 0.5));// Right
      // Sternothyroid

      myMuscles.add(createPeckMuscle("ta", 20.0, 35.1, 45.1, 0.5));// Transverse
      // Arytenoid
      // (midline
      // between
      // left
      // and
      // right
      // arytenoids)

      // get length ratios to update muscle lengths for new Amira geometry
      Muscle m;
      for (int k = 0; k < myMuscles.size(); k++) {
         m = myMuscles.get(k);
      }

   }

   public void printMuscleMaxForces() {

      for (AxialSpring s : myAxialSprings) {
         if (s instanceof Muscle) {
            Muscle m = (Muscle) s;
            String fullname = "";
            if (muscleInfo.containsKey(m.getName().substring(1))) {
               fullname = (m.getName().startsWith("l") ? "Left " : "Right ")
                     + muscleInfo.get(m.getName().substring(1)).fullName;
            }
            System.out.printf("%s %g -- %s\n", m.getName().toUpperCase(), 
               Muscle.getMaxForce (m), fullname);

         }
      }

   }

   public void assembleMuscleGroups() {

      for (MuscleGroupInfo info : muscleGroupInfo) {
         // bilateral excitor
         MuscleExciter bilateral = new MuscleExciter("bi_" + info.name);

         // add groups for left and right sides
         for (int i = 0; i < 2; i++) // add groups for left and right sides
         {
            String prefix = (i == 0 ? "l" : "r");
            String fullPrefix = (i == 0 ? "Left " : "Right ");
            MuscleExciter exciter = new MuscleExciter(prefix + info.name);
            for (String target : info.coactivators) {
               ExcitationComponent c = (Muscle) myAxialSprings.get(prefix
                     + target);
               if (c == null) { // look for target in excitors list
                  c = myExciterList.get(prefix + target);
               }
               if (c == null) continue;

               exciter.addTarget(c, 1.0);
            }
            addMuscleExciter(exciter);
            bilateral.addTarget(exciter, 1.0);
            muscleAbbreviations.put(prefix + info.name, fullPrefix
                  + info.fullName);
         }

         addMuscleExciter(bilateral);
         muscleAbbreviations.put("bi_" + info.name, "Bilateral "
               + info.fullName);

      }

   }

   public void updateMuscleLengthProps() {
      for (Muscle m : myMuscles)
         m.resetLengthProps();
   }

   public void pruneMuscleList() {
      for (int k = 0; k < myMuscles.size(); k++) {
         Muscle m = myMuscles.get(k);
         String name = m.getName();
         boolean inMuscleList = false;
         for (int i = 0; i < muscleList.size(); i++) {
            if (name.endsWith(muscleList.get(i))) {
               inMuscleList = true;
               break;
            }
         }
         if (!inMuscleList) {
            myMuscles.remove(k);
            k--; // update index to reflect purged kth muscle
         }
      }
   }

   /*
    * add FrameMarkers and Muscles to model
    */
   public void attachMuscles() {
      for (int k = 0; k < muscleList.size(); k++) {
         String name = muscleList.get(k);
         if (muscleInfo.get(muscleList.get(k)).isPaired()) {
            addMuscle("l" + name);
            muscleAbbreviations.put("l" + name, "Left "
                  + muscleInfo.get(name).fullName);
            addMuscle("r" + name);
            muscleAbbreviations.put("r" + name, "Right "
                  + muscleInfo.get(name).fullName);
         } else {
            addMuscle(name);
         }
      }
   }

   public void assembleBilateralExcitors() {
      for (int k = 0; k < muscleList.size(); k++) {
         String name = muscleList.get(k);
         if (muscleInfo.get(muscleList.get(k)).isPaired()) {
            Muscle left = (Muscle) myAxialSprings.get("l" + name);
            Muscle right = (Muscle) myAxialSprings.get("r" + name);
            if (left != null && right != null) {
               String excitorName = "bi_" + name;
               MuscleExciter bilateral = new MuscleExciter(excitorName);
               bilateral.addTarget(left, 1.0);
               bilateral.addTarget(right, 1.0);
               addMuscleExciter(bilateral);
               String fullName = muscleInfo.get(name).fullName;
               muscleAbbreviations.put(excitorName, "Bilateral " + fullName);
            }
         }
      }

   }

   public Muscle findMuscle(String name) {
      for (Muscle m : myMuscles) {
         if (name.compareTo(m.getName()) == 0) return m;
      }
      return null;
   }

   public void addMuscle(String name) {
      Muscle m = findMuscle(name);
      if (m == null) {
         System.err.println(name + " muscle not found.");
         return;
      }
      m.setFirstPoint(myFrameMarkers.get(name + "_origin"));
      m.setSecondPoint(myFrameMarkers.get(name + "_insertion"));
      AxialSpring.setDamping (m, myMuscleDamping);
      addAxialSpring(m);
   }

   public void detachMuscles() {
      for (Muscle m : myMuscles) {
         removeAxialSpring(m);
      }
   }





   public static Point3d createRightSidePoint(Point3d leftSidePt) {
      Point3d rightSidePt = new Point3d();
      if (leftSidePt != null) {
         rightSidePt.set(leftSidePt);
         rightSidePt.x = -rightSidePt.x; // right-left mirrored in x-axis
      }
      return rightSidePt;
   }

   private int findLargestZValue(Point3d[] pointList) {
      if (pointList.length < 1) return 0;
      double maxZ = pointList[0].z;
      int maxZindex = 0, k = 0;
      for (Point3d p : pointList) {
         if (p.z > maxZ) {
            maxZ = p.z;
            maxZindex = k;
         }
         k++;
      }
      return maxZindex;
   }

   public FrameMarker addFixedMarker(String bodyName, Point3d location,
         String markerName) {
      // add marker at location for body
      RigidBody body = myRigidBodies.get(bodyName);
      if (body == null) {
         System.err.println("addFixedMarker Error: "
               + "body specified don't exist, can't create marker");
         return null;
      }
      FrameMarker marker = new FrameMarker();
      addFrameMarker(marker, body, location);
      marker.setName(markerName);
      RenderProps fixedProps = marker.createRenderProps();
      fixedProps.setPointColor(Color.BLUE);
      fixedProps.setPointRadius(FIXED_PT_RADIUS);
      marker.setRenderProps(fixedProps);
      // myFrameMarkers.add(marker);
      return marker;
   }

   public void addComMarker(String bodyName) {
      // add marker at COM
      Point3d com = new Point3d();
      RigidBody body = myRigidBodies.get(bodyName);
      if (body == null) {
         // System.err.println("addComMarker Error: "
         // + "body specified don't exist, can't create com marker");
         return;
      }
      body.getCenterOfMass(com);
      FrameMarker comMarker = new FrameMarker();
      addFrameMarker(comMarker, body, com);
      comMarker.setName(bodyName + "Com");
      RenderProps comProps = RenderProps.createPointProps(null);
      comProps.setPointColor(Color.ORANGE);
      comProps.setPointRadius(FIXED_PT_RADIUS);
      comMarker.setRenderProps(comProps);
      // myFrameMarkers.add(comMarker);
   }

   public void addFixedMarkers() {
      // get fixed markers for jaw
      FrameMarker m;
      boolean hasRightPair;
      RigidBody body;
      String name;
      Vector3d model_tr= new Vector3d (0, 10.0335, -38.6615);
      AffineTransform3d X= new AffineTransform3d ();
      X.setTranslation (model_tr);
      Point3d[] pts;
      String[] list = new String[] { "jaw"};
      ArrayList<String> bodyList = new ArrayList<String>();
      for (int i = 0; i < list.length; i++)
         bodyList.add(list[i]);
      RenderProps fixedProps = RenderProps.createPointProps(null);
      fixedProps.setPointRadius(FIXED_PT_RADIUS);
      fixedProps.setPointColor(Color.WHITE);
      for (int i = 0; i < bodyList.size(); i++) {
         name = bodyList.get(i);
         body = myRigidBodies.get(name);
         try {
            pts = AmiraLandmarkReader.read(ArtisynthPath.getSrcRelativePath(
                  JawModel.class, "geometry/" + name
                        + "FixedMarkers.landmarkAscii"), CM_TO_MM);
         } catch (IOException e) {
            e.printStackTrace();
            return;
         }

         // find tmj point index for jaw
         int tmjPointIndex = -1;
         if (name.compareTo("jaw") == 0.0) {
            tmjPointIndex = findLargestZValue(pts);
         }

         for (int k = 0; k < pts.length; k++) {
            m = new FrameMarker();
            m.setRenderProps(fixedProps.clone());
            hasRightPair = true;
           
            addFrameMarker(m, body, pts[k]);
            m.transformGeometry (X);
            if (hasRightPair == true) {
               m = new FrameMarker();
               
               addFrameMarker(m, body, createRightSidePoint(pts[k]));
               
               m.transformGeometry (X);
               m.setRenderProps(fixedProps.clone());
               if (k == tmjPointIndex) {
                  m.setName("rtmj");
                  RenderProps.setPointRadius(m, CONTACT_PT_RADIUS);
                  RenderProps.setPointColor(m, Color.BLUE);
               }
            }

         }
      }

      // add C1 marker
      body = myRigidBodies.get("sternum");
      if (body != null) {
         m = new FrameMarker();
         addFrameMarker(m, body, c1Point);
         m.transformGeometry (X);
         m.setName("C1point");
         m.setRenderProps(fixedProps.clone());
         RenderProps.setPointColor(m, Color.MAGENTA);
      }

      // add COM markers
      addComMarker("jaw");
      addComMarker("hyoid");
      addComMarker("thyroid");
      addComMarker("cricoid");

      // add bite points
      FrameMarker fm;

      // add reference point for hyoid
      RigidBody hyoid = myRigidBodies.get("hyoid");
      if (hyoid != null) {
         m = new FrameMarker();
         addFrameMarker(m, hyoid, hyoidRefPos);
         m.transformGeometry (X);
         m.setName("hyoidRef");
      }

   }

   public double getUnitConversion() {
      return unitConversion;
   }

   public void setUnitConversion(double uc) {
      unitConversion = uc;
   }

   public int getNumMuscles() {
      return myMuscles.size();
   }
   
   public ArrayList<Muscle> getMuscles() {
    return myMuscles;
   }




   public boolean getTransparency() {
      return myTransparencyP;
   }

   public void setTransparency(boolean transparent) {
      myTransparencyP = transparent;
      for (RigidBody body : myRigidBodies) {
         if (body.getMesh() == null) {
            continue;
         }
         RenderProps.setAlpha(body, transparent ? transparentAlpha
               : opaqueAlpha);
      }
   }

   /**
    * Iterate through all rigidbodies associated with the MechModel and change
    * mesh render setting for faces to reflect show parameter
    * 
    * @param facesVisible visibility flag
    */
   public void showMeshFaces(boolean facesVisible) {
      myRenderFacesP = facesVisible;
      for (RigidBody body : myRigidBodies) {
         RenderProps.setVisible(body, facesVisible);
      }
      return;
   }

   public boolean getRenderFaces() {
      return myRenderFacesP;
   }

   public void showMeshEdges(boolean edgesVisible) {
      myRenderEdgesP = edgesVisible;
      for (RigidBody body : myRigidBodies) {
         RenderProps.setDrawEdges(body, edgesVisible);
         RenderProps.setLineColor(body, Color.WHITE);
      }
      return;
   }

   public boolean getRenderEdges() {
      return myRenderEdgesP;
   }

   public void setMuscleColor(Color newColor) {
      RenderProps.setLineColor(this, newColor);
   }

   public Color getMuscleColor() {
      return getRenderProps().getLineColor();
   }

   public void setRigidBodyColor(Color newColor) {
      for (RigidBody rb : myRigidBodies) {
         RenderProps.setFaceColor(rb, newColor);
      }
   }

   public Color getRigidBodyColor() {
      // assume all bodies have same color as first in list
      return myRigidBodies.get(0).getRenderProps().getFaceColor();
   }

   public void scaleDistance(double s) {
      super.scaleDistance(s);
      updateMuscleLengthProps();
      c1Point.scale(s);
   
 
   }

  

   public String getMuscleName(String shortName) {
      if (muscleInfo.get(shortName) != null) return muscleInfo.get(shortName).fullName;
      else if (muscleAbbreviations.get(shortName) != null) return muscleAbbreviations
            .get(shortName);
      else
         return "unknown";
   }



   public void setEnableMuscles(boolean enabled) {
      enableMuscles = enabled;
      for (Muscle m : myMuscles) {
         m.setEnabled(enabled);
      }
   }

   public boolean getEnableMuscles() {
      return enableMuscles;
   }

   public void showMarkers(String[] markersToShow) {
      FrameMarker m;
      for (int i = 0; i < markersToShow.length; i++) {
         // System.out.println("Show marker: " + markersToShow[i]);
         m = myFrameMarkers.get(markersToShow[i]);
         if (m != null) {
            RenderProps props = m.getRenderProps();
            if (props == null) {
               props = RenderProps.createLineProps(null);
            }
            props.setPointRadius(1.0);
            //props.setPointSlices(20);
            props.setPointColor(Color.PINK);
            m.setRenderProps(props);
         }
      }
   }

   public void showMasseterMarkers() {
      String markersToShow[] = new String[] { "lsm_insertion", "rsm_insertion",
            "ldm_insertion", "rdm_insertion", "lsm_origin", "rsm_origin",
            "ldm_origin", "rdm_origin" };
      showMarkers(markersToShow);
   }

   /*
    * getJawMuscleWrench -- computes wrench due to markers that are muscle
    * endpoints. XXX - note that if markers are also attached to non-muscle
    * force effectors that force will also be added to the returned wrench. For
    * jaw model all muscle end-point markers are not attached to additional
    * force effectors.
    */
   public Wrench getJawMuscleWrench() {
      jawMuscleWrench.setZero();
      RigidBody body = myRigidBodies.get("jaw");

      myTmpPos.transform(body.getPose(), body.getCenterOfMass());
      XBodyToCom.R.setIdentity();
      XBodyToCom.p.negate(myTmpPos); // XWorldToCom
      XBodyToCom.mul(body.getPose()); // XBodyToCom = XWorldToCom * XBodyToWorld

      for (FrameMarker m : myFrameMarkers) {
         if (m.getFrame() == body) {
            boolean muscleAttachedToMarker = false;
            for (int i = 0; i < myAxialSprings.size(); i++) {
               AxialSpring s = myAxialSprings.get(i);
               if (s instanceof Muscle) {
                  if (s.getFirstPoint() == m || s.getSecondPoint() == m) {
                     muscleAttachedToMarker = true;
                     break;
                  }
               }
            }
            if (muscleAttachedToMarker) {
               // get wrench in coordinate frame at COM point (aligned with
               // world-frame)
               myTmpPos.transform(XBodyToCom, m.getLocation());
               jawMuscleWrench.f.add(m.getForce(), jawMuscleWrench.f);
               jawMuscleWrench.m.crossAdd(myTmpPos, m.getForce(),
                     jawMuscleWrench.m);

               // System.out.println(m.getName () + " -- f = " + m.getForce ());
            }
         }
      }
      return jawMuscleWrench;
   }

   public Vector3d getJawMuscleForce() {
      return getJawMuscleWrench().f;
   }

   public Vector3d getJawMuscleMoment() {
      return getJawMuscleWrench().m;
   }

   public Point3d getJawComPosition() {
      RigidBody jaw = myRigidBodies.get("jaw");
      jawComPosition.transform(jaw.getPose(), jaw.getCenterOfMass());
      return jawComPosition;
   }

   /*
    * Traceable interface
    */

   public String[] getTraceables() {
      return new String[] { "jawMuscleForce", "jawMuscleMoment"};
   }
   
   public String getTraceablePositionProperty (String traceableName) {
      return "+jawComPosition";
   }

   public TracingProbe getTracingProbe(String traceableName) {
      VectorTracingProbe vecProbe = null;
      Property jawCom = getProperty("jawComPosition");
      if (traceableName.equals("jawMuscleForce")) {
          vecProbe = new VectorTracingProbe(this, getProperty("jawMuscleForce"),
             jawCom, 1.0);
      } else if (traceableName.equals("jawMuscleMoment")) {
         vecProbe =  new VectorTracingProbe(this, getProperty("jawMuscleMoment"),
            jawCom, 1.0);
      }
      
      if (vecProbe !=  null) {
         vecProbe.setRenderAsPush (false);
         return vecProbe;
      }
      else {
         throw new IllegalArgumentException(
            "Unknown traceable '" + traceableName + "'");
      }
   }

   public void resetInertia(RigidBody body) {
      // System.out.println( b.getName () + " old inertia:");
      // System.out.println( b.getCenterOfMass ().toString ("%8.2f") + "\n");
      // System.out.println( b.getRotationalInertia ().toString ("%8.2f"));

      if (body.getMesh() == null) return;

      double density = body.getMass() / body.getMesh().computeVolume();
      body.setInertiaFromDensity (density);

      // System.out.println( b.getName () + " new inertia:");
      // System.out.println( b.getCenterOfMass ().toString ("%8.2f") + "\n");
      // System.out.println( b.getRotationalInertia ().toString ("%8.2f"));

   }

   public RigidTransform3d centerAtCOM(MechModel mech, String bodyName) {
      RigidBody body = mech.rigidBodies().get(bodyName);
      body.setAxisLength(100);

      RigidTransform3d XComToBody = new RigidTransform3d();
      XComToBody.p.set(body.getCenterOfMass());

      RigidTransform3d XBodyToWorld = new RigidTransform3d();
      body.getPose(XBodyToWorld);

      RigidTransform3d XComToWorld = new RigidTransform3d();
      XComToWorld.mul(XBodyToWorld, XComToBody);
      body.setPose(XComToWorld);

      RigidTransform3d XMeshToCom = new RigidTransform3d();
      if (body.getMesh() != null) {
         PolygonalMesh mesh = body.getMesh();
         // XMeshToCom.mulInverseRight (mesh.getMeshToWorld (), XComToWorld);
         XMeshToCom.invert(XComToWorld);
         mesh.transform(XMeshToCom);
         body.setMesh(mesh, null);
      }

      for (FrameMarker mrk : mech.frameMarkers()) {
         if (mrk.getFrame() == body) {
            // System.out.println("transforming " + mrk.getName ());
            Point3d loc = new Point3d();
            mrk.getLocation(loc);
            loc.transform(XMeshToCom);
            mrk.setLocation(loc);
         }
      }

      for (BodyConnector con : mech.bodyConnectors()) {
         if (con.getBodyA() == body) {
            System.out.println("jaw con - " + con.getName());
            con.transformGeometry(XComToWorld);
         }
      }

      return XMeshToCom;
   }

   
   
  public static  ArrayList<RigidBody> assembleRigidBodies(ArrayList<BodyInfo> bodyInfoList, RigidTransform3d amiraTranformation, String myPath) throws IOException {
     ArrayList<RigidBody> bodies= new ArrayList<> ();
     for (BodyInfo bodyInfo : bodyInfoList) {
       bodies.add(createBodies(bodyInfo.name, bodyInfo.meshName, amiraTranformation, myPath));
      }

      // transformed vertebrae mesh has been saved to vertebrae_t.obj
      // therefore no need to transform here
      // transformVertebrae();

      // Rigidbody dampers for dynamic components
  
      
      return bodies;
   }
   
   public static RigidBody createBodies(String name, String meshName, RigidTransform3d amiraTranformation, String myPath) {

      RigidBody body = new RigidBody();
      body = new RigidBody();
      body.setName(name);
     // addRigidBody(body);
      body.setInertia(SpatialInertia.createBoxInertia(1, 1, 1, 1));
      body.setDynamic (body.getName ().equals ("jaw"));
      if (meshName.compareTo("none") != 0) setBodyMesh(body, meshName, amiraTranformation, myPath);
      RenderProps.setVisible (body, true);
    

      
      if (body.getName ().equals ("jaw")){setBodyDamping(body, myJawDampingT * unitConversion, myJawDampingR * unitConversion); setJawDynamicProps(body);}
      else if (body.getName ().equals ("hyoid")){setBodyDamping(body, myHyoidDampingT * unitConversion, myHyoidDampingR * unitConversion);}
      else if (body.getName ().equals ("thyroid")){setBodyDamping(body, myHyoidDampingT * unitConversion, myHyoidDampingR * unitConversion);}
      else if (body.getName ().equals ("cricoid")){setBodyDamping(body, 0.1, 10);}   
      return body;
   }
   
   public static void setBodyMesh(RigidBody body, String meshName, RigidTransform3d amiraTranformation, String myPath) {
      String meshFilename = (myPath+"geometry/" + meshName);
      PolygonalMesh mesh = new PolygonalMesh();
      try {
         mesh.read(new BufferedReader(new FileReader(meshFilename)));
      } catch (IOException e) {
         e.printStackTrace();
         return;
      }
    //  mesh.scale(scale);
      mesh.setFixed(true);
      mesh.triangulate ();
      //Vector3d centroid = new Vector3d ();
      //mesh.computeCentreOfVolume (centroid);
      mesh.transform (amiraTranformation);
      //mesh.translate (centroid.scale (-1));
      body.setMesh(mesh, meshFilename);
      //if(body.getName ()!="skull"){
      //body.setPose (new RigidTransform3d (centroid.scale (-1),new AxisAngle ()));
      //}
      //body.setMesh(mesh, meshFilename);
   }
   
   public static void setBodyDamping(RigidBody body, double td, double rd) {
      if (body == null) { return; }
      body.setFrameDamping(td);
      body.setRotaryDamping(rd);
   }
   
   public static ArrayList<FrameMarker> assembleMarkers(ArrayList<String> muscleList, HashMap<String, MuscleInfo> muscleInfo,
      ComponentList<RigidBody> myRigidBodies, RigidTransform3d amiraTranformation, String myPath) {
      Point3d[] pts;
      FrameMarker myMarker;
      ArrayList<FrameMarker> myMarkers = new ArrayList<> ();
      for (int k = 0; k < muscleList.size(); k++) {
         String name = muscleList.get(k);
         JawModel.MuscleInfo info = muscleInfo.get(name);
         RigidBody origin = myRigidBodies.get(info.origin);
         RigidBody insertion = myRigidBodies.get(info.insertion);

         try {
            if (origin == null || insertion == null) { throw new Exception(
                  "muscle attached to non-existent body"); }
            if (info.isPaired()) {
               pts = AmiraLandmarkReader.read(myPath+ "geometry/l" + name + ".landmarkAscii",
                     1);
               checkOriginInsertion(pts);
               pts[0].transform (amiraTranformation);
               pts[1].transform (amiraTranformation);
               

               myMarker= new FrameMarker("l" + name + "_origin");
               myMarker.setFrame (origin);
               myMarker.setLocation (pts[0]);
               myMarker.setRefPos (myMarker.getPosition ());
               myMarkers.add (myMarker);

               myMarker= new FrameMarker("l" + name + "_insertion");
               myMarker.setFrame (insertion);
               myMarker.setLocation (pts[1]);
               myMarker.setRefPos (myMarker.getPosition ());
               myMarkers.add (myMarker);
               // right side muscle is mirrored in x direction

               myMarker= new FrameMarker("r" + name + "_origin");
               myMarker.setFrame ( myRigidBodies.get(info.origin));
               myMarker.setLocation (createRightSidePoint(pts[0]));
               myMarker.setRefPos (myMarker.getPosition ());
               myMarkers.add (myMarker);

               myMarker= new FrameMarker("r" + name + "_insertion");
               myMarker.setFrame ( myRigidBodies.get(info.insertion));
               myMarker.setLocation (createRightSidePoint(pts[1]));
               myMarker.setRefPos (myMarker.getPosition ());
               myMarkers.add (myMarker);
            } else {
               pts = AmiraLandmarkReader.read(myPath+ "geometry/" + name + ".landmarkAscii",
                     1);          
               checkOriginInsertion(pts);

               pts[0].transform (amiraTranformation);
               pts[1].transform (amiraTranformation);
               myMarker= new FrameMarker(name + "_origin");
               myMarker.setFrame (origin);
               myMarker.setLocation (pts[0]);
               myMarker.setRefPos (myMarker.getPosition ());
               myMarkers.add (myMarker);
               // right side attachment is mirrored in x direction

               myMarker= new FrameMarker(name + "_insertion");
               myMarker.setFrame (insertion);
               myMarker.setLocation (pts[1]);
               myMarker.setRefPos (myMarker.getPosition ());
               myMarkers.add (myMarker);

            }
         } catch (Exception e) {
            System.out.println(e.getMessage());
            System.out.println("unable to add markers for muscle, removing: "
                  + info.fullName);
            muscleList.remove(k);
            k--; // update index to reflect purged kth muscle
            continue;
         }
      }
      return myMarkers;
   }
   
   public static ArrayList<Muscle> assembleandreturnMuscles()  {
      ArrayList<Muscle> myMuscles = new ArrayList<> ();
      
      /*
       * all muscle CSA values and 40 N/cm^2 constant taken from Peck 2000 Arch
       * Oral Biology
       */ 
      double shlpMaxForce = 66.9 / 0.7 * 0.3; // ihlp reported as 70% of muscle
      // [Peck 2000]
      double mylohyoidMaxForce = 177.0 / 100 / 2.0 * 40.0; // mylohyoid 177 mm^2
      // from
      // Buchilliard2009
      // JASA, divided
      // equally into
      // anterior and
      // posterior parts
      double geniohyoidMaxForce = 80.0 / 100 * 40.0; // geniohyoid 80 mm^2 from
      // Buchilliard2009 JASA
      double postdigMaxForce = 40.0; // same CSA as antdig from vanEijden 1997
      // Anat Rec
      double stylohyoidMaxForce = 0.39 * 40; // 0.39 cm^2 from van Eijden 1997
      // Anat Rec

      // NB - max length and opt length get overwritten in
      // updateMuscleLengthProps()
      // Skull - Jaw Muscles
      myMuscles.add(createPeckMuscle("lat", 158.0, 75.54, 95.92, 0.5)); // lat
      myMuscles.add(createPeckMuscle("ldm", 81.6, 29.07, 44.85, 0.29)); // ldm
      myMuscles.add(createPeckMuscle("lip", 66.9, 31.5, 41.5, 0.0)); // lip
      // (opener)
      myMuscles.add(createPeckMuscle("lmp", 174.8, 40.51, 50.63, 0.64)); // lmp
      myMuscles.add(createPeckMuscle("lmt", 95.6, 65.81, 93.36, 0.48)); // lmt
      myMuscles.add(createPeckMuscle("lpt", 75.6, 77.11, 101.08, 0.51)); // lpt
      myMuscles.add(createPeckMuscle("lsm", 190.4, 51.46, 66.88, 0.46)); // lsm
      myMuscles.add(createPeckMuscle("lsp", shlpMaxForce, 27.7, 37.7, 0.0)); // lsp
      // (opener)
      myMuscles.add(createPeckMuscle("rat", 158.0, 75.54, 95.92, 0.5)); // rat
      myMuscles.add(createPeckMuscle("rdm", 81.6, 29.07, 44.85, 0.29)); // rdm
      myMuscles.add(createPeckMuscle("rip", 66.9, 31.5, 41.5, 0.0)); // rip
      // (opener)
      myMuscles.add(createPeckMuscle("rmp", 174.8, 40.51, 50.63, 0.64)); // rmp
      myMuscles.add(createPeckMuscle("rmt", 95.6, 65.81, 93.36, 0.48)); // rmt
      myMuscles.add(createPeckMuscle("rpt", 75.6, 77.11, 101.08, 0.51)); // rpt
      myMuscles.add(createPeckMuscle("rsm", 190.4, 51.46, 66.88, 0.46)); // rsm
      myMuscles.add(createPeckMuscle("rsp", shlpMaxForce, 27.7, 37.7, 0.0)); // rsp
      // (opener)

      // Laryngeal Muscles (jaw-hyoid, skull-hyoid)
      myMuscles.add(createPeckMuscle("lad", 40.0, 35.1, 45.1, 0.0)); // lad
      // (opener)
      myMuscles.add(createPeckMuscle("lpd", postdigMaxForce, 35.1, 45.1,
            0.0)); // lpd
      myMuscles.add(createPeckMuscle("lam", mylohyoidMaxForce, 35.1, 45.1,
            0.0));// Left Anterior Mylohyoid
      myMuscles.add(createPeckMuscle("lpm", mylohyoidMaxForce, 35.1, 45.1,
            0.0));// Left Posterior Mylohyoid
      myMuscles.add(createPeckMuscle("lgh", geniohyoidMaxForce, 35.1, 45.1,
            0.0));// Left Geniohyoid
      myMuscles.add(createPeckMuscle("lsh", stylohyoidMaxForce, 35.1, 45.1,
            0.0));// Left Stylohyoid
      myMuscles.add(createPeckMuscle("rad", 40.0, 35.1, 45.1, 0.0)); // rad
      // (opener)
      myMuscles.add(createPeckMuscle("rpd", postdigMaxForce, 35.1, 45.1,
            0.0)); // rpd
      myMuscles.add(createPeckMuscle("ram", mylohyoidMaxForce, 35.1, 45.1,
            0.0));// Right Anterior Mylohyoid
      myMuscles.add(createPeckMuscle("rpm", mylohyoidMaxForce, 35.1, 45.1,
            0.0));// Right Posterior Mylohyoid
      myMuscles.add(createPeckMuscle("rgh", geniohyoidMaxForce, 35.1, 45.1,
            0.0));// Right Geniohyoid
      myMuscles.add(createPeckMuscle("rsh", stylohyoidMaxForce, 35.1, 45.1,
            0.0));// Right Stylohyoid

      // hyoid depressors
      myMuscles.add(createPeckMuscle("lth", 20.0, 35.1, 45.1, 0.5));// Left
      // Thyrohyoid
      myMuscles.add(createPeckMuscle("lsteh", 50.0, 35.1, 45.1, 0.5));// Left
      // Sternohyoid
      myMuscles.add(createPeckMuscle("loh", 50.0, 35.1, 45.1, 0.5));// Left
      // Omohyoid
      myMuscles.add(createPeckMuscle("rth", 20.0, 35.1, 45.1, 0.5));// Right
      // Thyrohyoid
      myMuscles.add(createPeckMuscle("rsteh", 50.0, 35.1, 45.1, 0.5));// Right
      // Sternohyoid
      myMuscles.add(createPeckMuscle("roh", 50.0, 35.1, 45.1, 0.5));// Right
      // Omohyoid

      // Laryngeal Muscles (thyroid-cricoid, crico-arytenoid, sternum)
      myMuscles.add(createPeckMuscle("lpc", 20.0, 35.1, 45.1, 0.5));// Left
      // Posterior
      // Cricoarytenoid
      myMuscles.add(createPeckMuscle("llc", 20.0, 35.1, 45.1, 0.5));// Left
      // Lateral
      // Cricoarytenoid
      myMuscles.add(createPeckMuscle("lpct", 20.0, 35.1, 45.1, 0.5));// Left
      // Posterior
      // Cricothyroid
      myMuscles.add(createPeckMuscle("lact", 20.0, 35.1, 45.1, 0.5));// Left
      // Anterior
      // Cricothyroid
      myMuscles.add(createPeckMuscle("lstet", 20.0, 35.1, 45.1, 0.5));// Left
      // Sternothyroid
      myMuscles.add(createPeckMuscle("rpc", 20.0, 35.1, 45.1, 0.5));// Right
      // Posterior
      // Cricoarytenoid
      myMuscles.add(createPeckMuscle("rlc", 20.0, 35.1, 45.1, 0.5));// Right
      // Lateral
      // Cricoarytenoid
      myMuscles.add(createPeckMuscle("rpct", 20.0, 35.1, 45.1, 0.5));// Right
      // Posterior
      // Cricothyroid
      myMuscles.add(createPeckMuscle("ract", 20.0, 35.1, 45.1, 0.5));// Right
      // Anterior
      // Cricothyroid
      myMuscles.add(createPeckMuscle("rstet", 20.0, 35.1, 45.1, 0.5));// Right
      // Sternothyroid

      myMuscles.add(createPeckMuscle("ta", 20.0, 35.1, 45.1, 0.5));// Transverse
      // Arytenoid
      // (midline
      // between
      // left
      // and
      // right
      // arytenoids)

      // get length ratios to update muscle lengths for new Amira geometry
      Muscle m;
      for (int k = 0; k < myMuscles.size(); k++) {
         m = myMuscles.get(k);
      }
      return myMuscles;
   }
   
   public static ArrayList<Muscle> attachMuscles(ArrayList<String> muscleList, HashMap<String, MuscleInfo> muscleInfo,
      HashMap<String, FrameMarker> myFrameMarkers, ArrayList<Muscle> muscles) {
      ArrayList<Muscle> myAttachedMuscles= new ArrayList<> ();
      
      for (int k = 0; k < muscleList.size(); k++) {
         String name = muscleList.get(k);
         if (muscleInfo.get(muscleList.get(k)).isPaired()) {
            myAttachedMuscles.add(addMuscle("l" + name, myFrameMarkers, muscles));
            myAttachedMuscles.add(addMuscle("r" + name, myFrameMarkers, muscles));
         } else {
            myAttachedMuscles.add (addMuscle(name, myFrameMarkers, muscles));
         }
      }
      return myAttachedMuscles;
   }
   
   private static Muscle addMuscle(String name, HashMap<String, FrameMarker> myFrameMarkers, ArrayList<Muscle> myMuscles) {
      Muscle m = findMuscle(name, myMuscles);
      if (m == null) {
         System.err.println(name + " muscle not found.");
         return null;
      }
      m.setFirstPoint(myFrameMarkers.get(name + "_origin"));
      m.setSecondPoint(myFrameMarkers.get(name + "_insertion"));
      AxialSpring.setDamping (m, myMuscleDamping);
      //addAxialSpring(m);
      return m;
   }
   
   private static Muscle findMuscle(String name, ArrayList<Muscle> myMuscles) {
      for (Muscle m : myMuscles) {
         if (name.compareTo(m.getName()) == 0) return m;
      }
      return null;
   }
   
   public static void updateMuscleLengthProps(ArrayList<Muscle> myMuscles) {
      for (Muscle m : myMuscles)
         m.resetLengthProps();
   }
   
   
   public static ArrayList<Muscle> createMuscleList(
      ArrayList<String> muscleAbbreviations, HashMap<String, MuscleInfo>  myMuscleInfo, ArrayList<Muscle> myMuscles) {
      ArrayList<Muscle> list = new ArrayList<Muscle>();
      for (String name : muscleAbbreviations) {
         if (myMuscleInfo.get(name).isPaired()) {
            list.add(findMuscle("l" + name, myMuscles));
            list.add(findMuscle("r" + name, myMuscles));
         } else {
              list.add(findMuscle(name, myMuscles));
         }
      }
      return list;
   }
   
   public static void assembleBilateralExcitors(ArrayList<String> muscleList, HashMap<String,MuscleInfo> muscleInfo, HashMap<String,ExcitationComponent> myMuscles, HashMap<String,String> muscleAbbreviations) {
      for (int k = 0; k < muscleList.size(); k++) {
         String name = muscleList.get(k);
         if (muscleInfo.get(muscleList.get(k)).isPaired()) {
            ExcitationComponent left = myMuscles.get("l" + name);
            ExcitationComponent right = myMuscles.get("r" + name);
            if (left != null && right != null) {
               String excitorName = "bi_" + name;
               MuscleExciter bilateral = new MuscleExciter(excitorName);
               bilateral.addTarget(left, 1.0);
               bilateral.addTarget(right, 1.0);
              // addMuscleExciter(bilateral);
               String fullName = muscleInfo.get(name).fullName;
               muscleAbbreviations.put(excitorName, "Bilateral " + fullName);
            }
         }
      }
   }
   
   public static HashMap<String,MuscleExciter>  assembleMuscleGroups(MuscleGroupInfo info, HashMap<String,ExcitationComponent> myMuscles, ComponentList<MuscleExciter> myMuscleExciters, HashMap<String,String> muscleAbbreviations) {
      HashMap<String,MuscleExciter> myExciters= new LinkedHashMap<> ();
 
         for (int i = 0; i < 2; i++) // add groups for left and right sides
         {
            String prefix = (i == 0 ? "l" : "r");
            String fullPrefix = (i == 0 ? "Left " : "Right ");
            MuscleExciter exciter = new MuscleExciter(prefix + info.name);
            for (String target : info.coactivators) {
               ExcitationComponent c = myMuscles.get(prefix
                     + target);
               if (c == null) { // look for target in excitors list
                  c = myMuscleExciters.get(prefix + target);
               }
               if (c == null) continue;

               exciter.addTarget(c, 1.0);
               muscleAbbreviations.put(prefix + info.name, fullPrefix
                  + info.fullName);
            }
            myExciters.put(exciter.getName (),exciter);
         }
      return myExciters;
   }
   
   public static ArrayList<MuscleExciter> assemblebilateralMuscleGroups(ArrayList<MuscleGroupInfo> muscleGroupInfo, ComponentList<MuscleExciter> mySingleExciters,  HashMap<String,String> muscleAbbreviations){
      // bilateral excitor
      ArrayList<MuscleExciter> myBilateralExciters = new ArrayList<> ();
      int j=0;
      for(int i=0; i<mySingleExciters.size ();i=i+2){
         MuscleExciter bilateral = new MuscleExciter("bi_" + muscleGroupInfo.get (j).name);
         bilateral.addTarget(mySingleExciters.get ("l"+muscleGroupInfo.get (j).name));
         bilateral.addTarget(mySingleExciters.get ("r"+muscleGroupInfo.get (j).name));
         
         myBilateralExciters.add(bilateral);
         muscleAbbreviations.put("bi_" + muscleGroupInfo.get (j).name, "Bilateral "
            + muscleGroupInfo.get (j).fullName);
         j++;
      }      
      return myBilateralExciters;
   }
}