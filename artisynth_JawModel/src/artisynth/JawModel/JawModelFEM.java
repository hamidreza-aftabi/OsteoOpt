package artisynth.JawModel;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;

import artisynth.JawModel.ElasticFoundationForceBehavior;
import artisynth.JawModel.JawModel;
import artisynth.core.femmodels.AbaqusReader;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.PointFem3dAttachment;
import artisynth.core.materials.AxialMuscleMaterial;
import artisynth.core.materials.LigamentAxialMaterial;
import artisynth.core.materials.LinearElasticContact;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.materials.NeoHookeanMaterial;
import artisynth.core.materials.RotAxisFrameMaterial;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.mechmodels.Collidable;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.MultiPointSpring;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.RigidCylinder;
import artisynth.core.mechmodels.RigidEllipsoid;
import artisynth.core.mechmodels.RigidMeshComp;
import artisynth.core.mechmodels.Wrappable;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.renderables.ColorBar;
import artisynth.core.mechmodels.CollisionBehavior.ColorMapType;
import artisynth.core.mechmodels.CollisionBehavior.Method;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.CollisionManager.ColliderType;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.util.ArtisynthIO;
import artisynth.core.util.ArtisynthPath;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.render.color.JetColorMap;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;
import artisynth.core.util.ScalarRange;
import artisynth.core.util.ScanToken;

public class JawModelFEM extends JawModel{
  
   
   
   double PCAscaling = 1;
   
   protected String recType ="bright";
   
   protected double defaultScarK = 0;

   protected boolean useFEMDisc = false;
   
   protected boolean usePlate = false;


   FrameMarker ml;
   FrameMarker mr;
   
   AxialSpring stm_R = new AxialSpring ("stm_R");
   AxialSpring sphm_R = new AxialSpring ("sphm_R");

  public ArrayList<Muscle> myAttachedMuscles = new ArrayList<Muscle>();
   


   protected static double sphm_slack = 5.5;
   protected static double stm_slack =1.5;
  
   
   
   public boolean debug = false; // set to true for debug printlns

   public static final String muscleListFilename = "muscleList.txt";
   
   public static final String wrappedMuscleListFilename = "wrappedMuscleList.txt";

   public static final String bodyListFilename = "bodyList.txt";

   public static final String muscleInfoFilename = "muscleInfo.txt";
   
   public static final String femListFilename = "femList_0.5_edgelength.txt";
   
   public static ArrayList<String> wrappedMuscleList = new ArrayList<String>();

   public static final String muscleGroupInfoFilename = "muscleGroupsInfo.txt";
   
   protected ArrayList<JawModel.BodyInfo> femInfoList = new ArrayList<JawModel.BodyInfo>();

   public ArrayList<Muscle> myMuscles = new ArrayList<Muscle>();

   protected HashMap<String, String> muscleGroupNames = new LinkedHashMap<String, String>();
   
   protected double tmjDiscWeight = 0.0006; // .6g 
   
   protected RigidTransform3d amiraTranformation = new RigidTransform3d (new Vector3d(0,0,0),new RotationMatrix3d (new AxisAngle(new Vector3d(0.975111,-0.20221,0.0909384),Math.toRadians (-9.54211))));
   
   protected static MooneyRivlinMaterial defaultMooneyRivlinMaterial = new MooneyRivlinMaterial (900000/unitConversion, 900/unitConversion, 0d, 0d, 0d, /*90000000*/10*9000000/unitConversion);
   
   protected double myParticleDamping = 0.005;  
   
   protected double myStiffnessDamping = 0.005;     
   
   
   /***Values for EF CONTACT ****/
   protected static final double DEFAULT_E = 12750000/unitConversion;    
   protected static final double DEFAULT_Thickness = 2.35; // mm   
   protected static final double DEFAULT_Damping = 30; 
   protected static final double DEFAULT_Nu = 0.49;
   ElasticFoundationForceBehavior EFContact;
   
   /***Values for EF CONTACT ****/
   protected static final double DEFAULT_E_FE = 6*450000/unitConversion;    
   protected static final double DEFAULT_Thickness_FE = .4; // mm   
   protected static final double DEFAULT_Damping_FE = 150; 
   protected static final double DEFAULT_Nu_FE = 0.49;
   ElasticFoundationForceBehavior EFContact_FE;
   
   protected boolean useMooneyRivlin = true;
   protected boolean useElasticFoundationContact = true;   
   protected boolean useFEMJoint = true;
   
   protected static RigidTransform3d hyoid_translation = new RigidTransform3d (new Vector3d(0,7, -7),new AxisAngle());
   
   LigamentAxialMaterial capsule_ligament_material = new LigamentAxialMaterial (250000000/unitConversion, 0, 50);
   
   protected static PropertyList myProps =
   new PropertyList (JawModelFEM.class, JawModel.class);
  
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }



   
   private void setupRenderProps() {

      // Line Renderprops
      RenderProps props = createRenderProps();
      props.setLineRadius(2.0);
      props.setLineWidth(3);
      props.setLineStyle(Renderer.LineStyle.LINE);
      props.setLineColor(Color.WHITE);

      // Mesh RenderProps
      props.setShading(Renderer.Shading.SMOOTH);
      props.setFaceColor(new Color(1f, 0.8f, 0.6f));
      props.setFaceStyle(Renderer.FaceStyle.FRONT_AND_BACK);
      setRenderProps(props);

      // Spring Render Props
      RenderProps.setLineRadius(myAxialSprings, 2.0);
      RenderProps.setLineStyle(myAxialSprings, Renderer.LineStyle.SPINDLE);
      RenderProps.setLineColor(myAxialSprings, Color.WHITE);      
      
      // Marker RenderProps
      frameMarkers ().getRenderProps ().setPointStyle (PointStyle.SPHERE);
      frameMarkers ().getRenderProps ().setPointSize (1);
      frameMarkers ().getRenderProps ().setPointColor (Color.PINK);      
   }
   
   //read list of FEM models
   private ArrayList<JawModel.BodyInfo> readFemInfoList(String filename)
      throws IOException {
      ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(filename));
      rtok.wordChars(".");
      ArrayList<JawModel.BodyInfo> bodyInfoList = new ArrayList<JawModel.BodyInfo>();
      
      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
         JawModel.BodyInfo bi = new JawModel.BodyInfo();
         bi.scan(rtok);
         bodyInfoList.add(bi);
      }
      return bodyInfoList;
   }
   
   //update intertia and center of mass for the new geometry
   public void setNewJawDynamicProps(){
      rigidBodies ().get ("jaw").setDensity (0.000002330);
      //rigidBodies().get("jaw").setInertiaFromMass (0.2);
      rigidBodies().get("jaw").setRotaryDamping (100);
      rigidBodies().get("jaw").setFrameDamping (50);
   }

  
   
   public void setNewResectedJawDynamicProps(){
      rigidBodies ().get("jaw_resected").setDensity (0.000002330);
      rigidBodies().get("jaw_resected").setDynamic (true);  
      //rigidBodies().get("jaw_resected").setInertiaFromMass (0.1);
      //rigidBodies().get("jaw_resected").setRotaryDamping (100);
      //rigidBodies().get("jaw_resected").setFrameDamping (50);
   }

   
   
   public void setNewDonorDynamicProps(){
      rigidBodies ().get("donor").setDensity (0.000002330);
      rigidBodies().get("donor").setDynamic (true);   
      //rigidBodies().get("donor").setInertiaFromMass (0.1);
      //rigidBodies().get("donor").setRotaryDamping (100);
      //rigidBodies().get("donor").setFrameDamping (50);
   }

   
   
   public FemModel3d createAndAddFemBody(String name, String meshName) {
      FemModel3d model = (FemModel3d) get(name);

      if (model == null) {
              model = new FemModel3d();
              model.setName(name);
              addModel(model);
              PolygonalMesh disc = loadFemMesh(meshName, 1);
            
              disc.transform (amiraTranformation);
              
              if (meshName.compareTo("none") != 0) {
                      FemFactory.createFromMesh(model, disc, 0); // use tetgen to create volume mesh
              }

              RenderProps.setVisible(model, true);
              model.setSurfaceRendering(SurfaceRender.Stress);
              model.setDensity(tmjDiscWeight / model.getVolume());
              model.getElements().getRenderProps().setVisible(false);
         }
              
      setMooneyRivlinMaterial(model);
    

      return model;
      
}
   

   
   
   // Chnage the method I imported the Discl, Fem Factory vs Abq
   
   /*
   public FemModel3d createAndAddFemBody(String name, String meshName) throws IOException {
      FemModel3d model = (FemModel3d)get (name);
      
      if (model == null) {
         model = new FemModel3d();
         model.setName(name);
         addModel(model);
      
         if (meshName.compareTo("none") != 0){      
               AbaqusReader.read (model, ArtisynthPath.getSrcRelativePath(JawModelFEM.class,
                  "geometry/" + meshName),1,0x2);
         }
         
         model.transformGeometry (amiraTranformation);
         
         RenderProps.setVisible (model, true);
         model.setSurfaceRendering (SurfaceRender.Stress);
         model.setDensity (tmjDiscWeight/model.getVolume ());
         model.getElements ().getRenderProps ().setVisible (false);
         
      }
      
      setMooneyRivlinMaterial(model);
      
      return model;
   }

   */
  
   
   
   public PolygonalMesh loadFemMesh(String meshName, double scale){
      String meshFilename = ArtisynthPath.getSrcRelativePath(JawModelFEM.class,
         "geometry/" + meshName);
      PolygonalMesh mesh = new PolygonalMesh();
      
      try {
         mesh.read(new BufferedReader(new FileReader(meshFilename)));
      } catch (IOException e) {
         e.printStackTrace();         
      }
      
      mesh.scale(scale);
      mesh.setFixed(true);
      
      return mesh;
   }
   
   public void assembleFemModels() throws IOException {
      for (JawModel.BodyInfo bodyInfo : femInfoList) {
         createAndAddFemBody(bodyInfo.name, bodyInfo.meshName);
      }
   }
   
   public void setMooneyRivlinMaterial(FemModel3d model){
      model.setMaterial (defaultMooneyRivlinMaterial);   
      model.setParticleDamping (myParticleDamping);
      model.setStiffnessDamping (myStiffnessDamping);
   }
   
   /**
    * used to translate the frames of each body to the center of the body and translate components accordingly
    */
   public void translateFrame(RigidBody body){
      Vector3d centroid= new Vector3d();
      body.getMesh ().computeCentroid (centroid);
      RigidTransform3d XComToBody = new RigidTransform3d();      
      XComToBody.p.set(centroid);
     
      
      RigidTransform3d XBodyToWorld = new RigidTransform3d();
      body.getPose(XBodyToWorld);
      
      RigidTransform3d XComToWorld = new RigidTransform3d();
      XComToWorld.mul(XBodyToWorld, XComToBody);
      body.setPose(XComToWorld);
   
      RigidTransform3d XMeshToCom = new RigidTransform3d();
      if (body.getMeshComps () != null) {
         for(int i=0; i< body.getMeshComps ().size (); i++ ){
            RigidMeshComp mesh =body.getMeshComps ().get(i);
            XMeshToCom.invert(XComToWorld);
            mesh.transformMesh(XMeshToCom);
           // body.setMesh(mesh, null);
         }
      }

      if (body.getName ().equals ("hyoid")){
         body.transformGeometry (hyoid_translation);
      }
     
      for (FrameMarker mrk : frameMarkers()) {
         if (mrk.getFrame() == body) {
            Point3d loc = new Point3d();
            
            mrk.getLocation(loc);
            loc.transform(XMeshToCom);
            mrk.setLocation(loc);
         }
      }

      for (BodyConnector con : bodyConnectors()) {
         if (con.getBodyA() == body) {
            con.transformGeometry(XComToWorld);
         }
      }      
   }
   
   
   // adds elastic foundation contact articular cartilage
   public void addCartilage(){
      CollisionBehavior behav1 = new CollisionBehavior (true, 0);
      CollisionBehavior behav2 = new CollisionBehavior (true, 0);
      CollisionBehavior behav3 = new CollisionBehavior (true, 0);
      CollisionBehavior behav4 = new CollisionBehavior (true, 0);
      CollisionBehavior behav5 = new CollisionBehavior (true, 0);
      CollisionBehavior behav6 = new CollisionBehavior (true, 0); 
      
      behav1.setMethod(Method.VERTEX_PENETRATION);
      behav2.setMethod(Method.VERTEX_PENETRATION);
      behav3.setMethod(Method.VERTEX_PENETRATION);
      behav4.setMethod(Method.VERTEX_PENETRATION);
      behav5.setMethod(Method.VERTEX_PENETRATION);
      behav6.setMethod(Method.VERTEX_PENETRATION);


      
      //orogonal Code
      /*
      behav1.setMethod(Method.VERTEX_PENETRATION_BILATERAL);
      behav2.setMethod(Method.VERTEX_PENETRATION_BILATERAL);
      behav3.setMethod(Method.VERTEX_PENETRATION_BILATERAL);
      behav4.setMethod(Method.VERTEX_PENETRATION_BILATERAL);
      behav5.setMethod(Method.VERTEX_PENETRATION_BILATERAL);
      behav6.setMethod(Method.VERTEX_PENETRATION_BILATERAL);
      behav7.setMethod(Method.VERTEX_PENETRATION_BILATERAL);
      */
      
      if (useElasticFoundationContact == true){

         //original Code
         /*
         ElasticFoundationForceBehavior EFContact =
          new ElasticFoundationForceBehavior (DEFAULT_E, DEFAULT_Nu, DEFAULT_Damping, DEFAULT_Thickness);
         */
          
         LinearElasticContact EFContact = new LinearElasticContact (DEFAULT_E, DEFAULT_Nu, DEFAULT_Damping, DEFAULT_Thickness);
         LinearElasticContact EFContact_FE = new LinearElasticContact (DEFAULT_E_FE, DEFAULT_Nu_FE, DEFAULT_Damping_FE, DEFAULT_Thickness_FE);

         if (useFEMDisc == true) {
           
          removeRigidBody (rigidBodies ().get ("disc_right_rigid"));
          removeRigidBody (rigidBodies ().get ("disc_left_rigid"));

           
            behav1.setForceBehavior (EFContact_FE);
            behav2.setForceBehavior (EFContact_FE);
            behav3.setForceBehavior (EFContact_FE);
            behav4.setForceBehavior (EFContact_FE);
            behav5.setForceBehavior (EFContact_FE);   
            behav6.setForceBehavior (EFContact_FE);   
            getCollisionManager ().setColliderType (ColliderType.AJL_CONTOUR);
            
            //behav1.setMethod (Method.DEFAULT);
            setCollisionBehavior (rigidBodies ().get ("jaw_resected"), (FemModel3d)models().get ("disc_right"), behav1);
            behav1.setName ("mand_disc_right");
            
            //behav2.setMethod (Method.DEFAULT);
            setCollisionBehavior (rigidBodies ().get ("skull"), (FemModel3d)models().get ("disc_right"), behav2);
            behav2.setName ("skull_disc_right");
            
            //behav3.setMethod (Method.DEFAULT);
            setCollisionBehavior (rigidBodies ().get ("jaw"), (FemModel3d)models().get ("disc_left"), behav3);
            behav3.setName ("mand_disc_left");
                  
            //behav4.setMethod (Method.DEFAULT);      
            setCollisionBehavior (rigidBodies ().get ("skull"), (FemModel3d)models().get ("disc_left"), behav4);
            behav4.setName ("skull_disc_left");
            
            behav5.setMethod (Method.DEFAULT);      
            setCollisionBehavior (rigidBodies ().get ("skull"), rigidBodies ().get ("jaw"), behav5);
            behav5.setName ("skull_mandible");
            
            behav6.setMethod (Method.DEFAULT);      
            setCollisionBehavior (rigidBodies ().get ("skull"), rigidBodies ().get ("jaw_resected"), behav6);
            behav6.setName ("skull_mandible_right");

         }
       
         else {
        
            
            rigidBodies ().get ("disc_right_rigid").setDynamic (true);
            rigidBodies ().get ("disc_left_rigid").setDynamic (true);
            rigidBodies ().get ("disc_right_rigid").setInertiaFromMass (.006);
            rigidBodies ().get ("disc_left_rigid").setInertiaFromMass (.006);

            //original Code
            
            /*
            ElasticFoundationForceBehavior EFContact2 =
             new ElasticFoundationForceBehavior (
                DEFAULT_E2, DEFAULT_Nu2, DEFAULT_Damping2, DEFAULT_Thickness2);
             */
             
             behav1.setForceBehavior (EFContact);
             behav2.setForceBehavior (EFContact);
             behav3.setForceBehavior (EFContact);
             behav4.setForceBehavior (EFContact);
             behav5.setForceBehavior (EFContact_FE);  
             behav6.setForceBehavior (EFContact_FE);  
             getCollisionManager ().setColliderType (ColliderType.AJL_CONTOUR);
             
             //behav1.setMethod (Method.DEFAULT);
             setCollisionBehavior (rigidBodies ().get ("jaw_resected"), rigidBodies ().get ("disc_right_rigid"), behav1);
             behav1.setName ("mand_disc_right");
             
             //behav2.setMethod (Method.DEFAULT);
             setCollisionBehavior (rigidBodies ().get ("skull"), rigidBodies ().get ("disc_right_rigid"), behav2);
             behav2.setName ("skull_disc_right");
             
             //behav3.setMethod (Method.DEFAULT);
             setCollisionBehavior (rigidBodies ().get ("jaw"),  rigidBodies ().get("disc_left_rigid"), behav3);
             behav3.setName ("mand_disc_left");
                   
             //behav4.setMethod (Method.DEFAULT);      
             setCollisionBehavior (rigidBodies ().get ("skull"), rigidBodies ().get ("disc_left_rigid"), behav4);
             behav4.setName ("skull_disc_left");
             
             //behav5.setMethod (Method.DEFAULT);      
             setCollisionBehavior (rigidBodies ().get ("skull"), rigidBodies ().get ("jaw"), behav5);
             behav5.setName ("skull_mandible");
             
             
             setCollisionBehavior (rigidBodies ().get ("skull"), rigidBodies ().get ("jaw_resected"), behav6);
             behav6.setName ("skull_mandible_right");
             
             
             /*
             behav6.setMethod (Method.DEFAULT);      
             setCollisionBehavior (rigidBodies ().get ("skull_cartilage_right"), rigidBodies ().get ("disc_right_rigid"), behav6);
             behav6.setName ("skull_disc_cartilage_right");
             
             behav7.setMethod (Method.DEFAULT);      
             setCollisionBehavior (rigidBodies ().get ("skull_cartilage_left"), rigidBodies ().get ("disc_left_rigid"), behav7);
             behav7.setName ("skull_disc_cartilage_left");
             */
             
         }     
         
      }
      

      rigidBodies ().get ("jaw").getMeshComp (0).setIsCollidable (true);
      rigidBodies ().get ("jaw").getMeshComp (1).setIsCollidable (true);
      rigidBodies ().get ("jaw").getMeshComp (1).setName ("ContactMesh");
      
      rigidBodies ().get ("jaw_resected").getMeshComp (0).setIsCollidable (true);
      rigidBodies ().get ("jaw_resected").getMeshComp (1).setIsCollidable (true);
      rigidBodies ().get ("jaw_resected").getMeshComp (1).setName ("ContactMesh_Right");
      
      //attach cartilage to respective bones
      attachFrame (rigidBodies ().get ("mandible_cartilage_right"), rigidBodies ().get ("jaw_resected"));
      attachFrame (rigidBodies ().get ("mandible_cartilage_left"), rigidBodies ().get ("jaw"));
      attachFrame (rigidBodies ().get ("skull_cartilage_right"), rigidBodies ().get ("skull"));
      attachFrame (rigidBodies ().get ("skull_cartilage_left"), rigidBodies ().get ("skull"));
         
      rigidBodies ().get ("mandible_cartilage_right").setMass (0);
      rigidBodies ().get ("mandible_cartilage_left").setMass (0);
      rigidBodies ().get ("skull_cartilage_right").setMass (0);
      rigidBodies ().get ("skull_cartilage_left").setMass (0);
      
      
   }
   
   //create planar constraints for simplified tooth contact
   protected void createBiteConstraints (){
 
      Vector3d pCAr = new Vector3d();
      
      for (FrameMarker mrk : frameMarkers()) {
         if (mrk.getName() == "rbite") {
           
            pCAr.x = mrk.getLocation ().x;
            pCAr.y = mrk.getLocation ().y;
            pCAr.z = mrk.getLocation ().z;
         
  
         }
      }
      
           
      /*
      Vector3d trans = new Vector3d(-17.540899, -66.518233,  -43.405483);
      RigidTransform3d XPWr = new RigidTransform3d (trans, new AxisAngle (new Vector3d(1,0,0), Math.toRadians (180)));
      XPWr.mulRotation (new RotationMatrix3d(new AxisAngle(new Vector3d(0,1,0),Math.toRadians (-0))));
      //XPWr.setTranslation (-20.85772, -67.02249, -43.405483);
      PlanarConnector conr = new PlanarConnector (rigidBodies ().get ("jaw_resected"), pCAr, XPWr);
      
      conr.setPlaneSize (20);
      conr.getRenderProps ().setAlpha (0.7);
      conr.getRenderProps ().setFaceColor (Color.GRAY);
      conr.setUnilateral (true);
      conr.setName ("RBITE");
      conr.setEnabled (true);
      RenderProps.setVisible (conr, true);
  
        */
      
      
      Vector3d pCAl = new Vector3d();
      
      for (FrameMarker mrk : frameMarkers()) {
         if (mrk.getName() == "lbite") {
           
            pCAl.x = mrk.getLocation ().x;
            pCAl.y = mrk.getLocation ().y;
            pCAl.z = mrk.getLocation ().z;

         }
      }
      
      Vector3d trans2 = new Vector3d(24.785562, -69.041624, -43.405483);      
      RigidTransform3d XPWl = new RigidTransform3d (trans2, new AxisAngle (new Vector3d(1,0,0), Math.toRadians (180)));
      XPWl.mulRotation (new RotationMatrix3d(new AxisAngle(new Vector3d(0,1,0),Math.toRadians (-0))));
      //XPWl.setTranslation (28.951207, -67.02249, -43.405483);
      PlanarConnector conl = new PlanarConnector (rigidBodies ().get ("jaw"), pCAl, XPWl);
      
      conl.setPlaneSize (20);
      conl.getRenderProps ().setAlpha (0.7);
      conl.getRenderProps ().setFaceColor (Color.GRAY);
      conl.setUnilateral (true);
      conl.setName ("LBITE");


      Vector3d pCA = new Vector3d(-2.26103, -44.0217, 6.87158);
      RigidTransform3d XPW = new RigidTransform3d (pCA, new AxisAngle (new Vector3d(1,0,0), Math.toRadians (180)));
      PlanarConnector con = new PlanarConnector (rigidBodies ().get ("jaw"), pCA, XPW);
      
      con.setPlaneSize (20);
      con.getRenderProps ().setAlpha (0.7);
      con.getRenderProps ().setFaceColor (Color.GRAY);
      con.setUnilateral (true);
      con.setName ("BiteICP");
      con.setEnabled (false);
      RenderProps.setVisible (con, false);
      
      
      Vector3d pCA2 = new Vector3d(24.9041, -18.4052, 4.9116);
      RigidTransform3d XPW2 = new RigidTransform3d (pCA2, new AxisAngle (new Vector3d(1,0,0), Math.toRadians (180)));
      XPW2.mulRotation (new RotationMatrix3d(new AxisAngle(new Vector3d(0,1,0),Math.toRadians (-0))));
      
      PlanarConnector con2 = new PlanarConnector (rigidBodies ().get ("jaw"), pCA2, XPW2);
      con2.setPlaneSize (20);
      con2.getRenderProps ().setAlpha (0.7);
      con2.getRenderProps ().setFaceColor (Color.GRAY);
      con2.setUnilateral (true);
      con2.setName ("Brux_M6");
      con2.setEnabled (false);
      RenderProps.setVisible (con2, false);
      
      
      Vector3d pCA3 = new Vector3d(11.8622,  -39.9166,    6.1264);
      RigidTransform3d XPW3 = new RigidTransform3d (pCA3, new AxisAngle (new Vector3d(1,0,0), Math.toRadians (180)));
      XPW3.mulRotation (new RotationMatrix3d(new AxisAngle(new Vector3d(0,1,0),Math.toRadians (-40))));
  
      PlanarConnector con3 = new PlanarConnector (rigidBodies ().get ("jaw"), pCA3, XPW3);
      con3.setPlaneSize (20);
      con3.getRenderProps ().setAlpha (0.7);
      con3.getRenderProps ().setFaceColor (Color.GRAY);
      con3.setUnilateral (true);
      con3.setName ("Brux_C");
      con3.setEnabled (false);
      RenderProps.setVisible (con3, false);
  
      
      Vector3d pCA4 = new Vector3d(24.9041, -18.4052, 4.9116);
      RigidTransform3d XPW4 = new RigidTransform3d (pCA4, new AxisAngle (new Vector3d(1,0,0), Math.toRadians (180)));
      XPW4.mulRotation (new RotationMatrix3d(new AxisAngle(new Vector3d(0,1,0),Math.toRadians (-50))));
      XPW4.mulRotation (new RotationMatrix3d(new AxisAngle(new Vector3d(0,0,1),Math.toRadians (15))));
      PlanarConnector con4 = new PlanarConnector (rigidBodies ().get ("jaw"), pCA4, XPW4);
      con4.setPlaneSize (20);
      con4.getRenderProps ().setAlpha (0.7);
      con4.getRenderProps ().setFaceColor (Color.GRAY);
      con4.setUnilateral (true);
      con4.setName ("Brux_PM5");
      con4.setEnabled (false);
      RenderProps.setVisible (con4, false);
      
      
      Vector3d pCA5 = new Vector3d(11.8622,  -39.9166,    6.1264);
      RigidTransform3d XPW5 = new RigidTransform3d (pCA5, new AxisAngle (new Vector3d(1,0,0), Math.toRadians (180)));
      XPW5.mulRotation (new RotationMatrix3d(new AxisAngle(new Vector3d(0,1,0),Math.toRadians (-36))));
      XPW5.mulRotation (new RotationMatrix3d(new AxisAngle(new Vector3d(1,0,0),Math.toRadians (15))));
      PlanarConnector con5 = new PlanarConnector (rigidBodies ().get ("jaw"), pCA5, XPW5);
      con5.setPlaneSize (20);
      con5.getRenderProps ().setAlpha (0.7);
      con5.getRenderProps ().setFaceColor (Color.GRAY);
      con5.setUnilateral (true);
      con5.setName ("Brux_PM4");
      con5.setEnabled (false);
      RenderProps.setVisible (con5, false);
      
      
      addBodyConnector (conl);
      //addBodyConnector (conr);
      //addBodyConnector (con);
      //addBodyConnector (con2);
      //addBodyConnector (con3);
      //addBodyConnector (con4);
      //addBodyConnector (con5);

   }
   
   void createFoodContact() {
    
      //create framemarkers for contact points of constraints
      FrameMarker m1=new FrameMarker (rigidBodies().get("jaw"), new Point3d(2.3768318, -99.827457, -40.301746)); 
      FrameMarker ml=new FrameMarker (rigidBodies().get("jaw"), new Point3d(24.785562, -69.041624, -43.405483));
      //FrameMarker mr=new FrameMarker (rigidBodies().get("jaw_resected"), new Point3d(-17.540899, -66.518233, -43.405483));
  
      
      m1.setName ("lowerincisor");
      ml.setName ("lbite");
      //mr.setName ("rbite");

      RenderProps.setPointColor (ml, Color.BLUE);
      //RenderProps.setPointColor (mr, Color.BLUE);
      
      addFrameMarker (m1);
      addFrameMarker (ml);
      //addFrameMarker (mr);
      

   }

   
   // attach ligaments for simplified capsule representation
   protected void attachLigaments() throws IOException{ 

      FrameMarker m_ml_r= new FrameMarker (rigidBodies ().get ("mandible_cartilage_right"), new Point3d(0.291939, -4.14064, 2.56426));
      m_ml_r.setName ("jaw_ligament_r");
      FrameMarker m_sl_r= new FrameMarker (rigidBodies ().get ("skull_cartilage_right"), new Point3d(0.448085, 4.52591, 2.58537));
      m_sl_r.setName ("skull_ligament_r");
      
      //FrameMarker m_ml_l= new FrameMarker (rigidBodies ().get ("mandible_cartilage_left"), new Point3d(-2.17999, -2.86218, 1.81019));
      FrameMarker m_ml_l= new FrameMarker (rigidBodies ().get ("mandible_cartilage_left"), new Point3d(1.70796, -3.90319, 1.19122));
      m_ml_l.setName ("jaw_ligament_l");
      FrameMarker m_sl_l= new FrameMarker (rigidBodies ().get ("skull_cartilage_left"), new Point3d(1.09947, 6.27995, 0.589989));
      m_sl_l.setName ("skull_ligament_l");
      
      FrameMarker lat_lig_l = new FrameMarker (rigidBodies ().get ("mandible_cartilage_left"), new Point3d(10.8907, -3.11902, 0.241905));
      lat_lig_l.setName ("lat_caps_l");
      
      FrameMarker med_lig_l = new FrameMarker (rigidBodies ().get ("mandible_cartilage_left"), new Point3d(-11.0365, 2.3139, 3.08688));
      med_lig_l.setName ("med_caps_l");
      
      FrameMarker lat_lig_r = new FrameMarker (rigidBodies ().get ("mandible_cartilage_right"), new Point3d(-10.7861, -1.52841, 1.46048));
      lat_lig_r.setName ("lat_caps_r");
      
      FrameMarker med_lig_r = new FrameMarker (rigidBodies ().get ("mandible_cartilage_right"), new Point3d(10.3989, 2.79619, 2.22426));
      med_lig_r.setName ("med_caps_r");
      
      ////////////////////////////get attachment position from geometry not list/////////////////////////
      Particle lig_left_lat = new Particle (0,new Point3d(65.019852, -11.481986, -4.1072206));
      lig_left_lat.setName ("lig_left_lat");
      
      Particle lig_left_med = new Particle (0,new Point3d(44.126351, -5.8700357, -1.3299661));
      lig_left_med.setName ("lig_left_med");
     
      Particle lig_left_ant = new Particle (0,new Point3d(55.350088, -11.363722, -2.5779645));
      lig_left_ant.setName ("lig_left_ant");
      
      Particle lig_left_post = new Particle (0,new Point3d(55.088378, -3.6865039, -0.41415746));
      lig_left_ant.setName ("lig_left_post");
      
      Particle lig_right_lat = new Particle (0,new Point3d(-54.213407, -7.701083, -5.6393926));
      lig_right_lat.setName ("lig_right_lat");
      
      Particle lig_right_med = new Particle (0,new Point3d(-34.190005, -1.8034389, -4.6513553));
      lig_right_med.setName ("lig_right_med");
     
      Particle lig_right_ant = new Particle (0,new Point3d(-43.165001, -8.5129663, -4.8143906));
      lig_right_ant.setName ("lig_righ_ant");
      
      Particle lig_right_post = new Particle (0,new Point3d(-43.927232, -1.7415594, -0.98541508));
      lig_right_post.setName ("lig_right_post");
      
      addParticle (lig_left_lat);
      addParticle (lig_left_med);
      addParticle (lig_left_ant);
      addParticle (lig_left_post);
      addParticle (lig_right_lat);
      addParticle (lig_right_med);
      addParticle (lig_right_ant);
      addParticle (lig_right_post);
      
      //////////////////////get attachment position from geometry not list/////////////
      
      
      addFrameMarker (m_ml_r);
      addFrameMarker (m_ml_l);
      addFrameMarker (m_sl_r);
      addFrameMarker (m_sl_l);
      addFrameMarker (lat_lig_l);
      addFrameMarker (med_lig_l);
      addFrameMarker (lat_lig_r);
      addFrameMarker (med_lig_r);
     
      
     
       double size=4;
     
       RigidCylinder cylinder = new RigidCylinder ("anterior_ligament_wrapping_left", size, 3*size, 0);
       cylinder.setDynamic (false);
       cylinder.setPose (new RigidTransform3d (new Vector3d(54.812, -6.8707, -4.7085),new AxisAngle (0.14312, 0.98485, -0.097955,  Math.toRadians (95.844))));
       cylinder.getRenderProps ().setFaceStyle (FaceStyle.NONE);
       cylinder.getRenderProps ().setLineColor (Color.BLUE);
       cylinder.getRenderProps ().setDrawEdges (true);
       addRigidBody (cylinder);
       attachFrame(cylinder, rigidBodies ().get ("jaw"));
      
       RigidCylinder cylinder2 = new RigidCylinder ("anterior_ligament_wrapping_right", size, 3*size, 0);
       cylinder2.setDynamic (false);
       cylinder2.setPose (new RigidTransform3d (new Vector3d(-45.3936, -5.09499, -6.75523),new AxisAngle (-0.015408, 0.99489, 0.099805, Math.toRadians (83.846))));
       cylinder2.getRenderProps ().setFaceColor (Color.BLUE);
       cylinder2.getRenderProps ().setFaceStyle (FaceStyle.NONE);
       cylinder2.getRenderProps ().setLineColor (Color.BLUE);
       cylinder2.getRenderProps ().setDrawEdges (true);
       addRigidBody (cylinder2);
       attachFrame(cylinder2, rigidBodies ().get ("jaw_resected"));
       
       //////////////SLACKLENGTH////////////////////
       createLigament(lig_left_ant,(FemModel3d)models ().get ("disc_left"), m_ml_l, 4, cylinder,1.5,0.6);   
       createLigament(lig_right_ant,(FemModel3d)models ().get ("disc_right"), m_ml_r, 4, cylinder2,1.5,0.6);      
       createLigament(lig_right_post,(FemModel3d)models ().get ("disc_right"), (m_sl_r), 7.5,1.25,99);     
       createLigament(lig_left_post,(FemModel3d)models ().get ("disc_left"), m_sl_l, 7.5,1.25,99);
       createLigament(lig_left_lat,(FemModel3d)models ().get ("disc_left"), lat_lig_l,2.5,1.75,99);
       createLigament(lig_left_med,(FemModel3d)models ().get ("disc_left"), med_lig_l, 1.9,1.25,99);
       createLigament(lig_right_lat,(FemModel3d)models ().get ("disc_right"), lat_lig_r, 2.5,1.75,99); 
       createLigament(lig_right_med,(FemModel3d)models ().get ("disc_right"), med_lig_r, 1.9,1.25,99);
       
       //////////////NO SLACKLENGTH///////
       /*createLigament(lig_left_ant,(FemModel3d)models ().get ("disc_left"), m_ml_l, 0, cylinder,1.0);   
       createLigament(lig_right_ant,(FemModel3d)models ().get ("disc_right"), m_ml_r, 0, cylinder2,1.0);      
       createLigament(lig_right_post,(FemModel3d)models ().get ("disc_right"), (m_sl_r), 0,1.0);     
       createLigament(lig_left_post,(FemModel3d)models ().get ("disc_left"), m_sl_l, 0,1.0);
       createLigament(lig_left_lat,(FemModel3d)models ().get ("disc_left"), lat_lig_l,01.25);
       createLigament(lig_left_med,(FemModel3d)models ().get ("disc_left"), med_lig_l, 0,1.0);
       createLigament(lig_right_lat,(FemModel3d)models ().get ("disc_right"), lat_lig_r, 0,1.25); 
       createLigament(lig_right_med,(FemModel3d)models ().get ("disc_right"), med_lig_r, 0,1.0);*/
      
     }

    public void createLigament (ArrayList<Integer> attached_points, FemModel3d model, FrameMarker m, double slack ){
       //create a particle at the middle of the FEM nodes where the force of the ligament will be distributed and attach these nodes to the particle
       
       Point3d pos=new Point3d();
       for (int i=0; i<attached_points.size ();i++){
          pos.add (model.getNode (attached_points.get (i)).getPosition ());
       }
      
       pos.x=pos.x/(attached_points.size ());
       pos.y=pos.y/(attached_points.size ());
       pos.z=pos.z/(attached_points.size ());
       
       Particle p1 = new Particle ();
       p1.setMass (0);
       p1.setPosition (pos);
       addParticle (p1);
       PointFem3dAttachment att = new PointFem3dAttachment (p1);
       att.setFromNodes (p1.getPosition (), collectNodes(model,attached_points));
       addAttachment (att);
       
       //create an axial spring between bone frame marker and particle
       AxialSpring as = new AxialSpring ();
       as.setFirstPoint (p1);
       as.setSecondPoint (m);
       as.setMaterial (capsule_ligament_material);
       as.getRenderProps ().setLineStyle (LineStyle.LINE);
       as.setRestLength (as.getLength () + slack);
       as.getRenderProps ().setLineColor (Color.GREEN);
       as.getRenderProps ().setLineStyle (LineStyle.CYLINDER);
       as.getRenderProps ().setLineRadius (0.75);
       addAxialSpring (as);

       for (FemNode n : att.getNodes()) {
          RenderProps.setSphericalPoints (n, 0.2, Color.GREEN);
       }
    }
    
    public void createLigament (ArrayList<Integer> attached_points, FemModel3d model, FrameMarker m, double slack, RigidBody wrappingBody){
      //create a particle at the middle of the FEM nodes where the force of the ligament will be distributed and attach these nodes to the particle
       
      Point3d pos=new Point3d();
      for (int i=0; i<attached_points.size ();i++){
         pos.add (model.getNode (attached_points.get (i)).getPosition ());
      }
     
      pos.x=pos.x/(attached_points.size ());
      pos.y=pos.y/(attached_points.size ());
      pos.z=pos.z/(attached_points.size ());
      Particle p1 = new Particle ();
      p1.setMass (0);
      p1.setPosition (pos);
      addParticle (p1);
      PointFem3dAttachment att = new PointFem3dAttachment (p1);
      att.setFromNodes (p1.getPosition (),collectNodes(model,attached_points));
      addAttachment (att);
       
      //create an multipoint spring between bone frame marker and particle
      MultiPointSpring as = new MultiPointSpring ();
      as.addPoint (m);
      as.setSegmentWrappable (30);
      as.addWrappable ((Wrappable)wrappingBody);
      as.addPoint (p1);
      as.setMaterial (capsule_ligament_material);
      as.getRenderProps ().setLineStyle (LineStyle.LINE);
      as.setRestLength (as.getLength ()+slack);
      as.getRenderProps ().setLineColor (Color.GREEN);
      as.getRenderProps ().setLineStyle (LineStyle.CYLINDER);
      as.getRenderProps ().setLineRadius (0.75);
      addMultiPointSpring (as);
        
      for (FemNode n : att.getNodes()) {
         RenderProps.setSphericalPoints (n, 0.2, Color.GREEN);
      }
    }
    
    public void createLigament (Particle p, FemModel3d model, FrameMarker m, double slack, double r, double zz){
       //create a particle at the middle of the FEM nodes where the force of the ligament will be distributed and attach these nodes to the particle
       
       ArrayList<FemNode3d> attached_points = findAttachmentNodes(r,p,model,zz);
       
       PointFem3dAttachment att = new PointFem3dAttachment (p);
       att.setFromNodes (p.getPosition (), attached_points);
       addAttachment (att);
       
       //create an axial spring between bone frame marker and particle
       AxialSpring as = new AxialSpring ();
       as.setFirstPoint (p);
       as.setSecondPoint (m);
       as.setMaterial (capsule_ligament_material);
       as.getRenderProps ().setLineStyle (LineStyle.LINE);
       as.setRestLength (as.getLength () + slack);
       as.getRenderProps ().setLineColor (Color.GREEN);
       as.getRenderProps ().setLineStyle (LineStyle.CYLINDER);
       as.getRenderProps ().setLineRadius (0.75);
       addAxialSpring (as);

       for (FemNode n : att.getNodes()) {
          RenderProps.setSphericalPoints (n, 0.2, Color.GREEN);
       }
    }
    
    public void createLigament (Particle p, FemModel3d model, FrameMarker m, double slack, RigidBody wrappingBody, double r, double zz){
       //create a particle at the middle of the FEM nodes where the force of the ligament will be distributed and attach these nodes to the particle
        
       ArrayList<FemNode3d> attached_points = findAttachmentNodes(r, p, model,zz);
       
       PointFem3dAttachment att = new PointFem3dAttachment (p);
       att.setFromNodes (p.getPosition (), attached_points);
       addAttachment (att);
        
       //create an multipoint spring between bone frame marker and particle
       MultiPointSpring as = new MultiPointSpring ();
       as.addPoint (m);
       as.setSegmentWrappable (30);
       as.addWrappable ((Wrappable)wrappingBody);
       as.addPoint (p);
       as.setMaterial (capsule_ligament_material);
       as.getRenderProps ().setLineStyle (LineStyle.LINE);
       as.setRestLength (as.getLength ()+slack);
       as.getRenderProps ().setLineColor (Color.GREEN);
       as.getRenderProps ().setLineStyle (LineStyle.CYLINDER);
       as.getRenderProps ().setLineRadius (1);
       addMultiPointSpring (as);
         
       for (FemNode n : att.getNodes()) {        
          RenderProps.setSphericalPoints (n, 0.2, Color.GREEN);
       }
     }
    
    public ArrayList<FemNode3d> findAttachmentNodes(double r, Particle p1, FemModel3d disc) {
       ArrayList<FemNode3d> nodes= new ArrayList<FemNode3d> ();

      
       double distance;
   
       Vector3d nrm= new Vector3d ();
      for (FemNode3d n : disc.getNodes ()) {
          if(disc.isSurfaceNode (n)) {
            
             distance=n.getPosition ().distance (p1.getPosition ());
             if(distance>r) {
                continue;
             }
             else {
                disc.getSurfaceVertex (n).computeNormal (nrm);
                nodes.add (n);
                }
          }
      }
       
    return nodes;   
    }
    
    public ArrayList<FemNode3d> findAttachmentNodes(double r, Particle p1, FemModel3d disc, double zz) {
       ArrayList<FemNode3d> nodes= new ArrayList<FemNode3d> ();

       double distance;
    
      Vector3d nrm= new Vector3d ();
      for (FemNode3d n : disc.getNodes ()) {
          if(disc.isSurfaceNode (n)) {
            
             distance=n.getPosition ().distance (p1.getPosition ());
             if(distance>r) {
                continue;
             }
             else {
                disc.getSurfaceVertex (n).computeNormal (nrm);
               if(zz!=99 && nrm.z> zz){
                   continue;
                }
                else {
                   nodes.add (n);
                }
             }
          }
      }
    return nodes;   
    }
    
    private HashSet<FemNode3d> collectNodes (FemModel3d fem, ArrayList<Integer> nodeNums) {
      HashSet<FemNode3d> nodes = new LinkedHashSet<FemNode3d>();
         for (int i=0; i<nodeNums.size (); i++) {
            FemNode3d e = fem.getNode (nodeNums.get (i));
            nodes.add (e);                
         }
      return nodes;
    }
       
   private ArrayList<Integer> readIntList(String filename) throws IOException {
      ArrayList<Integer> stringList = new ArrayList<Integer>();
      ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(filename));

      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
         if (rtok.ttype != ReaderTokenizer.TT_NUMBER) { throw new IOException(
            "readMarkerList Expecting number, got " + rtok.tokenName()); }
         stringList.add((int)rtok.lval);
      }
      return stringList;
   }
   
   public void addWrappedMuscles (ArrayList<String> wrappedMuscleList,  HashMap<String, ExcitationComponent> myMuscles){
      HashMap<String, RigidBody> wrappingBodies = new LinkedHashMap<String, RigidBody>();
      /* as of now only the superior head of the lateral pterygoid has wrapping geometry
       * wrapping bodies are created and only added if the related muscles in part of wrappedMuscleList
       */
      double size=10;
      double density = 150;
      RigidCylinder cylinder = new RigidCylinder (
         "cylinder_rsp", size/1.1, 2*size, density, 50);
      cylinder.setPose (new RigidTransform3d (new Vector3d(-40.1763, -10.8551, 2.3),new AxisAngle (-0.12701, 0.98571, 0.11067 ,  Math.toRadians (89.728))));
      cylinder.setDynamic (false);
      cylinder.getRenderProps ().setFaceColor (Color.BLUE);
      cylinder.getRenderProps ().setFaceColor (Color.BLUE);
      cylinder.getRenderProps ().setFaceStyle (FaceStyle.NONE);
      cylinder.getRenderProps ().setLineColor (Color.BLUE);
      cylinder.getRenderProps ().setDrawEdges (true);
      
      wrappingBodies.put ("rsp",cylinder);
      
      RigidCylinder cylinder2 = new RigidCylinder (
         "cylinder_lsp", size/1.1, 2*size, density, 50);
      cylinder2.setDynamic (false);
      cylinder2.setPose (new RigidTransform3d (new Vector3d(49.0791, -14.5805, 4.64117),new AxisAngle (0.14726, 0.98318, -0.10801, Math.toRadians ( 90.734))));
      cylinder2.getRenderProps ().setFaceColor (Color.BLUE);
      cylinder2.getRenderProps ().setFaceColor (Color.BLUE);
      cylinder2.getRenderProps ().setFaceStyle (FaceStyle.NONE);
      cylinder2.getRenderProps ().setLineColor (Color.BLUE);
      cylinder2.getRenderProps ().setDrawEdges (true);
      wrappingBodies.put ("lsp",cylinder2);
      
      ArrayList<AxialSpring> wrappedMuscles = new ArrayList<AxialSpring>();

      
      for(int i=0; i<wrappedMuscleList.size (); i++){
         wrappedMuscles.add (axialSprings ().get(wrappedMuscleList.get (i)));
         addRigidBody(wrappingBodies.get (wrappedMuscles.get (i).getName ()));
      }
      
      /*
       * sets up MultiPointMuscle using the same material properties that have been used for the respective axial spring muscle
       * and deletes axial spring muscle + replaces it in myMuscles List
       */
      for(int i=0; i<wrappedMuscles.size (); i++){
         MultiPointMuscle m= new MultiPointMuscle();
         m.addPoint(((Muscle)axialSprings ().get(wrappedMuscleList.get (i))).getFirstPoint ());
         m.setSegmentWrappable (20, new Point3d[] {new Point3d(0,0,0)});
         m.addWrappable ((Wrappable)wrappingBodies.get (wrappedMuscleList.get (i)));
         m.addPoint(((Muscle)axialSprings ().get(wrappedMuscleList.get (i))).getSecondPoint ());
         m.updateWrapSegments();
         m.setMaterial (((Muscle)axialSprings ().get(wrappedMuscleList.get (i))).getMaterial ());
         String name=((Muscle)axialSprings ().get(wrappedMuscleList.get (i))).getName();
      
         double length = m.getLength();
         double optLength = ((AxialMuscleMaterial)m.getMaterial ()).getOptLength();
         double maxLength = ((AxialMuscleMaterial)m.getMaterial ()).getMaxLength();
         double maxOptRatio =(optLength != 0.0) ? maxLength / optLength : 1.0;
         ((AxialMuscleMaterial)m.getMaterial ()).setOptLength(length);
         ((AxialMuscleMaterial)m.getMaterial ()).setMaxLength(length * maxOptRatio); 
         ((AxialMuscleMaterial)m.getMaterial ()).setMaxForce (((AxialMuscleMaterial)m.getMaterial ()).getMaxForce ()*0.001);
  
         m.setName (name);
         m.setExcitationColor (Color.RED);
         m.getRenderProps ().setLineStyle (LineStyle.CYLINDER);
         m.getRenderProps ().setLineRadius (0.75);
         addMultiPointSpring (m);
      
         myMuscles.remove (name);
         myMuscles.put (name, m);
      }
      
      ComponentUtils.removeComponents (wrappedMuscles, null);
   }
   
   public void renderCollisionForces(){
      CollisionBehavior behav = getCollisionBehavior ((Collidable)rigidBodies().get("skull"), (Collidable)models ().get ("disc_right"));
      behav.setDrawColorMap (ColorMapType.CONTACT_PRESSURE);
      behav.getPenetrationDepthRange().setUpdating (
      ScalarRange.Updating.AUTO_FIT);
       
      behav = getCollisionBehavior ((Collidable)rigidBodies().get("skull"), (Collidable)models ().get ("disc_left"));
      behav.setDrawColorMap (ColorMapType.CONTACT_PRESSURE);
      behav.getPenetrationDepthRange().setUpdating (
      ScalarRange.Updating.AUTO_FIT);
    
      CollisionManager cm = getCollisionManager();
      RenderProps.setVisible (cm, true);
      cm.setContactForceLenScale (10);

      
      JetColorMap map = new JetColorMap();
      map.setColorArray (
         new Color[] {
         new Color(0x0000FF), // blue
         new Color(0x007FFF), // dark cyan
         new Color(0x00FFFF), // cyan
         new Color(0x7FFF7F), // dark green
         new Color(0xFFFF00), // yellow
         new Color(0xFF7F00), // orange
         new Color(0xFF0000), // red
         new Color(0x7F0000), // dark red
         });
   }
  
   private static Color createColor (int r, int g, int b) {
      return new Color (r/255.0f, g/255.0f, b/255.0f);
   }   
   
// Creates and returns a ColorBar renderable object
   public ColorBar createColorBar() {
      ColorBar cbar = new ColorBar();
      cbar.setName("colorBar");
      cbar.setNumberFormat("%.2f");      // 2 decimal places
      cbar.populateLabels(0.0, 1.0, 10); // Start with range [0,1], 10 ticks
      cbar.setLocation(-100, 0.1, 20, 0.8);
      cbar.setTextColor (Color.WHITE);
      addRenderable(cbar);               // add to root model's renderables
      return cbar;
   }
   
   //creates a finite element TMJ capsule plus TMJ ligaments
   public void addCapsule() throws IOException {      
      FemModel3d caps_left = new FemModel3d ("capsule_l");
      FemModel3d caps_right = new FemModel3d ("capsule_r");
      
       PolygonalMesh m_l= new PolygonalMesh ();
       m_l= loadFemMesh ("caps_l_v11.obj", 1);
           FemFactory.createShellModel (caps_left, m_l, 2.3, true);
       FemModel3d caps_left_wedge= new FemModel3d ();
       FemFactory.createExtrusion(caps_left_wedge, 1, 2.3, 0,caps_left);
       caps_left=caps_left_wedge;
       caps_left.setName ("capsule_l");
             
       ArrayList<Integer> attached_points_l_man = readIntList(ArtisynthPath.getSrcRelativePath(JawModelFEM.class,"geometry"+"/capsule_l_fixedNodes_man_wedge_fine_5.txt"));
       for (int i : attached_points_l_man){
          attachPoint (caps_left.getNode (i), rigidBodies().get ("jaw"));
          caps_left.getNode (i).setDynamic (false);
          RenderProps.setPointColor (caps_left.getNode (i), Color.GREEN);
          RenderProps.setPointStyle (caps_left.getNode (i), PointStyle.SPHERE);
          RenderProps.setPointRadius (caps_left.getNode (i), 0.35);
          RenderProps.setAlpha (caps_left.getNode (i), 1);
       }
       
       ArrayList<Integer> attached_points_l_skull = readIntList(ArtisynthPath.getSrcRelativePath(JawModelFEM.class,"geometry"+"/capsule_l_fixedNodes_skull_wedge_fine_5.txt"));
       for (int i : attached_points_l_skull){
          attachPoint (caps_left.getNode (i), rigidBodies().get ("skull"));
          caps_left.getNode (i).setDynamic (false);
          RenderProps.setPointColor (caps_left.getNode (i), Color.GREEN);
          RenderProps.setPointStyle (caps_left.getNode (i), PointStyle.SPHERE);
          RenderProps.setPointRadius (caps_left.getNode (i), 0.35);
          RenderProps.setAlpha (caps_left.getNode (i), 1);
       }
       
       PolygonalMesh m_r= new PolygonalMesh ();
       m_r= loadFemMesh ("caps_r_v19.obj", 1);
       
       FemFactory.createShellModel (caps_right, m_r, 2.3, true);
       
       FemModel3d caps_right_wedge= new FemModel3d ();
       FemFactory.createExtrusion(caps_right_wedge, 1, 2.3, 0, caps_right);
       
       caps_right=caps_right_wedge;
       caps_right.setName ("capsule_r");
      
       ArrayList<Integer> attached_points_r_man = readIntList(ArtisynthPath.getSrcRelativePath(JawModelFEM.class,"geometry"+"/capsule_r_fixedNodes_man_wedge_fine_19.txt"));   
       for (int i : attached_points_r_man){
          attachPoint (caps_right.getNode (i), rigidBodies().get ("jaw_resected"));
          caps_right.getNode (i).setDynamic (false);
          RenderProps.setPointColor (caps_right.getNode (i), Color.GREEN);
          RenderProps.setPointStyle (caps_right.getNode (i), PointStyle.SPHERE);
          RenderProps.setPointRadius (caps_right.getNode (i), 0.35);
          RenderProps.setAlpha (caps_right.getNode (i), 1);
       }
       
       ArrayList<Integer> attached_points_r_skull = readIntList(ArtisynthPath.getSrcRelativePath(JawModelFEM.class,"geometry"+"/capsule_r_fixedNodes_skull_wedge_fine_19.txt"));
       for (int i : attached_points_r_skull){
          attachPoint (caps_right.getNode (i), rigidBodies().get ("skull"));
          caps_right.getNode (i).setDynamic (false);
          RenderProps.setPointColor (caps_right.getNode (i), Color.GREEN);
          RenderProps.setPointStyle (caps_right.getNode (i), PointStyle.SPHERE);
          RenderProps.setPointRadius (caps_right.getNode (i), 0.35);  
          RenderProps.setAlpha (caps_right.getNode (i), 1);
       }
      
     
      caps_left.setSurfaceRendering (SurfaceRender.Shaded);
      caps_right.setSurfaceRendering (SurfaceRender.Shaded);
      caps_left.setDensity (0.0000001195);
      caps_right.setDensity(0.0000001195);
            
      NeoHookeanMaterial mat = new NeoHookeanMaterial (1440, 0);
      caps_left.setMaterial (mat);
      caps_right.setMaterial (mat);
      caps_left.setMassDamping (0.001);
      caps_left.setStiffnessDamping (0.001);
      caps_right.setMassDamping (0.001);
      caps_right.setStiffnessDamping (0.001);
      RenderProps.setAlpha (caps_right, 0.15);
      RenderProps.setAlpha (caps_left, 0.15);
      
      addModel(caps_left);
      addModel(caps_right);
      
      CollisionBehavior behav1 = new CollisionBehavior (true, 0);
      CollisionBehavior behav2 = new CollisionBehavior (true, 0);
      CollisionBehavior behav3 = new CollisionBehavior (true, 0);
      CollisionBehavior behav4 = new CollisionBehavior (true, 0);
      
      behav1.setMethod(Method.VERTEX_PENETRATION_BILATERAL);
      behav2.setMethod(Method.VERTEX_PENETRATION_BILATERAL);
      behav3.setMethod(Method.VERTEX_PENETRATION_BILATERAL);
      behav4.setMethod(Method.VERTEX_PENETRATION_BILATERAL);
      behav3.setDamping (0.01);
      behav3.setCompliance (0.0001);
      
      behav4.setDamping (0.01);
      behav4.setCompliance (0.0001);
      
      getCollisionManager ().setColliderType (ColliderType.AJL_CONTOUR);
      
      behav1.setMethod (Method.DEFAULT);
      behav1.setName ("caps_cond_left");
      setCollisionBehavior (rigidBodies ().get ("jaw"), (FemModel3d)models().get ("capsule_l"), behav1);
      
      behav2.setMethod (Method.DEFAULT);
      behav2.setName ("caps_cond_right");
      setCollisionBehavior (rigidBodies ().get ("jaw_resected"), (FemModel3d)models().get ("capsule_r"), behav2);
      
      
      
      if (useFEMDisc == true) {
         
         behav3.setName ("caps_disc_right");
         setCollisionBehavior ((FemModel3d)models().get ("disc_right"), (FemModel3d)models().get ("capsule_r"), behav3);
         
         behav4.setName ("caps_disc_left");
         setCollisionBehavior ((FemModel3d)models().get ("disc_left"), (FemModel3d)models().get ("capsule_l"), behav4);      
         
      }
      
      else {
         behav3.setName ("caps_disc_right");
         setCollisionBehavior (rigidBodies().get ("disc_right_rigid"), (FemModel3d)models().get ("capsule_r"), behav3);
         
         behav4.setName ("caps_disc_left");
         setCollisionBehavior (rigidBodies().get ("disc_left_rigid"), (FemModel3d)models().get ("capsule_l"), behav4);  
      }
      

          
     /*
      FrameMarker sphm_mL= new FrameMarker (rigidBodies ().get("jaw"), new Point3d(46.98093, -24.014082, -31.215364));
      sphm_mL.setName ("sphm_mL");
      
      FrameMarker stm_mL= new FrameMarker (rigidBodies ().get("jaw"), new Point3d(51.56613, -15.529182, -53.196934));
      stm_mL.setName ("stm_mL");  

      FrameMarker tm_mL= new FrameMarker (rigidBodies ().get("jaw"), new Point3d(61.51143, -8.8986816, -17.783864));
      tm_mL.setName ("tm_mL");
      
      FrameMarker sphm_mR= new FrameMarker (rigidBodies ().get("jaw"), new Point3d(-39.32017, -18.560082, -33.919064));
      sphm_mR.setName ("sphm_mR");
      
      FrameMarker stm_mR= new FrameMarker (rigidBodies ().get("jaw"), new Point3d(-42.62647, -11.426282, -53.767444));
      stm_mR.setName ("stm_mR");
      
      FrameMarker tm_mR= new FrameMarker (rigidBodies ().get("jaw"), new Point3d(-50.26127, -4.3128816, -21.000664));
      tm_mR.setName ("tm_mR");
      */
      
      
      
      Vector3d centroid= new Vector3d();
      rigidBodies ().get ("jaw").getMesh ().computeCentroid (centroid);
      RigidTransform3d XComToBody = new RigidTransform3d();      
      XComToBody.p.set(centroid);
      RigidTransform3d XBodyToWorld = new RigidTransform3d();
      rigidBodies ().get ("jaw").getPose(XBodyToWorld);
      RigidTransform3d XComToWorld = new RigidTransform3d();
      XComToWorld.mul(XBodyToWorld, XComToBody);
      RigidTransform3d XMeshToCom = new RigidTransform3d();
      XMeshToCom.invert(XComToWorld);
      
     
      Vector3d centroid1= new Vector3d();
      rigidBodies ().get ("jaw_resected").getMesh ().computeCentroid (centroid1);
      RigidTransform3d XComToBody1 = new RigidTransform3d();      
      XComToBody1.p.set(centroid1);
      RigidTransform3d XBodyToWorld1 = new RigidTransform3d();
      rigidBodies ().get ("jaw_resected").getPose(XBodyToWorld1);
      RigidTransform3d XComToWorld1 = new RigidTransform3d();
      XComToWorld1.mul(XBodyToWorld1, XComToBody1);
      RigidTransform3d XMeshToCom1 = new RigidTransform3d();
      XMeshToCom1.invert(XComToWorld1);
      
      
      
      FrameMarker sphm_mL= new FrameMarker (rigidBodies ().get("jaw"), new Point3d(46.98093, -24.014082, -31.215364));
      sphm_mL.setName ("sphm_mL");
      Point3d tmp_loc  = new Point3d ();
      sphm_mL.getLocation (tmp_loc);
      tmp_loc.transform(XMeshToCom);
      sphm_mL.setLocation(tmp_loc);
      
      
      
      FrameMarker stm_mL= new FrameMarker (rigidBodies ().get("jaw"), new Point3d(51.56613, -15.529182, -53.196934));
      stm_mL.setName ("stm_mL");
      stm_mL.getLocation (tmp_loc);
      tmp_loc.transform(XMeshToCom);
      stm_mL.setLocation(tmp_loc);
      

      
      FrameMarker tm_mL= new FrameMarker (rigidBodies ().get("jaw"), new Point3d(61.51143, -8.8986816, -17.783864));
      tm_mL.setName ("tm_mL");
      tm_mL.getLocation (tmp_loc);
      tmp_loc.transform(XMeshToCom);
      tm_mL.setLocation(tmp_loc);
      
     
      
      FrameMarker sphm_mR= new FrameMarker (rigidBodies ().get("jaw_resected"), new Point3d(-39.32017, -18.560082, -33.919064));
      sphm_mR.setName ("sphm_mR");
      Point3d sphm_loc  = new Point3d ();
      sphm_mR.getLocation(sphm_loc);
      sphm_loc.transform(XMeshToCom1);
      sphm_mR.setLocation(sphm_loc);
      //sphm_mR.setLocation(new Point3d (1.37317, 4.14462, 0.120469));
      
      

      FrameMarker stm_mR= new FrameMarker (rigidBodies ().get("jaw_resected"), new Point3d(-42.62647, -11.426282, -53.767444));
      stm_mR.setName ("stm_mR");
      Point3d stm_loc  = new Point3d ();
      stm_mR.getLocation(stm_loc);
      stm_loc.transform(XMeshToCom1);
      stm_mR.setLocation(stm_loc);
      //stm_mR.setLocation( new Point3d (-3.06595, 12.3885, -13.5142));
      
      
      
      FrameMarker tm_mR= new FrameMarker (rigidBodies ().get("jaw_resected"), new Point3d(-50.26127, -4.3128816, -21.000664));
      tm_mR.setName ("tm_mR");
      Point3d tm_loc  = new Point3d ();
      tm_mR.getLocation(tm_loc);
      tm_loc.transform(XMeshToCom1);
      tm_mR.setLocation(tm_loc);
      //tm_mR.setLocation(new Point3d(-9.56666, 18.3916, 13.0395));
      


      FrameMarker sphm_sL= new FrameMarker (rigidBodies ().get("skull"), new Point3d(32.7444, 13.1171, -18.2766));
      sphm_sL.setName ("sphm_sL");
      FrameMarker stm_sL= new FrameMarker (rigidBodies ().get("skull"), new Point3d(38.4851, 17.5523, -27.0278));
      stm_sL.setName ("stm_sL");
      FrameMarker tm_sL= new FrameMarker (rigidBodies ().get("skull"), new Point3d(64.6107, 0.542694, -10.6094));
      tm_sL.setName ("tm_sL");
      FrameMarker sphm_sR= new FrameMarker (rigidBodies ().get("skull"), new Point3d(-27.0814, 10.9547, -12.5527));
      sphm_sR.setName ("sphm_sR");
      FrameMarker stm_sR= new FrameMarker (rigidBodies ().get("skull"), new Point3d(-38.954, 21.3211, -29.1513));
      stm_sR.setName ("stm_sR");
      FrameMarker tm_sR= new FrameMarker (rigidBodies ().get("skull"), new Point3d(-61.9129, 5.388, -14.0374));
      tm_sR.setName ("tm_sR");
      
      RigidEllipsoid ell_l = new RigidEllipsoid ("tm_L_wrap", 3*4, 5,5, 0);
      ell_l.setDynamic (false);
      ell_l.setPose (new RigidTransform3d (new Vector3d(60.9648, -9.88043, -11.2545),new AxisAngle (-0.0098936, 0.97489, -0.22246, Math.toRadians (131.01))));
      ell_l.getRenderProps ().setFaceStyle (FaceStyle.NONE);
      ell_l.getRenderProps ().setLineColor (Color.BLUE);
      ell_l.getRenderProps ().setDrawEdges (true);
      addRigidBody (ell_l);
      attachFrame(ell_l, rigidBodies ().get ("jaw"));

            
      RigidEllipsoid ell_r = new RigidEllipsoid ("tm_R_wrap", 3*4, 5,5, 0);
      ell_r.setDynamic (false);
      ell_r.setPose (new RigidTransform3d (new Vector3d(-49.2518, -6.33017, -14.1949),new AxisAngle (0.015407, -0.96632, 0.25689 , Math.toRadians (108.75))));
      ell_r.getRenderProps ().setFaceStyle (FaceStyle.NONE);
      ell_r.getRenderProps ().setLineColor (Color.BLUE);
      ell_r.getRenderProps ().setDrawEdges (true);
      addRigidBody (ell_r);
      //RenderProps.setVisible(ell_r, false);
      attachFrame(ell_r, rigidBodies ().get ("jaw_resected"));
        
      sphm_R.setFirstPoint (sphm_mR);
      sphm_R.setSecondPoint (sphm_sR);
      sphm_R.setMaterial (capsule_ligament_material);
      sphm_R.getRenderProps ().setLineStyle (LineStyle.LINE);
      sphm_R.setRestLength (sphm_R.getLength () + sphm_slack);
      sphm_R.getRenderProps ().setLineColor (Color.GREEN);
      sphm_R.getRenderProps ().setLineStyle (LineStyle.CYLINDER);
      sphm_R.getRenderProps ().setLineRadius (0.75);
      
      stm_R.setFirstPoint (stm_mR);
      stm_R.setSecondPoint (stm_sR);
      stm_R.setMaterial (capsule_ligament_material);
      stm_R.getRenderProps ().setLineStyle (LineStyle.LINE);
      stm_R.setRestLength (stm_R.getLength () + stm_slack);
      stm_R.getRenderProps ().setLineColor (Color.GREEN);
      stm_R.getRenderProps ().setLineStyle (LineStyle.CYLINDER);
      stm_R.getRenderProps ().setLineRadius (0.75);
      
      MultiPointSpring tm_R = new MultiPointSpring ("tm_R");
      tm_R.addPoint (tm_mR);
      tm_R.setSegmentWrappable (30);
      tm_R.addPoint (tm_sR);      
      tm_R.setMaterial (capsule_ligament_material);
      tm_R.getRenderProps ().setLineStyle (LineStyle.LINE);
      tm_R.setRestLength (22.8);
      tm_R.getRenderProps ().setLineColor (Color.GREEN);
      tm_R.getRenderProps ().setLineStyle (LineStyle.CYLINDER);
      tm_R.getRenderProps ().setLineRadius (0.75);
      tm_R.addWrappable (ell_r);
      
      AxialSpring sphm_L = new AxialSpring ("sphm_L");
      sphm_L.setFirstPoint (sphm_mL);
      sphm_L.setSecondPoint (sphm_sL);
      sphm_L.setMaterial (capsule_ligament_material);
      sphm_L.getRenderProps ().setLineStyle (LineStyle.LINE);
      sphm_L.setRestLength (sphm_L.getLength () + sphm_slack);
      sphm_L.getRenderProps ().setLineColor (Color.GREEN);
      sphm_L.getRenderProps ().setLineStyle (LineStyle.CYLINDER);
      sphm_L.getRenderProps ().setLineRadius (0.75);
      
      AxialSpring stm_L = new AxialSpring ("stm_L");
      stm_L.setFirstPoint (stm_mL);
      stm_L.setSecondPoint (stm_sL);
      stm_L.setMaterial (capsule_ligament_material);
      stm_L.getRenderProps ().setLineStyle (LineStyle.LINE);
      stm_L.setRestLength (stm_L.getLength () + stm_slack);
      stm_L.getRenderProps ().setLineColor (Color.GREEN);
      stm_L.getRenderProps ().setLineStyle (LineStyle.CYLINDER);
      stm_L.getRenderProps ().setLineRadius (0.75);
      
      MultiPointSpring tm_L = new MultiPointSpring ("tm_L");
      tm_L.addPoint (tm_mL);
      tm_L.setSegmentWrappable (30);
      tm_L.addPoint (tm_sL);      
      tm_L.setMaterial (capsule_ligament_material);
      tm_L.getRenderProps ().setLineStyle (LineStyle.LINE);
      tm_L.setRestLength (22.8);
      tm_L.getRenderProps ().setLineColor (Color.GREEN);
      tm_L.getRenderProps ().setLineStyle (LineStyle.CYLINDER);
      tm_L.getRenderProps ().setLineRadius (0.75);
      tm_L.setSegmentWrappable (30);
      tm_L.addWrappable ((Wrappable)rigidBodies ().get ("jaw"));
     
      
      addAxialSpring (sphm_R);
      addAxialSpring (stm_R);
      addMultiPointSpring (tm_R);
      addAxialSpring (sphm_L);
      addAxialSpring (stm_L);
      addMultiPointSpring (tm_L);
      
      
      addFrameMarker (sphm_mL);
      addFrameMarker (stm_mL);
      addFrameMarker (tm_mL);
      addFrameMarker (sphm_mR);
      addFrameMarker (stm_mR);
      addFrameMarker (tm_mR);
      addFrameMarker (sphm_sL);
      addFrameMarker (stm_sL);
      addFrameMarker (tm_sL);
      addFrameMarker (sphm_sR);
      addFrameMarker (stm_sR);
      addFrameMarker (tm_sR);
      
   }

   // needed for scan/load
   public JawModelFEM () throws IOException {
      super();
   }

   
   public JawModelFEM (String name) throws IOException {
      super();
      this.setName (name);

      setGravity(0, 0, -gravityVal * unitConversion);


      JawModel.muscleList = readStringList(ArtisynthPath.getSrcRelativePath(JawModelFEM.class,"geometry/"+muscleListFilename));
      JawModel.bodyInfoList = readBodyInfoList(ArtisynthPath.getSrcRelativePath(JawModelFEM.class,"geometry/"+bodyListFilename));
      femInfoList = readFemInfoList(ArtisynthPath.getSrcRelativePath(JawModelFEM.class,"geometry/"+femListFilename));
      
      JawModel.muscleInfo = readMuscleInfo(ArtisynthPath.getSrcRelativePath(JawModelFEM.class,"geometry/"+muscleInfoFilename));
      JawModel.muscleGroupInfo = readMuscleGroupsInfo(ArtisynthPath.getSrcRelativePath(JawModelFEM.class,"geometry/"+muscleGroupInfoFilename));
      ArrayList<RigidBody> bodies = JawModel.assembleRigidBodies(bodyInfoList, amiraTranformation,ArtisynthPath.getSrcRelativePath (JawModelFEM.class, ""));
      
      
      for (RigidBody body : bodies){
         addRigidBody (body);
      }
      
      
      String meshFilename = (ArtisynthPath.getSrcRelativePath(JawModelFEM.class,"geometry"+"/mandible_resected_left_with_cartilage2.obj"));
      PolygonalMesh mesh = new PolygonalMesh();
      try {
         mesh.read(new BufferedReader(new FileReader(meshFilename)));
      } catch (IOException e) {
         e.printStackTrace();
         return;
      }
      mesh.setFixed(true);
      mesh.triangulate ();
      mesh.transform (amiraTranformation);
    
      rigidBodies ().get ("jaw").addMesh (mesh);

      
      String meshFilename2 = (ArtisynthPath.getSrcRelativePath(JawModelFEM.class,"geometry"+"/mandible_resected_right_with_cartilage2.obj"));
      PolygonalMesh mesh2 = new PolygonalMesh();
      try {
         mesh2.read(new BufferedReader(new FileReader(meshFilename2)));
      } catch (IOException e) {
         e.printStackTrace();
         return;
      }
      mesh2.setFixed(true);
      mesh2.triangulate ();
      mesh2.transform (amiraTranformation);
      
      rigidBodies ().get ("jaw_resected").addMesh (mesh2);

     
      setNewJawDynamicProps();
      setNewResectedJawDynamicProps ();
      

   
      
      ArrayList<FrameMarker>markers = JawModel.assembleMarkers(muscleList, muscleInfo, myRigidBodies, amiraTranformation, ArtisynthPath.getSrcRelativePath (JawModelFEM.class, ""));
      HashMap<String, FrameMarker> myMarkerInfo = new LinkedHashMap<String, FrameMarker>();
      for (FrameMarker marker : markers){
         addFrameMarker (marker);
         myMarkerInfo.put(marker.getName (),marker);
      }   
      

      createFoodContact();
      createCondyleFramemarkers();
     
      
      
      if (recType.equals ("bright")){
         
         addBRightScarSprings();

      }
      else if (recType.equalsIgnoreCase ("bsright")) {
       
         addBSRightScarSprings();
         
      }
      
      else if (recType.equalsIgnoreCase ("rbsright")) {
         
         addRBSRightScarSprings();
         
      }
      
      else if (recType.equalsIgnoreCase ("rbright")) {
         
         addRBRightScarSprings();
         
      }
      
      else if (recType.equalsIgnoreCase ("bleft")) {
         
         addBleftScarSprings();
         
      }
      
      else if (recType.equalsIgnoreCase ("bsleft")) {
         
         addBSleftScarSprings();
         
      }
      else if (recType.equalsIgnoreCase ("rbleft")) {
         
         addRBleftScarSprings();
         
      }
      else if (recType.equalsIgnoreCase ("rbsleft")) {
         
         addRBSleftScarSprings();
         
      }
      
  else if (recType.equalsIgnoreCase ("bsb")) {
         
         addBSBScarSprings();
         
      }
      
      
      
      

      ArrayList<Muscle> myAssembledMuscles = JawModel.assembleandreturnMuscles();
      myAttachedMuscles = JawModel.attachMuscles(muscleList, muscleInfo, myMarkerInfo, myAssembledMuscles);
      HashMap<String, ExcitationComponent> myMuscles = new LinkedHashMap<String, ExcitationComponent>();
      
      for (Muscle muscle : myAttachedMuscles){
         muscle.setExcitationColor (Color.RED);
         muscle.setMaxColoredExcitation (1);
         addAxialSpring (muscle);
         myMuscles.put (muscle.getName (), muscle);
      }
      
      wrappedMuscleList = readStringList(ArtisynthPath.getSrcRelativePath(JawModelFEM.class,"geometry/"+wrappedMuscleListFilename));
      
      if(useFEMDisc==true){
         assembleFemModels();
         rigidBodies().get ("disc_right_rigid").setDynamic (false);
         rigidBodies().get ("disc_left_rigid").setDynamic (false);
         rigidBodies().get ("disc_right_rigid").setMass (.006);
         rigidBodies().get ("disc_left_rigid").setMass (.006);
      }
      else {
         
         rigidBodies().get ("disc_right_rigid").setDynamic (false);
         rigidBodies().get ("disc_left_rigid").setDynamic (false);
         rigidBodies().get ("disc_right_rigid").setMass (.006);
         rigidBodies().get ("disc_left_rigid").setMass (.006);
         
      } 
      
    
      for (RigidBody body : bodies){
         
            translateFrame(body);
        
      }
     

  
      /*
      for (RigidBody body : bodies){
      
         if (body != rigidBodies ().get ("jaw"))
         {
            translateFrame(body);
         }
        
      }
      */
    
   
      closerMuscleList = JawModel.createMuscleList(readStringList(ArtisynthPath.getSrcRelativePath(JawModelFEM.class,"geometry/"+"closerMuscleList.txt")), muscleInfo, myAttachedMuscles);
      assembleBilateralExcitors(muscleList, muscleInfo, myMuscles, muscleAbbreviations);
      HashMap<String,MuscleExciter>  exciters= new LinkedHashMap<String,MuscleExciter>  ();      
      for(MuscleGroupInfo info : muscleGroupInfo){
         exciters=assembleMuscleGroups(info, myMuscles, getMuscleExciters (),  muscleAbbreviations);
         addMuscleExciter (exciters.get ("l"+info.name));
         addMuscleExciter (exciters.get ("r"+info.name));
      }
      
      ArrayList<MuscleExciter> bilateral_exciters= new ArrayList<> ();
      bilateral_exciters=assemblebilateralMuscleGroups(muscleGroupInfo, getMuscleExciters (), muscleAbbreviations);      
      
      for(MuscleExciter exciter : bilateral_exciters){
         addMuscleExciter (exciter);
       }
      
      JawModel.updateMuscleLengthProps(myAttachedMuscles);
      if(useFEMJoint==true){
         addCartilage();
         addCapsule();
         
         // FOR Simplified Version
         //attachLigaments();
      }     
      
      if(useFEMJoint==false){
         //attach cartilage to respective bones
         attachFrame (rigidBodies ().get ("mandible_cartilage_right"), rigidBodies ().get ("jaw_resected"));
         attachFrame (rigidBodies ().get ("mandible_cartilage_left"), rigidBodies ().get ("jaw"));
         attachFrame (rigidBodies ().get ("skull_cartilage_right"), rigidBodies ().get ("skull"));
         attachFrame (rigidBodies ().get ("skull_cartilage_left"), rigidBodies ().get ("skull"));
            
         rigidBodies ().get ("mandible_cartilage_right").setMass (0);
         rigidBodies ().get ("mandible_cartilage_left").setMass (0);
         rigidBodies ().get ("skull_cartilage_right").setMass (0);
         rigidBodies ().get ("skull_cartilage_left").setMass (0);
         
         addFixedMarkers();

      }
          
     
      createBiteConstraints();
      
      setupRenderProps();
      
      //attachFrame (rigidBodies ().get ("jaw_resected"), rigidBodies ().get ("jaw"));

      
      if (usePlate ==  true) {
         
         rigidBodies ().get ("plate").setDensity (0.00000442);
         rigidBodies ().get ("plate").setDynamic (true);
         attachFrame (rigidBodies ().get ("plate"), rigidBodies ().get ("jaw"));

         RenderProps.setFaceColor (rigidBodies ().get ("plate"), Color.GRAY);
      }
         else {
        
            removeRigidBody (rigidBodies().get("plate"));
            
       }
   
      editMuscleMaxForce();
    
   }
   
   
   public void addBSBScarSprings()
   {
      addMidScarSpring ("ant_scar", new Point3d(-0.065037822, -70.39615, -65.119159));
      addMidScarSpring ("med_scar", new Point3d(6.5819631, -69.876826, -64.435753));
      addMidScarSpring ("post_scar", new Point3d(13.579149, -69.320039, -63.512411));
   }
   
   public void addRBSleftScarSprings()
   {
      addLeftScarSpring ("ant_scar", new Point3d(22.799607, -56.236447, -65.119159));
      addLeftScarSpring ("med_scar", new Point3d(26.642532, -50.788122, -64.435753));
      addLeftScarSpring ("post_scar", new Point3d(30.680137, -45.046313, -63.512411));
   }

   
   public void addRBleftScarSprings()
   {
      addLeftScarSpring ("ant_scar", new Point3d(31.752744, -48.801105, -65.119159));
      addLeftScarSpring ("med_scar", new Point3d(35.717033, -43.440439, -64.435753));
      addLeftScarSpring ("post_scar", new Point3d(39.882542, -37.790735, -63.512411));
   }

   
   public void addBSleftScarSprings()
   {
      addLeftScarSpring ("ant_scar", new Point3d(13.961641, -70.89199, -65.119159));
      addLeftScarSpring ("med_scar", new Point3d(17.92593 ,-65.531324, -64.435753));
      addLeftScarSpring ("post_scar", new Point3d(22.091439, -59.88162, -63.512411));
   }
   
   
   public void addBleftScarSprings()
   {
      addLeftScarSpring ("ant_scar", new Point3d(22.685756, -60.987878, -65.119159));
      addLeftScarSpring ("med_scar", new Point3d(26.650045, -55.627212, -64.435753));
      addLeftScarSpring ("post_scar", new Point3d(30.815554, -49.977508, -63.512411));
   }
   
   
   public void addRBRightScarSprings()
   {
      addRightScarSpring ("ant_scar", new Point3d(-25.092224, -46.646418, -65.119159));
      addRightScarSpring ("med_scar", new Point3d(-27.546633, -41.149303, -64.435753));
      addRightScarSpring ("post_scar", new Point3d(-30.096884, -35.634337, -63.512411));
   }
   

   public void addRBSRightScarSprings()
   {
      addRightScarSpring ("ant_scar", new Point3d(-14.462292, -58.111754, -65.119159));
      addRightScarSpring ("med_scar", new Point3d(-17.738106, -52.304744, -64.435753));
      addRightScarSpring ("post_scar", new Point3d(-21.178114, -46.186168, -63.512411));
   }
   
   public void addBSRightScarSprings()
   {
      addRightScarSpring ("ant_scar", new Point3d(-4.3146676, -69.167172, -65.119159));
      addRightScarSpring ("med_scar", new Point3d(-8.2147951, -63.759647, -64.435753));
      addRightScarSpring ("post_scar", new Point3d(-12.312684, -58.060706, -63.512411));
   }
   
   public void addBRightScarSprings()
   {
      addRightScarSpring ("ant_scar", new Point3d(-15.463112, -62.631761, -65.119159));
      addRightScarSpring ("med_scar", new Point3d(-17.740297, -56.365442, -64.435753));
      addRightScarSpring ("post_scar", new Point3d(-20.128253, -49.764814, -63.512411));
   }
   
   
   
   public void addMidScarSpring(String name, Point3d loc)
   {
      FrameMarker insertion_point = new FrameMarker();
      insertion_point.setName(name+"_insertion");

      RigidBody jaw = rigidBodies ().get ("jaw");
      RigidBody fixed = rigidBodies ().get ("skull");
      addFrameMarker(insertion_point, jaw, loc);
     
      Point3d offset = new Point3d (-2.95, 22.49, -8.74);
         
      
      loc.x = loc.x + offset.x -.3 ;
      loc.y = loc.y + offset.y;
      loc.z = loc.z + offset.z;
     
    
      RigidTransform3d X = new RigidTransform3d();
      X.p.set (loc);

      
      
      ScarSpring scar = new ScarSpring(name);
      scar.setFrameB (fixed);
      scar.setAttachFrameB (X);
      scar.setCollidingPoint (insertion_point);
      scar.setPlaneSize (10.0);
      scar.setMaterial (new RotAxisFrameMaterial (defaultScarK/3, 0, 10, 0));
      addFrameSpring (scar);
      
      RenderProps.setFaceColor (scar, new Color(0.8f,0.8f,1f));
      RenderProps.setLineColor (scar, new Color(0.5f,0.5f,1f));
      RenderProps.setPointColor (insertion_point, new Color(0.2f,0.2f,1f));
   }
   
   
   
   
   public void addLeftScarSpring(String name, Point3d loc)
   {
      FrameMarker insertion_point = new FrameMarker();
      insertion_point.setName(name+"_insertion");

      RigidBody jaw = rigidBodies ().get ("jaw");
      RigidBody fixed = rigidBodies ().get ("skull");
      addFrameMarker(insertion_point, jaw, loc);
     
      Point3d offset = new Point3d (-2.95, 22.49, -8.74);
         
      
      loc.x = loc.x + offset.x ;
      loc.y = loc.y + offset.y;
      loc.z = loc.z + offset.z;
     
    
      RigidTransform3d X = new RigidTransform3d();
      X.p.set (loc);

      
      
      ScarSpring scar = new ScarSpring(name);
      scar.setFrameB (fixed);
      scar.setAttachFrameB (X);
      scar.setCollidingPoint (insertion_point);
      scar.setPlaneSize (10.0);
      scar.setMaterial (new RotAxisFrameMaterial (defaultScarK/3, 0, 10, 0));
      addFrameSpring (scar);
      
      RenderProps.setFaceColor (scar, new Color(0.8f,0.8f,1f));
      RenderProps.setLineColor (scar, new Color(0.5f,0.5f,1f));
      RenderProps.setPointColor (insertion_point, new Color(0.2f,0.2f,1f));
   }
   
   
   public void addRightScarSpring(String name, Point3d loc)
   {
      FrameMarker insertion_point = new FrameMarker();
      insertion_point.setName(name+"_insertion");

      RigidBody jaw = rigidBodies ().get ("jaw");
      RigidBody fixed = rigidBodies ().get ("skull");
      addFrameMarker(insertion_point, jaw, loc);
     
      Point3d offset = new Point3d (-3.46, 22.49, -8.74);
         
      
      loc.x = loc.x + offset.x ;
      loc.y = loc.y + offset.y;
      loc.z = loc.z + offset.z;
     
    
      RigidTransform3d X = new RigidTransform3d();
      X.p.set (loc);

      
      
      ScarSpring scar = new ScarSpring(name);
      scar.setFrameB (fixed);
      scar.setAttachFrameB (X);
      scar.setCollidingPoint (insertion_point);
      scar.setPlaneSize (10.0);
      scar.setMaterial (new RotAxisFrameMaterial (defaultScarK/3, 0, 10, 0));
      addFrameSpring (scar);
      
      RenderProps.setFaceColor (scar, new Color(0.8f,0.8f,1f));
      RenderProps.setLineColor (scar, new Color(0.5f,0.5f,1f));
      RenderProps.setPointColor (insertion_point, new Color(0.2f,0.2f,1f));
   }
   
   void editMuscleMaxForce(){

      for (Muscle m : myAttachedMuscles) {   
         AxialMuscleMaterial m_material = (AxialMuscleMaterial) m.getMaterial ();
         double max_force = m_material.getMaxForce ();
         m_material.setMaxForce (max_force*PCAscaling);
      }
   
   }
   
   
   public void createCondyleFramemarkers(){
     
      /*
      Vector3d centroid= new Vector3d();
      rigidBodies ().get ("jaw").getMesh ().computeCentroid (centroid);
      RigidTransform3d XComToBody = new RigidTransform3d();      
      XComToBody.p.set(centroid);
      RigidTransform3d XBodyToWorld = new RigidTransform3d();
      rigidBodies ().get ("jaw").getPose(XBodyToWorld);
      RigidTransform3d XComToWorld = new RigidTransform3d();
      XComToWorld.mul(XBodyToWorld, XComToBody);
      RigidTransform3d XMeshToCom = new RigidTransform3d();
      XMeshToCom.invert(XComToWorld);
      */
      
      FrameMarker condyleLeft = new FrameMarker(rigidBodies ().get ("jaw"), new Point3d (54.80443, -6.4923816, -1.3121639));
    /*
     Point3d tmp_loc  = new Point3d ();
     condyleLeft.getLocation (tmp_loc);
     tmp_loc.transform(XMeshToCom);
     condyleLeft.setLocation(tmp_loc);
     */
     
      FrameMarker condyleright = new FrameMarker(rigidBodies ().get ("jaw_resected"), new Point3d (-45.08047, -4.3705816, -3.1194639));
    /*
     Point3d tmp_loc2  = new Point3d ();
     condyleright.getLocation (tmp_loc2);
     tmp_loc2.transform(XMeshToCom);
     condyleright.setLocation(tmp_loc2);
     */
     
     condyleLeft.setName ("CondyleLeft");
     condyleright.setName ("CondyleRight");
     
     addFrameMarker (condyleLeft);
     addFrameMarker (condyleright);

     RenderProps.setPointColor (condyleLeft,Color.RED);
     RenderProps.setPointColor (condyleright,Color.RED);
   
     
      
   }
   

   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens) throws IOException {
   rtok.nextToken();
   if (scanAttributeName (rtok, "EFContact")) {
      EFContact = new ElasticFoundationForceBehavior();
      EFContact.scan (rtok, tokens);
      return true;
   }
   rtok.pushBack();
   return super.scanItem (rtok, tokens);
   }

   protected void writeItems (PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor) throws IOException {
           super.writeItems (pw, fmt, ancestor);
           if (EFContact != null) {
                   pw.print ("EFContact=");
                   EFContact.write (pw, fmt, ancestor);
           }
   }
}