package artisynth.JawModel;


import java.awt.Color;

import artisynth.core.femmodels.FemCutPlane;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.fields.ScalarNodalField;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.modelbase.Monitor;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.renderables.ColorBar;
import artisynth.core.util.ScalarRange;
import artisynth.core.workspace.RootModel;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.color.ColorMap;
import maspack.render.color.ColorMapBase;
import maspack.render.color.HueColorMap;
import maspack.render.color.RainbowColorMap;
import maspack.util.DoubleInterval;



/**
 * Illustrates visualization of a scalar nodal field.
 */
public class  ScalarVisual extends RootModel {
   
   FemModel3d fem;
   ScalarNodalField field;
   FemCutPlane cutplane;
   
private class StressUpdateMonitor extends MonitorBase {
   private FemModel3d fem;
   private ScalarNodalField field;

   
   public StressUpdateMonitor(FemModel3d fem, ScalarNodalField field) {
      this.fem = fem;
      this.field = field;
   }

   @Override
   public void apply(double t0, double t1) {
      // Update the scalar field based on current nodal stresses
      for (FemNode3d n : fem.getNodes()) {
         double stressValue = n.getStress().get(0, 0);  // Assuming we're interested in the xx component of the stress tensor
         field.setValue(n, 2*t1);
      }
   }

   @Override
   public void initialize(double t) {
      // Initialization code if needed
   }
}




@Override
   public void build (String[] args) {
      MechModel mech = new MechModel ("mech");
      addModel (mech);

      // create a hex FEM cylinder to use for the field
      fem = FemFactory.createHexCylinder (
         null, /*height=*/1.0, /*radius=*/0.5, /*nh=*/5, /*nt=*/10);
      fem.setMaterial (new LinearMaterial (10000, 0.45));
      fem.setName ("fem");
      mech.addModel (fem);

      // fix the top nodes of the FEM
      for (FemNode3d n : fem.getNodes()) {
         if (n.getPosition().z == 0.5) {
            n.setDynamic (false);
         }
      }

      
      fem.setComputeNodalStress (true);
      
      // create a scalar field whose value is r^2, where r is the radial
      // distance from FEM axis
      field = new ScalarNodalField (fem);
      fem.addField (field);
      for (FemNode3d n : fem.getNodes()) {
         field.setValue (n, n.getStress ().get (0, 0));
         
         //field.setValue (n, n.getPosition ().z);
      }
      
      // create a FemCutPlane to provide the visualization surface, rotated
      // into the z-x plane.
       cutplane = new FemCutPlane (
         new RigidTransform3d (0,0,0, 0,0,Math.toRadians(90)));
      fem.addCutPlane (cutplane);

      // set the field's visualization and the cut plane to it as a render mesh
      field.setVisualization (ScalarNodalField.Visualization.SURFACE);
      field.addRenderMeshComp (cutplane);
      field.setRenderRange (new ScalarRange (ScalarRange.Updating.FIXED));
      field.setRenderRange (new ScalarRange (0, 50));
     
      //RainbowColorMap rainbowMap = new RainbowColorMap();
      //field.setColorMap(hueColorMap);
      
      
       ColorBar cbar = new ColorBar ();
       cbar. setName("colorBar");
       cbar. setNumberFormat ("%.2f"); // 2 decimal places
       cbar. populateLabels (0.0 , 50, 10); // Start with range [0,1], 10 ticks
       cbar. setLocation (-100, 0.1, 20, 0.8) ;
       addRenderable (cbar);
       cbar. setColorMap (field. getColorMap ());
      
       
      /*
       // create a control panel to set properties
      ControlPanel panel = new ControlPanel();
      panel.addWidget (field, "visualization");
      panel.addWidget (field, "renderRange");
      panel.addWidget (field, "colorMap");
      addControlPanel (panel);
       */
       
      // set render properties
      // set FEM line color to render edges blue grey:
      RenderProps.setLineColor (fem, new Color (0.7f, 0.7f, 1f));
      // make cut plane visible via its coordinate axes; make surface invisible
      // to avoid conflicting with field rendering:
      cutplane.setSurfaceRendering (SurfaceRender.None);
      cutplane.setAxisLength (0.4);
      RenderProps.setLineWidth (cutplane, 2);
      // for point visualization: render points as spheres with radius 0.01
      RenderProps.setSphericalPoints (field, 0.02, Color.GRAY); // color ignored
      
      addMonitor(new StressUpdateMonitor(fem, field));

   }
   

@Override
public void prerender ( RenderList list ) {
  super.prerender (list );
  ColorBar cbar = ( ColorBar)( renderables ().get("colorBar"));
  cbar.setColorMap (field.getColorMap ());
  cbar.updateLabels (field.getValueRange ().getLowerBound (),field.getValueRange ().getUpperBound());
  System.out.println (field.getValueRange ().getUpperBound());
}
     
   
 
}
