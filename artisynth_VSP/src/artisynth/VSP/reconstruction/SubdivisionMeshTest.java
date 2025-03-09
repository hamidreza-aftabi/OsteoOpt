package artisynth.VSP.reconstruction;

import java.awt.Color;
import java.io.*;
import java.util.*;

import artisynth.core.workspace.*;
import artisynth.core.renderables.*;
import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.*;
import artisynth.core.femmodels.*;
import artisynth.core.materials.*;
import artisynth.core.probes.*;
import artisynth.core.gui.*;

import maspack.util.*;
import maspack.matrix.*;
import maspack.geometry.*;
import maspack.render.*;
import maspack.render.Renderer.*;
import maspack.properties.*;

public class SubdivisionMeshTest extends RootModel {

   public void build (String[] args) {
      MechModel mech = new MechModel ("mech");
      addModel (mech);

      PolygonalMesh mesh = MeshFactory.createQuadBox (
         2.0, 1.0, 1.0, new Point3d(), 2, 1, 2);

      SubdivisionMesh smesh = new SubdivisionMesh (mesh);
      mech.addRenderable (smesh);
      
      ControlPanel panel = new ControlPanel();
      panel.addWidget (smesh, "flatDivisions");
      panel.addWidget (smesh, "subDivisions");
      addControlPanel (panel);
   }
}
