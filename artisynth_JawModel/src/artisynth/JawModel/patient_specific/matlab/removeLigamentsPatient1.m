sphm_R = ah1.find ('models/jawmodel/axialSprings/sphm_R');
sphm_R_parent = sphm_R.getParent();
sphm_R_parent.remove (sphm_R);

sphm_R = ah1.find ('models/jawmodel/frameMarkers/sphm_sR');
sphm_R_parent = sphm_R.getParent();
sphm_R_parent.remove (sphm_R);

sphm_R = ah1.find ('models/jawmodel/frameMarkers/sphm_mR');
sphm_R_parent = sphm_R.getParent();
sphm_R_parent.remove (sphm_R);




stm_R = ah1.find ('models/jawmodel/axialSprings/stm_R');
stm_R_parent = stm_R.getParent();
stm_R_parent.remove (stm_R);

stm_R = ah1.find ('models/jawmodel/frameMarkers/stm_mR');
stm_R_parent = stm_R.getParent();
stm_R_parent.remove (stm_R);

stm_R = ah1.find ('models/jawmodel/frameMarkers/stm_sR');
stm_R_parent = stm_R.getParent();
stm_R_parent.remove (stm_R);

