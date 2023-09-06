##libraries
## import librarys
from abaqus import *
from abaqusConstants import *
import os
import math
import section
import regionToolset
import displayGroupMdbToolset as dgm
import part
import material
import assembly
import step
import interaction
import load
import mesh
import optimization
import job
import sketch
import visualization
import xyPlot
import displayGroupOdbToolset as dgo
import connectorBehavior
####
##create model
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=1000.0)
g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=STANDALONE)
s.CircleByCenterPerimeter(center=(300.0, 0.0), point1=(0.0, 0.0))
p = mdb.models['Model-1'].Part(name='Cylinder', dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
p = mdb.models['Model-1'].parts['Cylinder']
p.BaseShellExtrude(sketch=s, depth=705.0)
s.unsetPrimaryObject()
p = mdb.models['Model-1'].parts['Cylinder']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
del mdb.models['Model-1'].sketches['__profile__']

###### Create Cylindrical coordinate system
p = mdb.models['Model-1'].parts['Cylinder']
p.DatumCsysByThreePoints(name='Cylindrical_CSYS', coordSysType=CYLINDRICAL, 
	origin=(0.0, 0.0, 0.0), line1=(0.0, 1.0, 0.0), line2=(-1.0, 0.0, 0.0))

################ Partition###########
###datum plane
HH=5.0								
for i in range(140):								
	p = mdb.models['Model-1'].parts['Cylinder']
	p.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=HH)
	HH=HH+5.0

################### Formulation ##########
## inputs A - phi - w - alpha
#data = [(69.74,0.139626,0.84,0.01623156),(22.38,0.1523672,0.06,0.78312924)]
data = [(71.3008,0.0174864538,0.1,0.78),(34.7835,1.5708,0.05,0.00872665)]  ### radian
h = 705
teta_0 = []
teta_1 = []
for x in range(0, 705, 5):
    ans = []
    for k in range(len(data)):
        sample = data[k]
        y = sample[0] * sin((sample[2] * (2 * pi / h) * x) + sample[1]) + tan(sample[3]) * x
        moshtagh = sample[0] * sample[2] * (2 * math.pi / h) * math.cos(
            (sample[2] * (2 * math.pi / h) * x) + sample[1]) + math.tan(sample[3])
        teta = math.atan(moshtagh)  #### radian
        teta = teta * 57.295827  #### degree
        ans.append(teta)
    teta_0.append(ans[0])
    teta_1.append(ans[1])
########################### create section ###################
for j in range(141):
	##create section
	#   
	sectionLayer1 = section.SectionLayer(material='AL', thickness=0.181, 
			orientAngle=teta_0[j], numIntPts=3, plyName='')
	sectionLayer2 = section.SectionLayer(material='AL', thickness=0.181, 
			orientAngle=-teta_0[j], numIntPts=3, plyName='')
	sectionLayer3 = section.SectionLayer(material='AL', thickness=0.181, 
			orientAngle=teta_1[j], numIntPts=3, plyName='')
	sectionLayer4 = section.SectionLayer(material='AL', thickness=0.181, 
			orientAngle=-teta_1[j], numIntPts=3, plyName='')
	##Layer definition
	mdb.models['Model-1'].CompositeShellSection(name='Section-'+str(j), preIntegrate=OFF, 
			idealization=NO_IDEALIZATION, symmetric=True, thicknessType=UNIFORM, 
			poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT, 
			useDensity=OFF, integrationRule=SIMPSON, layup=(sectionLayer1, 
			sectionLayer2, sectionLayer3, sectionLayer4, ))
			
##################Partitioning###############
##########
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#1 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[3], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#2 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[4], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#4 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[5], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#8 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[6], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#10 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[7], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#20 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[8], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#40 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[9], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#80 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[10], faces=pickedFaces)
##8 ta
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#100 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[11], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#200 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[12], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#400 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[13], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#800 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[14], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#1000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[15], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#2000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[16], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#4000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[17], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#8000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[18], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#10000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[19], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#20000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[20], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#40000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[21], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#80000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[22], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#100000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[23], faces=pickedFaces)
##21 ta
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#200000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[24], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#400000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[25], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#800000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[26], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#1000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[27], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#2000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[28], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#4000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[29], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#8000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[30], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#10000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[31], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#20000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[32], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#40000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[33], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#80000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[34], faces=pickedFaces)
#32 ta
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #1 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[35], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #2 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[36], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #4 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[37], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #8 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[38], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #10 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[39], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #20 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[40], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #40 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[41], faces=pickedFaces)
##39 ta
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #80 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[42], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #100 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[43], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #200 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[44], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #400 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[45], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #800 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[46], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #1000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[47], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #2000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[48], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #4000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[49], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #8000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[50], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #10000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[51], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #20000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[52], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #40000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[53], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #80000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[54], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #100000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[55], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #200000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[56], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #400000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[57], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #800000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[58], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #1000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[59], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #2000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[60], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #4000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[61], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #8000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[62], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #10000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[63], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #20000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[64], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #40000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[65], faces=pickedFaces)
########63 ta
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0 #80000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[66], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #1 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[67], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #2 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[68], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #4 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[69], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #8 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[70], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #10 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[71], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #20 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[72], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #40 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[73], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #80 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[74], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #100 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[75], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #200 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[76], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #400 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[77], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #800 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[78], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #1000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[79], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #2000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[80], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #4000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[81], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #8000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[82], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #10000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[83], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #20000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[84], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #40000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[85], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #80000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[86], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #100000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[87], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #200000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[88], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #400000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[89], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #800000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[90], faces=pickedFaces)
###88 ta
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #1000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[91], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #2000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[92], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #4000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[93], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #8000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[94], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #10000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[95], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #20000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[96], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #40000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[97], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:2 #80000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[98], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #1 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[99], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #2 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[100], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #4 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[101], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #8 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[102], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #10 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[103], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #20 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[104], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #40 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[105], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #80 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[106], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #100 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[107], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #200 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[108], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #400 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[109], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #800 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[110], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #1000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[111], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #2000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[112], faces=pickedFaces)
###110 ta
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #4000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[113], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #8000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[114], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #10000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[115], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #20000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[116], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #40000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[117], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #80000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[118], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #100000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[119], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #200000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[120], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #400000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[121], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #800000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[122], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #1000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[123], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #2000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[124], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #4000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[125], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #8000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[126], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #10000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[127], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #20000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[128], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #40000000 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[129], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:3 #80000000 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[130], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:4 #1 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[131], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:4 #2 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[132], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:4 #4 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[133], faces=pickedFaces)
###131 ta
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:4 #8 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[134], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:4 #10 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[135], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:4 #20 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[136], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:4 #40 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[137], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:4 #80 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[138], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:4 #100 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[139], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:4 #200 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[140], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:4 #400 ]', ), )
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[141], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
pickedFaces = f.getSequenceFromMask(mask=('[#0:4 #800 ]', ), )
d = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d[142], faces=pickedFaces)

####################################################################
###################  End Of Partitioning ###########################
################### Material Orientation ###########################

p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #800 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #400 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #200 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #100 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #80 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #40 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #20 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #10 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #8 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #4 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #2 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #1 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #80000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #40000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #20000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #10000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #8000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #4000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #2000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #1000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #800000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #400000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #200000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #100000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #80000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #40000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #20000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #10000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #8000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #4000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #2000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #1000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #800 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #400 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #200 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #100 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #80 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #40 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #20 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #10 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #8 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #4 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #2 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #1 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #80000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #40000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #20000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #10000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #8000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #4000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #2000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #1000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #800000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #400000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #200000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #100000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #80000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #40000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #20000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #10000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #8000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #4000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #2000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #1000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #800 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #400 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #200 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #100 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #80 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #40 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #20 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #10 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #8 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #4 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #2 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #1 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #80000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #40000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #20000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #10000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #8000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #4000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #2000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #1000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #800000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #400000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #200000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #100000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #80000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #40000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #20000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #10000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #8000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #4000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #2000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #1000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #800 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #400 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #200 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #100 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #80 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #40 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #20 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #10 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #8 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #4 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #2 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #1 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#80000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#40000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#20000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#10000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#8000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#4000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#2000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#1000000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#800000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#400000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#200000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#100000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#80000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#40000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#20000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#10000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#8000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#4000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#2000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#1000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#800 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#400 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#200 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#100 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#80 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#40 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#20 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#10 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#8 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#4 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#2 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#1 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #1000 ]', ), )
region = regionToolset.Region(faces=faces)
orientation = mdb.models['Model-1'].parts['Cylinder'].datums[2]
mdb.models['Model-1'].parts['Cylinder'].MaterialOrientation(region=region, 
	orientationType=SYSTEM, axis=AXIS_2, localCsys=orientation, 
	fieldName='', additionalRotationType=ROTATION_NONE, angle=0.0, 
	additionalRotationField='')
##########################################################################
#################### End of Orientation ##################################
############# Create Material ############

mdb.models['Model-1'].Material(name='AL')
mdb.models['Model-1'].materials['AL'].Elastic(type=LAMINA, table=((141.0, 10.3, 
	0.33, 4.5, 4.5, 4.5), ))
####### 
################# Assign Section #################
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #800 ]', ), )
region = p.Set(faces=faces, name='Set-142')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-0', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #400 ]', ), )
region = p.Set(faces=faces, name='Set-143')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #200 ]', ), )
region = p.Set(faces=faces, name='Set-144')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-2', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #100 ]', ), )
region = p.Set(faces=faces, name='Set-145')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-3', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #80 ]', ), )
region = p.Set(faces=faces, name='Set-146')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-4', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #40 ]', ), )
region = p.Set(faces=faces, name='Set-147')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-5', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #20 ]', ), )
region = p.Set(faces=faces, name='Set-148')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-6', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #10 ]', ), )
region = p.Set(faces=faces, name='Set-149')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-7', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #8 ]', ), )
region = p.Set(faces=faces, name='Set-150')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-8', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #4 ]', ), )
region = p.Set(faces=faces, name='Set-151')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-9', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #2 ]', ), )
region = p.Set(faces=faces, name='Set-152')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-10', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #1 ]', ), )
region = p.Set(faces=faces, name='Set-153')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-11', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #80000000 ]', ), )
region = p.Set(faces=faces, name='Set-154')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-12', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #40000000 ]', ), )
region = p.Set(faces=faces, name='Set-155')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-13', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #20000000 ]', ), )
region = p.Set(faces=faces, name='Set-156')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-14', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #10000000 ]', ), )
region = p.Set(faces=faces, name='Set-157')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-15', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #8000000 ]', ), )
region = p.Set(faces=faces, name='Set-158')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-16', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #4000000 ]', ), )
region = p.Set(faces=faces, name='Set-159')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-17', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #2000000 ]', ), )
region = p.Set(faces=faces, name='Set-160')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-18', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #1000000 ]', ), )
region = p.Set(faces=faces, name='Set-161')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-19', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #800000 ]', ), )
region = p.Set(faces=faces, name='Set-162')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-20', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #400000 ]', ), )
region = p.Set(faces=faces, name='Set-163')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-21', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #200000 ]', ), )
region = p.Set(faces=faces, name='Set-164')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-22', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #100000 ]', ), )
region = p.Set(faces=faces, name='Set-165')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-23', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #80000 ]', ), )
region = p.Set(faces=faces, name='Set-166')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-24', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #40000 ]', ), )
region = p.Set(faces=faces, name='Set-167')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-25', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #20000 ]', ), )
region = p.Set(faces=faces, name='Set-168')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-26', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #10000 ]', ), )
region = p.Set(faces=faces, name='Set-169')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-27', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #8000 ]', ), )
region = p.Set(faces=faces, name='Set-170')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-28', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #4000 ]', ), )
region = p.Set(faces=faces, name='Set-171')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-29', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #2000 ]', ), )
region = p.Set(faces=faces, name='Set-172')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-30', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #1000 ]', ), )
region = p.Set(faces=faces, name='Set-173')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-31', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #800 ]', ), )
region = p.Set(faces=faces, name='Set-174')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-32', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #400 ]', ), )
region = p.Set(faces=faces, name='Set-175')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-33', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #200 ]', ), )
region = p.Set(faces=faces, name='Set-176')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-34', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #100 ]', ), )
region = p.Set(faces=faces, name='Set-177')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-35', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #80 ]', ), )
region = p.Set(faces=faces, name='Set-178')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-36', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #40 ]', ), )
region = p.Set(faces=faces, name='Set-179')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-37', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #20 ]', ), )
region = p.Set(faces=faces, name='Set-180')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-38', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #10 ]', ), )
region = p.Set(faces=faces, name='Set-181')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-39', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #8 ]', ), )
region = p.Set(faces=faces, name='Set-182')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-40', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #4 ]', ), )
region = p.Set(faces=faces, name='Set-183')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-41', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #2 ]', ), )
region = p.Set(faces=faces, name='Set-184')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-42', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:3 #1 ]', ), )
region = p.Set(faces=faces, name='Set-185')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-43', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #80000000 ]', ), )
region = p.Set(faces=faces, name='Set-186')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-44', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #40000000 ]', ), )
region = p.Set(faces=faces, name='Set-187')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-45', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #20000000 ]', ), )
region = p.Set(faces=faces, name='Set-188')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-46', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #10000000 ]', ), )
region = p.Set(faces=faces, name='Set-189')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-47', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #8000000 ]', ), )
region = p.Set(faces=faces, name='Set-190')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-48', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #4000000 ]', ), )
region = p.Set(faces=faces, name='Set-191')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-49', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #2000000 ]', ), )
region = p.Set(faces=faces, name='Set-192')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-50', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #1000000 ]', ), )
region = p.Set(faces=faces, name='Set-193')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-51', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #800000 ]', ), )
region = p.Set(faces=faces, name='Set-194')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-52', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #400000 ]', ), )
region = p.Set(faces=faces, name='Set-195')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-53', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #200000 ]', ), )
region = p.Set(faces=faces, name='Set-196')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-54', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #100000 ]', ), )
region = p.Set(faces=faces, name='Set-197')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-55', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #80000 ]', ), )
region = p.Set(faces=faces, name='Set-198')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-56', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #40000 ]', ), )
region = p.Set(faces=faces, name='Set-199')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-57', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #20000 ]', ), )
region = p.Set(faces=faces, name='Set-200')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-58', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #10000 ]', ), )
region = p.Set(faces=faces, name='Set-201')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-59', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #8000 ]', ), )
region = p.Set(faces=faces, name='Set-202')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-60', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #4000 ]', ), )
region = p.Set(faces=faces, name='Set-203')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-61', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #2000 ]', ), )
region = p.Set(faces=faces, name='Set-204')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-62', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #1000 ]', ), )
region = p.Set(faces=faces, name='Set-205')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-63', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #800 ]', ), )
region = p.Set(faces=faces, name='Set-206')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-64', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #400 ]', ), )
region = p.Set(faces=faces, name='Set-207')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-65', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #200 ]', ), )
region = p.Set(faces=faces, name='Set-208')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-66', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #100 ]', ), )
region = p.Set(faces=faces, name='Set-209')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-67', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #80 ]', ), )
region = p.Set(faces=faces, name='Set-210')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-68', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #40 ]', ), )
region = p.Set(faces=faces, name='Set-211')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-69', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #20 ]', ), )
region = p.Set(faces=faces, name='Set-212')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-70', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #10 ]', ), )
region = p.Set(faces=faces, name='Set-213')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-71', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #8 ]', ), )
region = p.Set(faces=faces, name='Set-214')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-72', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #4 ]', ), )
region = p.Set(faces=faces, name='Set-215')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-73', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #2 ]', ), )
region = p.Set(faces=faces, name='Set-216')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-74', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:2 #1 ]', ), )
region = p.Set(faces=faces, name='Set-217')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-75', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #80000000 ]', ), )
region = p.Set(faces=faces, name='Set-218')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-76', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #40000000 ]', ), )
region = p.Set(faces=faces, name='Set-219')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-77', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #20000000 ]', ), )
region = p.Set(faces=faces, name='Set-220')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-78', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #10000000 ]', ), )
region = p.Set(faces=faces, name='Set-221')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-79', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #8000000 ]', ), )
region = p.Set(faces=faces, name='Set-222')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-80', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #4000000 ]', ), )
region = p.Set(faces=faces, name='Set-223')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-81', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #2000000 ]', ), )
region = p.Set(faces=faces, name='Set-224')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-82', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #1000000 ]', ), )
region = p.Set(faces=faces, name='Set-225')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-83', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #800000 ]', ), )
region = p.Set(faces=faces, name='Set-226')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-84', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #400000 ]', ), )
region = p.Set(faces=faces, name='Set-227')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-85', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #200000 ]', ), )
region = p.Set(faces=faces, name='Set-228')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-86', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #100000 ]', ), )
region = p.Set(faces=faces, name='Set-229')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-87', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #80000 ]', ), )
region = p.Set(faces=faces, name='Set-230')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-88', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #40000 ]', ), )
region = p.Set(faces=faces, name='Set-231')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-89', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #20000 ]', ), )
region = p.Set(faces=faces, name='Set-232')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-90', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #10000 ]', ), )
region = p.Set(faces=faces, name='Set-233')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-91', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #8000 ]', ), )
region = p.Set(faces=faces, name='Set-234')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-92', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #4000 ]', ), )
region = p.Set(faces=faces, name='Set-235')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-93', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #2000 ]', ), )
region = p.Set(faces=faces, name='Set-236')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-94', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #1000 ]', ), )
region = p.Set(faces=faces, name='Set-237')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-95', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #800 ]', ), )
region = p.Set(faces=faces, name='Set-238')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-96', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #400 ]', ), )
region = p.Set(faces=faces, name='Set-239')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-97', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #200 ]', ), )
region = p.Set(faces=faces, name='Set-240')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-98', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #100 ]', ), )
region = p.Set(faces=faces, name='Set-241')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-99', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #80 ]', ), )
region = p.Set(faces=faces, name='Set-242')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-100', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #40 ]', ), )
region = p.Set(faces=faces, name='Set-243')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-101', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #20 ]', ), )
region = p.Set(faces=faces, name='Set-244')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-102', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #10 ]', ), )
region = p.Set(faces=faces, name='Set-245')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-103', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #8 ]', ), )
region = p.Set(faces=faces, name='Set-246')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-104', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #4 ]', ), )
region = p.Set(faces=faces, name='Set-247')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-105', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #2 ]', ), )
region = p.Set(faces=faces, name='Set-248')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-106', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0 #1 ]', ), )
region = p.Set(faces=faces, name='Set-249')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-107', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#80000000 ]', ), )
region = p.Set(faces=faces, name='Set-250')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-108', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#40000000 ]', ), )
region = p.Set(faces=faces, name='Set-251')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-109', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#20000000 ]', ), )
region = p.Set(faces=faces, name='Set-252')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-110', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#10000000 ]', ), )
region = p.Set(faces=faces, name='Set-253')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-111', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#8000000 ]', ), )
region = p.Set(faces=faces, name='Set-254')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-112', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#4000000 ]', ), )
region = p.Set(faces=faces, name='Set-255')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-113', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#2000000 ]', ), )
region = p.Set(faces=faces, name='Set-256')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-114', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#1000000 ]', ), )
region = p.Set(faces=faces, name='Set-257')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-115', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#800000 ]', ), )
region = p.Set(faces=faces, name='Set-258')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-116', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#400000 ]', ), )
region = p.Set(faces=faces, name='Set-259')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-117', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#200000 ]', ), )
region = p.Set(faces=faces, name='Set-260')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-118', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#100000 ]', ), )
region = p.Set(faces=faces, name='Set-261')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-119', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#80000 ]', ), )
region = p.Set(faces=faces, name='Set-262')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-120', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#40000 ]', ), )
region = p.Set(faces=faces, name='Set-263')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-121', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#20000 ]', ), )
region = p.Set(faces=faces, name='Set-264')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-122', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#10000 ]', ), )
region = p.Set(faces=faces, name='Set-265')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-123', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#8000 ]', ), )
region = p.Set(faces=faces, name='Set-266')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-124', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#4000 ]', ), )
region = p.Set(faces=faces, name='Set-267')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-125', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#2000 ]', ), )
region = p.Set(faces=faces, name='Set-268')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-126', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#1000 ]', ), )
region = p.Set(faces=faces, name='Set-269')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-127', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#800 ]', ), )
region = p.Set(faces=faces, name='Set-270')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-128', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#400 ]', ), )
region = p.Set(faces=faces, name='Set-271')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-129', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#200 ]', ), )
region = p.Set(faces=faces, name='Set-272')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-130', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#100 ]', ), )
region = p.Set(faces=faces, name='Set-273')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-131', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#80 ]', ), )
region = p.Set(faces=faces, name='Set-274')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-132', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#40 ]', ), )
region = p.Set(faces=faces, name='Set-275')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-133', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#20 ]', ), )
region = p.Set(faces=faces, name='Set-276')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-134', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#10 ]', ), )
region = p.Set(faces=faces, name='Set-277')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-135', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#8 ]', ), )
region = p.Set(faces=faces, name='Set-278')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-136', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#4 ]', ), )
region = p.Set(faces=faces, name='Set-279')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-137', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#2 ]', ), )
region = p.Set(faces=faces, name='Set-280')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-138', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#1 ]', ), )
region = p.Set(faces=faces, name='Set-281')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-139', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
p = mdb.models['Model-1'].parts['Cylinder']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#0:4 #1000 ]', ), )
region = p.Set(faces=faces, name='Set-282')
p = mdb.models['Model-1'].parts['Cylinder']
p.SectionAssignment(region=region, sectionName='Section-140', offset=0.0, 
	offsetType=MIDDLE_SURFACE, offsetField='', 
	thicknessAssignment=FROM_SECTION)
##########################################################################################
################# End Of Assign Section #################################################
### hide datum planes
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
        datumPoints=OFF, datumAxes=OFF, datumPlanes=OFF, datumCoordSystems=OFF)
session.viewports['Viewport: 1'].assemblyDisplay.geometryOptions.setValues(
        datumPoints=OFF, datumAxes=OFF, datumPlanes=OFF, datumCoordSystems=OFF)
################ Assembly ################
a = mdb.models['Model-1'].rootAssembly
a.DatumCsysByDefault(CARTESIAN)
p = mdb.models['Model-1'].parts['Cylinder']
a.Instance(name='Cylinder-1', part=p, dependent=OFF)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
	adaptiveMeshConstraints=ON)
	
################ Step ###################
mdb.models['Model-1'].BuckleStep(name='Buckling', previous='Initial', 
	numEigen=1, vectors=2, maxIterations=1000)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(step='Buckling')
session.viewports['Viewport: 1'].assemblyDisplay.setValues(mesh=ON, 
	adaptiveMeshConstraints=OFF)
session.viewports['Viewport: 1'].assemblyDisplay.meshOptions.setValues(
	meshTechnique=ON)
	
############### Meshing ##################
a = mdb.models['Model-1'].rootAssembly
partInstances =(a.instances['Cylinder-1'], )
a.seedPartInstance(regions=partInstances, size=5.0, deviationFactor=0.1, 
	minSizeFactor=0.1)
a = mdb.models['Model-1'].rootAssembly
partInstances =(a.instances['Cylinder-1'], )
a.generateMesh(regions=partInstances)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(mesh=OFF, 
	interactions=ON, constraints=ON, connectors=ON, engineeringFeatures=ON)
session.viewports['Viewport: 1'].assemblyDisplay.meshOptions.setValues(
	meshTechnique=OFF)

############# 2 RP #############
a = mdb.models['Model-1'].rootAssembly
v1 = a.instances['Cylinder-1'].vertices
a.ReferencePoint(point=v1[140])
session.viewports['Viewport: 1'].view.fitView()
a = mdb.models['Model-1'].rootAssembly
v11 = a.instances['Cylinder-1'].vertices
a.ReferencePoint(point=v11[141])
session.viewports['Viewport: 1'].view.fitView()
session.viewports['Viewport: 1'].assemblyDisplay.setValues(mesh=ON)

################# 2 Sets Of Nodes ############
a = mdb.models['Model-1'].rootAssembly
n1 = a.instances['Cylinder-1'].nodes
nodes1 = n1.getSequenceFromMask(mask=(
	'[#0:4 #2000 #0:1656 #ffffffc0 #ffffffff:10 #3fffffff ]', ), )
a.Set(nodes=nodes1, name='Top_node')
session.viewports['Viewport: 1'].assemblyDisplay.setValues(mesh=OFF)
session.viewports['Viewport: 1'].view.fitView()
session.viewports['Viewport: 1'].assemblyDisplay.setValues(mesh=ON)
a = mdb.models['Model-1'].rootAssembly
n1 = a.instances['Cylinder-1'].nodes
nodes1 = n1.getSequenceFromMask(mask=(
	'[#0:4 #1000 #0:1644 #ffffc000 #ffffffff:11 #3f ]', ), )
a.Set(nodes=nodes1, name='Bot_node')
session.viewports['Viewport: 1'].assemblyDisplay.setValues(mesh=OFF)

############ Constraint ##################
a = mdb.models['Model-1'].rootAssembly
region4=a.sets['Bot_node']
a = mdb.models['Model-1'].rootAssembly
r1 = a.referencePoints
refPoints1=(r1[5], )
region1=regionToolset.Region(referencePoints=refPoints1)
mdb.models['Model-1'].RigidBody(name='Bot_cons', refPointRegion=region1, 
	tieRegion=region4)
session.viewports['Viewport: 1'].view.fitView()
a = mdb.models['Model-1'].rootAssembly
region4=a.sets['Top_node']
a = mdb.models['Model-1'].rootAssembly
r1 = a.referencePoints
refPoints1=(r1[6], )
region1=regionToolset.Region(referencePoints=refPoints1)
mdb.models['Model-1'].RigidBody(name='Top_cons', refPointRegion=region1, 
	tieRegion=region4)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=ON, bcs=ON, 
	predefinedFields=ON, interactions=OFF, constraints=OFF, 
	engineeringFeatures=OFF)
	
############## Load ##############
a = mdb.models['Model-1'].rootAssembly
r1 = a.referencePoints
refPoints1=(r1[6], )
region = a.Set(referencePoints=refPoints1, name='Set-5')
mdb.models['Model-1'].ConcentratedForce(name='Load-1', 
	createStepName='Buckling', region=region, cf3=-1.0, 
	distributionType=UNIFORM, field='', localCsys=None)

########### Boundry Condition #############
a = mdb.models['Model-1'].rootAssembly
r1 = a.referencePoints
refPoints1=(r1[6], )
region = a.Set(referencePoints=refPoints1, name='Set-6')
mdb.models['Model-1'].DisplacementBC(name='BC_Top', createStepName='Buckling', 
	region=region, u1=0.0, u2=0.0, u3=UNSET, ur1=0.0, ur2=0.0, ur3=0.0, 
	amplitude=UNSET, buckleCase=PERTURBATION_AND_BUCKLING, fixed=OFF, 
	distributionType=UNIFORM, fieldName='', localCsys=None)
a = mdb.models['Model-1'].rootAssembly
r1 = a.referencePoints
refPoints1=(r1[5], )
region = a.Set(referencePoints=refPoints1, name='Set-7')
mdb.models['Model-1'].EncastreBC(name='BC_bot', createStepName='Buckling', 
	region=region, localCsys=None)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=OFF, bcs=OFF, 
	predefinedFields=OFF, connectors=OFF)
	
##################### Job ###########################
mdb.Job(name='Job20-1', model='Model-1', description='', type=ANALYSIS, 
	atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
	memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
	explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
	modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
	scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=1, 
	numGPUs=0)
