# My code to understand 
from abaqus import *
from abaqusConstants import *
import sketch
import part
from mesh import *
import math
import random
from sketch import *
from connectorBehavior import *
session.journalOptions.setValues(replayGeometry=COORDINATE, recoverGeometry=COORDINATE) # To get ride of getSequenceFromMask in abaqus scripting 


# Decision on the number of inclusions within the RVE based on the given volume fraction of the inclusions
 
# v_f = 0.1                                                                            # Given volume fraction of the inclusions, here is 20%
# d = 0.15                                                                             # Diameter of the incluions
# n_total_Inclusions = (24*v_f*a*b*c)/(4*pi*d**3)                                      # Calculating the number of inclusions
# n_total_Inclusions = math.floor(n_total_Inclusions)                                  # Rounding the calculated 'n_total_Inclusions'
# n_total_Inclusions = int(n_total_Inclusions)                                         # Conversion of float to integer to be used in the loop
# print ('The number of inclusion is:' , n_total_Inclusions)
d = 0.13
n_total_Inclusions = 40
# Identifying the coordinates of the random Inclusions
X=[]             # X coordinate of the inclusion without knowing if the minimum distance criteria is met or not
Y=[]             # Y coordinate of the inclusion without knowing if the minimum distance criteria is met or not
Z=[]             # Z coordinate of the inclusion without knowing if the minimum distance criteria is met or not
 
x = []           # x coordinate of the inclusion that satisfy the minimum distance criteria
y = []           # y coordinate of the inclusion that satisfy the minimum distance criteria
z = []           # z coordinate of the inclusion that satisfy the minimum distance criteria 

               # matrix of distance between each generated coordinates
min_distance = 0.15          # Defined minimum distance between each coordinate points      
n_Inclusions = 0            
min_dis_Inclusions=[]


# Generating the matrix
a=1   # 'a' represents the lenght of the box
b=1   # 'b' represents the width of the box
c=1   # 'c' represent the depth of the box
myModel = mdb.Model(name='Model A')
mySketch = myModel.ConstrainedSketch(name='Sketch A', sheetSize=10.0)
mySketch.rectangle(point1=(0.0, 0.0), point2=(a, b))
myPart=myModel.Part(name = 'Matrix', dimensionality=THREE_D, type=DEFORMABLE_BODY)
myPart.BaseSolidExtrude(sketch=mySketch, depth=c)




while (n_Inclusions < n_total_Inclusions):

    random_x = random.uniform (d/2+0.05, 0.9)
    random_y = random.uniform (d/2+0.05, 0.9)
    random_z = random.uniform (d/2+0.05, 0.9)
    X.append(random_x)
    Y.append(random_y)
    Z.append(random_z)
    distance = []
    
    for j in range (0,len(X)-1):
        
        distance_Inclusions = sqrt((random_x-X[j])**2 + (random_y-Y[j])**2 + (random_z-Z[j])**2)
        distance.append(distance_Inclusions)
        min_dis_Inclusions = min(distance)
    if ( min_dis_Inclusions > min_distance):
        x.append(random_x)
        y.append(random_y)
        z.append(random_z)
        n_Inclusions+=1            
    else: 
        continue

  

# Generating the spherical inclusions 
for k in range(0,n_total_Inclusions):

    myModel.ConstrainedSketch(name='Sketch-B%d' %(k), sheetSize=10.0)
    myModel.sketches['Sketch-B%d' %(k)].ConstructionLine(point1=(0.0, -5.0), point2=(0.0, 5.0))
    myModel.sketches['Sketch-B%d' %(k)].FixedConstraint(entity= mdb.models['Model A'].sketches['Sketch-B%d' %(k)].geometry[2])
    myModel.sketches['Sketch-B%d' %(k)].Arc3Points(point1=(0.0, d/2), point2=(0.0, -d/2), point3=(d/2, 0.0))
    myModel.sketches['Sketch-B%d' %(k)].Line(point1=(0.0, d/2), point2=(0.0, -d/2))
    myModel.Part(dimensionality=THREE_D, name='Inclusion-%d' %(k), type=DEFORMABLE_BODY)
    myModel.parts['Inclusion-%d' %(k)].BaseSolidRevolve(angle=360.0, flipRevolveDirection=OFF, sketch=mdb.models['Model A'].sketches['Sketch-B%d' %(k)])
    del myModel.sketches['Sketch-B%d' %(k)]

# Generating the spherical holes within the matrix solid

part_Matrix = myModel.parts['Matrix']
for i in range (0,n_total_Inclusions):
    # Step 1: Creation of Datum plans
    datum_plan = part_Matrix.DatumPlaneByPrincipalPlane(offset = z[i], principalPlane=XYPLANE)

    # Step 2: Generation of the extrude profile w.r.t generated datum plans
    myModel.ConstrainedSketch(gridSpacing=0.09, name='Sketch S%i' %(i), 
        sheetSize=4, transform = part_Matrix.MakeSketchTransform( sketchPlane= part_Matrix.datums[datum_plan.id], 
        sketchPlaneSide=SIDE1, sketchUpEdge= part_Matrix.edges.findAt((1, 0.5, 1), ), sketchOrientation=RIGHT, origin=(x[i], y[i], z[i])))
        
    myModel.sketches['Sketch S%i' %(i)].Arc3Points(point1=(0, d/2), point2=(0, -d/2), point3=(d/2, 0))
    myModel.sketches['Sketch S%i' %(i)].Line(point1=(0, d/2), point2=(0, -d/2))
    myModel.sketches['Sketch S%i' %(i)].ConstructionLine(point1=(0, 1), point2=(0, -1))        

    
    
    part_Matrix.projectReferencesOntoSketch(filter=COPLANAR_EDGES, sketch = myModel.sketches['Sketch S%i' %(i)])
    

    
    part_Matrix.CutRevolve(angle=360.0, flipRevolveDirection=OFF, sketch=
       myModel.sketches['Sketch S%i' %(i)], sketchOrientation=RIGHT, 
       sketchPlane=part_Matrix.datums[datum_plan.id], sketchPlaneSide=SIDE1, sketchUpEdge= part_Matrix.edges.findAt((1, 0.50, 1), ))
     
    mdb.models['Model A'].sketches['Sketch S%i' %(i)].delete(objectList=(mdb.models['Model A'].sketches['Sketch S%i' %(i)].geometry[4], ))
    
    del myModel.sketches['Sketch S%i' %(i)]  
    
# Generating the assembly

myAssembly = myModel.rootAssembly
myCoordinateSys = myAssembly. DatumCsysByDefault(CARTESIAN)
#myAssembly.Instance(dependent=ON, name='Matrix', part=myModel.parts['Matrix'])

for m in range (0,n_total_Inclusions):
   myAssembly.Instance(dependent=ON, name='Matrix', part=myModel.parts['Matrix'])
   myAssembly.Instance(dependent=ON, name='Inclusion-%d' %(m), part=myModel.parts['Inclusion-%d' %(m)])
   myAssembly.translate(instanceList=('Inclusion-%d' %(m), ), vector=(x[m], y[m], z[m]))


## Merging the incluions and matrix to create the composite 
# instance_Matrix = myAssembly.instances['Matrix']
# instances_assembly = [instance_Matrix]

# for i in range(0,n_total_Inclusions):
    # instances_assembly.append(myAssembly.instances['Inclusion-%i' %(i)])

# myAssembly.InstanceFromBooleanMerge(domain=GEOMETRY, instances =(instances_assembly), keepIntersections=ON, name='Composite', originalInstances=SUPPRESS)


# Discretization of the inclusions contains the following steps: 
for i in range (0,n_total_Inclusions):

    # Step 1: Seeding the inclusions
    myModel.parts['Inclusion-%i' %(i)].seedPart(deviationFactor=0.1, minSizeFactor=0.1, size=d/10)
    
    # Step 2: Determing the element type
    myModel.parts['Inclusion-%i' %(i)].setElementType(elemTypes=(ElemType(elemCode=C3D8, elemLibrary=STANDARD), ElemType(elemCode=C3D6, elemLibrary=STANDARD), ElemType(elemCode=C3D4, elemLibrary=STANDARD)), 
    regions=(myModel.parts['Inclusion-%i' %(i)].cells.findAt(((0.0,0.0,0.0), ), ), ))
        
    # Step 3: Assigning the meshing technique    
    myModel.parts['Inclusion-%i' %(i)].setMeshControls(elemShape=TET, regions = myModel.parts['Inclusion-%i' %(i)].cells.findAt(((0.0,0.0,0.0), ), ), technique=FREE)
    
    # Step 4: Discretization of each inclusion
    myModel.parts['Inclusion-%i' %(i)].generateMesh()
    
    # Step 5: Generation of node and element sets
    element = myModel.parts['Inclusion-%i' %(i)].elements.getByBoundingBox(-a, -b, -c, a, b,c)
    myModel.parts['Inclusion-%i' %(i)].Set(elements=element, name='Inclusion-%i-elem' %(i))    
    
    node = myModel.parts['Inclusion-%i' %(i)].nodes.getByBoundingBox(-a, -b, -c, a, b, c)
    myModel.parts['Inclusion-%i' %(i)].Set(nodes=node, name='Inclusion-%i-node' %(i))
   

# Discretization of the matrix: 

# Step 1: Seeding the inclusions
part_Matrix.seedPart(deviationFactor=0.1, minSizeFactor=0.1, size=a/10)
    
# Step 2: Determing the element type and mesh technique
part_Matrix.setMeshControls(allowMapped=False, regions = part_Matrix.cells.findAt(((0, 0, 0), )), elemShape=TET)
part_Matrix.setElementType(elemTypes=(ElemType(elemCode=C3D8, elemLibrary=STANDARD), ElemType(elemCode=C3D6, elemLibrary=STANDARD), ElemType(elemCode=C3D4, elemLibrary=STANDARD)), 
regions=(part_Matrix.cells.findAt(((0.0,0.0,0.0), ), ), ))
           
# Step 4: Discretization of each inclusion
part_Matrix.generateMesh()

# Step 5: Generation of node and element sets
element = part_Matrix.elements.getByBoundingBox(-a, -b, -c, a, b,c) 
node = part_Matrix.nodes.getByBoundingBox(-a, -b, -c, a, b,c) 
part_Matrix.Set(elements=element, name='Matrix-elem')
part_Matrix.Set(nodes=node, name='Matrix-node')

#LET'S CREATE JOBS 
mdb.Job(activateLoadBalancing=False, atTime=None, contactPrint=OFF, 
    description='model-1', echoPrint=OFF, explicitPrecision=SINGLE, 
    getMemoryFromAnalysis=True, historyPrint=OFF, memory=90, memoryUnits=
    PERCENTAGE, model='Model A', modelPrint=OFF, multiprocessingMode=DEFAULT, 
    name='Job-1', nodalOutputPrecision=SINGLE, numCpus=1, numDomains=1, 
    numGPUs=0, parallelizationMethodExplicit=DOMAIN, queue=None, resultsFormat=
    ODB, scratch='', type=ANALYSIS, userSubroutine='', waitHours=0, 
    waitMinutes=0)

mdb.jobs['Job-1'].writeInput()

